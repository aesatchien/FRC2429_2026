"""
Scope2429 - 2429's FRC robot logs and live NetworkTables data in one dashboard layout.

  * Log files:   open a .wpilog (a match, or any shop log), scrub through it, optionally sync a match video
                 to it and render an MP4.
  * Live robot:  connect to a roboRIO (10.24.29.2), a simulator (localhost) or any NT4 server by address.

Run:  python scope2429.py
"""
from __future__ import annotations

import base64
import os
import queue
import re
import subprocess
import sys
import threading
import time
import traceback
import tkinter as tk
from pathlib import Path
from tkinter import ttk, filedialog, messagebox

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "helpers"))     # core, dash, telemetry, ... live in helpers/
sys.path.insert(0, HERE)
import core  # noqa: E402

APP_NAME = "Scope2429"
SIZES = {
    "Compact  1280×720": 360,
    "Standard 1920×1080": 540,
    "Large    2560×1440": 720,
}
QUALITY = {"Small file (CRF 30)": 30, "Balanced (CRF 27)": 27, "High (CRF 23)": 23}
GENERIC_FIELD = "(generic drawn field — labelled hub/bump/trench/tower)"
LOG, LIVE = "log", "live"
LIVE_PRESETS = {
    "Robot (roboRIO 10.24.29.2)": "10.24.29.2",
    "Simulator (this computer)": "localhost",
    "roboRIO over USB": "172.22.11.2",
}


class ColorBar(tk.Canvas):
    """A progress bar whose color tells the state: blue while working, green when done, red on an error."""
    COLORS = {"idle": "#c8c8c8", "busy": "#2f7fd8", "done": "#2eaa4a", "error": "#d33a3a"}

    def __init__(self, master, width=260, height=20):
        super().__init__(master, width=width, height=height, highlightthickness=1, highlightbackground="#999",
                         background="#eeeeee")
        self.frac, self.state = 0.0, "idle"
        self.bind("<Configure>", lambda e: self._draw())

    def set(self, frac, state):
        self.frac, self.state = max(0.0, min(1.0, frac)), state
        self._draw()

    def _draw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if self.state in ("done", "error"):
            self.create_rectangle(0, 0, w, h, fill=self.COLORS[self.state], width=0)
            self.create_text(w / 2, h / 2, text="DONE" if self.state == "done" else "ERROR", fill="white",
                             font=("Segoe UI", 9, "bold"))
        elif self.frac > 0:
            self.create_rectangle(0, 0, w * self.frac, h, fill=self.COLORS[self.state], width=0)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(f"{APP_NAME} — robot logs & live NetworkTables")
        self.geometry("1180x780")
        self.minsize(980, 560)
        self.settings = core.load_settings()
        self.q = queue.Queue()
        self.busy = False
        self.cancel_flag = False
        self.ml: core.MatchLog | None = None
        self.video_path: str | None = None
        self.field_obj = None
        self.log_files: list[Path] = []
        self.log_numeric: list[str] = []
        self.session = None              # live.LiveSession
        self.live_dash = None            # dash.DashRenderer fed by the live session
        self.mode = LOG
        self._preview_after = None
        self._preview_img = None
        self._last_frame = None
        self.viewer = None               # the separate, resizable video window (see _open_viewer)
        self._viewer_img = None
        self._viewer_after = None
        self._viewer_dismissed = False   # the user closed the video window: do not bring it back by itself
        self._out_auto = True
        self._live_after = None
        self._live_entries_t = 0.0
        self.trends: list[str] = list(self.settings.get("trends", []))
        self.days: list = []             # webcast.DayVideo: the whole-day event videos
        # clock error per day video, only ever filled by webcast.calibrate (several matches that agree).  The old key 'day_cal'
        # could hold a single wrong match's value (Q13 matched to Q14: +321 s), so it is no longer read.
        self.day_cal: dict = {k: list(v) for k, v in self.settings.get("day_cal_v3", {}).items()}
        self.settings.pop("day_cal", None)
        try:
            import webcast
            self.days = [webcast.DayVideo.from_dict(d) for d in self.settings.get("day_videos", [])]
        except Exception:  # noqa
            self.days = []
        self._build()
        self.after(100, self._poll)
        folder = self.settings.get("log_folder")
        if folder and os.path.isdir(folder):
            self.folder_var.set(folder)
            self._scan_folder()
        self._refresh_fields()
        self._set_mode(self.settings.get("mode", LOG), first=True)

    # ------------------------------------------------------------------ UI
    def _build(self):
        pad = dict(padx=4, pady=2)
        # ---- source selector
        src = ttk.Frame(self); src.pack(fill="x", **pad)
        ttk.Label(src, text="Source:", font=("Segoe UI", 10, "bold")).pack(side="left")
        self.mode_var = tk.StringVar(value=LOG)
        ttk.Radiobutton(src, text="Log files (.wpilog)", value=LOG, variable=self.mode_var,
                        command=lambda: self._set_mode(self.mode_var.get())).pack(side="left", padx=8)
        ttk.Radiobutton(src, text="Live robot / simulator (NetworkTables 4)", value=LIVE, variable=self.mode_var,
                        command=lambda: self._set_mode(self.mode_var.get())).pack(side="left")

        # log source row
        self.log_row = ttk.Frame(self); self.log_row.pack(fill="x", **pad)
        ttk.Label(self.log_row, text="Logs folder").pack(side="left")
        self.folder_var = tk.StringVar()
        ttk.Entry(self.log_row, textvariable=self.folder_var, width=70).pack(side="left", fill="x", expand=True, padx=4)
        ttk.Button(self.log_row, text="Browse…", command=self._browse_folder).pack(side="left")
        ttk.Button(self.log_row, text="Open log file…", command=self._open_log_file).pack(side="left", padx=2)
        ttk.Button(self.log_row, text="Rescan", command=self._scan_folder).pack(side="left")

        # live source row
        self.live_row = ttk.Frame(self)
        ttk.Label(self.live_row, text="Address").pack(side="left")
        self.host_var = tk.StringVar(value=self.settings.get("host", "10.24.29.2"))
        self.host_cb = ttk.Combobox(self.live_row, textvariable=self.host_var, width=26,
                                    values=[f"{v}   ({k})" for k, v in LIVE_PRESETS.items()])
        self.host_cb.pack(side="left", padx=4)
        self.host_cb.bind("<<ComboboxSelected>>", self._host_preset)
        self.connect_btn = ttk.Button(self.live_row, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=2)
        self.live_status = tk.StringVar(value="not connected")
        ttk.Label(self.live_row, textvariable=self.live_status, foreground="#555").pack(side="left", padx=8)
        ttk.Label(self.live_row, text="(IP, host name, host:port or a team number)", foreground="#888").pack(side="left")

        body = ttk.Panedwindow(self, orient="horizontal"); body.pack(fill="both", expand=True, **pad)
        # ---- left: log list / live entries
        lf = ttk.Frame(body); body.add(lf, weight=0)
        self.left_title = tk.StringVar(value="Logs")
        ttk.Label(lf, textvariable=self.left_title).pack(anchor="w")
        self.logs = tk.Listbox(lf, width=38, exportselection=False)
        self.logs.pack(fill="both", expand=True)
        self.logs.bind("<<ListboxSelect>>", lambda e: self._select_log())
        self.entries_lb = tk.Listbox(lf, width=38, exportselection=False)
        self.entries_lb.bind("<Double-Button-1>", lambda e: self._add_trend_from_list())
        self.batch_btn = ttk.Button(lf, text="Batch: all matches (auto)", command=self._batch)
        self.batch_btn.pack(fill="x", pady=(4, 0))
        self.entries_hint = ttk.Label(lf, text="double-click an entry to plot it", foreground="#888")

        # ---- right: controls + preview
        rf = ttk.Frame(body); body.add(rf, weight=1)
        self.info_var = tk.StringVar(value="Select a log on the left.")
        self.info_lbl = ttk.Label(rf, textvariable=self.info_var, font=("Segoe UI", 10, "bold"))
        self.info_lbl.pack(anchor="w")

        g = ttk.LabelFrame(rf, text="Data"); g.pack(fill="x", **pad)
        # the pose entries are fixed (core.DEFAULT_DRIVE / DEFAULT_QUEST); there is nothing to pick
        self.drive_var = tk.StringVar(value=core.DEFAULT_DRIVE)
        self.quest_var = tk.StringVar(value=core.DEFAULT_QUEST)
        ttk.Label(g, text="field image").grid(row=2, column=0, sticky="e")
        self.field_var = tk.StringVar()
        self.field_cb = ttk.Combobox(g, textvariable=self.field_var, width=48, state="readonly")
        self.field_cb.grid(row=2, column=1, sticky="we", **pad)
        self.field_cb.bind("<<ComboboxSelected>>", lambda e: self._field_changed())
        ff = ttk.Frame(g); ff.grid(row=2, column=2, sticky="w")
        ttk.Button(ff, text="Browse config.json…", command=self._browse_field).pack(side="left")
        ttk.Button(ff, text="Download 2026 field", command=self._download_field).pack(side="left", padx=2)
        self.redleft_var = tk.BooleanVar(value=self.settings.get("red_left", True))
        ttk.Checkbutton(g, text="Red alliance on left (rotate field 180°)", variable=self.redleft_var,
                        command=self._field_changed).grid(row=3, column=1, sticky="w")
        g.columnconfigure(1, weight=1)

        # trend plots (top-right slot when there is no match video)
        tr = ttk.LabelFrame(rf, text="Trend plots  (top-right slot when there is no match video)"); tr.pack(fill="x", **pad)
        self.trend_lb = tk.Listbox(tr, height=3, exportselection=False)
        self.trend_lb.grid(row=0, column=0, rowspan=2, sticky="we", **pad)
        tb = ttk.Frame(tr); tb.grid(row=0, column=1, sticky="we")
        self.trend_var = tk.StringVar()
        self.trend_cb = ttk.Combobox(tb, textvariable=self.trend_var, width=50)
        self.trend_cb.pack(side="left", fill="x", expand=True, padx=4)
        self.trend_cb.bind("<Return>", lambda e: self._add_trend())
        ttk.Button(tb, text="Add", command=self._add_trend).pack(side="left")
        ttk.Button(tb, text="Remove", command=self._remove_trend).pack(side="left", padx=2)
        ttk.Button(tb, text="Defaults", command=self._clear_trends).pack(side="left")
        ttk.Label(tr, text="empty list = shooter / feeder rpm, wheel speed and Quest error", foreground="#888"
                  ).grid(row=1, column=1, sticky="w", padx=4)
        tr.columnconfigure(0, weight=1); tr.columnconfigure(1, weight=2)
        self._fill_trend_list()

        # deviation indicators: how far the actual motor rpm / swerve state may be from the set value before it changes color
        dv = ttk.LabelFrame(rf, text="Indicator thresholds  (deviation: color changes when actual vs set differs by more than; battery: PDH volts marks)")
        dv.pack(fill="x", **pad)
        thr0 = self.settings.get("thresholds", {})
        self.thr_vars = {}
        for i, (key, text, default, step, fmt) in enumerate((("rpm", "Motor rpm ±", 500.0, 50, "%.0f"),
                                                             ("speed", "Swerve speed ± m/s", 0.3, 0.05, "%.2f"),
                                                             ("angle", "Swerve angle ± deg", 10.0, 1, "%.0f"),
                                                             ("warn_v", "Battery low (orange) V <", 8.0, 0.1, "%.1f"),
                                                             ("brown_v", "Brownout (red) V <", 7.0, 0.1, "%.1f"))):
            ttk.Label(dv, text=text).grid(row=i // 3, column=2 * (i % 3), sticky="e", padx=(8, 2))
            var = tk.DoubleVar(value=thr0.get(key, default))
            self.thr_vars[key] = var
            sb = ttk.Spinbox(dv, from_=0, to=100000, increment=step, textvariable=var, width=8, format=fmt,
                             command=self._thr_changed)
            sb.grid(row=i // 3, column=2 * (i % 3) + 1, sticky="w")
            sb.bind("<Return>", lambda e: self._thr_changed())
            sb.bind("<FocusOut>", lambda e: self._thr_changed())

        # match video (log mode)
        self.video_frame = v = ttk.LabelFrame(rf, text="Match video  (optional — a log renders fine without one)")
        ttk.Label(v, text="TBA event key").grid(row=0, column=0, sticky="e")
        self.event_var = tk.StringVar(value=self.settings.get("event_key", "2026cass"))
        ttk.Entry(v, textvariable=self.event_var, width=14).grid(row=0, column=1, sticky="w", **pad)
        ttk.Label(v, text="TBA API key (optional)").grid(row=0, column=2, sticky="e")
        self.tbakey_var = tk.StringVar(value=self.settings.get("tba_key", ""))
        ttk.Entry(v, textvariable=self.tbakey_var, width=34, show="•").grid(row=0, column=3, sticky="we", **pad)
        ttk.Button(v, text="Find on TBA", command=self._find_tba).grid(row=0, column=4, **pad)
        ttk.Label(v, text="YouTube URL").grid(row=1, column=0, sticky="e")
        self.url_var = tk.StringVar()
        self.url_cb = ttk.Combobox(v, textvariable=self.url_var, width=60)
        self.url_cb.grid(row=1, column=1, columnspan=3, sticky="we", **pad)
        bf = ttk.Frame(v); bf.grid(row=1, column=4, sticky="w")
        ttk.Button(bf, text="Download", command=self._download).pack(side="left")
        ttk.Button(bf, text="Local file…", command=self._local_video).pack(side="left", padx=2)
        ttk.Button(bf, text="No video", command=self._clear_video).pack(side="left")
        self.video_var = tk.StringVar(value="No video loaded")
        ttk.Label(v, textvariable=self.video_var, foreground="#555").grid(row=2, column=1, columnspan=4, sticky="w")
        v.columnconfigure(3, weight=1)

        # whole-day event videos: used for matches that have no video of their own
        self.day_frame = df = ttk.LabelFrame(rf, text="Whole-day event videos  (for matches with no video of their own: each match is cut out of them)")
        self.day_query = tk.StringVar(value=self.settings.get("day_query", "SoCal Showdown Fall 2026 Day"))
        ttk.Entry(df, textvariable=self.day_query, width=34).grid(row=0, column=0, padx=4, pady=2, sticky="we")
        ttk.Button(df, text="Find on YouTube", command=self._find_days).grid(row=0, column=1, padx=2)
        self.day_url = tk.StringVar()
        ttk.Entry(df, textvariable=self.day_url, width=34).grid(row=0, column=2, padx=4, sticky="we")
        ttk.Button(df, text="Add URL", command=self._add_day_url).grid(row=0, column=3, padx=2)
        self.day_lb = tk.Listbox(df, height=3, selectmode="extended", exportselection=False)
        self.day_lb.grid(row=1, column=0, columnspan=3, sticky="we", padx=4, pady=2)
        bf = ttk.Frame(df); bf.grid(row=1, column=3, sticky="n")
        ttk.Button(bf, text="Use selected", command=self._use_days).pack(fill="x")
        ttk.Button(bf, text="Clip this match", command=self._clip_this).pack(fill="x", pady=2)
        self.day_var = tk.StringVar()
        ttk.Label(df, textvariable=self.day_var, foreground="#555").grid(row=2, column=0, columnspan=4, sticky="w", padx=4)
        df.columnconfigure(0, weight=1); df.columnconfigure(2, weight=1)
        self.day_found: list = []
        self._day_text()

        # the sync (offset) and the time slider are in the video window (_build_viewer_controls): they only mean something next to
        # the picture, and the sync only when there is a TBA / YouTube / day video.  Their variables are made here.
        self.offset_var = tk.DoubleVar(value=0.0)
        self.conf_var = tk.StringVar()
        self.t_var = tk.DoubleVar(value=0.0)
        self.t_scale = self.t_lbl = None
        self.jump_btns = {}
        self._scrub_max = 170.0
        self._scrub_has_match = True

        # CTRE hoot (log mode): the Phoenix 6 device log that goes with the robot log; used by the match report
        self.hoot_frame = hf = ttk.LabelFrame(rf, text="CTRE hoot  (optional: swerve drive motor currents, faults, CAN bus; used by the match report)")
        self.hoot_var = tk.StringVar(value="none found next to this log")
        ttk.Label(hf, textvariable=self.hoot_var, foreground="#555").pack(side="left", padx=6, fill="x", expand=True)
        ttk.Button(hf, text="Find", command=self._find_hoots).pack(side="left", padx=2)
        ttk.Button(hf, text="Browse…", command=self._browse_hoot).pack(side="left", padx=2)
        ttk.Button(hf, text="Convert hoot to wpilog…", command=self._convert_hoot).pack(side="left", padx=2)
        self.hoot_paths: list = []
        self.hoot_log = None            # hoot.HootLog of the selected log, shown in the swerve / PDH panels
        self._hoot_key = None

        self.preview_frame = p = ttk.LabelFrame(rf, text="Video window  (the time slider and, with a video, the sync controls are in it)")
        p.pack(fill="both", expand=True, **pad)
        self.live_bar = lb = ttk.Frame(p)
        self.freeze_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(lb, text="Pause display", variable=self.freeze_var).pack(side="left", padx=4)
        ttk.Button(lb, text="Save frame as PNG…", command=self._save_frame).pack(side="left", padx=4)
        ttk.Button(lb, text="Forget history", command=self._live_reset).pack(side="left", padx=4)
        ttk.Button(lb, text="Pop out ⧉", command=self._open_viewer).pack(side="right", padx=4)
        self.show_row = ttk.Frame(p)
        ttk.Button(self.show_row, text="Show video window ⧉", command=self._open_viewer).pack(side="left", padx=4, pady=2)
        ttk.Label(self.show_row, text="The picture is in its own window (it opens by itself, drag it to another screen). "
                                      "Keys there: arrows scrub, F11 / double-click full screen, Esc.",
                  foreground="#666").pack(side="left", padx=6)
        self.show_row.pack(fill="x")

        self.out_frame = o = ttk.LabelFrame(rf, text="Output")
        ttk.Label(o, text="Title").grid(row=0, column=0, sticky="e")
        self.title_var = tk.StringVar()
        self.title_var.trace_add("write", lambda *a: self._schedule_preview(400))
        ttk.Entry(o, textvariable=self.title_var, width=30).grid(row=0, column=1, sticky="we", **pad)
        ttk.Label(o, text="Size").grid(row=0, column=2, sticky="e")
        self.size_var = tk.StringVar(value=self.settings.get("size", "Standard 1920×1080"))
        if self.size_var.get() not in SIZES:
            self.size_var.set("Standard 1920×1080")
        ttk.Combobox(o, textvariable=self.size_var, values=list(SIZES), state="readonly", width=18).grid(row=0, column=3, **pad)
        ttk.Label(o, text="Quality").grid(row=0, column=4, sticky="e")
        self.qual_var = tk.StringVar(value=self.settings.get("quality", "Balanced (CRF 27)"))
        ttk.Combobox(o, textvariable=self.qual_var, values=list(QUALITY), state="readonly", width=18).grid(row=0, column=5, **pad)
        ttk.Label(o, text="Save as").grid(row=1, column=0, sticky="e")
        self.out_var = tk.StringVar()
        ttk.Entry(o, textvariable=self.out_var).grid(row=1, column=1, columnspan=4, sticky="we", **pad)
        ob = ttk.Frame(o); ob.grid(row=1, column=5, sticky="w")
        ttk.Button(ob, text="…", width=3, command=self._browse_out).pack(side="left")
        ttk.Button(ob, text="Match report…", command=self._report).pack(side="left")
        self.render_btn = ttk.Button(ob, text="Render MP4", command=self._render)
        self.render_btn.pack(side="left", padx=2)
        ttk.Button(ob, text="Open folder", command=self._open_out_folder).pack(side="left")
        o.columnconfigure(1, weight=1)

        st = ttk.Frame(self); st.pack(fill="x", side="bottom", before=body, **pad)
        self.prog = ColorBar(st); self.prog.pack(side="left")
        ttk.Button(st, text="Cancel", command=self._cancel).pack(side="left", padx=4)
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(st, textvariable=self.status_var).pack(side="left", padx=6)

    def _set_mode(self, mode, first=False):
        """Show the controls that belong to the log source or to the live source."""
        if mode not in (LOG, LIVE):
            mode = LOG
        if not first and mode == self.mode:
            return
        if self.mode == LIVE and mode != LIVE:
            self._disconnect()
        self.mode = mode
        self.mode_var.set(mode)
        for w in (self.video_frame, self.day_frame, self.hoot_frame, self.out_frame, self.preview_frame):
            w.pack_forget()
        if mode == LOG:
            self.live_row.pack_forget()
            self.log_row.pack(fill="x", padx=4, pady=2, after=self.log_row.master.winfo_children()[0])
            self.entries_lb.pack_forget(); self.entries_hint.pack_forget()
            self.logs.pack(fill="both", expand=True); self.batch_btn.pack(fill="x", pady=(4, 0))
            self.left_title.set("Logs")
            self.live_bar.pack_forget()
            self.video_frame.pack(fill="x", padx=4, pady=2)
            self.day_frame.pack(fill="x", padx=4, pady=2)
            self.hoot_frame.pack(fill="x", padx=4, pady=2)
            self.preview_frame.pack(fill="x", padx=4, pady=2)
            self.out_frame.pack(side="bottom", fill="x", padx=4, pady=2, before=self.info_lbl)
            self.preview_frame.configure(text="Video window  (the time slider and, with a video, the sync controls are in it)")
            self.info_var.set(self._log_info if getattr(self, "_log_info", None) else "Select a log on the left.")
            self._schedule_preview(50)
            self._viewer_controls_refresh()
        else:
            self.log_row.pack_forget()
            self.live_row.pack(fill="x", padx=4, pady=2, after=self.log_row.master.winfo_children()[0])
            self.logs.pack_forget(); self.batch_btn.pack_forget()
            self.entries_lb.pack(fill="both", expand=True); self.entries_hint.pack(anchor="w")
            self.left_title.set("Live entries (numbers)")
            self.live_bar.pack(fill="x", before=self.show_row)
            self.preview_frame.pack(fill="x", padx=4, pady=2)
            self.preview_frame.configure(text="Live view  (shown in the video window)")
            self.info_var.set("Enter the robot's address and press Connect.")
        self._trend_choices()
        self._viewer_controls_refresh()

    # ------------------------------------------------------------ threading
    def _run(self, fn, done=None, label="Working…"):
        if self.busy:
            messagebox.showinfo("Busy", "Please wait for the current task to finish.")
            return
        self.busy = True
        self.cancel_flag = False
        self._status(0, label)

        def work():
            try:
                res = fn()
                self.q.put(("done", done, res))
            except Exception as e:  # noqa
                self.q.put(("error", None, (e, traceback.format_exc())))
        threading.Thread(target=work, daemon=True).start()

    def _progress_cb(self, frac, msg):
        self.q.put(("progress", frac, msg))

    def _poll(self):
        try:
            while True:
                kind, a, b = self.q.get_nowait()
                if kind == "progress":
                    self._status(a, b)
                elif kind == "done":
                    self.busy = False
                    self._status(1, "Done")
                    if a:
                        a(b)
                elif kind == "error":
                    self.busy = False
                    e, tb = b
                    self._status(0, f"Error: {e}", error=True)
                    print(tb, file=sys.stderr)
                    messagebox.showerror("Error", str(e))
                elif kind == "call":
                    a(b)
        except queue.Empty:
            pass
        self.after(100, self._poll)

    def _status(self, frac, msg, error=False):
        if error:
            self.prog.set(1.0, "error")
        elif frac is not None:
            self.prog.set(frac, "done" if (frac >= 1 and not self.busy) else "busy")
        self.status_var.set(msg)

    def _cancel(self):
        self.cancel_flag = True

    # ------------------------------------------------------------ settings
    @staticmethod
    def _clip_ver(video):
        """webcast.CLIP_VERSION for a video cut from a day video, else None."""
        try:
            import webcast
            return webcast.CLIP_VERSION if video and str(webcast.CLIP_DIR) in str(video) else None
        except Exception:  # noqa
            return None

    @staticmethod
    def _stale(rec):
        """A render of a video cut from a day video by an older version may show the neighbouring match: do it again."""
        try:
            import webcast
            v = rec.get("video") or ""
            return bool(str(webcast.CLIP_DIR) in str(v) and rec.get("clip_ver") != webcast.CLIP_VERSION)
        except Exception:  # noqa
            return False

    def _save(self):
        s = self.settings
        code = getattr(self, "_cur_event", None)
        if code:                                   # remembered per event, so logs of other events do not use this event's key
            p = self._event_prefs(code)
            p["tba"], p["day"] = self.event_var.get().strip(), self.day_query.get().strip()
        s.update(log_folder=self.folder_var.get(),
                 event_key=self.event_var.get().strip(), tba_key=self.tbakey_var.get().strip(),
                 size=self.size_var.get(), quality=self.qual_var.get(), red_left=self.redleft_var.get(),
                 field=self.field_var.get(), mode=self.mode, host=self.host_var.get(), trends=self.trends,
                 thresholds=self._thr())
        s.pop("show_status", None)
        if self.ml and self.mode == LOG:
            per = s.setdefault("per_log", {})
            rec = per.setdefault(Path(self.ml.path).name, {})
            rec.update(url=self.url_var.get(), offset=self._offset(), video=self.video_path, title=self.title_var.get(),
                       hoot=list(self.hoot_paths), clip_ver=self._clip_ver(self.video_path))
        core.save_settings(s)

    # ---------------------------------------------------------------- logs
    def _browse_folder(self):
        d = filedialog.askdirectory(initialdir=self.folder_var.get() or str(Path.home()))
        if d:
            self.folder_var.set(d)
            self._scan_folder()

    def _open_log_file(self):
        f = filedialog.askopenfilename(initialdir=self.folder_var.get() or str(Path.home()),
                                       filetypes=[("WPILib logs", "*.wpilog"), ("All", "*.*")])
        if not f:
            return
        self.folder_var.set(str(Path(f).parent))
        self._scan_folder()
        names = [p.name for p in self.log_files]
        if Path(f).name in names:
            i = names.index(Path(f).name)
            self.logs.selection_clear(0, "end"); self.logs.selection_set(i); self.logs.see(i)
            self._select_log()

    def _scan_folder(self):
        self.logs.delete(0, "end")
        d = self.folder_var.get()
        if not os.path.isdir(d):
            return
        self.log_files = sorted(Path(d).glob("*.wpilog"))
        per = self.settings.get("per_log", {})
        for f in self.log_files:
            mark = "✓ " if per.get(f.name, {}).get("rendered") else "   "
            self.logs.insert("end", mark + f.name)
        self._save()

    def _select_log(self):
        sel = self.logs.curselection()
        if not sel:
            return
        if self.ml:
            self._save()
        path = str(self.log_files[sel[0]])
        self.info_var.set("Loading " + Path(path).name + " …")
        self._load_log(path)

    def _load_log(self, path):
        poses = [self.drive_var.get(), self.quest_var.get()]

        def work():
            pose_names, numeric = core.list_log_entries(path)
            ml = core.load_match_log(path, poses)
            ml.pose_entries = pose_names
            self.log_numeric = numeric
            return ml
        self._run(work, self._log_loaded, "Reading log…")

    def _log_loaded(self, ml: core.MatchLog):
        self.ml = ml
        self._trend_choices()
        name = Path(ml.path).name
        if ml.has_match:
            ev = ml.phase_events
            txt = (f"{ml.match_label}  ·  {ml.alliance_text}  ·  event {ml.event_name or '?'}  ·  "
                   f"auto start at log t={ev['auto_start']:.2f}s  ·  match length {ml.match_len:.1f}s")
        else:
            txt = (f"{name}  ·  no match start (no FMS / DS autonomous) — plots start at "
                   f"{'the first enable' if ml.first_enable is not None else 'the start of the log'}, "
                   f"{ml.match_len:.0f}s long; match clock and scoring show defaults")
        missing = [n for n in (self.drive_var.get(), self.quest_var.get())
                   if n and len(ml.poses.get(n, core.PoseSeries(n, [], [], [], [])).t) == 0]
        if missing:
            txt += "   ⚠ no data: " + ", ".join(m.split('/')[-1] for m in missing)
        self._log_info = txt
        self.info_var.set(txt)
        self._scrub_max, self._scrub_has_match = ml.match_len + 5, ml.has_match
        self.t_var.set(0.0)
        per = self.settings.get("per_log", {}).get(name, {})
        self.url_var.set(per.get("url", ""))
        self.url_cb["values"] = []
        self.offset_var.set(round(per.get("offset", 0.0), 3))
        self.title_var.set(per.get("title") or self._default_title(ml))
        self._event_for_log(ml)
        vp = per.get("video")
        self.video_path = vp if vp and os.path.exists(vp) else None
        if not self.video_path and self.url_var.get():
            vid = core.youtube_id(self.url_var.get())
            cached = list(core.CACHE_DIR.glob(f"{vid}.mp4")) if vid else []
            self.video_path = str(cached[0]) if cached else None
        self.video_var.set(self.video_path or "No video loaded")
        self.conf_var.set("")
        saved = [p for p in per.get("hoot", []) if os.path.exists(p)]
        self._set_hoots(saved if saved else self._auto_hoots(), auto=not saved)
        self._out_auto = True
        self._set_default_out()
        self._viewer_controls_refresh()
        self._schedule_preview(50)
        if not self.url_var.get() and ml.match_type in (2, 3) and ml.has_match:
            self._find_tba(auto=True)

    def _event_prefs(self, code):
        return self.settings.setdefault("event_prefs", {}).setdefault(code, {})

    def _event_key(self, ml, prefs=None):
        """TBA event key for a log: what the user typed for this event, else the built-in / guessed one (core.EVENT_INFO)."""
        code = ml.event_code
        p = (prefs if prefs is not None else self.settings.get("event_prefs", {})).get(code, {})
        return p.get("tba") or core.event_tba_key(code)

    def _event_for_log(self, ml):
        """A log of another event: show that event's TBA key and whole-day video search, not the previous log's."""
        code = ml.event_code
        if code == getattr(self, "_cur_event", None):
            return
        self._cur_event = code
        p = self.settings.get("event_prefs", {}).get(code, {})
        self.event_var.set(p.get("tba") or core.event_tba_key(code) or self.event_var.get())
        self.day_query.set(p.get("day") or (f"{core.event_title(code)} 2026 Day" if code else self.day_query.get()))

    @staticmethod
    def _default_title(ml: core.MatchLog):
        if ml.match_type and ml.match_number:
            return f"2429 · {core.event_title(ml.event_code)} {ml.match_label} · {ml.alliance_text}"
        return Path(ml.path).stem

    def _set_default_out(self):
        if not (self.ml and self._out_auto):
            return
        tag = "sidebyside" if self.video_path else "dashboard"
        out_dir = Path(self.folder_var.get() or Path(self.ml.path).parent) / "videos"
        self.out_var.set(str(out_dir / f"{Path(self.ml.path).stem}_{tag}.mp4"))

    # --------------------------------------------------------- thresholds
    def _thr(self):
        import dash
        out = {}
        for k, v in self.thr_vars.items():
            try:
                out[k] = max(0.0, float(v.get()))
            except (tk.TclError, ValueError):
                out[k] = dash.DEFAULT_THR[k]
        return out

    def _thr_changed(self):
        self._save()
        if self.mode == LIVE and self.live_dash:
            self.live_dash.thr = {**self.live_dash.thr, **self._thr()}
        else:
            self._schedule_preview(50)

    # ------------------------------------------------------------- trends
    def _fill_trend_list(self):
        self.trend_lb.delete(0, "end")
        for n in self.trends:
            self.trend_lb.insert("end", n)

    def _trend_choices(self):
        if self.mode == LIVE and self.session is not None:
            names = self.session.tel.numeric_entries()
        else:
            names = self.log_numeric
        self.trend_cb["values"] = names

    def _trends_changed(self):
        self._fill_trend_list()
        self._save()
        if self.mode == LIVE:
            if self.session:
                self.session.set_trends(self.trends)
            if self.live_dash:
                self.live_dash.trend_panel.extra = list(self.trends)[:6]
        else:
            self._schedule_preview(50)

    def _add_trend(self, name=None):
        name = (name or self.trend_var.get()).strip()
        if name and name not in self.trends and len(self.trends) < 6:
            self.trends.append(name)
            self._trends_changed()

    def _add_trend_from_list(self):
        sel = self.entries_lb.curselection()
        if sel:
            self._add_trend(self.entries_lb.get(sel[0]))

    def _remove_trend(self):
        for i in reversed(self.trend_lb.curselection()):
            del self.trends[i]
        self._trends_changed()

    def _clear_trends(self):
        self.trends.clear()
        self._trends_changed()

    # --------------------------------------------------------------- field
    def _refresh_fields(self, select=None):
        cfgs = core.find_field_configs("2026")
        self.field_paths = {GENERIC_FIELD: None}
        for c in cfgs:
            self.field_paths[os.path.basename(os.path.dirname(c)) + "  —  " + c] = c
        extra = self.settings.get("field_custom")
        if extra and os.path.exists(extra):
            self.field_paths["custom  —  " + extra] = extra
        self.field_cb["values"] = list(self.field_paths)
        want = select or self.settings.get("field")
        if want not in self.field_paths:
            want = next((k for k in self.field_paths if k != GENERIC_FIELD), GENERIC_FIELD)
        self.field_var.set(want)
        self._field_changed()

    def _field_changed(self):
        try:
            self.field_obj = core.load_field(self.field_paths.get(self.field_var.get()))
        except Exception as e:  # noqa
            self._status(None, f"Field image error ({e}); using drawn field")
            self.field_obj = core.synthetic_field()
        if self.mode == LIVE:
            self.live_dash = None                      # rebuilt on the next tick
        else:
            self._schedule_preview(50)

    def _browse_field(self):
        f = filedialog.askopenfilename(title="AdvantageScope Field2d config.json",
                                       filetypes=[("config.json", "config.json"), ("JSON", "*.json")])
        if f:
            self.settings["field_custom"] = f
            self._refresh_fields("custom  —  " + f)

    def _download_field(self):
        self._run(lambda: core.download_field_asset("2026", self._progress_cb),
                  lambda c: self._refresh_fields(os.path.basename(os.path.dirname(c)) + "  —  " + c),
                  "Downloading field image…")

    # --------------------------------------------------------------- video
    def _find_tba(self, auto=False):
        if not self.ml:
            return
        key = core.tba_match_key(self.event_var.get().strip(), self.ml)    # (event_var follows the log's event: _event_for_log)
        if not key:
            if not auto:
                messagebox.showinfo("TBA", "This log is not a qualification / playoff match, so it is not on "
                                           "The Blue Alliance.\nPaste a YouTube URL or pick a local file — or "
                                           "leave the video empty.")
            return

        def done(res):
            urls, info = res
            self.url_cb["values"] = urls
            if urls:
                self.url_var.set(urls[0])
                teams = ""
                if info.get("red") or info.get("blue"):
                    teams = f"   red {' '.join(info['red'])}  /  blue {' '.join(info['blue'])}"
                self._status(1, f"TBA {key}: {len(urls)} video(s){teams}")
                self._save()
            else:
                self._status(1, f"TBA {key}: no video posted yet")
        api = self.tbakey_var.get().strip()                 # read here: Tk variables belong to the UI thread
        self._run(lambda: core.tba_find_video(key, api), done, f"Looking up {key} on TBA…")

    def _download(self):
        url = self.url_var.get().strip()
        if not url:
            return messagebox.showinfo("Video", "Enter a YouTube URL (or use Find on TBA).")
        self._run(lambda: core.download_youtube(url, self._progress_cb), self._video_loaded, "Downloading video…")

    def _local_video(self):
        f = filedialog.askopenfilename(filetypes=[("Video", "*.mp4 *.mkv *.webm *.mov"), ("All", "*.*")])
        if f:
            self._video_loaded(f)

    def _clear_video(self):
        self.video_path = None
        self.url_var.set("")
        self.video_var.set("No video loaded")
        self.conf_var.set("")
        self._set_default_out()
        self._save()
        self._viewer_controls_refresh()
        self._schedule_preview(50)

    def _video_loaded(self, path):
        self.video_path = path
        info = core.video_info(path)
        self.video_var.set(f"{path}   ({info['width']}×{info['height']}, {info['duration'] or 0:.0f}s"
                           f"{', audio' if info['has_audio'] else ', NO AUDIO'})")
        self._set_default_out()
        self._auto_red_left(path)
        self._save()
        self._viewer_controls_refresh()
        if self.ml and self.ml.has_match and abs(self._offset()) < 1e-6:
            self._auto_sync()
        else:
            self._schedule_preview(50)

    def _auto_red_left(self, path):
        """Set the "Red alliance on left" box to what the loaded broadcast video shows (the left score box of the
        score bar is the alliance on the left of the field camera).  If it cannot tell, the box is left as it was.
        You can still change the box by hand afterwards; it is decided again the next time a video is loaded."""
        try:
            red_left = core.detect_red_left(path)
        except Exception:  # noqa - a video it cannot read must not stop the load
            red_left = None
        if red_left is None:
            return
        if red_left != self.redleft_var.get():
            self.redleft_var.set(red_left)
            self._field_changed()
        self._status(None, f"Video shows {'red' if red_left else 'blue'} on the left: field panel set to match "
                           f"('Red alliance on left' {'on' if red_left else 'off'})")

    # ---------------------------------------------------------------- sync
    def _offset(self):
        try:
            return float(self.offset_var.get())
        except (tk.TclError, ValueError):
            return 0.0

    def _nudge(self, d):
        self.offset_var.set(round(self._offset() + d, 3))
        self._schedule_preview(150)

    def _auto_sync(self):
        if not (self.ml and self.video_path):
            return messagebox.showinfo("Sync", "Load a log and a video first.")
        if not self.ml.has_match:
            return messagebox.showinfo("Sync", "This log has no match start, so there is nothing to sync the video to.")

        def done(res):
            off, conf, det = res
            self.offset_var.set(round(off, 3))
            q = "good" if conf > 6 and det["ratio"] > 1.15 else ("fair" if conf > 4 else "LOW — check preview")
            if det.get("picture"):
                self.conf_var.set("synced to the match clock in the picture"
                                  + (f" (the sound said {det['sound_off']:.1f} s)" if abs(det["sound_off"] - off) > 1.0 else ""))
            else:
                self.conf_var.set(f"auto-sync confidence {conf:.1f} ({q}); match clock not readable, sound only")
            self._save()
            self._schedule_preview(50)
        import webcast
        self._run(lambda: webcast.sync_video(self.video_path, self.ml, progress=self._progress_cb), done,
                  "Auto-syncing from the match clock and the sound…")

    def _jump(self, label, tv):
        if self.ml and self.ml.has_match:
            ev = self.ml.phase_events
            m = {"Auto end": "auto_end", "Teleop": "teleop_start", "End": "teleop_end"}
            if label in m and m[label] in ev:
                tv = ev[m[label]] - ev["auto_start"]
        elif self.ml and label == "End":
            tv = self.ml.match_len
        self.t_var.set(tv)
        self._schedule_preview(10)

    # ------------------------------------------------------------- preview
    def _schedule_preview(self, ms=300):
        if self.mode != LOG:
            return
        if self._preview_after:
            self.after_cancel(self._preview_after)
        self._preview_after = self.after(ms, self._update_preview)

    def _job(self, out=None):
        st = core.RenderStyle(red_left=self.redleft_var.get(), title=self.title_var.get())
        return core.RenderJob(log=self.ml, video=self.video_path, offset=self._offset(),
                              out_path=out or self.out_var.get(), field=self.field_obj or core.synthetic_field(),
                              quest_name=self.quest_var.get(), drive_name=self.drive_var.get(),
                              height=SIZES.get(self.size_var.get(), 540), crf=QUALITY.get(self.qual_var.get(), 27),
                              style=st, trends=list(self.trends), thresholds=self._thr(), hoot=self.hoot_log)

    def _update_preview(self):
        self._preview_after = None
        t = float(self.t_var.get())
        if self.t_lbl is not None:
            self.t_lbl.configure(text=f"{t:.1f} s")
        if not self.ml or self.field_obj is None or self.mode != LOG:
            return
        job = self._job()
        try:
            img = core.preview(job, t)
        except Exception as e:  # noqa
            self._status(None, f"Preview error: {e}")
            return
        self._show(img)

    def _show(self, img):
        """Show a BGR frame in the video window (opened the first time a frame arrives, unless the user closed it)."""
        self._last_frame = img
        if self.viewer is None and not self._viewer_dismissed:
            self._open_viewer(auto=True)
        else:
            self._draw_viewer()

    # ------------------------------------------------------ separate video window
    def _open_viewer(self, auto=False):
        """The picture in its own window that can be dragged to another screen, resized or made full screen
        (double-click or F11).  Esc leaves full screen, then closes it.  In log mode the arrow keys scrub:
        left / right 1 s, with Shift 10 s, with Ctrl one frame (1/30 s)."""
        if not auto:
            self._viewer_dismissed = False
        if self.viewer is not None and self.viewer.winfo_exists():
            self.viewer.lift()
            self._draw_viewer()
            return
        v = self.viewer = tk.Toplevel(self)
        v.title(f"{APP_NAME} - view")
        v.geometry("1440x860")
        v.configure(background="black")
        self._build_viewer_controls(v)                                  # (packed at the bottom first, so it is never cut off)
        v.lbl = tk.Label(v, background="black", borderwidth=0)
        v.lbl.pack(fill="both", expand=True)
        v.bind("<Configure>", self._viewer_resized)
        v.bind("<F11>", lambda e: self._viewer_fullscreen())
        v.lbl.bind("<Double-Button-1>", lambda e: self._viewer_fullscreen())
        v.bind("<Escape>", self._viewer_escape)
        v.bind("<Left>", lambda e: self._viewer_step(-1, e))
        v.bind("<Right>", lambda e: self._viewer_step(1, e))
        v.protocol("WM_DELETE_WINDOW", lambda: self._close_viewer(by_user=True))
        self._viewer_controls_refresh()
        if auto:
            self.after(80, self.focus_force)           # the controls stay in front of the first picture
        else:
            v.focus_set()
        self._draw_viewer()

    def _close_viewer(self, by_user=False):
        if by_user:
            self._viewer_dismissed = True
        if self.viewer is not None:
            try:
                self.viewer.destroy()
            except tk.TclError:
                pass
        self.viewer = None
        self._viewer_img = None
        self.t_scale = self.t_lbl = None
        self.jump_btns = {}

    def _build_viewer_controls(self, v):
        """The time slider (log mode) and the sync controls (only when the log has a TBA / YouTube / day video) sit at the bottom
        of the video window, next to the picture they act on."""
        pad = dict(padx=4, pady=2)
        v.ctl = ttk.Frame(v)
        v.ctl.pack(side="bottom", fill="x")
        # row 1: seconds from match start / first enable
        v.scrub = ttk.Frame(v.ctl)
        ttk.Label(v.scrub, text="seconds from match start / first enable").pack(side="left", padx=(6, 2))
        self.t_scale = ttk.Scale(v.scrub, from_=-5, to=self._scrub_max, variable=self.t_var, orient="horizontal",
                                 command=lambda e: self._schedule_preview(250))
        self.t_scale.pack(side="left", fill="x", expand=True, padx=4)
        self.t_lbl = ttk.Label(v.scrub, text=f"{float(self.t_var.get()):.1f} s", width=8)
        self.t_lbl.pack(side="left")
        self.jump_btns = {}
        for lbl, tv in (("Start", 0), ("Auto end", 20), ("Teleop", 23.2), ("End", 163)):
            b = ttk.Button(v.scrub, text=lbl, command=lambda tv=tv, l=lbl: self._jump(l, tv))
            b.pack(side="left")
            self.jump_btns[lbl] = b
        # row 2: the video's sync (offset = time in the video when autonomous starts)
        v.sync = ttk.Frame(v.ctl)
        ttk.Button(v.sync, text="Auto-sync (match clock + sound)", command=self._auto_sync).pack(side="left", **pad)
        ttk.Label(v.sync, text="Video offset (s) = time in the video when autonomous starts:").pack(side="left")
        sp = ttk.Spinbox(v.sync, from_=-600, to=3600, increment=0.05, textvariable=self.offset_var, width=9,
                         command=self._schedule_preview, format="%.2f")
        sp.pack(side="left", **pad)
        sp.bind("<Return>", lambda e: self._schedule_preview())
        sp.bind("<FocusOut>", lambda e: self._schedule_preview())
        for d in (-1, -0.1, -0.033, 0.033, 0.1, 1):
            ttk.Button(v.sync, text=f"{d:+g}", width=6, command=lambda d=d: self._nudge(d)).pack(side="left")
        ttk.Label(v.sync, textvariable=self.conf_var, foreground="#555").pack(side="left", padx=8)

    def _viewer_controls_refresh(self):
        """Show the time slider in log mode, and the sync row only when a video is loaded (no video: the offset means nothing)."""
        v = self.viewer
        if v is None or not hasattr(v, "ctl"):
            return
        try:
            if not v.winfo_exists():
                return
            v.scrub.pack_forget()
            v.sync.pack_forget()
            if self.mode == LOG and self.ml:
                v.scrub.pack(fill="x", **dict(padx=2, pady=2))
                if self.video_path:
                    v.sync.pack(fill="x")
                self.t_scale.configure(to=self._scrub_max)
                for lbl, b in self.jump_btns.items():
                    b.state(["!disabled"] if (self._scrub_has_match or lbl in ("Start", "End")) else ["disabled"])
        except tk.TclError:
            pass

    def _viewer_fullscreen(self):
        v = self.viewer
        if v is not None:
            v.attributes("-fullscreen", not bool(v.attributes("-fullscreen")))

    def _viewer_escape(self, _e=None):
        if self.viewer is not None and self.viewer.attributes("-fullscreen"):
            self.viewer.attributes("-fullscreen", False)
        else:
            self._close_viewer(by_user=True)

    def _viewer_step(self, sign, e):
        """arrow keys: move the preview time (log mode)"""
        if self.mode != LOG or not self.ml:
            return
        shift, ctrl = bool(e.state & 0x1), bool(e.state & 0x4)
        self.seek(float(self.t_var.get()) + sign * (10.0 if shift else 1 / 30 if ctrl else 1.0))

    def _viewer_resized(self, _e=None):
        if self._viewer_after:
            self.after_cancel(self._viewer_after)
        self._viewer_after = self.after(60, self._draw_viewer)

    def _draw_viewer(self):
        v = self.viewer
        if v is None or self._last_frame is None:
            return
        try:
            if not v.winfo_exists():
                self.viewer = None
                return
            img = self._last_frame
            w, h = max(200, v.lbl.winfo_width()), max(120, v.lbl.winfo_height())
            k = min(w / img.shape[1], h / img.shape[0])
            small = cv2.resize(img, (max(1, int(img.shape[1] * k)), max(1, int(img.shape[0] * k))),
                               interpolation=cv2.INTER_AREA if k < 1 else cv2.INTER_LINEAR)
            ok, png = cv2.imencode(".png", small, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if ok:
                self._viewer_img = tk.PhotoImage(data=base64.b64encode(png.tobytes()))
                v.lbl.configure(image=self._viewer_img)
        except tk.TclError:
            self.viewer = None

    def _save_frame(self):
        if self._last_frame is None:
            return
        f = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG", "*.png")],
                                         initialfile=time.strftime("scope2429_%Y%m%d_%H%M%S.png"))
        if f:
            ok, buf = cv2.imencode(".png", self._last_frame)
            if ok:
                Path(f).write_bytes(buf.tobytes())
                self._status(None, f"Saved {f}")

    # ---------------------------------------------------------------- live
    def _toggle_connect(self):
        if self.session:
            self._disconnect()
        else:
            self._connect()

    def _host_preset(self, _e=None):
        v = self.host_var.get()
        if "   (" in v:
            self.host_var.set(v.split("   (")[0])

    def _connect(self):
        host = self.host_var.get().split("   (")[0].strip()
        if not host:
            return messagebox.showinfo("Connect", "Enter the robot's IP address, e.g. 10.24.29.2, or localhost for a simulator.")
        try:
            import live
            self.session = live.LiveSession(host, {"drive": self.drive_var.get(), "quest": self.quest_var.get()},
                                            self.trends)
            self.session.start()
        except Exception as e:  # noqa
            self.session = None
            return messagebox.showerror("Live connection", str(e))
        self.host_var.set(host)
        self.live_dash = None
        self.connect_btn.configure(text="Disconnect")
        self.info_var.set(f"Live: {host}")
        self._save()
        self._live_tick()

    def _disconnect(self):
        if self._live_after:
            self.after_cancel(self._live_after)
            self._live_after = None
        if self.session:
            self.session.stop()
        self.session = None
        self.live_dash = None
        self.connect_btn.configure(text="Connect")
        self.live_status.set("not connected")
        self.entries_lb.delete(0, "end")

    def _reconnect(self):
        self._disconnect()
        self._connect()

    def _live_reset(self):
        if self.session:
            self._reconnect()

    def _live_tick(self):
        """Move new NetworkTables data in and draw a frame; reschedules itself while connected."""
        self._live_after = None
        sess = self.session
        if sess is None or self.mode != LIVE:
            return
        t0 = time.time()
        try:
            sess.poll()
            self.live_status.set(sess.status_text())
            if sess.tel.sig and self.live_dash is None:
                import dash
                self.live_dash = dash.DashRenderer(sess.ml, self.field_obj or core.synthetic_field(),
                                                   f"LIVE  {sess.host_text}", self.redleft_var.get(),
                                                   trends=self.trends, tel=sess.tel)
                self.live_dash.thr = {**self.live_dash.thr, **self._thr()}
            if t0 - self._live_entries_t > 2.0:
                self._live_entries_t = t0
                names = sess.tel.numeric_entries()
                self.trend_cb["values"] = names
                if len(names) != self.entries_lb.size():
                    top = self.entries_lb.yview()[0]
                    self.entries_lb.delete(0, "end")
                    for n in names:
                        self.entries_lb.insert("end", n)
                    self.entries_lb.yview_moveto(top)
            if self.live_dash is not None:
                al = sess.ml.alliance_text
                self.info_var.set(f"LIVE  {sess.host_text}  ·  {len(sess.tel.sig)} entries recorded"
                                  + ("" if al == "?" else f"  ·  {al}"))
                if not self.freeze_var.get():
                    self._show(self.live_dash.render(sess.now()))
            else:
                self.info_var.set(f"Live: {sess.status_text()}"
                                  + ("" if sess.connected else "   (nothing received yet)"))
        except Exception as e:  # noqa
            traceback.print_exc()
            self._status(None, f"Live error: {e}")
        dt = int((time.time() - t0) * 1000)
        self._live_after = self.after(max(15, 80 - dt), self._live_tick)

    # -------------------------------------------------------------- render
    def _browse_out(self):
        f = filedialog.asksaveasfilename(defaultextension=".mp4", filetypes=[("MP4", "*.mp4")],
                                         initialfile=os.path.basename(self.out_var.get()),
                                         initialdir=os.path.dirname(self.out_var.get()))
        if f:
            self._out_auto = False
            self.out_var.set(f)

    def _open_out_folder(self):
        d = os.path.dirname(self.out_var.get()) or "."
        os.makedirs(d, exist_ok=True)
        if sys.platform == "win32":
            os.startfile(d)  # noqa
        elif sys.platform == "darwin":
            subprocess.run(["open", d])
        else:
            subprocess.run(["xdg-open", d])

    def _render(self):
        if not self.ml:
            return messagebox.showinfo("Render", "Load a log first.")
        job = self._job()
        if job.video and not self.ml.has_match:
            messagebox.showinfo("Render", "This log has no match start, so the video cannot be synced to it.\n"
                                          "Rendering the dashboard without the video.")
            job.video = None
        self._save()
        name = Path(self.ml.path).name

        def done(path):
            self.settings.setdefault("per_log", {}).setdefault(name, {})["rendered"] = True
            core.save_settings(self.settings)
            mb = os.path.getsize(path) / 1e6
            self._status(1, f"Saved {path}  ({mb:.1f} MB)")
            self._scan_folder_keep()
        self._run(lambda: core.render(job, self._progress_cb, lambda: self.cancel_flag), done, "Rendering…")

    def _scan_folder_keep(self):
        sel = self.logs.curselection()
        self._scan_folder()
        if sel:
            self.logs.selection_set(sel[0])

    # ------------------------------------------------------ whole-day videos
    def _day_text(self):
        if self.days:
            self.day_var.set("using: " + "   |   ".join(d.label() for d in self.days))
        else:
            self.day_var.set("none yet: press Find on YouTube, pick the Day 1 / Day 2 videos and Use selected")

    def _find_days(self):
        q = self.day_query.get().strip()
        if not q:
            return

        def work():
            import webcast
            return webcast.search_day_videos(q)

        def done(res):
            self.day_found = res
            self.day_lb.delete(0, "end")
            for r in res:
                self.day_lb.insert("end", f"{r['title']}   ({r['hours']:.1f} h)   {r['channel']}")
            for i, r in enumerate(res):
                if re.search(r"\bday\s*\d", r["title"], re.I) and q.split()[0].lower() in r["title"].lower():
                    self.day_lb.selection_set(i)
            self._status(1, f"{len(res)} long video(s) found; check the selection and press Use selected")
        self.settings["day_query"] = q
        self._run(work, done, "Searching YouTube…")

    def _add_day_url(self):
        u = self.day_url.get().strip()
        if u:
            self.day_found = [{"id": core.youtube_id(u) or u, "title": u, "hours": 0.0, "channel": "", "url": u}] + self.day_found
            self.day_lb.insert(0, u)
            self.day_lb.selection_set(0)
            self.day_url.set("")

    def _use_days(self):
        sel = [self.day_found[i] for i in self.day_lb.curselection()]
        if not sel:
            return messagebox.showinfo("Whole-day videos", "Select the Day 1 / Day 2 videos in the list first.")

        def work():
            import webcast
            return [webcast.day_video(r["id"]) for r in sel]

        def done(days):
            self.days = sorted(days, key=lambda d: d.start_epoch)
            self.settings["day_videos"] = [d.to_dict() for d in self.days]
            core.save_settings(self.settings)
            self._day_text()
        self._run(work, done, "Reading the videos…")

    def _clip_this(self):
        if not self.ml:
            return messagebox.showinfo("Whole-day videos", "Select a log first.")
        if not self.days:
            return messagebox.showinfo("Whole-day videos", "Choose the whole-day videos first (Find on YouTube, Use selected).")
        ml, path, days = self.ml, self.ml.path, list(self.days)
        others = [str(f) for f in self.log_files]
        poses = [self.drive_var.get(), self.quest_var.get()]

        def work():
            import webcast
            day, _est = webcast.locate(path, ml, days)
            if day is not None and not self.day_cal.get(day.id):
                # a day video is full of look-alike matches (Q13 and Q14 start 7 minutes apart): first learn this day's clock
                # error from a few of the logs in the folder, then the match can only be the one at that offset
                items = []
                for f in others:
                    try:
                        m = core.load_match_log(f, poses)
                        if m.has_match:
                            items.append((f, m))
                    except Exception:  # noqa
                        pass
                webcast.calibrate(day, items, self.day_cal, lambda fr, msg: self._progress_cb(fr * 0.5, msg))
            return webcast.clip_match(path, ml, days, progress=lambda fr, msg: self._progress_cb(0.5 + fr / 2, msg),
                                      cal=self.day_cal)

        def done(c):
            self.settings["day_cal_v3"] = self.day_cal
            core.save_settings(self.settings)
            if self.ml is None or self.ml.path != path:
                return
            self.video_path = c.path
            self.url_var.set("")
            self.offset_var.set(round(c.offset, 3))
            q = "good" if c.good else "LOW - check the preview"
            self.conf_var.set(f"day video: found {c.note}; sync confidence {c.confidence:.1f} ({q})")
            info = core.video_info(c.path)
            self.video_var.set(f"{c.path}   ({info['width']}x{info['height']}, {info['duration'] or 0:.0f}s, clipped from {c.day.title})")
            self._set_default_out()
            self._save()
            self._viewer_controls_refresh()
            self._schedule_preview(50)
        self._run(work, done, "Cutting the match out of the day video…")

    # ---------------------------------------------------------------- hoot
    def _auto_hoots(self):
        import hoot
        return hoot.find_hoots_for_log(self.ml.path, self.folder_var.get() or None) if self.ml else []

    def _set_hoots(self, paths, auto=False):
        self.hoot_paths = list(paths)
        if not paths:
            self.hoot_var.set("none found next to this log (Find / Browse…)")
        else:
            names = ", ".join(Path(p).name for p in paths)
            self.hoot_var.set(("found: " if auto else "") + names)
        self._load_hoot_bg()

    def _load_hoot_bg(self):
        """Convert (owlet, cached) and load the hoot in a background thread so the dashboard can show it."""
        self.hoot_log = None
        if not (self.hoot_paths and self.ml):
            self._hoot_key = None
            return
        key = (tuple(self.hoot_paths), self.ml.path)
        self._hoot_key = key
        paths, log, owlet = list(self.hoot_paths), self.ml.path, self.settings.get("owlet_path") or None

        def work():
            try:
                import hoot
                conv = [hoot.convert_hoot(p, owlet_path=owlet) for p in paths]
                h = hoot.load_many([w for w, _ in conv], log, [x for _, i in conv for x in i["warnings"]])
                self.q.put(("call", self._hoot_ready, (key, h, None)))
            except Exception as e:  # noqa
                self.q.put(("call", self._hoot_ready, (key, None, str(e).splitlines()[0] if str(e) else type(e).__name__)))
        threading.Thread(target=work, daemon=True).start()
        self.hoot_var.set(self.hoot_var.get().split("   [")[0] + "   [loading…]")

    def _hoot_ready(self, res):
        key, h, err = res
        if key != self._hoot_key:
            return                                   # another log was selected meanwhile
        base = self.hoot_var.get().split("   [")[0]
        if h is not None:
            self.hoot_log = h
            self.hoot_var.set(base + f"   [loaded, clock offset {h.offset:+.2f} s]")
            self._schedule_preview(50)
        else:
            self.hoot_var.set(base + f"   [not used: {err}]")

    def _find_hoots(self):
        if not self.ml:
            return
        self._set_hoots(self._auto_hoots(), auto=True)
        if not self.hoot_paths:
            messagebox.showinfo("CTRE hoot", "No .hoot file whose name or folder contains this match "
                                             f"({self._match_token()}) was found under the logs folder.\nUse Browse… to pick it.")
        self._save()

    def _match_token(self):
        import hoot
        return hoot.match_token(self.ml.path) or "event_match"

    def _browse_hoot(self):
        fs = filedialog.askopenfilenames(initialdir=self.folder_var.get() or str(Path.home()),
                                         filetypes=[("CTRE hoot logs", "*.hoot"), ("All", "*.*")])
        if fs:
            self._set_hoots(fs)
            self._save()

    def _convert_hoot(self):
        fs = filedialog.askopenfilenames(initialdir=self.folder_var.get() or str(Path.home()),
                                         filetypes=[("CTRE hoot logs", "*.hoot")], title="Hoot files to convert")
        if not fs:
            return
        owlet = self.settings.get("owlet_path") or None

        def work():
            import hoot
            out = []
            for i, f in enumerate(fs):
                self._progress_cb(i / len(fs), f"Converting {Path(f).name} with owlet…")
                dst = Path(f).with_suffix(".wpilog")
                w, info = hoot.convert_hoot(f, out=dst, owlet_path=owlet, force=True)
                out.append((w, info["warnings"]))
            return out

        def done(res):
            msg = "\n".join(f"{Path(w).name}" + (("   (" + "; ".join(x) + ")") if x else "") for w, x in res)
            self._scan_folder()
            messagebox.showinfo("Converted", "Saved next to the hoot file(s), ready for AdvantageScope:\n\n" + msg)
        self._run(work, done, "Converting hoot…")

    # -------------------------------------------------------------- report
    def seek(self, t):
        """Jump the preview to t seconds after time zero (used by the report window)."""
        if self.mode != LOG or not self.ml:
            return
        self.t_var.set(max(-5.0, min(float(t), float(self._scrub_max))))
        self._schedule_preview(10)

    def _report(self):
        if not self.ml:
            return messagebox.showinfo("Match report", "Load a log first.")
        ml, path = self.ml, self.ml.path
        poses = {"drive": self.drive_var.get(), "quest": self.quest_var.get()}
        thr = self._thr()
        hoots = list(self.hoot_paths)
        owlet = self.settings.get("owlet_path") or None

        def work():
            import hoot
            import report
            import telemetry
            tel = telemetry.Telemetry(path, ml, poses)
            h, problem = None, None
            if hoots:
                try:
                    self._progress_cb(0.3, "Converting the hoot with owlet…")
                    conv = [hoot.convert_hoot(p, owlet_path=owlet) for p in hoots]
                    h = hoot.load_many([w for w, _ in conv], path, [x for _, i in conv for x in i["warnings"]])
                except hoot.HootError as e:
                    problem = str(e)
                except Exception as e:  # noqa
                    problem = f"{type(e).__name__}: {e}"
            rep = report.build_report(tel, ml, thr, h)
            if problem:
                rep.sections.append(report.Section("CTRE hoot", [report.Item(None, "warn", "the hoot was not used: " + problem)]))
            return rep, problem

        def done(res):
            rep, problem = res
            import report_window
            report_window.ReportWindow(self, rep, Path(path).stem)
            if problem and "was not found" in problem:
                f = filedialog.askopenfilename(title="Locate owlet (CTRE's hoot converter)",
                                               filetypes=[("owlet", "owlet*"), ("All", "*.*")])
                if f:
                    self.settings["owlet_path"] = f
                    core.save_settings(self.settings)
                    self._report()
        self._run(work, done, "Building match report…")

    # --------------------------------------------------------------- batch
    def _batch(self):
        if not self.log_files:
            return
        if not messagebox.askyesno("Batch", "For every log with a match: use the match's own video from TBA if there is one, "
                                            "otherwise cut the match out of the whole-day event videos "
                                            + ("(chosen: %d)" % len(self.days) if self.days else "(none chosen yet)")
                                            + ", auto-sync and render the MP4, and save a one-page match report "
                                            "(PNG + HTML) for every log. Nothing more is asked once it starts.\n\nVideos and reports that "
                                            "already exist are skipped. Low-confidence syncs are listed at the end "
                                            "(and in videos/batch_summary.txt). Cutting a match out of a day video "
                                            "takes a few minutes.\n\nContinue?"):
            return
        self._save()
        files = list(self.log_files)
        prefs = {k: dict(v) for k, v in self.settings.get("event_prefs", {}).items()}    # per-event TBA keys (the batch may cover several events)
        ev_default = self.event_var.get().strip()
        api = self.tbakey_var.get().strip()
        drive, quest = self.drive_var.get(), self.quest_var.get()
        red_left = self.redleft_var.get()
        height = SIZES.get(self.size_var.get(), 540)
        crf = QUALITY.get(self.qual_var.get(), 27)
        field_obj = self.field_obj or core.synthetic_field()
        out_dir = Path(self.folder_var.get()) / "videos"
        per = self.settings.setdefault("per_log", {})
        trends = list(self.trends)
        thr = self._thr()
        folder = self.folder_var.get() or None
        days = list(self.days)

        def work():
            report = []
            # ---- 1. which matches have a video of their own (TBA) and which must be cut out of a day video
            plan = {}
            for i, f in enumerate(files):
                if self.cancel_flag:
                    break
                rec = per.setdefault(f.name, {})
                has_file = any((out_dir / (f.stem + tag)).exists() for tag in ("_sidebyside.mp4", "_dashboard.mp4"))
                if rec.get("rendered") and has_file and not self._stale(rec):      # (a deleted or moved video is made again)
                    continue
                self._progress_cb(i / len(files) * 0.2, f"[{i + 1}/{len(files)}] {f.name}: looking for the match video")
                try:
                    ml = core.load_match_log(str(f), [drive, quest])
                    if not ml.has_match:                     # a shop log: the dashboard on its own
                        plan[f] = (ml, None, None)
                        continue
                    url = rec.get("url")
                    key = core.tba_match_key(self._event_key(ml, prefs) or ev_default, ml)
                    if not url and key:
                        try:
                            urls, _ = core.tba_find_video(key, api)
                            url = urls[0] if urls else None
                        except Exception:  # noqa
                            url = None
                    plan[f] = (ml, url, key)
                except Exception as e:  # noqa
                    report.append(f"{f.name}: ERROR {e}")
            # ---- 2. cut those without one out of the whole-day videos (the first of each day alone, then 3 at a time)
            clips = {}
            need = [(str(f), v[0]) for f, v in plan.items() if not v[1] and v[0].has_match]
            if days and plan:
                import webcast
                # (also learns each day's clock error from all the matches, which a match with a video of its own may need later)
                clips = webcast.clip_matches(need, days, self.day_cal, lambda fr, msg: self._progress_cb(0.2 + 0.3 * fr, msg),
                                             cancel=lambda: self.cancel_flag,
                                             calib_items=[(str(f), v[0]) for f, v in plan.items()])
                self.settings["day_cal_v3"] = self.day_cal
                core.save_settings(self.settings)
            # ---- 3. sync (TBA videos) and render each
            for n, (f, (ml, url, key)) in enumerate(plan.items()):
                if self.cancel_flag:
                    break
                rec = per.setdefault(f.name, {})
                pf = lambda fr, msg, n=n, f=f: self._progress_cb(0.5 + 0.4 * (n + fr) / len(plan), f"[{n + 1}/{len(plan)}] {f.name}: {msg}")
                try:
                    src = "match video"
                    if url:
                        pf(0.1, "downloading")
                        vp = core.download_youtube(url)
                        pf(0.3, "syncing")
                        import webcast
                        off, conf, det = webcast.sync_video(vp, ml)
                        good = conf > 6 and det["ratio"] > 1.15
                        if not good and days:                # a weak sync on the match's own video: try the day video instead
                            try:
                                import webcast
                                c = webcast.clip_match(str(f), ml, days, cal=self.day_cal)
                                if c.good:
                                    vp, off, conf, good, src = c.path, c.offset, c.confidence, True, "day video (the match video synced weakly)"
                            except Exception:  # noqa
                                pass
                    else:
                        c = clips.get(str(f))
                        if not ml.has_match:
                            vp, off, conf, good, src = None, 0.0, 0.0, True, "dashboard only (no match start in the log)"
                        elif c is None or isinstance(c, Exception):
                            why = "no whole-day videos chosen" if c is None else str(c)
                            vp, off, conf, good, src = None, 0.0, 0.0, True, (
                                "dashboard only (no match video: " + ("none on TBA" + (f" ({key})" if key else "")) + "; " + why + ")")
                        else:
                            vp, off, conf, good, src = c.path, c.offset, c.confidence, c.good, "day video"
                    hl = None
                    try:                                     # the CTRE hoot of this match, if there is one
                        import hoot
                        hs = hoot.find_hoots_for_log(str(f), folder)
                        if hs:
                            conv = [hoot.convert_hoot(h_) for h_ in hs]
                            hl = hoot.load_many([w for w, _ in conv], str(f))
                    except Exception:  # noqa
                        hl = None
                    title = rec.get("title") or self._default_title(ml)
                    rl = red_left                            # the video decides which side red is on, event by event
                    if vp:
                        try:
                            seen = core.detect_red_left(vp)
                        except Exception:  # noqa
                            seen = None
                        if seen is not None:
                            rl = seen
                    job = core.RenderJob(ml, vp, off, str(out_dir / (f.stem + ("_sidebyside.mp4" if vp else "_dashboard.mp4"))), field_obj,
                                         quest, drive, height, crf=crf, trends=trends, thresholds=thr, hoot=hl,
                                         style=core.RenderStyle(red_left=rl, title=title))
                    core.render(job, lambda fr, msg: pf(0.35 + 0.65 * fr, msg), lambda: self.cancel_flag)
                    rec.update(url=url or "", video=vp, offset=off, title=title, rendered=True, clip_ver=self._clip_ver(vp))
                    core.save_settings(self.settings)
                    flag = "" if good else "   ⚠ CHECK SYNC"
                    report.append(f"{f.name}: {src}, offset {off:.2f}s, confidence {conf:.1f}{flag}")
                except Exception as e:  # noqa
                    report.append(f"{f.name}: ERROR {e}")

            # ---- 4. the one-page match report (PNG for Slack, HTML for the details) of every log
            def make_report(f):
                import hoot
                import report as rp
                import report_image
                import telemetry
                png = out_dir / (f.stem + "_report.png")
                if png.exists():
                    return False
                ml = core.load_match_log(str(f), [drive, quest])
                tel = telemetry.Telemetry(str(f), ml, {"drive": drive, "quest": quest})
                hl = None
                try:
                    hs = hoot.find_hoots_for_log(str(f), folder)
                    if hs:
                        conv = [hoot.convert_hoot(h_) for h_ in hs]
                        hl = hoot.load_many([w for w, _ in conv], str(f))
                except Exception:  # noqa
                    hl = None
                r = rp.build_report(tel, ml, thr, hl)
                out_dir.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(png), report_image.render_image(r))
                (out_dir / (f.stem + "_report.html")).write_text(r.to_html(), encoding="utf-8")
                ov = r.overview()
                n_bad = sum(1 for _, i in ov if i.sev == "bad")
                n_warn = sum(1 for _, i in ov if i.sev == "warn")
                report.append(f"{f.name}: report saved ({n_bad} problem(s), {n_warn} warning(s))")
                return True
            for n, f in enumerate(files):
                if self.cancel_flag:
                    break
                self._progress_cb(0.9 + 0.1 * n / len(files), f"[{n + 1}/{len(files)}] {f.name}: making the match report")
                try:
                    make_report(f)
                except Exception as e:  # noqa
                    report.append(f"{f.name}: report ERROR {e}")
            try:                                                      # a record of what happened, for later
                out_dir.mkdir(parents=True, exist_ok=True)
                (out_dir / "batch_summary.txt").write_text("\n".join(report) + "\n", encoding="utf-8")
            except OSError:
                pass
            return report

        def done(report):
            self._scan_folder()
            messagebox.showinfo("Batch finished", "\n".join(report) or "Nothing to do.")
        self._run(work, done, "Batch…")

    def close(self):
        try:
            self._save()
            self._close_viewer()
        finally:
            if self.session:
                self.session.stop()
            self.destroy()


def main():
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.close)
    app.mainloop()


if __name__ == "__main__":
    main()
