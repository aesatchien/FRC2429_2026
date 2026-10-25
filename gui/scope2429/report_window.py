"""
The window that shows a report.Report: an Overview tab with every warning, and one tab per section with its findings
and its table.  Double-click a finding that has a time to jump the main window's preview to that moment.
"""
from __future__ import annotations

import base64
import os
import tempfile
import tkinter as tk
import webbrowser
from tkinter import ttk, filedialog

SEV_TEXT = {"bad": "PROBLEM", "warn": "warning", "info": "info", "ok": "ok"}
SEV_COLOR = {"bad": "#c00000", "warn": "#b45f06", "info": "#222222", "ok": "#2e7d32"}


class ReportWindow(tk.Toplevel):
    def __init__(self, app, report, name="match"):
        super().__init__(app)
        self.app, self.report, self.name = app, report, name
        self.title(report.title)
        self.geometry("1100x700")
        top = ttk.Frame(self)
        top.pack(fill="x", padx=6, pady=4)
        ttk.Label(top, text=report.title, font=("Segoe UI", 11, "bold")).pack(side="left")
        ttk.Button(top, text="Save for Slack (PNG)…", command=self._save_png).pack(side="right")
        ttk.Button(top, text="Save HTML / text…", command=self._save).pack(side="right", padx=4)
        ttk.Button(top, text="Open in browser", command=self._browser).pack(side="right", padx=4)
        ttk.Button(top, text="Copy as text", command=self._copy).pack(side="right", padx=4)
        ttk.Label(self, text="Double-click a finding with a time to jump the preview to it.  Times are seconds after "
                             "auto start (or the first enable).", foreground="#666").pack(anchor="w", padx=8)
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=6, pady=4)
        self._picture_tab(nb)
        ov = report.overview()
        n_bad = sum(1 for _, i in ov if i.sev == "bad")
        n_warn = sum(1 for _, i in ov if i.sev == "warn")
        self._tab(nb, f"Overview ({n_bad} problems, {n_warn} warnings)",
                  [(sec, it) for sec, it in ov], None, None, show_section=True)
        for s in report.sections:
            self._tab(nb, s.title.split(":")[0].split(" (")[0], [(None, it) for it in s.items], s.columns, s.rows, s.note)

    def _picture(self):
        import cv2
        import report_image
        return report_image.render_image(self.report), cv2

    def _picture_tab(self, nb):
        """The one-page picture, as it will look in Slack."""
        f = ttk.Frame(nb)
        nb.add(f, text="One-page picture")
        img, cv2 = self._picture()
        self._png = cv2.imencode(".png", img)[1].tobytes()
        scale = 1060 / img.shape[1]
        small = cv2.resize(img, (1060, int(img.shape[0] * scale)), interpolation=cv2.INTER_AREA)
        self._preview = tk.PhotoImage(data=base64.b64encode(cv2.imencode(".png", small)[1].tobytes()))
        cv = tk.Canvas(f, highlightthickness=0)
        sb = ttk.Scrollbar(f, orient="vertical", command=cv.yview)
        cv.configure(yscrollcommand=sb.set, scrollregion=(0, 0, 1060, small.shape[0]))
        cv.create_image(0, 0, anchor="nw", image=self._preview)
        sb.pack(side="right", fill="y")
        cv.pack(fill="both", expand=True)

    def _tab(self, nb, title, items, columns, rows, note="", show_section=False):
        f = ttk.Frame(nb)
        nb.add(f, text=title)
        pw = ttk.Panedwindow(f, orient="vertical")
        pw.pack(fill="both", expand=True)
        cols = ("t", "level", "section", "finding") if show_section else ("t", "level", "finding")
        tv = ttk.Treeview(pw, columns=cols, show="headings", selectmode="browse")
        tv.heading("t", text="time")
        tv.heading("level", text="level")
        tv.column("t", width=70, anchor="e", stretch=False)
        tv.column("level", width=80, stretch=False)
        if show_section:
            tv.heading("section", text="section")
            tv.column("section", width=170, stretch=False)
        tv.heading("finding", text="finding")
        tv.column("finding", width=800)
        for sev, color in SEV_COLOR.items():
            tv.tag_configure(sev, foreground=color)
        sb = ttk.Scrollbar(tv, orient="vertical", command=tv.yview)
        tv.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        times = {}
        for sec, it in items:
            vals = ["" if it.t is None else f"{it.t:+.1f} s", SEV_TEXT[it.sev]]
            if show_section:
                vals.append(sec.split(":")[0].split(" (")[0])
            vals.append(it.msg)
            iid = tv.insert("", "end", values=vals, tags=(it.sev,))
            times[iid] = it.t
        if not items:
            tv.insert("", "end", values=["", "", "nothing to report"], tags=("info",)) if not show_section else \
                tv.insert("", "end", values=["", "ok", "", "no problems or warnings found in the logged data"], tags=("ok",))
        tv.bind("<Double-Button-1>", lambda e, tv=tv, times=times: self._seek(tv, times))
        pw.add(tv, weight=3)
        if columns:
            tb = ttk.Treeview(pw, columns=list(range(len(columns))), show="headings", height=8)
            for i, c in enumerate(columns):
                tb.heading(i, text=c)
                tb.column(i, width=110, anchor="center")
            for r in rows:
                tb.insert("", "end", values=r)
            pw.add(tb, weight=2)
        if note:
            ttk.Label(f, text=note, wraplength=1000, foreground="#666", justify="left").pack(anchor="w", padx=6, pady=3)

    def _seek(self, tv, times):
        sel = tv.selection()
        if sel and times.get(sel[0]) is not None:
            self.app.seek(times[sel[0]])

    def _save_png(self):
        f = filedialog.asksaveasfilename(parent=self, defaultextension=".png", initialfile=f"{self.name}_report.png",
                                         filetypes=[("PNG picture (drag into Slack)", "*.png")])
        if f:
            with open(f, "wb") as fh:
                fh.write(self._png)

    def _save(self):
        f = filedialog.asksaveasfilename(parent=self, defaultextension=".html", initialfile=f"{self.name}_report.html",
                                         filetypes=[("HTML page (colorized)", "*.html"), ("Plain text", "*.txt")])
        if f:
            with open(f, "w", encoding="utf-8") as fh:
                fh.write(self.report.to_text() if f.lower().endswith(".txt") else self.report.to_html())

    def _browser(self):
        """Write the HTML report to a temporary file and open it in the default browser."""
        path = os.path.join(tempfile.gettempdir(), f"{self.name}_report.html")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.report.to_html())
        webbrowser.open("file:///" + path.replace(os.sep, "/"))

    def _copy(self):
        self.clipboard_clear()
        self.clipboard_append(self.report.to_text())
