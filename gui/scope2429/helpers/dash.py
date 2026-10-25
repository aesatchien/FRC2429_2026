"""
Dashboard-style frame: the one layout Scope2429 uses for a match video, a plain log and a live robot.

  top-left   an emulation of 2429's own PyQt dashboard (gui/dashboard_2026.py): field with the
             robot / QuestNav icons, QuestNav + camera indicators, PDH monitors, pose read-outs ...
  bottom     scoring estimate, PDH plot, swerve module states, motor temperatures,
             Mechanism2d, driver-controller and button-box state.

Only data that is logged from NetworkTables / the Driver Station is shown.  The top-right
960x540 slot is the match video; ffmpeg overlays it, so it is left black in a render.  Without a video (a log
with no recording, or a live connection) it shows scrolling trend plots instead (trends.py).
"""
from __future__ import annotations

import colorsys
import math
from pathlib import Path

import cv2
import numpy as np

import core
import telemetry as T

W, H = 1920, 1080          # full canvas
PW, PH = 960, 540          # one quadrant
F = cv2.FONT_HERSHEY_SIMPLEX
FD = cv2.FONT_HERSHEY_DUPLEX
ASSETS = Path(__file__).resolve().parent.parent / "assets"

# --- colors (BGR).  The "STYLE_*" ones are the dashboard's own stylesheet colors.
GREEN, RED, GRAY_T = (0, 235, 80), (0, 0, 220), (180, 180, 180)
BLUE_T, BLACK, WHITE = (255, 20, 0), (0, 0, 0), (255, 255, 255)
HUB_RED, HUB_BLUE = (0, 0, 225), (225, 0, 0)
BG, PANEL, EDGE = (24, 24, 24), (34, 34, 34), (78, 78, 78)
TXT, DIM, CYAN = (225, 225, 225), (150, 150, 150), (60, 220, 255)
AMBER, LIME = (0, 190, 255), (80, 220, 60)
ACT_C, DES_C = (255, 200, 40), (0, 140, 255)       # swerve actual / desired
DEFAULT_THR = {"rpm": 500.0, "speed": 0.3, "angle": 10.0,      # deviation indicators: motor rpm, swerve m/s, swerve deg
               "warn_v": 8.0, "brown_v": 7.0}                    # battery volts: yellow marks under warn_v, red under brown_v
OVER, UNDER, WITHIN, IDLE = (40, 40, 235), (225, 120, 20), (0, 200, 80), (95, 95, 95)    # BGR indicator colors
# (label, commanded rpm key, measured rpm key) for the motor indicators
MOTORS = [("FLY L", "shooter_target", "fly_l"), ("FLY R", "shooter_target", "fly_r"), ("ROLLER", "roller_set", "roller_a"),
          ("HOPPER", "hopper_rpm", "hop_a"), ("IDX L", "indexer_rpm", "idx_l"), ("IDX R", "indexer_rpm", "idx_r"),
          ("INT L", "intake_rpm", "intake_l"), ("INT R", "intake_rpm", "intake_r")]
HOOT_C = (255, 215, 90)                       # the CTRE hoot's traces / tags
FAULT_SHORT = {"BootDuringEnable": "BOOT", "BridgeBrownout": "BROWNOUT", "Hardware": "HARDWARE", "Undervoltage": "UNDERVOLT",
               "RotorFault1": "ROTOR", "RotorFault2": "ROTOR", "ProcTemp": "TEMP", "DeviceTemp": "TEMP",
               "RemoteSensorDataInvalid": "SENSOR"}
MOD_NAMES = ["LF", "RF", "LB", "RB"]
BAR_SYN, BAR_UNS = (0, 150, 230), (200, 50, 200)      # accuracy bars: QuestNav synced / unsynced
DOT_SYN, DOT_UNS = (0, 170, 255), (255, 80, 255)
MOD_COL = [(255, 180, 60), (80, 220, 60), (60, 160, 255), (230, 90, 230)]


# ----------------------------------------------------------------------------- drawing helpers
# OpenCV's Hershey fonts draw ASCII only: every other character (each UTF-8 byte of it) comes out as '?', which is how
# "2429 · SoCal Showdown" became "2429 ?? SoCal Showdown" and "-54°" became "-54??".  Swap the ones we use for ASCII.
_ASCII_FOR = {"·": "|", "°": " deg", "—": "-", "–": "-", "…": "...", "±": "+/-", "→": "->", "←": "<-", "×": "x",
              "µ": "u", "‘": "'", "’": "'", "“": '"', "”": '"', " ": " "}


def ascii_text(s) -> str:
    s = str(s)
    for ch, rep in _ASCII_FOR.items():
        s = s.replace(ch, rep)
    return s.encode("ascii", "replace").decode("ascii")        # anything left over is still a '?', one per character


def put(img, s, x, y, scale=0.5, color=TXT, th=1, anchor="l", font=F):
    s = ascii_text(s)
    (w, _h), _ = cv2.getTextSize(s, font, scale, th)
    if anchor == "c":
        x -= w // 2
    elif anchor == "r":
        x -= w
    cv2.putText(img, s, (int(x), int(y)), font, scale, color, th, cv2.LINE_AA)


def put_mid(img, s, cx, cy, scale=0.5, color=TXT, th=1, font=F):
    s = ascii_text(s)
    (w, h), _ = cv2.getTextSize(s, font, scale, th)
    cv2.putText(img, s, (int(cx - w / 2), int(cy + h / 2)), font, scale, color, th, cv2.LINE_AA)


def fit_scale(s, width, scale, th=1, font=F):
    w = cv2.getTextSize(s, font, scale, th)[0][0]
    return scale if w <= width else scale * width / w


def rrect(img, x0, y0, x1, y1, color, r=6, th=-1):
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    r = max(0, min(r, (x1 - x0) // 2, (y1 - y0) // 2))
    if th < 0:
        cv2.rectangle(img, (x0 + r, y0), (x1 - r, y1), color, -1)
        cv2.rectangle(img, (x0, y0 + r), (x1, y1 - r), color, -1)
        for cx, cy in ((x0 + r, y0 + r), (x1 - r, y0 + r), (x0 + r, y1 - r), (x1 - r, y1 - r)):
            cv2.circle(img, (cx, cy), r, color, -1, cv2.LINE_AA)
    else:
        cv2.line(img, (x0 + r, y0), (x1 - r, y0), color, th, cv2.LINE_AA)
        cv2.line(img, (x0 + r, y1), (x1 - r, y1), color, th, cv2.LINE_AA)
        cv2.line(img, (x0, y0 + r), (x0, y1 - r), color, th, cv2.LINE_AA)
        cv2.line(img, (x1, y0 + r), (x1, y1 - r), color, th, cv2.LINE_AA)
        for (cx, cy), a in (((x0 + r, y0 + r), 180), ((x1 - r, y0 + r), 270), ((x1 - r, y1 - r), 0), ((x0 + r, y1 - r), 90)):
            cv2.ellipse(img, (cx, cy), (r, r), 0, a, a + 90, color, th, cv2.LINE_AA)


def tile(img, x, y, w, h, text, bg, fg=BLACK, border=None, scale=0.5, th=1, r=7):
    """A rounded status label like the dashboard's QLabels; '\\n' makes a second line."""
    if border is not None:
        rrect(img, x, y, x + w, y + h, border, r)
        rrect(img, x + 3, y + 3, x + w - 3, y + h - 3, bg, max(2, r - 3))
    else:
        rrect(img, x, y, x + w, y + h, bg, r)
    lines = text.split("\n")
    sc = min(fit_scale(ln, w - 8, scale, th) for ln in lines) if lines else scale
    lh = int(h / len(lines))
    for i, ln in enumerate(lines):
        put_mid(img, ln, x + w / 2, y + lh * i + lh / 2, sc, fg, th)


def group(img, x, y, w, h, title=""):
    rrect(img, x, y, x + w, y + h, EDGE, 6, 1)
    if title:
        tw = cv2.getTextSize(title, F, 0.45, 1)[0][0]
        cv2.rectangle(img, (x + 8, y - 2), (x + 14 + tw, y + 3), BG, -1)
        put(img, title, x + 11, y + 4, 0.45, CYAN)


def hue_color(v, lo, hi, red_high):
    """The dashboard's WarningLabel: hue sweeps red -> green across [lo, hi]."""
    n = (min(hi, max(lo, v)) - lo) / (hi - lo)
    hue = ((1 - n) if red_high else n) * 101 / 360.0
    r, g, b = colorsys.hsv_to_rgb(hue, 240 / 255, 240 / 255)
    return (int(b * 255), int(g * 255), int(r * 255))


def warning_tile(img, x, y, w, h, value, lo, hi, red_high, text, label, t):
    col = hue_color(value, lo, hi, red_high)
    bad = (value > hi) if red_high else (value < lo)
    fg = (WHITE if int(t * 4) % 2 else BLACK) if bad else BLACK
    rrect(img, x, y, x + w, y + h, col, 7)
    put(img, label, x + 6, y + 11, 0.34, BLACK)
    put_mid(img, text, x + w / 2, y + h * 0.66, 0.62, fg, 2)


def hex_bgr(s, default=(52, 137, 235)):
    try:
        s = s.lstrip("#")
        return (int(s[4:6], 16), int(s[2:4], 16), int(s[0:2], 16))
    except Exception:
        return default


def blit(dst, src_bgra, cx, cy, alpha=1.0):
    """Alpha-blend a BGRA image centred at (cx, cy)."""
    h, w = src_bgra.shape[:2]
    x0, y0 = int(round(cx - w / 2)), int(round(cy - h / 2))
    X0, Y0, X1, Y1 = max(0, x0), max(0, y0), min(dst.shape[1], x0 + w), min(dst.shape[0], y0 + h)
    if X0 >= X1 or Y0 >= Y1:
        return
    s = src_bgra[Y0 - y0:Y1 - y0, X0 - x0:X1 - x0]
    a = (s[:, :, 3:4].astype(np.float32) / 255.0) * alpha
    roi = dst[Y0:Y1, X0:X1]
    roi[:] = (s[:, :, :3] * a + roi * (1 - a)).astype(np.uint8)


def load_icon(name, size):
    im = cv2.imread(str(ASSETS / name), cv2.IMREAD_UNCHANGED)
    if im is None:
        return None
    if im.shape[2] == 3:
        im = cv2.cvtColor(im, cv2.COLOR_BGR2BGRA)
    return cv2.resize(im, (size, size), interpolation=cv2.INTER_AREA)


def rotated(icon, angle_ccw_deg, pad=1.5):
    s = int(icon.shape[0] * pad)
    canvas = np.zeros((s, s, 4), np.uint8)
    o = (s - icon.shape[0]) // 2
    canvas[o:o + icon.shape[0], o:o + icon.shape[1]] = icon
    M = cv2.getRotationMatrix2D((s / 2, s / 2), angle_ccw_deg, 1.0)
    return cv2.warpAffine(canvas, M, (s, s), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                          borderValue=(0, 0, 0, 0))


def bar_h(img, x, y, w, h, frac, color, bg=(55, 55, 55)):
    cv2.rectangle(img, (x, y), (x + w, y + h), bg, -1)
    f = max(0.0, min(1.0, frac))
    if f > 0:
        cv2.rectangle(img, (x, y), (x + int(w * f), y + h), color, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), EDGE, 1)


def bar_v(img, x, y, w, h, frac, color, bg=(55, 55, 55)):
    cv2.rectangle(img, (x, y), (x + w, y + h), bg, -1)
    f = max(0.0, min(1.0, frac))
    if f > 0:
        cv2.rectangle(img, (x, y + h - int(h * f)), (x + w, y + h), color, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), EDGE, 1)


class Plot:
    """A time-series plot whose static part is drawn once; only the cursor is drawn per frame."""

    def __init__(self, rect, t0, t1, y0, y1, grid, fmt="{:g}", unit="", xticks=None):
        self.xticks = xticks            # [(x value, label)] to replace the default 30 s time ticks
        self.x, self.y, self.w, self.h = rect
        self.t0, self.t1, self.y0, self.y1 = t0, t1, y0, y1
        self.grid, self.fmt, self.unit = grid, fmt, unit

    def px(self, t, v):
        return (self.x + (t - self.t0) / (self.t1 - self.t0) * self.w,
                self.y + self.h - (min(self.y1, max(self.y0, v)) - self.y0) / (self.y1 - self.y0) * self.h)

    def frame(self, img):
        cv2.rectangle(img, (self.x, self.y), (self.x + self.w, self.y + self.h), (20, 20, 20), -1)
        for g in self.grid:
            _, py = self.px(self.t0, g)
            cv2.line(img, (self.x, int(py)), (self.x + self.w, int(py)), (58, 58, 58), 1)
            put(img, self.fmt.format(g), self.x - 4, py + 4, 0.34, DIM, anchor="r")
        if self.xticks is not None:
            ticks = self.xticks
        else:
            span = self.t1 - self.t0
            step = 30 if span <= 300 else 60 if span <= 600 else 120 if span <= 1200 else 300 if span <= 3600 else 900
            ticks = [(tt, f"{tt}s") for tt in range(0, int(self.t1), step) if tt > self.t0]
        for tt, lab in ticks:
            px, _ = self.px(tt, self.y0)
            cv2.line(img, (int(px), self.y), (int(px), self.y + self.h), (48, 48, 48), 1)
            put(img, lab, px, self.y + self.h + 11, 0.3, DIM, anchor="c")
        cv2.rectangle(img, (self.x, self.y), (self.x + self.w, self.y + self.h), EDGE, 1)

    def line(self, img, t, v, color, th=1):
        t = np.asarray(t, float)
        v = np.asarray(v, float)
        m = np.isfinite(v)
        t, v = t[m], v[m]
        if len(t) == 0:
            return
        pts = []
        i0 = int(np.searchsorted(t, self.t0, side="right")) - 1
        if i0 >= 0:                                         # value held from before the axis starts
            pts.append(self.px(self.t0, v[i0]))
        inside = (t > self.t0) & (t <= self.t1)
        pts += [self.px(a, b) for a, b in zip(t[inside], v[inside])]
        i1 = int(np.searchsorted(t, self.t1, side="right")) - 1
        if i1 >= 0 and pts:
            pts.append(self.px(self.t1, v[i1]))             # hold the last sample to the end of the axis
        if len(pts) >= 2:
            cv2.polylines(img, [np.array(pts, np.float32).round().astype(np.int32)], False, color, th, cv2.LINE_AA)

    def cursor(self, img, t):
        if self.t0 <= t <= self.t1:
            px, _ = self.px(t, self.y0)
            cv2.line(img, (int(px), self.y), (int(px), self.y + self.h), (255, 255, 255), 1)


# ----------------------------------------------------------------------------- renderer
class DashRenderer:
    LIVE_WIN = 60.0                    # seconds shown by the live (scrolling) plots
    PDH_WIN = 180.0                    # ... and by the live PDH plot: the last 3 minutes of running
    STRIP_Y = (414, 431, 448, 465)          # the camera / Quest / Wi-Fi strips under the PDH plots (canvas rows of the lower half)
    STRIP_H = 15
    CAN_AXES = (5, 10, 25, 50, 100)        # the CAN plot's top, the smallest of these that fits the data: a healthy bus is a few percent

    def _can_axis(self, tel, scale):
        """(top, gridlines) for the CAN utilization plot."""
        top = 0.0
        for bus, _ in T.CAN_BUSES:
            s = tel.by_key.get(f"{bus}_util")
            if s is not None and len(s):
                top = max(top, float(np.nanmax(s.numeric())) * scale)
        y1 = next((a for a in self.CAN_AXES if a >= top * 1.15), 100)
        return y1, [0, y1 / 2, y1]

    CAN_COL = {"can_s0": (255, 170, 60), "can_s1": (120, 220, 120), "can_s2": (230, 120, 220)}   # BGR

    def __init__(self, ml: core.MatchLog, field: core.FieldImage, title: str = "", red_left: bool = True,
                 pose_names: dict | None = None, flux: dict | None = None, trends=(), tel: "T.Telemetry | None" = None,
                 hoot=None):
        """`tel` is a live Telemetry (see live.py); without it the data is read from ml.path.  `hoot` is a hoot.HootLog:
        its swerve drive motor currents and faults are shown in the swerve and PDH panels."""
        self.ml = ml
        self.hoot = hoot
        self.live = tel is not None
        self.tel = tel or T.Telemetry(ml.path, ml, pose_names, extra=trends)
        self.shots = T.ShotModel(self.tel)
        self.vshots = (T.VideoShotModel(self.tel, self.shots, flux, not ml.is_blue)
                       if flux and ml.has_match else None)
        self.score = self.vshots if (self.vshots is not None and self.vshots.available) else self.shots   # only a video-based one scores
        self.title = title
        self.thr = dict(DEFAULT_THR)       # deviation thresholds (set by the GUI)
        self.t_lo = -6.0                                           # plots start a bit before time zero
        self.t_hi = (ml.match_len or 160.0) + 6.0
        self._scale_cache = {}
        self._tick = {}
        self.yaw_offset = None
        self._update_driver_view()
        from dash_left import GuiPanel          # imported here: dash_left imports helpers from this module
        from trends import TrendPanel
        self.gui = GuiPanel(self.tel, ml, field, red_left, title)
        self.trend_panel = TrendPanel(self.tel, ml, trends)
        self.corr_base = self._make_corr()
        self.low_base, self.plots = self._make_lower_base()

    @property
    def thr(self):
        return self._thr

    @thr.setter
    def thr(self, v):
        """The low-battery levels are drawn into the cached lower base (log mode), so a change of them rebuilds it."""
        old = getattr(self, "_thr", None)
        self._thr = v
        if old is not None and hasattr(self, "low_base") and not self.live \
                and any(old.get(k) != v.get(k) for k in ("warn_v", "brown_v")):
            self.low_base, self.plots = self._make_lower_base()

    @property
    def a0(self):
        return self.tel.a0

    def _every(self, key, t, period):
        """True when `period` seconds have passed since this was last true (used to refresh live state)."""
        if t - self._tick.get(key, -1e9) >= period or t < self._tick.get(key, -1e9):
            self._tick[key] = t
            return True
        return False

    def update_live(self, t):
        """Live data keeps arriving: refresh what was worked out once for a log (cheap, a few times a second)."""
        if self._every("mech", t, 1.0):
            self.tel.refresh_mech()
            self._scale_cache.clear()
        if self._every("driver", t, 2.0):
            self._update_driver_view(t)
        if self._every("shots", t, 1.5):
            self.shots = T.ShotModel(self.tel)
            self.score = self.shots

    # ------------------------------------------------------------- driver view
    def _update_driver_view(self, t_now=None):
        """navX yaw is zeroed wherever the robot started, so align it with the field heading using the
        (steady) offset to the odometry heading; then 'up' on screen is the direction the driver looks
        (away from their own alliance wall: -x for red, +x for blue).  `t_now` (live): use the last 30 s."""
        tel = self.tel
        yaw, drv = tel.by_key.get("navx_yaw"), tel.by_key.get("drive")
        if yaw is not None and drv is not None and len(yaw) and len(drv):
            if t_now is None:
                rels = np.arange(-5.0, (self.ml.match_len or 160.0), 1.0)
            else:
                rels = np.arange(t_now - self.a0 - 30.0, t_now - self.a0, 1.0)
            d = []
            for rel in rels:
                y, p = yaw.at(self.a0 + rel), drv.at(self.a0 + rel)
                if y is not None and p is not None:
                    d.append(math.radians(y) - p[2])
            if d:
                self.yaw_offset = math.degrees(math.atan2(np.median(np.sin(d)), np.median(np.cos(d))))   # circular median
        self.driver_fwd = 0.0 if self.ml.is_blue else 180.0

    def driver_rotation(self, t):
        """(rotation deg CCW from driver-forward, raw navX yaw) at log time t."""
        yaw = self.tel.get("navx_yaw", t)
        if yaw is not None and self.yaw_offset is not None:
            return yaw - self.yaw_offset - self.driver_fwd, yaw
        p = self.tel.get("drive", t)
        if p is not None:
            return math.degrees(p[2]) - self.driver_fwd, yaw
        return (yaw if yaw is not None else 0.0), yaw

    # --------------------------------------------------------------- static bases

    # ------------------------------------------------------------------- left





    # ------------------------------------------- Quest accuracy vs shot accuracy
    MOTOR_Y, MOTOR_H = 436, 104               # motor-rpm strip at the bottom of the top-left quadrant
    CORR2_H = 170                             # Quest-vs-shot-accuracy plot at the top of the lower-left quadrant

    def _burst_points(self):
        """Per indexer burst: (start, end, Quest-vs-drive error m, shot accuracy 0..1, shots fired, QuestNav synced)."""
        vs = self.vshots
        if vs is None or not vs.available:
            return []
        pts = []
        for s in vs.summary:
            if s["fired"] >= 5:
                pts.append((s["a"], s["b"], s["odo_m"], min(1.0, s["scored"] / s["fired"]), s["fired"],
                            self._synced(s["a"], s["b"])))
        return pts

    def _synced(self, a, b):
        """Was QuestNav synced (questnav_synched true) for most of the burst?  Assumed yes if it is not logged."""
        sig = self.tel.by_key.get("q_synched")
        if sig is None or not len(sig):
            return True
        vals = [sig.at(self.a0 + x) for x in np.linspace(a, b, 5)]
        vals = [v for v in vals if v is not None]
        return not vals or sum(bool(v) for v in vals) * 2 > len(vals)

    def _make_corr(self):
        pts = self.corr_pts = self._burst_points()
        h = self.CORR2_H
        im = np.full((h, PW, 3), PANEL, np.uint8)
        cv2.rectangle(im, (2, 2), (PW - 3, h - 3), EDGE, 1)
        self.corr = None
        dl = self.tel.by_key.get("delta")
        if self.live:                       # scrolling Quest-vs-drive error; drawn per frame in _draw_corr
            put(im, "QUESTNAV vs DRIVE ERROR (live)", 12, 22, 0.55, CYAN)
            self.corr_live = Plot((50, 34, 860, 100), -self.PDH_WIN, 0, 0, 2.5, [0, 1, 2], "{:g}",
                                  xticks=[(-150, "-150s"), (-120, "-120s"), (-90, "-90s"), (-60, "-60s"), (-30, "-30s")])
            return im
        put(im, "QUESTNAV vs SHOT ACCURACY" if pts else "QUESTNAV vs DRIVE ERROR", 12, 22, 0.55, CYAN)
        tp = Plot((50, 34, 860, 100), self.t_lo, self.t_hi, 0, 2.5, [0, 1, 2], "{:g}")
        have_err = dl is not None and len(dl) > 0
        if not have_err:
            put(im, "no Quest / drive pose in this data", 12, 60, 0.45, DIM)
            return im
        tp.frame(im)
        # accuracy bars (right axis, 0..100 %) behind the Quest error line
        for a, b, odo, acc, n, syn in pts:
            x0, _ = tp.px(a, 0)
            x1, _ = tp.px(b + 1.0, 0)
            top = tp.y + tp.h - int(acc * tp.h)
            ov = im.copy()
            cv2.rectangle(ov, (int(x0), top), (int(max(x1, x0 + 3)), tp.y + tp.h), BAR_SYN if syn else BAR_UNS, -1)
            im[:] = cv2.addWeighted(ov, 0.55, im, 0.45, 0)
        err = np.array([math.hypot(v[0], v[1]) for v in dl.v])
        tp.line(im, dl.t - self.a0, err, (255, 220, 60), 1)
        put(im, "Quest-vs-drive error (m)", 400, 20, 0.42, (255, 220, 60))
        if pts:
            n_uns = sum(1 for p in pts if not p[5])
            put(im, "accuracy per burst (right axis)", 600, 20, 0.42, DOT_SYN)
            if n_uns:
                put(im, "| Quest UNSYNCED", 600 + cv2.getTextSize("accuracy per burst (right axis) ", F, 0.42, 1)[0][0], 20,
                    0.42, DOT_UNS)
            for frac, lab in ((0, "0%"), (0.5, "50%"), (1, "100%")):
                put(im, lab, tp.x + tp.w + 4, tp.y + tp.h - frac * tp.h + 4, 0.32, (0, 170, 255))
        self.corr = (tp, None)
        return im

    # ------------------------------------------------- deviation indicators
    def _rpm_dev(self, set_key, act_key, t):
        """(state, signed deviation) of a motor: deviation = actual - commanded, positive = beyond the command in the
        commanded direction.  state: over / under / ok, or none (no data) / off (commanded off) / ramp (command just changed)."""
        s_sig, a_sig = self.tel.by_key.get(set_key), self.tel.by_key.get(act_key)
        if s_sig is None or a_sig is None or not len(s_sig) or not len(a_sig):
            return "none", 0.0
        st, ac = s_sig.at(t), a_sig.at(t)
        if st is None or ac is None:
            return "none", 0.0
        thr = self.thr["rpm"]
        if abs(st) < 50.0:
            return "off", float(ac)                                     # commanded off: spinning down is not a fault
        prev = s_sig.at(t - 1.0)
        if prev is not None and abs(st - prev) > thr:
            return "ramp", float(ac - st)                               # still spinning up to a new command
        d = float(ac - st) * (1.0 if st > 0 else -1.0)
        return ("over" if d > thr else "under" if d < -thr else "ok"), d

    def _draw_motors(self, im, t):
        """The strip under the dashboard replica: one tile per motor, colored by how far the actual rpm is from the command."""
        thr = self.thr["rpm"]
        put(im, "MOTOR RPM  actual vs set", 12, 22, 0.55, CYAN)
        put(im, f"+/-{thr:g} rpm", PW - 12, 22, 0.45, TXT, anchor="r")
        w, hgt, gap = 110, 40, 8
        for i, (label, set_key, act_key) in enumerate(MOTORS):
            x, y = 12 + i * (w + gap), 34
            state, d = self._rpm_dev(set_key, act_key, t)
            if state == "none":
                tile(im, x, y, w, hgt, f"{label}\n--", IDLE, TXT, scale=0.5)
            elif state in ("off", "ramp"):
                tile(im, x, y, w, hgt, label + "\n" + ("off" if state == "off" else "..."), IDLE, TXT, scale=0.5)
            else:
                col = {"over": OVER, "under": UNDER, "ok": WITHIN}[state]
                word = {"over": "OVER", "under": "UNDER", "ok": ""}[state]
                tile(im, x, y, w, hgt, f"{label}\n{d:+.0f}" + (f" {word}" if word else ""), col,
                     BLACK if state == "ok" else WHITE, scale=0.5)
        put(im, "red: over the set rpm   blue: under   grey: off / changing", 12, 92, 0.38, DIM)

    def _draw_corr(self, t):
        """The Quest-vs-drive error / shot-accuracy plot (960 x CORR2_H image) with its cursor (or scrolling, live)."""
        tr = t - self.a0
        im = self.corr_base.copy()
        if self.live:
            dl = self.tel.by_key.get("delta")
            if dl is None or len(dl) == 0:
                put(im, "no Quest pose / drive pose yet", 12, 60, 0.45, DIM)
            else:
                lp = self.corr_live
                lp.frame(im)
                self._live_line(im, lp, dl, t, (255, 220, 60), 1, fn=lambda v: np.hypot(v[:, 0], v[:, 1]), win=self.PDH_WIN)
        elif self.corr is not None:
            self.corr[0].cursor(im, tr)
        return im

    def _live_line(self, im, plot, sig, t, color, th=1, col=None, fn=None, win=None):
        """Draw the last LIVE_WIN seconds of `sig` (ending at log/session time t) on a live Plot (x = seconds before now).
        `col` picks one element of an array signal; `fn` maps the window's values (an (n, k) array) to the line."""
        st = sig.t
        i1 = int(np.searchsorted(st, t, side="right"))
        i0 = max(0, int(np.searchsorted(st, t - (win or self.LIVE_WIN), side="right")) - 1)
        if i1 - i0 < 1:
            return
        vals = sig.v[i0:i1]
        try:
            if fn is not None:
                y = fn(np.array([np.asarray(v, float) for v in vals]))
            elif col is not None:
                y = np.array([v[col] if len(v) > col else np.nan for v in vals], float)
            else:
                y = np.asarray(vals, float)
        except (ValueError, TypeError):
            return
        plot.line(im, st[i0:i1] - t, y, color, th)

    def render_left(self, t):
        self.gui.title = self.title
        img = self.gui.render(t)
        strip = np.full((self.MOTOR_H, PW, 3), BG, np.uint8)
        self._draw_motors(strip, t)
        img[self.MOTOR_Y:self.MOTOR_Y + self.MOTOR_H] = strip
        return img

    # ------------------------------------------------------------------ lower
    def _rng(self, key, pct=99.0, floor=1.0):
        if key not in self._scale_cache:
            s = self.tel.by_key.get(key)
            v = np.abs(s.numeric()) if s is not None and len(s) else np.array([floor])
            top = float(np.percentile(v, pct)) if len(v) else floor
            mag = 10 ** math.floor(math.log10(max(top, floor)))
            self._scale_cache[key] = math.ceil(max(top, floor) / mag * 1.1) * mag
        return self._scale_cache[key]

    def _make_lower_base(self):
        """Lower half (1920 x 540).  Left 960: Quest-vs-shot-accuracy plot (top, drawn per frame by _draw_corr) and the PDH
        power panel.  Right 960: three panels of 320 x 270 in each of two rows."""
        im = np.full((PH, W, 3), BG, np.uint8)
        a0, tl, th_ = self.a0, self.t_lo, self.t_hi
        plots = {}
        panels = [(0, 170, 960, 370, "PDH POWER"),
                  (960, 0, 320, 270, "MECHANISM 2D"), (1280, 0, 320, 270, "SWERVE MODULES"), (1600, 0, 320, 270, "SWERVE MOTOR TEMPERATURE"),
                  (960, 270, 320, 270, "2429 SCORING (estimate)"), (1280, 270, 320, 270, "DRIVER CONTROLLER"),
                  (1600, 270, 320, 270, "BUTTON BOX")]
        for x, y, w, h, title in panels:
            cv2.rectangle(im, (x + 2, y + 2), (x + w - 3, y + h - 3), PANEL, -1)
            cv2.rectangle(im, (x + 2, y + 2), (x + w - 3, y + h - 3), EDGE, 1)
            put(im, title, x + 12, y + 22, 0.55, CYAN, 1)
        tel = self.tel
        live = self.live
        if live:
            tl, th_ = -self.PDH_WIN, 0.0
            xt = [(-150, "-150s"), (-120, "-120s"), (-90, "-90s"), (-60, "-60s"), (-30, "-30s")]
            tlt, xtt = -self.LIVE_WIN, [(-45, "-45s"), (-30, "-30s"), (-15, "-15s")]
        else:
            xt = xtt = None
            tlt = tl

        def rel(sig):
            return sig.t - a0

        pv = Plot((50, 204, 860, 56), tl, th_, 5, 13, [6, 9, 12], "{:g}", xticks=xt)
        pa = Plot((50, 278, 860, 56), tl, th_, 0, 240, [0, 100, 200], "{:g}", xticks=xt)
        cy1, cgrid = self._can_axis(tel, T.can_util_scale(tel))
        pc = Plot((50, 354, 860, 40), tl, th_, 0, cy1, cgrid, "{:g}", xticks=xt)      # CAN bus utilization, percent
        pt = Plot((1634, 38, 270, 192), tlt, th_, 20, 80, [20, 40, 60, 80], "{:g}", xticks=xtt)
        if not live:                                  # live plots are redrawn every frame (see _live_plots)
            pv.frame(im), pa.frame(im), pc.frame(im), pt.frame(im)
            for p_, key, col in ((pv, "pdh_v", (0, 235, 255)), (pa, "pdh_a", (60, 120, 255))):
                s = tel.by_key.get(key)
                if s is not None and len(s):
                    p_.line(im, rel(s), s.numeric(), col, 1)
            for key, thick, dim in (("temp_drive", 2, 1.0), ("temp_turn", 1, 0.6)):
                s = tel.by_key.get(key)
                if s is None or len(s) == 0:
                    continue
                arr = np.array([a if len(a) == 4 else np.full(4, np.nan) for a in s.v])
                for m in range(4):
                    col = tuple(int(c * dim) for c in MOD_COL[m])
                    pt.line(im, rel(s), arr[:, m], col, thick)
        self.plot_labels = {}                            # key -> [(text, x, y, colour, scale)], redrawn after each live frame()

        def label(key, text, x, y, col, scale=0.42):
            put(im, text, x, y, scale, col)
            self.plot_labels.setdefault(key, []).append((text, x, y, col, scale))
        label("pdh_v", "volts", 56, pv.y + 13, (0, 235, 255))
        label("pdh_a", "amps", 56, pa.y + 13, (60, 120, 255))
        if not live and self.hoot is not None:
            dt_, dv_ = self._hoot_drive_sum()
            if dt_ is not None:
                pa.line(im, dt_ - a0, dv_, HOOT_C, 1)
                put(im, "swerve drive (CTRE hoot)", 112, pa.y + 13, 0.42, HOOT_C)
        # CAN bus utilization: one line per bus, from the robot's helpers/can_monitor.py (older logs have none)
        scale = T.can_util_scale(tel)
        lx = 56
        if any(len(tel.by_key.get(f"{b}_util") or []) for b, _ in T.CAN_BUSES) or live:
            label("pdh_can", "% busy:", lx, pc.y + 12, DIM, 0.36)
            lx += 52
            for bus, what in T.CAN_BUSES:
                col = self.CAN_COL[bus]
                if not live:
                    s = tel.by_key.get(f"{bus}_util")
                    if s is not None and len(s):
                        pc.line(im, rel(s), s.numeric() * scale, col, 1)
                lab = f"{bus} {what}"
                label("pdh_can", lab, lx, pc.y + 12, col, 0.36)
                lx += int(len(lab) * 6.2) + 14
        else:
            put(im, "CAN bus utilization %: not in this log (the robot has logged it since 2026-10-25)", 56, pc.y + 12, 0.36, DIM)
        self._pdh_strip_frames(im, pa)
        if not live:
            self._pdh_marks(im, pv, pa, a0, a0 + tl, a0 + th_)
        plots["pdh_v"], plots["pdh_a"], plots["pdh_can"], plots["temp"] = pv, pa, pc, pt
        return im, plots

    # ------------------------------------------------- brownouts, camera / Quest / Wi-Fi under the PDH plots
    STRIPS = (("ARDU R", "ArduCam right"), ("ARDU B", "ArduCam back"), ("QUEST", "Quest"), ("NET", "Wi-Fi / NT link"))
    C_LINK, C_STALL, C_REBOOT, C_QUEST, C_WIFI, C_BROWN = (52, 104, 235), (0, 175, 235), (70, 70, 235), (235, 140, 150), (80, 80, 255), (60, 60, 255)
    C_NODATA = (90, 60, 140)
    C_WARN = (0, 130, 255)                                # battery under warn_v (orange, so it stands out from the yellow volts line); under brown_v it is C_BROWN (red)

    def _pdh_strip_frames(self, im, pa):
        x0, x1 = pa.x, pa.x + pa.w
        for row, (lab, _long) in enumerate(self.STRIPS):
            y = self.STRIP_Y[row]
            cv2.rectangle(im, (x0, y), (x1, y + self.STRIP_H), (20, 20, 20), -1)
            cv2.rectangle(im, (x0, y), (x1, y + self.STRIP_H), (58, 58, 58), 1)
            put(im, lab, x0 - 4, y + 12, 0.34, DIM, anchor="r")
        x = x0
        ly = self.STRIP_Y[-1] + self.STRIP_H + 18
        for txt, col in (("brownout", self.C_BROWN), ("battery low", self.C_WARN), ("camera lost + coprocessor link drop", self.C_LINK),
                         ("camera lost, link up", self.C_STALL), ("camera restart", self.C_REBOOT), ("no camera data", self.C_NODATA),
                         ("Quest off", self.C_QUEST), ("Wi-Fi gap", self.C_WIFI)):
            cv2.rectangle(im, (int(x), ly - 8), (int(x) + 9, ly + 1), col, -1)
            put(im, txt, x + 14, ly, 0.4, DIM)
            x += 14 + len(txt) * 6.4 + 12
        put(im, "brownout = the supply sagged and a motor controller (Kraken) or the roboRIO cut its output.  Dots = battery under %g V (orange) / %g V (red)." % (self.thr["warn_v"], self.thr["brown_v"]),
            x0, ly + 18, 0.36, DIM)

    def _brownout_events(self, t0, t1):
        """[(time, who)] of brownouts in [t0, t1]: the roboRIO flag, and a CTRE Kraken's bridge-brownout fault (hoot).
        Reports within 0.6 s of each other are one event; `who` lists what reported it."""
        raw = [(t, "roboRIO") for t in T.rising_times(self.tel.by_key.get("rio_brownout"), t0, t1)]
        if self.hoot is not None:
            import hoot as H
            for (dev, sg), sig in self.hoot.sigs.items():
                if sg == "Fault_BridgeBrownout":
                    name = H.DEVICE_NAMES.get(dev, dev)
                    raw += [(t, name) for t in T.rising_times(sig, t0, t1)]
        out = []
        for t, who in sorted(raw):
            if out and t - out[-1][0] <= 0.6:
                if who not in out[-1][1]:
                    out[-1][1].append(who)
            else:
                out.append((t, [who]))
        return [(t, ", ".join(w)) for t, w in out]

    def _pdh_marks(self, im, pv, pa, tref, t0, t1):
        """Brownouts and low-battery dips on the volts plot, and the camera / Quest / Wi-Fi strips.  t0..t1 are log (or
        session) times; the plots' x axis is seconds after `tref`."""
        tel = self.tel
        X = lambda t: int(round(pa.px(t - tref, 0)[0]))
        x0, x1 = pa.x, pa.x + pa.w

        def block(row, a, b, col, text=""):
            xa, xb = max(X(a), x0), min(X(b), x1)
            if xb < xa:
                return
            y = self.STRIP_Y[row]
            cv2.rectangle(im, (xa, y + 1), (max(xb, xa + 2), y + self.STRIP_H - 1), col, -1)
            if text:
                put(im, text, xa + 5, y + 12, 0.36, BLACK if col == self.C_STALL else WHITE, 1)

        # battery dips: orange under warn_v, red under brown_v; then brownouts
        warn_v, brown_v = self.thr["warn_v"], self.thr["brown_v"]
        for volts, col in ((warn_v, self.C_WARN), (brown_v, self.C_BROWN)):
            yb = int(pv.px(0, volts)[1])
            for xx in range(x0, x1, 6):
                cv2.line(im, (xx, yb), (xx + 3, yb), col, 1)
        sv = tel.by_key.get("pdh_v")
        if sv is not None and len(sv):
            i0, i1 = T._win(sv, t0, t1)
            tt, vv = sv.t[i0:i1], np.asarray(sv.v[i0:i1], float)
            for lo, hi, col in ((brown_v, warn_v, self.C_WARN), (-1e9, brown_v, self.C_BROWN)):
                m = (vv >= lo) & (vv < hi)
                for a, b in zip(tt[m], vv[m]):
                    px, py = pv.px(a - tref, b)
                    c = (int(px), int(py))
                    cv2.circle(im, c, 5, WHITE, -1, cv2.LINE_AA)          # white ring so the dot stands out from the volts line
                    cv2.circle(im, c, 3, col, -1, cv2.LINE_AA)
        row_end = [-1e9, -1e9]                        # right end of the last label on each of two text rows
        for t, who in self._brownout_events(t0, t1):
            px = X(t)
            if not x0 <= px <= x1:
                continue
            cv2.line(im, (px, pv.y), (px, pv.y + pv.h), self.C_BROWN, 1)
            cv2.drawMarker(im, (px, pv.y + 6), self.C_BROWN, cv2.MARKER_DIAMOND, 10, 2)
            # say what it is: "BROWNOUT" and who reported it (a motor controller reset its output when its supply sagged)
            l1 = "BROWNOUT"
            l2 = who.replace(" drive", "") + (" drive motor" if "roboRIO" not in who else " reset")
            wid = max(cv2.getTextSize(l1, F, 0.4, 1)[0][0], cv2.getTextSize(l2, F, 0.36, 1)[0][0])
            left = px + 8 + wid > x1                    # near the right edge: write to the left of the line
            xa = px - 8 - wid if left else px + 8
            for r_, yy in enumerate((pv.y + 20, pv.y + 52)):
                if xa > row_end[r_] + 4:
                    cv2.rectangle(im, (int(xa) - 3, yy - 13), (int(xa) + wid + 3, yy + 17), (20, 20, 20), -1)
                    put(im, l1, xa, yy, 0.4, self.C_BROWN, 1)
                    put(im, l2, xa, yy + 13, 0.36, TXT, 1)
                    row_end[r_] = xa + wid
                    break
        # the two ArduCam Raspberry Pis, one strip each: a camera counts as lost when its frame counter stops for > 2.5 s
        # (or it never sent any); orange when the coprocessor also dropped off NetworkTables around then
        link = tel.net_intervals("coproc", t0, t1)
        for row, key, name in ((0, "cam_right_frames", "ARDU RIGHT"), (1, "cam_back_frames", "ARDU BACK")):
            s = tel.by_key.get(key)
            first = float(s.t[0]) if s is not None and len(s) else None
            if first is None or first > t0 + 2.5:           # nothing received before `first` (or at all)
                end = t1 if first is None else min(first, t1)
                block(row, t0, end, self.C_NODATA, f"{name}: no data" if X(end) - X(t0) > 120 else "")
            for a, b in T.stall_intervals(s, t0, t1):
                near = any(la - 3 <= b and lb + 3 >= a for la, lb in link)
                block(row, a, b, self.C_LINK if near else self.C_STALL,
                      f"{name} LOST {b - a:.0f} s" if X(b) - X(a) > 110 else (f"{b - a:.0f}s" if X(b) - X(a) > 34 else ""))
            for t in T.restart_times(s, t0, t1):
                cv2.line(im, (X(t), self.STRIP_Y[row]), (X(t), self.STRIP_Y[row] + self.STRIP_H), self.C_REBOOT, 2)
        for a, b in T.false_intervals(tel.by_key.get("q_connected"), t0, t1):
            block(2, a, b, self.C_QUEST, f"Quest disconnected {b - a:.0f} s" if X(b) - X(a) > 150 else "")
        wifi = tel.net_intervals("self", t0, t1) + tel.net_intervals("other", t0, t1) + T.data_gaps(sv, t0, t1)
        for a, b in wifi:
            block(3, a, b, self.C_WIFI)

    def _live_plots(self, im, t, keys):
        """Live: draw the scrolling PDH / temperature plots (a log has them in the static base instead)."""
        tel = self.tel
        for key in keys:
            p = self.plots[key]
            if key == "pdh_can":
                p.y1, p.grid = self._can_axis(tel, T.can_util_scale(tel))      # follows the data as it arrives
            p.frame(im)
            for text, x, y, col, sc in self.plot_labels.get(key, ()):
                put(im, text, x, y, sc, col)
            if key == "pdh_v":
                s = tel.by_key.get("pdh_v")
                if s is not None and len(s):
                    self._live_line(im, p, s, t, (0, 235, 255), win=self.PDH_WIN)
            elif key == "pdh_a":
                s = tel.by_key.get("pdh_a")
                if s is not None and len(s):
                    self._live_line(im, p, s, t, (60, 120, 255), win=self.PDH_WIN)
                self._pdh_strip_frames(im, p)
                self._pdh_marks(im, self.plots["pdh_v"], p, t, t - self.PDH_WIN, t)
            elif key == "pdh_can":
                scale = T.can_util_scale(tel)
                for bus, _what in T.CAN_BUSES:
                    s = tel.by_key.get(f"{bus}_util")
                    if s is not None and len(s):
                        self._live_line(im, p, s, t, self.CAN_COL[bus], 1, fn=lambda v, k=scale: v * k, win=self.PDH_WIN)
            else:
                for sk, thick, dim in (("temp_drive", 2, 1.0), ("temp_turn", 1, 0.6)):
                    s = tel.by_key.get(sk)
                    if s is None or len(s) == 0:
                        continue
                    for m in range(4):
                        self._live_line(im, p, s, t, tuple(int(c * dim) for c in MOD_COL[m]), thick, col=m)

    def _plot_overlay(self, im, t, keys):
        if self.live:
            self._live_plots(im, t, keys)
        else:
            for k in keys:
                self.plots[k].cursor(im, t - self.a0)

    def render_lower(self, t):
        tel = self.tel
        im = self.low_base.copy()
        tr = t - self.a0                                    # seconds since auto start
        g = lambda k, d=None: tel.get(k, t, d)
        im[0:self.CORR2_H, 0:PW] = self._draw_corr(t)
        # ======================= PDH (x 0..960, y 170..540)
        self._plot_overlay(im, t, ("pdh_v", "pdh_a", "pdh_can"))
        pa = self.plots["pdh_a"]
        if not self.live and pa.t0 <= tr <= pa.t1:
            cx = int(pa.px(tr, 0)[0])
            cv2.line(im, (cx, self.STRIP_Y[0]), (cx, self.STRIP_Y[-1] + self.STRIP_H), (255, 255, 255), 1)
        wh = g("pdh_wh")
        put(im, "" if wh is None else f"{wh:.1f} Wh used", 910, 202, 0.42, DIM, anchor="r")
        # ======================= right half: two rows of three 320 x 270 panels
        self._draw_mech(im[0:270, 960:1280], t)
        self._draw_swerve(im[0:270, 1280:1600], g("mod"), g("mod_des"), t)
        self._plot_overlay(im, t, ("temp",))
        self._draw_temps(im[0:270, 1600:1920], t)
        self._draw_scoring(im[270:540, 960:1280], t)
        self._draw_driver(im[270:540, 1280:1600], t)
        self._draw_bbox(im[270:540, 1600:1920], t)
        return im

    def _draw_temps(self, im, t):
        """Under the plot: each module's drive / turn motor temperature (the plot's thick lines are drive, thin are turn)."""
        td, tt = self.tel.get("temp_drive", t), self.tel.get("temp_turn", t)
        put(im, "thick = drive, thin = turn", 12, 252, 0.34, DIM)
        for m in range(4):
            lab = MOD_NAMES[m]
            if td is not None and len(td) == 4 and tt is not None and len(tt) == 4:
                lab += f" {float(td[m]):.0f}/{float(tt[m]):.0f}C"
            put(im, lab, 12 + m * 76, 266, 0.4, MOD_COL[m])

    def _draw_scoring(self, im, t):
        tr = t - self.a0
        g = lambda k, d=None: self.tel.get(k, t, d)
        # points come only from the official fuel counter in the match video; without it the score is 0
        sm = self.score
        from_video = sm is self.vshots
        put(im, "points (from video fuel counter)" if from_video else "points (needs match video: 0)", 14, 50, 0.42, DIM)
        pts = sm.points(tr) if (from_video and tr >= 0) else 0
        put(im, f"{pts}", 14, 128, 3.0, AMBER if from_video else DIM, 5, font=FD)
        k = self.shots.fired(tr) if tr >= 0 else 0
        acc = sm.accuracy(tr) if (from_video and tr >= 0) else None
        put(im, "shots fired", 14, 160, 0.45, DIM)
        put(im, f"{k}", 130, 160, 0.7, TXT, 2)
        npass = self.shots.passed(tr) if tr >= 0 else 0
        if npass:
            s_ = f"+ {npass} passes (G407)"
            put(im, s_, 175, 160, fit_scale(s_, 138, 0.38), DIM)
        put(im, "accuracy", 14, 192, 0.45, DIM)
        put(im, "--" if acc is None else f"{acc * 100:.0f}%", 130, 192, 0.7, TXT, 2)
        bar_h(im, 14, 202, 292, 12, 0 if acc is None else acc, (0, 200, 90))
        dp = g("delta")
        err = float(np.hypot(dp[0], dp[1])) if dp is not None else None
        put(im, "odometry err (Quest vs drive)", 14, 232, 0.4, DIM)
        if err is not None:
            col = (0, 200, 90) if err < 0.3 else ((0, 190, 255) if err < 1.0 else (60, 60, 230))
            put(im, f"{err:.2f} m", 306, 232, 0.5, col, 1, anchor="r")
        put(im, f"rpm {g('shooter_rpm', 0):.0f}  /  target {g('shooter_target', 0):.0f}", 14, 248, 0.4, TXT)
        foot = ("official fuel counter, only while 2429 shoots from its zone (partners incl.)" if from_video
                else "shots are counted from flywheel dips; scoring needs the match video")
        put(im, foot, 14, 264, fit_scale(foot, 296, 0.34), DIM)

    # ----------------------------------------------------------------- swerve
    def _draw_swerve(self, im, act, des, t):
        """`im` is the 320 x 270 view of this panel (local coordinates)."""
        cx, cy, half = 160, 140, 52
        put(im, "actual", 14, 44, 0.42, ACT_C)
        put(im, "desired", 74, 44, 0.42, DES_C)
        if self.hoot is not None:
            self._hoot_banner(im, 0, t)
        # the whole diagram is turned by the navX yaw, aligned to the field and seen from the driver's
        # position: up = the way the driver looks, CCW positive
        rot_deg, yaw = self.driver_rotation(t)
        psi = math.radians(rot_deg)
        cs, sn = math.cos(psi), math.sin(psi)
        rot = lambda x, y: (x * cs + y * sn, -x * sn + y * cs)      # screen coords (y down), CCW on screen
        put(im, "driver fwd", cx, cy - 88, 0.34, (90, 90, 90), anchor="c")
        cv2.line(im, (cx, cy - 84), (cx, cy - 78), (90, 90, 90), 1)
        corners = np.array([[cx + rot(sx * half, sy * half)[0], cy + rot(sx * half, sy * half)[1]]
                            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))], np.int32)
        cv2.polylines(im, [corners], True, (120, 120, 120), 2, cv2.LINE_AA)
        a0_, a1_ = rot(0, 20), rot(0, -26)
        cv2.arrowedLine(im, (int(cx + a0_[0]), int(cy + a0_[1])), (int(cx + a1_[0]), int(cy + a1_[1])),
                        (110, 110, 110), 2, cv2.LINE_AA, tipLength=0.4)
        fl = rot(0, -half - 12)
        put(im, "front", cx + fl[0], cy + fl[1] + 4, 0.36, DIM, anchor="c")
        put(im, f"rotation (driver view) {(rot_deg + 180) % 360 - 180:+.0f}°", 14, 252, 0.42, TXT)
        put(im, "navX yaw n/a" if yaw is None else f"navX yaw {yaw:+.0f}°", 14, 266, 0.36, DIM)   # short: shares the line with the wheel speed
        pos = [(-1, -1), (1, -1), (-1, 1), (1, 1)]         # LF, RF, LB, RB as (screen sx, sy)
        SCALE = 34.0 / 4.5                                  # px per m/s
        for m, (sx, sy) in enumerate(pos):
            rx_, ry_ = rot(sx * half, sy * half)
            mx, my = cx + rx_, cy + ry_
            cv2.circle(im, (int(mx), int(my)), 4, MOD_COL[m], -1, cv2.LINE_AA)
            for arr, col, th in ((des, DES_C, 3), (act, ACT_C, 2)):
                if arr is None or len(arr) <= m:
                    continue
                spd, ang = float(arr[m][0]), float(arr[m][1])
                dx, dy = rot(-math.sin(ang), -math.cos(ang))
                ex, ey = mx + dx * spd * SCALE, my + dy * spd * SCALE
                if abs(spd) > 0.03:
                    cv2.arrowedLine(im, (int(mx), int(my)), (int(ex), int(ey)), col, th, cv2.LINE_AA, tipLength=0.28)
            lx_, ly_ = rot(sx * 12, sy * 12)
            put(im, MOD_NAMES[m], mx + lx_, my + ly_ + 4, 0.42, MOD_COL[m], anchor="c")
            if self.hoot is not None:
                self._hoot_module_tag(im, m, mx, my, t, rx_, ry_)
        if act is not None and len(act) == 4:
            put(im, f"mean speed {np.mean(np.abs(act[:, 0])):.2f} m/s", 312, 266, 0.38, DIM, anchor="r")

    # ------------------------------------------------------------ CTRE hoot
    def _hoot_drive_sum(self):
        """(times after time zero, total swerve drive supply current) from the hoot, or (None, None)."""
        import hoot as H
        sigs = [self.hoot.sig(d, "SupplyCurrent") for d in self.hoot.devices if d in H.MODULE_OF]
        sigs = [x for x in sigs if x is not None and len(x)]
        if not sigs:
            return None, None
        grid = np.unique(np.concatenate([x.t for x in sigs]))
        tot = np.zeros(len(grid))
        for x in sigs:
            i = np.searchsorted(x.t, grid, side="right") - 1
            tot += np.where(i >= 0, x.numeric()[np.clip(i, 0, None)], 0.0)
        return grid, tot

    def _hoot_faults(self, dev, t, hold=1.5):
        """Short names of the bad / warning faults of a CTRE device seen in the last `hold` seconds (a fault can be
        gone in 0.2 s, which would flash by in the video)."""
        import hoot as H
        out = []
        for fk, sev in H.FAULTS.items():
            if sev not in ("bad", "warn"):
                continue
            sg = self.hoot.sig(dev, fk)
            if sg is None or not len(sg):
                continue
            i1 = sg.idx(t)
            if i1 < 0:
                continue
            i0 = max(int(np.searchsorted(sg.t, t - hold, side="right")) - 1, 0)
            if any(float(v) != 0 for v in sg.v[i0:i1 + 1]):
                out.append(FAULT_SHORT.get(fk[6:], fk[6:].upper()[:8]))
        return out

    def _hoot_module_tag(self, im, m, mx, my, t, rx, ry):
        """Under a module in the swerve diagram: its drive motor's supply current, and a red ring with the fault name while
        the device reports a fault (or 'NO DATA' when it has stopped reporting)."""
        import hoot as H
        dev = next((d for d, k in H.MODULE_OF.items() if k == m), None)
        sup = self.hoot.sig(dev, "SupplyCurrent") if dev else None
        if sup is None or not len(sup) or t < sup.t[0]:
            return
        age = sup.age(t)
        faults = self._hoot_faults(dev, t)
        n = math.hypot(rx, ry) or 1.0
        ux = rx / n                                     # beside the module, on the side away from the chassis centre
        tx, ty = mx + (34 if ux >= 0 else -34), my + 4
        if age is not None and age > 1.5:
            put(im, "NO DATA", tx, ty, 0.4, RED, 1, anchor="c")
        else:
            v = float(sup.at(t, 0.0))
            put(im, f"{v:.0f} A", tx, ty, 0.42, hue_color(v, 20, 60, True), 1, anchor="c")
        if faults:
            cv2.circle(im, (int(mx), int(my)), 13, RED, 2, cv2.LINE_AA)
            put(im, "/".join(sorted(set(faults))), tx, ty + 14, 0.4, RED, 1, anchor="c")

    def _hoot_banner(self, im, ox, t):
        """Next to the swerve panel title: what the CTRE devices are reporting right now that is wrong."""
        import hoot as H
        msgs = []
        cu = self.hoot.can_up
        if cu is not None and len(cu) and cu.idx(t) >= 0 and not cu.at(t, True):
            msgs.append("CAN BUS DOWN")
        for dev in self.hoot.devices:
            f = self._hoot_faults(dev, t)
            if f:
                msgs.append(f"{H.DEVICE_NAMES.get(dev, dev).split()[0]} {'/'.join(sorted(set(f)))}")
        if msgs:
            txt = "  ".join(msgs[:3])
            put(im, txt, ox + 312, 22, fit_scale(txt, 120, 0.45, 1), RED, 1, anchor="r")

    # -------------------------------------------------------------- mechanism
    def _mech_val(self, node, attr, t, default=None):
        s = node.attrs.get(attr)
        return default if s is None else s.at(t, default)

    def _draw_mech(self, im, t):
        """`im` is the 320 x 270 view of this panel."""
        mech = self.tel.mech
        dw, dh = mech["dims"]
        x0, y0, bw, bh = 12, 32, 296, 226
        s = min(bw / dw, bh / dh)
        pw, ph = int(dw * s), int(dh * s)
        ox, oy = x0 + (bw - pw) // 2, y0 + (bh - ph) // 2
        canvas = np.full((ph, pw, 3), hex_bgr(mech["bg"], (32, 0, 0)), np.uint8)
        to_px = lambda mx, my: (mx * s, ph - my * s)
        thick_k = s / 245.0 * 0.55

        def walk(node, sx, sy, parent_ang, parent_col, is_root):
            if is_root:
                ang_base = 0.0
                x = self._mech_val(node, "x", t, 0.0)
                y = self._mech_val(node, "y", t, 0.0)
                px, py = x, y
                col = parent_col
            else:
                ang = self._mech_val(node, "angle", t, 0.0) + parent_ang
                ln = self._mech_val(node, "length", t, 0.0)
                col = hex_bgr(self._mech_val(node, "color", t, None), parent_col) if node.attrs.get("color") else parent_col
                wt = self._mech_val(node, "weight", t, 6.0)
                a = math.radians(ang)
                px, py = sx + ln * math.cos(a), sy + ln * math.sin(a)
                if ln != 0.0 and wt > 0:
                    p0, p1 = to_px(sx, sy), to_px(px, py)
                    cv2.line(canvas, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), col,
                             max(1, int(round(wt * thick_k))), cv2.LINE_AA)
                ang_base = ang
            for ch in node.children:
                walk(ch, px, py, ang_base, col, False)

        for name, root in mech["roots"].items():
            walk(root, 0, 0, 0.0, (52, 137, 235), True)
        im[oy:oy + ph, ox:ox + pw] = canvas
        cv2.rectangle(im, (ox, oy), (ox + pw, oy + ph), EDGE, 1)
    # ------------------------------------------------------------ controllers
    def _draw_driver(self, im, t):
        """`im` is the 320 x 270 view.  The pad is drawn at 640 x 265 and shrunk to fit; the names of the pressed buttons
        are written at full size under it."""
        btn = self.tel.joy(0, "buttons", t)
        axes = self.tel.joy(0, "axes", t)
        connected = btn is not None and len(btn) > 0
        pressed = lambda i: bool(connected and i < len(btn) and btn[i])
        ax = lambda i: float(axes[i]) if axes is not None and i < len(axes) else 0.0
        put(im, "port 0 PS5" + ("" if connected else " (not connected)"), 312, 22, 0.34, DIM if connected else RED, anchor="r")
        sub = np.full((265, 640, 3), PANEL, np.uint8)
        ON, OFFC = (0, 235, 80), (70, 70, 70)

        def pill(x0, y0, x1, y1, i, label, sc=0.5):
            on = pressed(i)
            rrect(sub, x0, y0, x1, y1, ON if on else OFFC, 6)
            put_mid(sub, label, (x0 + x1) / 2, (y0 + y1) / 2, sc, BLACK if on else TXT, 1)

        # triggers (analog) + bumpers
        for (x0, i_btn, i_ax, lab) in ((20, 9, 4, "L1"), (510, 10, 5, "R1")):
            v = max(0.0, ax(i_ax))
            bar_v(sub, x0, 10, 110, 26, 0, ON)
            cv2.rectangle(sub, (x0 + 1, 11), (x0 + 1 + int(108 * v), 35), (0, 190, 255), -1)
            put_mid(sub, ("L2" if lab == "L1" else "R2") + f" {v:.2f}", x0 + 55, 23, 0.5, TXT)
            pill(x0, 40, x0 + 110, 66, i_btn, lab, 0.55)
        # d-pad
        dpx, dpy = 95, 128
        for dx, dy, i, lab in ((0, -34, 11, "U"), (0, 34, 12, "D"), (-34, 0, 13, "L"), (34, 0, 14, "R")):
            pill(dpx + dx - 15, dpy + dy - 15, dpx + dx + 15, dpy + dy + 15, i, lab, 0.5)
        # face buttons
        fx, fy = 545, 128
        for dx, dy, i, lab in ((0, -36, 3, "TRI"), (0, 36, 0, "X"), (-38, 0, 2, "SQ"), (38, 0, 1, "O")):
            on = pressed(i)
            cv2.circle(sub, (fx + dx, fy + dy), 17, ON if on else OFFC, -1, cv2.LINE_AA)
            put_mid(sub, lab, fx + dx, fy + dy, 0.45, BLACK if on else TXT, 1)
        # centre buttons
        pill(240, 82, 320, 106, 4, "CREATE", 0.45)
        pill(330, 82, 410, 106, 6, "OPTIONS", 0.42)
        pill(262, 112, 378, 148, 20, "TOUCHPAD", 0.5)
        pill(290, 156, 350, 178, 5, "PS", 0.5)
        pill(290, 184, 350, 204, 15, "MIC", 0.45)
        # sticks
        for cx, cy, ix, iy, i_btn, lab in ((185, 190, 0, 1, 7, "L3"), (455, 190, 2, 3, 8, "R3")):
            on = pressed(i_btn)
            cv2.circle(sub, (cx, cy), 48, ON if on else (90, 90, 90), 2, cv2.LINE_AA)
            cv2.line(sub, (cx - 48, cy), (cx + 48, cy), (60, 60, 60), 1)
            cv2.line(sub, (cx, cy - 48), (cx, cy + 48), (60, 60, 60), 1)
            cv2.circle(sub, (int(cx + ax(ix) * 40), int(cy + ax(iy) * 40)), 9, ON if on else (0, 190, 255), -1, cv2.LINE_AA)
            put(sub, lab, cx - 14, cy + 66, 0.5, DIM)
        im[34:34 + 127, 7:7 + 306] = cv2.resize(sub, (306, 127), interpolation=cv2.INTER_AREA)
        # what is pressed, and what it does
        actions = {"CROSS": "auto shoot", "SQUARE": "shoot 3300 rpm", "CIRCLE": "fixed-rpm feed", "TRIANGLE": "gyro reset + Quest sync",
                   "L1": "intake rpm", "R1": "track hub + spin up", "L2": "swerve X", "CREATE": "intake stop",
                   "OPTIONS": "reverse intake", "UP": "intake up", "DOWN": "intake down"}
        names = [T.PS5_BUTTONS.get(i, f"b{i}") for i in range(len(btn)) if btn[i]] if connected else []
        if ax(5) > 0.5:
            names.append("R2")
        if ax(4) > 0.5:
            names.append("L2")
        lines = [f"{n}: {actions[n]}" if n in actions else n for n in names] or ["no buttons pressed"]
        for j, ln in enumerate(lines[:5]):
            put(im, ln, 14, 186 + j * 17, fit_scale(ln, 292, 0.45), TXT if names else DIM)

    # The operator button box as it is built (photo 2026-10-25).  Buttons are numbered from 0, the index of the Driver Station
    # button on port 2; 10 and 11 are the two ends of a three-position toggle that adjusts the shooting power: 10 = -50, 11 = +50, middle = no
    # adjustment (the robot's own bindings put the offset back to 0 when the toggle is released).
    BBOX_LABELS = ["ZEROINT", "147°", "AFTERBURNER", "40A", "QUESTOFF", "QUEST ON", "UNSYNC", "STOW", "SHOOT", "INTAKE", "-50", "+50"]
    BBOX_RED = (2, 4, 6)                                    # the red arcade buttons; every other one is blue
    BBOX_XY = {0: (61, 148), 1: (127, 66), 2: (127, 121), 3: (127, 176), 4: (199, 66), 5: (199, 121), 6: (199, 176),
               7: (266, 66), 8: (266, 121), 9: (266, 176)}   # button centres in the 320 x 270 panel, same arrangement as the box
    BBOX_BLUE_C, BBOX_RED_C = ((150, 60, 10), (230, 110, 30), (255, 190, 110)), ((30, 30, 140), (60, 60, 230), (140, 140, 255))   # BGR: edge, body, lit

    def _bbox_button(self, im, i, on):
        cx, cy = self.BBOX_XY[i]
        edge, body, lit = self.BBOX_RED_C if i in self.BBOX_RED else self.BBOX_BLUE_C
        r = 17
        if on:                                              # pressed: a glow, a bright body and a white ring
            cv2.circle(im, (cx, cy), r + 6, tuple(int(c * 0.55) for c in body), -1, cv2.LINE_AA)
        cv2.circle(im, (cx, cy), r + 3, (25, 25, 25), -1, cv2.LINE_AA)                          # bezel
        cv2.circle(im, (cx, cy), r, lit if on else edge, -1, cv2.LINE_AA)
        cv2.circle(im, (cx, cy), r - 4, lit if on else body, -1, cv2.LINE_AA)
        cv2.circle(im, (cx, cy), r + 1, WHITE if on else (60, 60, 60), 2 if on else 1, cv2.LINE_AA)
        put_mid(im, str(i), cx, cy, 0.42, BLACK if on else WHITE, 1)
        label = self.BBOX_LABELS[i]                         # white tape under the button, as on the box
        w = 60
        ty = cy + r + 5
        cv2.rectangle(im, (cx - w // 2, ty), (cx + w // 2, ty + 12), (235, 235, 235), -1)
        put_mid(im, label, cx, ty + 6, fit_scale(label, w - 4, 0.34), BLACK, 1)

    def _bbox_toggle(self, im, pos):
        """The three-position toggle: pos is 'minus' (button 10), 'mid' (neither) or 'plus' (button 11)."""
        cx, cy = 61, 78
        rrect(im, cx - 31, cy - 17, cx + 31, cy + 17, (45, 45, 45), 5)
        cv2.line(im, (cx - 20, cy), (cx + 20, cy), (15, 15, 15), 5, cv2.LINE_AA)               # the slot
        kx = {"minus": cx - 19, "mid": cx, "plus": cx + 19}[pos]
        cv2.line(im, (cx, cy + 10), (kx, cy), (170, 170, 170), 3, cv2.LINE_AA)                   # the lever
        cv2.circle(im, (kx, cy), 8, (60, 60, 225) if pos == "mid" else (80, 235, 100), -1, cv2.LINE_AA)
        cv2.circle(im, (kx, cy), 8, WHITE, 1, cv2.LINE_AA)
        put_mid(im, "10", cx - 19, cy - 27, 0.36, DIM)
        put_mid(im, "11", cx + 19, cy - 27, 0.36, DIM)
        for name, word, x in (("minus", "-50", cx - 22), ("plus", "+50", cx + 22)):                  # no caption for the middle
            put_mid(im, word, x, cy + 28, 0.28, LIME if pos == name else DIM)

    def _draw_bbox(self, im, t):
        """`im` is the 320 x 270 view of this panel."""
        btn = self.tel.joy(2, "buttons", t)
        axes = self.tel.joy(2, "axes", t)
        connected = btn is not None and len(btn) > 0
        put(im, "port 2" + ("" if connected else " (not connected)"), 312, 22, 0.34, DIM if connected else RED, anchor="r")
        pressed = lambda i: bool(connected and i < len(btn) and btn[i])
        rrect(im, 8, 30, 312, 222, (48, 46, 44), 10)                                              # the clear plate
        rrect(im, 8, 30, 312, 222, EDGE, 10, 1)
        for sx, sy in ((17, 39), (303, 39), (17, 213), (303, 213)):                               # its four screws
            cv2.circle(im, (sx, sy), 4, (150, 150, 150), -1, cv2.LINE_AA)
            cv2.line(im, (sx - 2, sy - 2), (sx + 2, sy + 2), (60, 60, 60), 1)
        for i in self.BBOX_XY:
            self._bbox_button(im, i, pressed(i))
        self._bbox_toggle(im, "minus" if pressed(10) else "plus" if pressed(11) else "mid")
        if axes is not None and any(abs(float(v)) > 0.05 for v in axes[:5]):                       # the box has no analog axes; shown only if one moves
            put(im, "axes " + " ".join(f"{float(v):+.2f}" for v in axes[:5]), 12, 238, 0.34, AMBER)
        b2 = self.tel.joy(3, "buttons", t)
        cp = self.tel.joy(1, "buttons", t)
        put(im, f"bbox 2 (port 3): {'connected' if b2 is not None and len(b2) else 'not connected'}", 12, 250, 0.36, DIM)
        put(im, f"copilot (port 1): {'connected' if cp is not None and len(cp) else 'not connected'}", 12, 264, 0.36, DIM)

    # -------------------------------------------------------------- compose
    def render(self, t, video=None, trends=True):
        """Full 1920x1080 frame.  The top-right slot shows `video` (a BGR frame) if given; otherwise the trend
        plots, or black when trends=False (ffmpeg overlays the match video there)."""
        if self.live:
            self.update_live(t)
        c = np.zeros((H, W, 3), np.uint8)
        c[:PH, :PW] = self.render_left(t)
        c[PH:, :] = self.render_lower(t)
        if video is not None:
            c[:PH, PW:] = cv2.resize(video, (PW, PH), interpolation=cv2.INTER_AREA)
        elif trends:
            c[:PH, PW:] = self.trend_panel.render(t, live=self.live)
        return c
