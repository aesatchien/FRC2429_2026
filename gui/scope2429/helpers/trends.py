"""
The top-right 960x540 slot of the dashboard when there is no match video: scrolling trend plots.

By default four panes (shooter rpm, feeder rpm, wheel speed, Quest-vs-drive error).  If the user picked entries
in the GUI, those are drawn instead, one pane each (an array entry gets one line per element).  The same code
draws a log being replayed and a live connection: the window is always the last WINDOW seconds up to "now".
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from dash import BG, CYAN, DIM, EDGE, PANEL, PH, PW, TXT, Plot, fit_scale, put

WINDOW = 30.0
COLORS = [(255, 200, 40), (0, 140, 255), (80, 220, 60), (230, 90, 230), (60, 220, 255), (235, 235, 235)]
MAX_PANES = 6

# (title, [(kind, ref, label)])   kind: key = Telemetry.by_key[ref], wheel / odo = derived, name = raw entry
DEFAULT_PANES = [
    ("shooter rpm", [("key", "shooter_rpm", "measured"), ("key", "shooter_target", "target")]),
    ("intake / hopper / indexer rpm", [("key", "intake_rpm", "intake"), ("key", "hopper_rpm", "hopper"),
                                       ("key", "indexer_rpm", "indexer")]),
    ("wheel speed (mean of 4 modules, m/s)", [("wheel", "mod", "actual"), ("wheel", "mod_des", "desired")]),
    ("Quest vs drive pose error (m)", [("odo", "delta", "error")]),
]


def nice_ticks(lo, hi, n=3):
    span = hi - lo
    raw = span / n
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw)
    first = math.ceil(lo / step) * step
    return [round(first + i * step, 10) for i in range(int((hi - first) / step) + 1)]


class TrendPanel:
    def __init__(self, tel, ml, extra=()):
        self.tel, self.ml = tel, ml
        self.extra = list(extra)[:MAX_PANES]

    # ----------------------------------------------------------------- data
    def _window(self, sig, t):
        """(seconds relative to t, list of values) for the last WINDOW seconds, including the value held from before"""
        st = sig.t
        i1 = int(np.searchsorted(st, t, side="right"))
        i0 = max(0, int(np.searchsorted(st, t - WINDOW, side="right")) - 1)
        if i1 - i0 < 1:
            return None
        return st[i0:i1] - t, sig.v[i0:i1]

    def _lines(self, kind, ref, label, t):
        """[(label, rel_t, y)] for one line spec (several for an array entry)."""
        tel = self.tel
        sig = tel.by_key.get(ref) if kind in ("key", "wheel", "odo") else tel.sig.get(ref)
        if sig is None or len(sig) == 0:
            return []
        w = self._window(sig, t)
        if w is None:
            return []
        ts, vs = w
        try:
            if kind == "wheel":
                y = np.array([np.mean(np.abs(v[:, 0])) if len(v) else np.nan for v in vs], float)
            elif kind == "odo":
                y = np.array([math.hypot(v[0], v[1]) for v in vs], float)
            else:
                first = vs[-1]
                if isinstance(first, np.ndarray) and first.ndim == 1:
                    k = len(first)
                    arr = np.array([v if len(v) == k else np.full(k, np.nan) for v in vs], float)
                    return [(f"{label}[{i}]", ts, arr[:, i]) for i in range(min(k, 6))]
                y = np.array(vs, float)
        except (ValueError, TypeError, IndexError):
            return []
        return [(label, ts, y)]

    def _panes(self):
        if not self.extra:
            return DEFAULT_PANES
        return [(n.rsplit("/", 1)[-1], [("name", n, n.rsplit("/", 1)[-1])]) for n in self.extra]

    # ----------------------------------------------------------------- draw
    def render(self, t, live=False):
        im = np.full((PH, PW, 3), BG, np.uint8)
        panes = self._panes()
        head = f"TRENDS  -  last {WINDOW:.0f} s"
        put(im, head, 10, 20, 0.55, CYAN)
        if live:
            put(im, "LIVE", PW - 12, 20, 0.55, (60, 60, 230), 2, anchor="r")
        else:
            put(im, f"t = {t - self.tel.a0:+.1f} s", PW - 12, 20, 0.5, TXT, 1, anchor="r")
        top, avail = 30, PH - 34
        ph = avail / len(panes)
        for i, (title, specs) in enumerate(panes):
            y0 = int(top + i * ph)
            h = int(ph) - 6
            cv2.rectangle(im, (2, y0), (PW - 3, y0 + h), PANEL, -1)
            cv2.rectangle(im, (2, y0), (PW - 3, y0 + h), EDGE, 1)
            put(im, title, 10, y0 + 14, fit_scale(title, 300, 0.45), CYAN)
            data = []
            for kind, ref, label in specs:
                data += self._lines(kind, ref, label, t)
            plot_x, plot_w = 66, 700
            if not data:
                put(im, "no data", plot_x + 10, y0 + h // 2 + 4, 0.45, DIM)
                continue
            allv = np.concatenate([d[2][np.isfinite(d[2])] for d in data]) if data else np.zeros(0)
            if len(allv) == 0:
                continue
            lo, hi = float(allv.min()), float(allv.max())
            if hi - lo < 1e-9:
                lo, hi = lo - 1.0, hi + 1.0
            pad = (hi - lo) * 0.1
            lo, hi = lo - pad, hi + pad
            plot = Plot((plot_x, y0 + 20, plot_w, h - 36), -WINDOW, 0, lo, hi, nice_ticks(lo, hi), "{:g}",
                        xticks=[(-20, "-20s"), (-10, "-10s")])
            plot.frame(im)
            for j, (label, ts, y) in enumerate(data):
                plot.line(im, ts, y, COLORS[j % len(COLORS)], 2)
            # read-out of the current values at the right
            for j, (label, ts, y) in enumerate(data[:8]):
                col = COLORS[j % len(COLORS)]
                yy = y0 + 34 + j * 18
                if yy > y0 + h - 4:
                    break
                cv2.line(im, (plot_x + plot_w + 14, yy - 4), (plot_x + plot_w + 30, yy - 4), col, 3)
                v = y[-1]
                put(im, f"{label} {v:.1f}" if np.isfinite(v) else f"{label} --", plot_x + plot_w + 36, yy,
                    fit_scale(f"{label} 00000.0", 150, 0.4), TXT)
        return im
