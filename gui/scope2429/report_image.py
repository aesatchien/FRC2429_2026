"""
One-page picture of a match report (PNG), made to be dragged into Slack or an email: a header with the overall
verdict, the things that need attention (problems red, warnings orange), and a tile per headline number for
health, swerve, odometry and cycles.  The full list of findings and the tables are in the HTML report.

    cv2.imwrite("report.png", render_image(report))
"""
from __future__ import annotations

import time

import cv2
import numpy as np

W, M = 1600, 28
BG, CARD, EDGE = (24, 24, 26), (38, 38, 42), (78, 78, 84)
TXT, DIM = (240, 240, 240), (160, 160, 166)
SEV = {"bad": (75, 75, 238), "warn": (35, 160, 245), "ok": (95, 195, 85), "info": (205, 175, 120)}
SEV_WORD = {"bad": "PROBLEM", "warn": "WARNING", "ok": "OK", "info": "INFO"}
F, FD = cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_DUPLEX
MAX_ROWS = 10


def _ascii(s):
    for a, b in (("·", "-"), ("°", " deg"), ("±", "+/-"), ("–", "-"), ("—", "-"), ("…", "...")):
        s = s.replace(a, b)
    return s.encode("ascii", "replace").decode()


def _tint(color, k, base=CARD):
    return tuple(int(base[i] * (1 - k) + color[i] * k) for i in range(3))


def _rr(img, x0, y0, x1, y1, color, r=10, th=-1):
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    r = max(0, min(r, (x1 - x0) // 2, (y1 - y0) // 2))
    if th < 0:
        cv2.rectangle(img, (x0 + r, y0), (x1 - r, y1), color, -1)
        cv2.rectangle(img, (x0, y0 + r), (x1, y1 - r), color, -1)
        for cx, cy in ((x0 + r, y0 + r), (x1 - r, y0 + r), (x0 + r, y1 - r), (x1 - r, y1 - r)):
            cv2.circle(img, (cx, cy), r, color, -1, cv2.LINE_AA)
    else:
        cv2.rectangle(img, (x0 + r, y0), (x1 - r, y1), color, th, cv2.LINE_AA)
        cv2.rectangle(img, (x0, y0 + r), (x1, y1 - r), color, th, cv2.LINE_AA)


def _text(img, s, x, y, scale=0.6, color=TXT, th=1, font=F, anchor="l"):
    s = _ascii(s)
    w = cv2.getTextSize(s, font, scale, th)[0][0]
    if anchor == "r":
        x -= w
    elif anchor == "c":
        x -= w // 2
    cv2.putText(img, s, (int(x), int(y)), font, scale, color, th, cv2.LINE_AA)
    return w


def _fit(s, width, scale, th=1, font=F):
    w = cv2.getTextSize(_ascii(s), font, scale, th)[0][0]
    return scale if w <= width else scale * width / w


def _wrap(s, width, scale, th=1, font=F):
    lines, cur = [], ""
    for word in _ascii(s).split(" "):
        t = (cur + " " + word).strip()
        if cv2.getTextSize(t, font, scale, th)[0][0] <= width or not cur:
            cur = t
        else:
            lines.append(cur)
            cur = word
    lines.append(cur)
    return lines


def render_image(report) -> np.ndarray:
    """BGR picture, W pixels wide, as tall as the content needs."""
    img = np.full((3400, W, 3), BG, np.uint8)
    ov = report.overview()
    n_bad = sum(1 for _, i in ov if i.sev == "bad")
    n_warn = sum(1 for _, i in ov if i.sev == "warn")
    overall = "bad" if n_bad else "warn" if n_warn else "ok"
    # ---------------------------------------------------------------- header
    cv2.rectangle(img, (0, 0), (W, 10), SEV[overall], -1)
    _text(img, report.title, M, 70, 1.55, TXT, 2, FD)
    _text(img, f"Scope2429 match report  -  {time.strftime('%Y-%m-%d %H:%M')}  -  times are seconds after auto start "
               f"(or the first enable)", M, 106, 0.6, DIM)
    badge = "ALL CLEAR" if overall == "ok" else (f"{n_bad} PROBLEM" + ("S" if n_bad != 1 else "") if n_bad else
                                                  f"{n_warn} WARNING" + ("S" if n_warn != 1 else ""))
    if n_bad and n_warn:
        badge += f" + {n_warn} WARN"
    bw = max(330, cv2.getTextSize(badge, FD, 1.15, 2)[0][0] + 60)
    _rr(img, W - M - bw, 30, W - M, 96, SEV[overall], 14)
    _text(img, badge, W - M - bw // 2, 76, 1.15, (255, 255, 255) if overall != "ok" else (20, 40, 20), 2, FD, "c")
    y = 134
    # ---------------------------------------------------------------- needs attention
    _text(img, "NEEDS ATTENTION" if ov else "NOTHING NEEDS ATTENTION", M, y + 22, 0.8, SEV["bad" if n_bad else "warn" if ov else "ok"], 2, FD)
    y += 38
    if not ov:
        _rr(img, M, y, W - M, y + 54, _tint(SEV["ok"], 0.2), 10)
        _text(img, "No problems or warnings in the logged data.", M + 24, y + 36, 0.75, SEV["ok"], 2)
        y += 66
    msg_x = M + 232
    msg_w = W - M - msg_x - 170
    for sec, it in ov[:MAX_ROWS]:
        lines = _wrap(it.msg, msg_w, 0.66)
        h = 20 + 25 * len(lines)
        col = SEV[it.sev]
        _rr(img, M, y, W - M, y + h, _tint(col, 0.16), 10)
        cv2.rectangle(img, (M, y + 4), (M + 8, y + h - 4), col, -1)
        _rr(img, M + 22, y + 10, M + 112, y + 34, col, 8)
        _text(img, SEV_WORD[it.sev], M + 67, y + 28, 0.55, (255, 255, 255), 2, F, "c")
        _text(img, "" if it.t is None else f"{it.t:+.0f} s", M + 205, y + 28, 0.62, DIM, 1, F, "r")
        for k, ln in enumerate(lines):
            _text(img, ln, msg_x, y + 30 + 25 * k, 0.66, TXT, 1)
        _text(img, sec.split(":")[0].split(" (")[0], W - M - 14, y + 28, 0.5, DIM, 1, F, "r")
        y += h + 8
    if len(ov) > MAX_ROWS:
        _text(img, f"+ {len(ov) - MAX_ROWS} more warning(s) in the HTML report", M + 8, y + 22, 0.62, DIM)
        y += 36
    y += 14
    # ---------------------------------------------------------------- panels
    pw = (W - 3 * M) // 2
    secs = [x for x in report.sections if x.kpis]               # the sections that have headline numbers
    for i in range(0, len(secs), 2):
        pair = secs[i:i + 2]
        heights = [_panel_height(s, pw) for s in pair]
        for j, s in enumerate(pair):
            _panel(img, s, M + j * (pw + M), y, pw, max(heights))
        y += max(heights) + M
    # ---------------------------------------------------------------- footer
    note = next((x.note for x in report.sections if x.note.startswith("Not in this log")), "")
    if note:
        for ln in _wrap(note, W - 2 * M, 0.55):
            _text(img, ln, M, y + 14, 0.55, DIM)
            y += 22
    _text(img, "Measured from the entries in the log. The HTML report has every finding and table.", M, y + 18, 0.55, DIM)
    return img[:y + 36].copy()


def _tiles(s):
    return s.kpis or [("data", "not in this log", "info")]


def _panel_height(s, pw):
    rows = -(-len(_tiles(s)) // 3)
    return 62 + rows * 96 + 10


def _panel(img, s, x, y, pw, h):
    worst = s.worst() if s.items else "ok"
    worst = "ok" if worst == "info" else worst
    col = SEV[worst]
    _rr(img, x, y, x + pw, y + h, CARD, 14)
    cv2.rectangle(img, (x + 14, y), (x + pw - 14, y + 5), col, -1)
    name = s.title.split(":")[0].split(" (")[0]
    _text(img, name.upper(), x + 20, y + 40, 0.95, TXT, 2, FD)
    nb = sum(1 for it in s.items if it.sev == "bad")
    nw = sum(1 for it in s.items if it.sev == "warn")
    parts = ([f"{nb} PROBLEM" + ("S" if nb != 1 else "")] if nb else []) + ([f"{nw} WARNING" + ("S" if nw != 1 else "")] if nw else [])
    _text(img, ", ".join(parts) if parts else "ALL OK", x + pw - 20, y + 38, 0.8, col, 2, FD, "r")
    tiles = _tiles(s)
    tw = (pw - 2 * 20 - 2 * 12) // 3
    for k, (label, value, sev) in enumerate(tiles):
        r_, c_ = divmod(k, 3)
        tx, ty = x + 20 + c_ * (tw + 12), y + 62 + r_ * 96
        sc = SEV[sev]
        _rr(img, tx, ty, tx + tw, ty + 86, sc if sev != "info" else EDGE, 10)            # border = a slightly larger fill
        _rr(img, tx + 2, ty + 2, tx + tw - 2, ty + 84, _tint(sc, 0.22 if sev != "info" else 0.10), 8)
        _text(img, label, tx + 12, ty + 25, _fit(label, tw - 24, 0.55), DIM, 1)
        vcol = TXT if sev == "info" else sc
        _text(img, value, tx + 12, ty + 68, _fit(value, tw - 24, 1.15, 2, FD), vcol, 2, FD)
