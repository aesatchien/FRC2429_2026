"""
Match report: what a log says about the robot's health, the swerve modules, QuestNav odometry and the cycle
times.  Everything is measured from the logged entries; nothing is guessed.  An item is only reported when the
entry it needs is in the log (CAN utilization and loop times are not logged by this robot, and the report says so).

    rep = build_report(tel, ml, thresholds)      # tel: telemetry.Telemetry of the log, ml: core.MatchLog
    print(rep.to_text())

Times are seconds after time zero (auto start, else the first enable), the same axis as the preview slider.
"""
from __future__ import annotations

import os

from dataclasses import dataclass, field

import html
import time

import numpy as np

import core
import telemetry as T

SEV_ORDER = {"bad": 0, "warn": 1, "info": 2, "ok": 3}


@dataclass
class Item:
    t: float | None          # seconds after time zero (None = whole match)
    sev: str                 # bad | warn | info | ok
    msg: str
    event: bool = False      # a discrete event (a fault, a jump, a dropout), as opposed to a statistic: see _correlate
    grouped: bool = False    # already part of an "events that line up" finding, so the overview does not list it twice


@dataclass
class Section:
    title: str
    items: list = field(default_factory=list)
    columns: list = field(default_factory=list)
    rows: list = field(default_factory=list)
    note: str = ""
    kpis: list = field(default_factory=list)       # (label, value text, sev): the headline numbers of the one-page image

    def add(self, sev, msg, t=None, event=False):
        self.items.append(Item(t, sev, msg, event))

    def kpi(self, label, value, sev="info"):
        self.kpis.append((label, value, sev))

    def worst(self):
        """the most severe level among this section's items"""
        return min((i.sev for i in self.items), key=lambda x: SEV_ORDER[x], default="ok")


@dataclass
class Report:
    title: str
    window: tuple                       # (start, end) log time of the part of the log that is analysed
    sections: list = field(default_factory=list)

    def overview(self):
        """The warnings and problems of every section, worst first."""
        out = []
        for s in self.sections:
            out += [(s.title, it) for it in s.items if it.sev in ("bad", "warn") and not it.grouped]
        out.sort(key=lambda x: (SEV_ORDER[x[1].sev], x[1].t if x[1].t is not None else 1e9))
        return out

    def to_html(self):
        """The whole report as one self-contained, colorized HTML page (light and dark mode, no external files)."""
        ov = self.overview()
        n_bad = sum(1 for _, i in ov if i.sev == "bad")
        n_warn = sum(1 for _, i in ov if i.sev == "warn")
        e = html.escape
        h = ['<!doctype html><html lang="en"><head><meta charset="utf-8">',
             '<meta name="viewport" content="width=device-width, initial-scale=1">',
             f"<title>{e(self.title)}</title><style>{_CSS}</style></head><body><main>",
             f"<h1>{e(self.title)}</h1>",
             f'<div class="sub">Scope2429 match report, made {time.strftime("%Y-%m-%d %H:%M")}. Times are seconds after '
             f"auto start (or the first enable).</div>",
             '<div class="chips">' +
             (f'<span class="chip-bad">{n_bad} problem(s)</span>' if n_bad else "") +
             (f'<span class="chip-warn">{n_warn} warning(s)</span>' if n_warn else "") +
             ('<span class="chip-ok">no problems or warnings</span>' if not ov else "") + "</div>",
             "<nav>" + "".join(f'<a href="#{_anchor(s.title)}">{e(_short(s.title))}</a>' for s in self.sections) + "</nav>",
             "<h2>Problems and warnings</h2>"]
        h.append(_html_items(ov[:60], with_section=True) if ov else '<p class="note">Nothing to report.</p>')
        if len(ov) > 60:
            h.append(f'<p class="note">... and {len(ov) - 60} more, listed under their sections.</p>')
        for s in self.sections:
            h.append(f'<h2 id="{_anchor(s.title)}">{e(s.title)}</h2>')
            h.append(_html_items([(None, it) for it in s.items]))
            if s.rows:
                h.append("<table><thead><tr>" + "".join(f"<th>{e(str(c))}</th>" for c in s.columns) + "</tr></thead><tbody>")
                for r in s.rows:
                    h.append("<tr>" + "".join(f"<td>{e(str(x))}</td>" for x in r) + "</tr>")
                h.append("</tbody></table>")
            if s.note:
                h.append(f'<p class="note">{e(s.note)}</p>')
        h.append("<footer>Everything above is measured from the entries in the log; a check that needs an entry the log "
                 "does not have is skipped and says so.</footer></main></body></html>")
        return "\n".join(h)

    def to_text(self):
        lines = [self.title, "=" * len(self.title), ""]
        ov = self.overview()
        lines.append("Problems and warnings" if ov else "No problems or warnings found in the logged data")
        for sec, it in ov[:30]:
            lines.append(f"  [{it.sev.upper():4s}] {sec}: {it.msg}" + ("" if it.t is None else f"  (t={it.t:+.1f}s)"))
        if len(ov) > 30:
            lines.append(f"  ... and {len(ov) - 30} more, listed under their sections")
        for s in self.sections:
            lines += ["", s.title, "-" * len(s.title)]
            for it in s.items:
                lines.append(f"  [{it.sev.upper():4s}] {it.msg}" + ("" if it.t is None else f"  (t={it.t:+.1f}s)"))
            if s.rows:
                lines.append("")
                widths = [max(len(str(x)) for x in [c] + [r[i] for r in s.rows]) for i, c in enumerate(s.columns)]
                lines.append("  " + "  ".join(str(c).ljust(w) for c, w in zip(s.columns, widths)))
                for r in s.rows:
                    lines.append("  " + "  ".join(str(x).ljust(w) for x, w in zip(r, widths)))
            if s.note:
                lines += ["", "  " + s.note]
        return "\n".join(lines) + "\n"


_CSS = """
:root { --bg:#ffffff; --fg:#1d2127; --dim:#667; --line:#d9dde3; --card:#f6f8fa;
        --bad:#b00020; --warn:#a85a00; --ok:#1b7a34; --badbg:#fdecee; --warnbg:#fff4e0; --okbg:#e9f6ec; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#14171b; --fg:#e4e7eb; --dim:#98a1ad; --line:#2c323a; --card:#1c2127;
          --bad:#ff6b7d; --warn:#ffb454; --ok:#5fd07a; --badbg:#3a1a20; --warnbg:#3a2a12; --okbg:#173022; } }
body { background:var(--bg); color:var(--fg); font:15px/1.45 system-ui,Segoe UI,Roboto,sans-serif; margin:0; }
main { max-width:1000px; margin:0 auto; padding:20px 16px 48px; }
h1 { margin:0 0 4px; font-size:26px; } h2 { margin:30px 0 8px; font-size:19px; border-bottom:1px solid var(--line); padding-bottom:4px; }
.sub { color:var(--dim); margin-bottom:14px; }
.chips span { display:inline-block; margin:0 8px 8px 0; padding:3px 12px; border-radius:14px; font-weight:600; }
.chip-bad { background:var(--badbg); color:var(--bad); } .chip-warn { background:var(--warnbg); color:var(--warn); }
.chip-ok { background:var(--okbg); color:var(--ok); }
ul.items { list-style:none; margin:0; padding:0; }
ul.items li { display:flex; gap:10px; padding:5px 8px; border-radius:6px; margin:2px 0; }
ul.items li.bad { background:var(--badbg); } ul.items li.warn { background:var(--warnbg); } ul.items li.ok { background:var(--okbg); }
.lvl { flex:0 0 74px; font-weight:700; font-size:12px; text-transform:uppercase; padding-top:2px; }
.bad .lvl, .bad .msg { color:var(--bad); } .warn .lvl, .warn .msg { color:var(--warn); } .ok .lvl, .ok .msg { color:var(--ok); }
.t { flex:0 0 70px; text-align:right; color:var(--dim); font-variant-numeric:tabular-nums; }
.sec { color:var(--dim); font-size:12px; display:block; }
table { border-collapse:collapse; margin:12px 0; width:100%; font-variant-numeric:tabular-nums; }
th, td { border:1px solid var(--line); padding:4px 10px; text-align:right; } th { background:var(--card); }
td:first-child, th:first-child { text-align:left; }
.note { color:var(--dim); font-size:13px; margin:6px 0; }
nav a { margin-right:14px; color:var(--dim); } footer { color:var(--dim); font-size:12px; margin-top:34px; }
"""


def _anchor(title):
    return "".join(c.lower() if c.isalnum() else "-" for c in title.split(":")[0].split(" (")[0]).strip("-")


def _short(title):
    return title.split(":")[0].split(" (")[0]


def _html_items(items, with_section=False):
    out = ['<ul class="items">']
    for sec, it in items:
        t = "" if it.t is None else f"{it.t:+.1f} s"
        label = {"bad": "Problem", "warn": "Warning", "info": "Info", "ok": "OK"}[it.sev]
        msg = html.escape(it.msg) + (f'<span class="sec">{html.escape(_short(sec))}</span>' if with_section and sec else "")
        out.append(f'<li class="{it.sev}"><span class="t">{t}</span><span class="lvl">{label}</span>'
                   f'<span class="msg">{msg}</span></li>')
    out.append("</ul>")
    return "\n".join(out)


# ------------------------------------------------------------------------------------------ helpers
def _window(ml):
    """Log times of the part to analyse: the match (auto start .. match end), else time zero .. end of the log."""
    a = ml.t_ref
    b = a + ml.match_len if ml.has_match else ml.t_last
    return a, max(b, a + 1.0)


def _held(sig, a, b):
    """(times, values) of a Sig inside [a, b], starting with the value held at a, ending at b (for time weighting)."""
    if sig is None or not len(sig):
        return None
    t = sig.t
    i0 = max(int(np.searchsorted(t, a, side="right")) - 1, 0)
    i1 = int(np.searchsorted(t, b, side="right"))
    tt = np.clip(t[i0:i1], a, b)
    vv = sig.v[i0:i1]
    if len(tt) == 0:
        return None
    return np.append(tt, b), vv


def _num(sig, a, b):
    h = _held(sig, a, b)
    if h is None:
        return None
    t, v = h
    try:
        return t, np.asarray(v, float)
    except (ValueError, TypeError):
        return None


def _intervals(sig, a, b, truthy=True):
    """[(start, end)] where the boolean Sig is true (or false) inside [a, b]."""
    h = _held(sig, a, b)
    if h is None:
        return []
    t, v = h
    out, start = [], None
    for i, x in enumerate(v):
        on = bool(x) == truthy
        if on and start is None:
            start = t[i]
        elif not on and start is not None:
            out.append((start, t[i]))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def _rises(sig, a, b):
    """times at which a counter / boolean Sig went up inside [a, b]"""
    h = _held(sig, a, b)
    if h is None:
        return []
    t, v = h
    out = []
    for i in range(1, len(v)):
        try:
            if float(v[i]) > float(v[i - 1]):
                out.append(float(t[i]))
        except (TypeError, ValueError):
            pass
    return out


def _tw_mean(t, v):
    d = np.diff(t)
    return float((v * d).sum() / d.sum()) if d.sum() > 0 else float(v[0])


# ------------------------------------------------------------------------------------------ health
def _health(tel, ml, win, rel, hoot=None):
    s = Section("Health: battery, brownouts, current, cameras")
    a, b = win
    v = tel.by_key.get("pdh_v")
    nv = _num(v, a, b)
    if nv is not None:
        t, x = nv
        rest = v.at(a - 1.0)
        i = int(np.argmin(x))
        vmin = float(x[i])
        d = np.diff(t)                      # t has one more entry than x: the end of the window
        below8 = float(d[x < 8.0].sum())
        below7 = float(d[x < 7.0].sum())
        sev = "bad" if vmin < 7.0 else "warn" if vmin < 8.0 else "info"
        s.add(sev, f"battery minimum {vmin:.2f} V" + (f", sag of {rest - vmin:.1f} V from {rest:.1f} V before the match"
                                                      if rest is not None else "") +
              f"; {below8:.1f} s below 8 V, {below7:.1f} s below 7 V", rel(t[i]))
    else:
        s.add("info", "battery voltage (_pdh_voltage) is not in this log")
    if nv is not None:
        s.kpi("Battery min", f"{vmin:.1f} V", "bad" if vmin < 7.0 else "warn" if vmin < 8.0 else "ok")
    br = tel.by_key.get("rio_brownout")
    if br is not None and len(br):
        ivs = _intervals(br, ml.t_first, ml.t_last)
        for x0, x1 in ivs:
            s.add("bad", f"roboRIO brownout for {x1 - x0:.2f} s", rel(x0), event=True)
        if not ivs:
            s.add("ok", "no roboRIO brownout flagged")
    else:
        s.add("info", "roboRIO brownout flag (_rio_browned_out) is not in this log")
    n_dev = 0
    if hoot is not None:                    # the CTRE devices' own brownout faults (the roboRIO flag can stay quiet)
        for dev in hoot.devices:
            fs = hoot.sig(dev, "Fault_BridgeBrownout")
            if fs is not None and len(fs):
                n_dev += len(_intervals(fs, a, b))
    if br is not None and len(br):
        n_rio = len(ivs)
        s.kpi("Brownouts", ("RIO " + (str(n_rio) if n_rio else "none") + (f", drives {n_dev}" if hoot is not None else "")),
              "bad" if n_rio else "warn" if n_dev else "ok")
    bm = tel.by_key.get("brownout")
    if bm is not None and len(bm):
        for x0, x1 in _intervals(bm, a, b):
            s.add("warn", f"brownout mode on for {x1 - x0:.1f} s", rel(x0), event=True)
    na = _num(tel.by_key.get("pdh_a"), a, b)
    if na is not None:
        t, x = na
        i = int(np.argmax(x))
        s.add("info", f"peak total current {float(x.max()):.0f} A, average {_tw_mean(t, x):.0f} A", rel(t[i]))
    if na is not None:
        s.kpi("Peak current", f"{float(na[1].max()):.0f} A")
    # per mechanism current
    cols = ["Motor", "Peak A", "at t", "Average A"]
    pairs = [("Flywheel L", "cur_fly_l"), ("Flywheel R", "cur_fly_r"), ("Roller", "cur_roller"), ("Hopper", "cur_hop"),
             ("Indexer L", "cur_idx_l"), ("Indexer R", "cur_idx_r"), ("Intake L", "cur_int_l"), ("Intake R", "cur_int_r"),
             ("Intake deploy", "cur_deploy")]
    means = {}
    s.columns = cols
    for label, key in pairs:
        n = _num(tel.by_key.get(key), a, b)
        if n is None:
            continue
        t, x = n
        i = int(np.argmax(x))
        means[label] = _tw_mean(t, x)
        s.rows.append([label, f"{x.max():.0f}", f"{rel(t[i]):+.1f} s", f"{means[label]:.1f}"])
    for l1, l2 in (("Flywheel L", "Flywheel R"), ("Indexer L", "Indexer R"), ("Intake L", "Intake R")):
        if l1 in means and l2 in means and max(means[l1], means[l2]) > 2.0:
            hi, lo = max(means[l1], means[l2]), max(min(means[l1], means[l2]), 0.1)
            if hi / lo > 1.5:
                big = l1 if means[l1] > means[l2] else l2
                s.add("warn", f"{l1} / {l2} currents differ: {big} draws {hi / lo:.1f}x the other on average "
                              f"(binding, a failing motor, a loose wire, or the two are not measured the same way)")
    # camera dropouts
    for label, key in (("ArduCam right", "cam_right_frames"), ("ArduCam back", "cam_back_frames")):
        sg = tel.by_key.get(key)
        if sg is None or not len(sg):
            continue
        t = np.concatenate([[a], np.clip(sg.t[(sg.t > a) & (sg.t < b)], a, b), [b]])
        gaps = [(t[i], t[i + 1] - t[i]) for i in range(len(t) - 1) if t[i + 1] - t[i] > 2.5]
        if gaps:
            s.add("warn", f"{label}: no frames for more than 2.5 s, {len(gaps)} time(s), {sum(g for _, g in gaps):.0f} s in all "
                          f"(longest {max(g for _, g in gaps):.0f} s)", rel(gaps[0][0]), event=True)
            s.kpi(label, f"{sum(g for _, g in gaps):.0f} s lost", "warn")
        else:
            s.add("ok", f"{label}: frames arrived throughout")
            s.kpi(label, "ok", "ok")
    if means:
        n_imb = sum(1 for it in s.items if "currents differ" in it.msg)
        s.kpi("Motor current pairs", f"{n_imb} differ" if n_imb else "balanced", "warn" if n_imb else "ok")
    s.note = ("Not in this log, so not checked: CAN bus utilization / timeouts, loop times and overruns, "
              "Driver Station disconnects.")
    return s


# ------------------------------------------------------------------------------------------ swerve
def _can(tel, ml, win, rel):
    """CAN bus health from the robot's once-a-second readings (comp_system_core/helpers/can_monitor.py): how busy each bus
    was, and any bus-off or TX-queue-full event, with its time."""
    sec = Section("CAN buses")
    a, b = win
    scale = T.can_util_scale(tel)
    cols = ["bus", "mean %", "max %", "s over 70 %", "bus-off", "TX-full", "rx errors", "tx errors"]
    rows, worst, n_off, n_full = [], 0.0, 0, 0
    for bus, what in T.CAN_BUSES:
        nu = _num(tel.by_key.get(f"{bus}_util"), a, b)
        if nu is None:
            continue
        t, x = nu
        x = x * scale
        d = np.diff(t)
        i = int(np.argmax(x))
        mean, top = _tw_mean(t, x), float(x[i])
        over = float(d[x > 70.0].sum())
        worst = max(worst, top)
        label = f"{bus} ({what})"
        if top > 90.0:
            sec.add("bad", f"{label} reached {top:.0f} % busy", rel(t[i]), event=True)
        elif top > 70.0:
            sec.add("warn", f"{label} reached {top:.0f} % busy ({over:.0f} s over 70 %)", rel(t[i]), event=True)
        counts = []
        for key, name in (("busoff", "bus-off"), ("txfull", "TX queue full"), ("rxerr", "receive errors"), ("txerr", "transmit errors")):
            s = tel.by_key.get(f"{bus}_{key}")
            base, end = (s.at(a, 0.0), s.at(b, 0.0)) if s is not None and len(s) else (0.0, 0.0)
            counts.append(int(max(end - base, 0.0)))
            if key in ("busoff", "txfull"):
                for tr in _rises(s, a, b):
                    sec.add("bad" if key == "busoff" else "warn", f"{label} {name}", rel(tr), event=True)
                    if key == "busoff":
                        n_off += 1
                    else:
                        n_full += 1
        rows.append([bus, f"{mean:.1f}", f"{top:.1f}", f"{over:.0f}", *counts])
    if not rows:
        sec.add("info", "no CAN bus data in this log (the robot has logged it since 2026-10-25)")
        return sec
    sec.columns, sec.rows = cols, rows
    px = _num(tel.by_key.get("can_phx_util"), a, b)
    s0 = _num(tel.by_key.get("can_s0_util"), a, b)
    if px is not None and s0 is not None:
        sec.add("info", f"can_s0 mean: {_tw_mean(s0[0], s0[1] * scale):.1f} in the utilization topic, {_tw_mean(px[0], px[1]):.1f} as Phoenix "
                        f"returns it (the same number when Phoenix was the reader; a different one means WPILib's unit differs)")
    if not sec.items:
        sec.add("ok", f"no CAN bus above 70 % busy, no bus-off or TX-full events (busiest {worst:.0f} %)")
    sec.kpi("CAN max busy", f"{worst:.0f} %", "bad" if worst > 90 else "warn" if worst > 70 else "ok")
    sec.kpi("CAN bus-off", str(n_off), "bad" if n_off else "ok")
    sec.note = ("Utilization is read once a second.  Bus-off and TX-full are running totals on the robot; a rise during the match is "
                "the event.  A bus above about 70 % leaves little room for a burst, and bus-off means the controller disconnected "
                f"from that bus.  The unit is taken as percent and is not checked against a bus analyzer{' (this log held fractions, shown x100)' if scale == 100.0 else ''}.")
    return sec


def _swerve(tel, ml, win, rel, thr):
    s = Section("Swerve modules")
    a, b = win
    act, des = tel.by_key.get("mod"), tel.by_key.get("mod_des")
    if act is None or des is None or not len(act) or not len(des):
        s.add("info", "swerve module states (actual / desired) are not in this log")
        return s
    ia = np.nonzero((act.t >= a) & (act.t <= b))[0]
    ta, A = [], []
    for i in ia:
        if np.shape(act.v[i]) == (4, 2):
            ta.append(act.t[i])
            A.append(act.v[i])
    if len(A) < 50:
        s.add("info", "too little swerve data inside the match to compare modules")
        return s
    ta, A = np.array(ta), np.array(A, float)
    j = np.searchsorted(des.t, ta, side="right") - 1
    ok = j >= 0
    ta, A, j = ta[ok], A[ok], j[ok]
    D = np.array([des.v[k] if np.shape(des.v[k]) == (4, 2) else np.full((4, 2), np.nan) for k in j], float)
    sa, aa, sd, ad = A[:, :, 0], A[:, :, 1], D[:, :, 0], D[:, :, 1]
    dang = np.arctan2(np.sin(aa - ad), np.cos(aa - ad))
    flip = np.abs(dang) > np.pi / 2
    sd2 = np.where(flip, -sd, sd)
    dang2 = np.where(flip, np.arctan2(np.sin(dang + np.pi), np.cos(dang + np.pi)), dang)
    dspd = (sa - sd2) * np.where(sd2 >= 0, 1.0, -1.0)
    dang_deg = np.degrees(dang2)
    moving = (np.abs(sd2) > 0.1) | (np.abs(sa) > 0.1)
    names = ["LF", "RF", "LB", "RB"]
    th_s, th_a = thr["speed"], thr["angle"]
    stat = []
    td, tt = tel.by_key.get("temp_drive"), tel.by_key.get("temp_turn")

    def tmax(sig, m):
        n = _held(sig, a, b)
        if n is None:
            return None
        vals = [x[m] for x in n[1] if np.shape(x) == (4,)]
        return float(np.nanmax(vals)) if vals else None

    for m in range(4):
        mv = moving[:, m] & np.isfinite(dspd[:, m])
        n = int(mv.sum())
        if n < 20:
            stat.append(None)
            continue
        ds, da = dspd[mv, m], dang_deg[mv, m]
        stat.append({"n": n, "rms_s": float(np.sqrt(np.mean(ds ** 2))), "pct_s": float(np.mean(np.abs(ds) > th_s) * 100),
                     "rms_a": float(np.sqrt(np.mean(da ** 2))), "pct_a": float(np.mean(np.abs(da) > th_a) * 100),
                     "td": tmax(td, m), "tt": tmax(tt, m)})
    s.columns = ["Module", f"speed RMS dev m/s", f"% moving > {th_s:g} m/s", "angle RMS dev deg", f"% moving > {th_a:g} deg",
                 "max drive C", "max turn C"]
    f = lambda x, fmt: "--" if x is None else fmt.format(x)
    for m, st in enumerate(stat):
        if st is None:
            s.rows.append([names[m]] + ["--"] * 6)
        else:
            s.rows.append([names[m], f"{st['rms_s']:.2f}", f"{st['pct_s']:.1f}", f"{st['rms_a']:.1f}", f"{st['pct_a']:.1f}",
                           f(st["td"], "{:.0f}"), f(st["tt"], "{:.0f}")])
    flagged = False
    for key, label, floor, ratio in (("rms_s", "speed deviation", 0.5 * th_s, 1.5), ("rms_a", "angle deviation", 0.5 * th_a, 1.5),
                                     ("pct_s", "time beyond the speed threshold", 3.0, 2.0),
                                     ("pct_a", "time beyond the angle threshold", 3.0, 2.0)):
        for m, st in enumerate(stat):
            others = [o[key] for k, o in enumerate(stat) if o is not None and k != m]
            if st is None or len(others) < 2:
                continue
            med = float(np.median(others))
            if st[key] > ratio * max(med, 1e-9) and st[key] > floor:
                flagged = True
                s.add("warn", f"{names[m]} stands out in {label}: {st[key]:.2f} vs {med:.2f} for the others"
                              if key.startswith("rms") else
                              f"{names[m]} stands out in {label}: {st[key]:.1f}% vs {med:.1f}% for the others")
    for key, label in (("td", "drive motor"), ("tt", "turn motor")):
        for m, st in enumerate(stat):
            others = [o[key] for k, o in enumerate(stat) if o is not None and o[key] is not None and k != m]
            if st is None or st[key] is None or len(others) < 2:
                continue
            med = float(np.median(others))
            if st[key] >= med + 8.0:
                flagged = True
                s.add("warn", f"{names[m]} {label} ran hot: {st[key]:.0f} C max, {st[key] - med:.0f} C above the others")
    if not flagged:
        s.add("ok", "no module stands out from the others in tracking error or temperature")
    n_flag = sum(1 for it in s.items if it.sev == "warn")
    s.kpi("Module outliers", str(n_flag) if n_flag else "none", "warn" if n_flag else "ok")
    temps = [st[k] for st in stat if st for k in ("td", "tt") if st[k] is not None]
    if temps:
        s.kpi("Hottest motor", f"{max(temps):.0f} C", "warn" if any("ran hot" in it.msg for it in s.items) else "info")
    rms = [st["rms_s"] for st in stat if st]
    if rms:
        s.kpi("Speed error RMS", f"{max(rms):.2f} m/s")
    s.note = ("Deviation = actual minus desired state, over the samples where the module is moving. Angle differences "
              "are wrapped to +-180 deg. A module is flagged when it is clearly worse than the median of the other three.")
    return s


# ------------------------------------------------------------------------------------------ odometry
def _odometry(tel, ml, win, rel):
    s = Section("QuestNav odometry")
    a, b = win
    dl = tel.by_key.get("delta")
    syn = tel.by_key.get("q_synched")
    if dl is None or not len(dl):
        s.add("info", "Quest-vs-drive error (quest_error_pose) is not in this log")
        return s
    m = (dl.t >= a) & (dl.t <= b)
    t = dl.t[m]
    if len(t) < 10:
        s.add("info", "too little Quest data inside the match")
        return s
    e = np.array([np.hypot(v[0], v[1]) for v, k in zip(dl.v, m) if k], float)
    synced = np.array([bool(syn.at(x, True)) if syn is not None and len(syn) else True for x in t])
    unsynced_pct = float((~synced).mean() * 100)
    s.add("warn" if unsynced_pct > 20 else "info",
          f"QuestNav unsynced for {unsynced_pct:.0f}% of the analysed time" + ("" if syn is not None else " (questnav_synched not logged)"))
    es = e[synced] if synced.any() else e
    imax = int(np.argmax(es if synced.any() else e))
    tt_ = t[synced][imax] if synced.any() else t[imax]
    p95 = float(np.percentile(es, 95))
    s.add("warn" if p95 > 1.0 else "info",
          f"Quest vs drive error while synced: median {np.median(es):.2f} m, 95th percentile {p95:.2f} m, maximum {es.max():.2f} m",
          rel(tt_))
    for thr_m in (0.5, 1.0):
        s.add("info", f"error above {thr_m:g} m for {float(np.mean(es > thr_m) * 100):.1f}% of the synced time")
    # jumps: the error changes by more than 0.8 m between two samples that are close in time.  While QuestNav is
    # unsynced its pose is not used, so those are only counted.
    de, dt = np.abs(np.diff(e)), np.diff(t)
    last, n_j, n_u = -9.0, 0, 0
    for i in np.nonzero((de > 0.8) & (dt < 0.5))[0]:
        tj = t[i + 1]
        if tj - last <= 1.0:
            last = tj
            continue
        last = tj
        if synced[i + 1]:
            n_j += 1
            s.add("warn", f"error jumped from {e[i]:.2f} m to {e[i + 1]:.2f} m", rel(tj), event=True)
        else:
            n_u += 1
    if n_u:
        s.add("info", f"{n_u} further jump(s) of the error while QuestNav was unsynced (not listed)")
    if not n_j:
        s.add("ok", "no sudden jumps in the Quest-vs-drive error while synced")
    # events from the other Quest entries
    for x in _rises(tel.by_key.get("q_lost"), a, b):
        s.add("warn", "QuestNav tracking lost", rel(x), event=True)
    for x in _rises(tel.by_key.get("q_dtap"), a, b):
        s.add("info", "double tap on the Quest", rel(x))
    for x0, x1 in _intervals(tel.by_key.get("q_reset_cmd"), a, b):
        s.add("info", "QuestNav odometry reset command", rel(x0))
    for x0, x1 in _intervals(tel.by_key.get("q_connected"), a, b, truthy=False):
        s.add("bad", f"Quest not connected for {x1 - x0:.1f} s", rel(x0), event=True)
    for x0, x1 in _intervals(syn, a, b, truthy=False):
        if x1 - x0 >= 2.0:
            s.add("info", f"QuestNav unsynced for {x1 - x0:.1f} s", rel(x0))
    nb = _num(tel.by_key.get("q_battery"), a, b)
    if nb is not None and float(nb[1].min()) < 20.0:
        s.add("warn", f"Quest battery fell to {float(nb[1].min()):.0f}%")
    # drift while synced: error growth per minute over each synced stretch of at least 10 s
    s.columns = ["Synced from", "length s", "mean error m", "max error m", "drift m/min"]
    drifts = []
    sy = np.append(synced, False)
    start = None
    for i, on in enumerate(sy):
        if on and start is None:
            start = i
        elif not on and start is not None:
            seg_t, seg_e = t[start:i], e[start:i]
            if seg_t[-1] - seg_t[0] >= 10.0:
                k = float(np.polyfit(seg_t - seg_t[0], seg_e, 1)[0] * 60.0)
                drifts.append(k)
                s.rows.append([f"{rel(seg_t[0]):+.1f} s", f"{seg_t[-1] - seg_t[0]:.0f}", f"{seg_e.mean():.2f}",
                               f"{seg_e.max():.2f}", f"{k:+.2f}"])
            start = None
    if drifts:
        s.add("warn" if (float(np.median(drifts)) > 0.3 and len(drifts) >= 2) else "info",
              f"median error drift while synced: {float(np.median(drifts)):+.2f} m per minute over {len(drifts)} stretch(es)")
    s.kpi("Unsynced", f"{unsynced_pct:.0f}%", "warn" if unsynced_pct > 20 else "ok")
    s.kpi("Median error", f"{np.median(es):.2f} m")
    s.kpi("95th pct error", f"{p95:.2f} m", "warn" if p95 > 1.0 else "ok")
    s.kpi("Jumps (synced)", str(n_j) if n_j else "none", "warn" if n_j else "ok")
    n_lost = len(_rises(tel.by_key.get("q_lost"), a, b))
    s.kpi("Tracking lost", str(n_lost) if n_lost else "none", "warn" if n_lost else "ok")
    disc = sum(x1 - x0 for x0, x1 in _intervals(tel.by_key.get("q_connected"), a, b, truthy=False))
    s.kpi("Disconnected", f"{disc:.0f} s" if disc else "never", "bad" if disc else "ok")
    s.note = ("Error = distance between the Quest pose and the fused drive pose, a proxy: neither is the true position. "
              "Drift is the slope of that error over a synced stretch.")
    return s


# ------------------------------------------------------------------------------------------ cycles
def _cycles(tel, ml, win, rel):
    s = Section("Cycles (shooting bursts from the alliance zone)")
    sm = T.ShotModel(tel)
    if not sm.available:
        s.add("info", "shooter data (indexer_on / flywheel rpm) is not in this log")
        return s
    bursts = T.VideoShotModel._merge(sm)
    a, b = win
    end_rel = rel(b)
    if not bursts:
        s.add("warn", "no shots from inside the alliance zone were found")
    tele0 = rel(ml.teleop_start) if (ml.has_match and ml.teleop_start is not None) else None

    def phase(x):
        if not ml.has_match:
            return "-"
        return "auto" if (tele0 is None and x < 20.0) or (tele0 is not None and x < tele0) else "teleop"
    intake = tel.by_key.get("intake_on")
    iv = [(rel(x0), rel(x1)) for x0, x1 in _intervals(intake, tel.a0 - 5, tel.a0 + end_rel + 5)] if intake is not None and len(intake) else None
    if iv is None:
        rpm = tel.by_key.get("intake_rpm")
        if rpm is not None and len(rpm):
            h = _held(rpm, a - 5, b + 5)
            on = np.abs(np.asarray(h[1], float)) > 100
            iv, st = [], None
            for i, x in enumerate(on):
                if x and st is None:
                    st = h[0][i]
                elif not x and st is not None:
                    iv.append((rel(st), rel(h[0][i])))
                    st = None

    def overlap(x0, x1):
        return sum(max(0.0, min(x1, q1) - max(x0, q0)) for q0, q1 in iv) if iv is not None else None
    s.columns = ["#", "phase", "start", "balls", "shoot s", "collect s", "intake on s", "cycle s"]
    prev_a, prev_b = None, 0.0
    cyc = []
    for k, bu in enumerate(bursts):
        ca, cb = bu["a"], bu["b"]
        coll = ca - prev_b
        ion = overlap(prev_b, ca)
        cycle = ca - prev_a if prev_a is not None else None
        cyc.append({"k": k + 1, "phase": phase(ca), "a": ca, "balls": len(bu["dips"]), "shoot": cb - ca, "collect": coll,
                    "intake": ion, "cycle": cycle})
        s.rows.append([k + 1, phase(ca), f"{ca:+.1f} s", len(bu["dips"]), f"{cb - ca:.1f}", f"{coll:.1f}",
                       "--" if ion is None else f"{ion:.1f}", "--" if cycle is None else f"{cycle:.1f}"])
        prev_a, prev_b = ca, cb
    if cyc:
        s.add("info", f"first shot {cyc[0]['a']:+.1f} s after time zero; {len(cyc)} shooting bursts, "
                      f"{sum(c['balls'] for c in cyc)} balls fired in all", cyc[0]["a"])
        for ph in ("auto", "teleop"):
            cs = [c for c in cyc if c["phase"] == ph]
            cycles = [c["cycle"] for c in cs if c["cycle"] is not None]
            if cs:
                msg = f"{ph}: {len(cs)} burst(s), {sum(c['balls'] for c in cs)} balls, {np.mean([c['balls'] for c in cs]):.1f} per burst"
                if cycles:
                    msg += (f"; cycle time median {np.median(cycles):.1f} s, best {min(cycles):.1f} s, worst {max(cycles):.1f} s")
                s.add("info", msg)
        allc = [c["cycle"] for c in cyc if c["cycle"] is not None and c["phase"] != "auto"]
        if len(allc) >= 3:
            med = float(np.median(allc))
            for c in cyc:
                if c["phase"] != "auto" and c["cycle"] is not None and c["cycle"] > 2 * med and c["cycle"] > 15:
                    s.add("warn", f"slow cycle: {c['cycle']:.0f} s between bursts (median {med:.0f} s); "
                                  f"{c['collect']:.0f} s collecting", c["a"])
        gap_end = end_rel - cyc[-1]["a"]
        if gap_end > 25 and ml.has_match:
            s.add("info", f"no shots in the last {gap_end:.0f} s", cyc[-1]["a"])
    # balls fed from outside the zone (G407 passes)
    groups, last = [], -99.0
    for x in sorted(sm.passes):
        if x - last > 2.5:
            groups.append([x, 0])
        groups[-1][1] += 1
        last = x
    for x, n in groups:
        s.add("info", f"fed {n} ball(s) from outside the alliance zone (a pass, cannot score)", x)
    if cyc:
        s.kpi("First shot", f"{cyc[0]['a']:+.0f} s")
        s.kpi("Bursts / balls", f"{len(cyc)} / {sum(c['balls'] for c in cyc)}")
        tele = [c["cycle"] for c in cyc if c["phase"] != "auto" and c["cycle"] is not None]
        if tele:
            s.kpi("Median cycle", f"{float(np.median(tele)):.0f} s")
        n_slow = sum(1 for it in s.items if it.msg.startswith("slow cycle"))
        s.kpi("Slow cycles", str(n_slow) if n_slow else "none", "warn" if n_slow else "ok")
    n_pass = sum(n for _, n in groups)
    s.kpi("Passes (balls)", str(n_pass) if n_pass else "none")
    s.note = ("A burst is a run of flywheel-rpm dips while the indexer runs (balls fired), merged when less than 2.5 s apart. "
              "Cycle = start of this burst minus start of the previous one; collect = gap between the end of the previous "
              "burst and the start of this one.")
    return s


# ------------------------------------------------------------------------------------------ hoot
def _hoot_section(tel, ml, win, rel, hoot):
    import hoot as H
    s = Section("CTRE hoot: swerve drive motors")
    a, b = win
    m, tot = hoot.matched
    s.add("ok" if tot and m == tot else "warn",
          f"hoot lined up with the robot log by the enable edges: offset {hoot.offset:+.2f} s, {m} of {tot} edges within 0.1 s")
    for w in hoot.warnings:
        s.add("info", "hoot: " + w)
    if hoot.t_last < b - 1.0:
        s.add("warn", f"the hoot ends at {rel(hoot.t_last):+.0f} s, before the match ends at {rel(b):+.0f} s: "
                      f"nothing after that is analysed", rel(hoot.t_last))
    if hoot.can_up is not None and len(hoot.can_up):
        down = _intervals(hoot.can_up, a, b, truthy=False)
        for x0, x1 in down:
            s.add("bad", f"CAN bus reported down for {x1 - x0:.2f} s", rel(x0), event=True)
        s.kpi("CAN bus", f"down {sum(x1 - x0 for x0, x1 in down):.1f} s" if down else "up", "bad" if down else "ok")
    stats = {}
    labels = {d: H.DEVICE_NAMES.get(d, d) for d in hoot.devices}
    mod = tel.by_key.get("mod")
    n_faults = 0
    for dev in hoot.devices:
        lab = labels[dev]
        sup, sta, vel = hoot.sig(dev, "SupplyCurrent"), hoot.sig(dev, "StatorCurrent"), hoot.sig(dev, "Velocity")
        mv, sv = hoot.sig(dev, "MotorVoltage"), hoot.sig(dev, "SupplyVoltage")
        st = {"label": lab}
        nsup, nsta = _num(sup, a, b), _num(sta, a, b)
        mt = None
        if vel is not None and len(vel):
            k = (vel.t >= a) & (vel.t <= b)
            tv, vv = vel.t[k], np.abs(vel.numeric()[k])
            if len(tv) > 10:
                dtv = np.diff(tv, append=tv[-1])
                mt = (tv, vv, dtv)
        if nsup is not None:
            t, x = nsup
            st["peak_sup"], st["t_peak"] = float(x.max()), rel(t[int(np.argmax(x))])
            if mt is not None:
                tv, vv, dtv = mt
                sup_at = np.array([sup.at(q, 0.0) for q in tv[::5]], float)
                mv_ = vv[::5] > 1.0
                st["avg_sup_moving"] = float(sup_at[mv_].mean()) if mv_.any() else None
        if nsta is not None:
            st["peak_sta"] = float(nsta[1].max())
        nsv = _num(sv, a, b)
        if nsv is not None:
            st["min_v"], st["t_min_v"] = float(nsv[1].min()), rel(nsv[0][int(np.argmin(nsv[1]))])
        nmv = _num(mv, a, b)
        if nmv is not None and nsv is not None:
            t, x = nmv
            svv = np.array([sv.at(q, 12.0) for q in t[:-1]], float)
            full = np.abs(x) >= 0.95 * np.maximum(svv, 1.0)
            d = np.diff(t)
            st["full_pct"] = float(d[full].sum() / max(d.sum(), 1e-9) * 100)
        if mt is not None and nsta is not None:
            tv, vv, dtv = mt
            sta_at = np.array([sta.at(q, 0.0) for q in tv], float)
            st["stall_s"] = float(dtv[(vv < 0.5) & (sta_at > 40.0)].sum())
        # wheel scale: m/s of the module (NT) per rps of the motor (hoot)
        mi = H.MODULE_OF.get(dev)
        if mod is not None and len(mod) and mi is not None and mt is not None:
            im = np.nonzero((mod.t >= a) & (mod.t <= b))[0]
            ts_ = np.array([mod.t[i] for i in im if np.shape(mod.v[i]) == (4, 2)])
            sp = np.array([abs(mod.v[i][mi][0]) for i in im if np.shape(mod.v[i]) == (4, 2)])
            if len(ts_) > 100:
                j = np.clip(np.searchsorted(vel.t, ts_, side="right") - 1, 0, len(vel) - 1)
                vh = np.abs(vel.numeric()[j])
                ok = (sp > 0.5) & (vh > 0.5)
                if ok.sum() > 100:
                    st["scale"] = float((sp[ok] * vh[ok]).sum() / (vh[ok] ** 2).sum())
        # faults, device resets, data dropouts
        for fk, sev in H.FAULTS.items():
            fs = hoot.sig(dev, fk)
            if fs is None or not len(fs):
                continue
            for x0, x1 in _intervals(fs, a, b):
                n_faults += sev in ("bad", "warn")
                s.add(sev, f"{lab}: {fk[6:]} fault for {x1 - x0:.1f} s", rel(x0), event=sev in ("bad", "warn"))
        rc = hoot.sig(dev, "ResetCount")
        if rc is not None and len(rc) and float(rc.v[-1]) > 0:
            s.add("info", f"{lab}: the device reports {int(rc.v[-1])} reset(s) since power-up")
        if sta is not None and len(sta):
            tt = np.concatenate([[a], np.clip(sta.t[(sta.t > a) & (sta.t < min(b, hoot.t_last))], a, b), [min(b, hoot.t_last)]])
            gaps = [(tt[i], tt[i + 1] - tt[i]) for i in range(len(tt) - 1) if tt[i + 1] - tt[i] > 1.0]
            if gaps:
                s.add("warn", f"{lab}: no data for more than 1 s, {len(gaps)} time(s), longest {max(g for _, g in gaps):.1f} s "
                              f"(device off the bus?)", rel(gaps[0][0]), event=True)
        stats[dev] = st
    # ---- table and comparisons between the modules
    s.columns = ["Motor", "peak supply A", "avg supply A (moving)", "peak stator A", "min supply V", "full voltage %",
                 "stall s", "m/s per rps"]
    f = lambda x, fmt: "--" if x is None else fmt.format(x)
    for dev, st in stats.items():
        s.rows.append([st["label"], f(st.get("peak_sup"), "{:.0f}"), f(st.get("avg_sup_moving"), "{:.1f}"),
                       f(st.get("peak_sta"), "{:.0f}"), f(st.get("min_v"), "{:.1f}"), f(st.get("full_pct"), "{:.1f}"),
                       f(st.get("stall_s"), "{:.1f}"), f(st.get("scale"), "{:.4f}")])
    scales = {d: st["scale"] for d, st in stats.items() if "scale" in st}
    spread = 0.0
    if len(scales) >= 3:
        med = float(np.median(list(scales.values())))
        spread = max(abs(v / med - 1) for v in scales.values()) * 100
        for d, v in scales.items():
            if abs(v / med - 1) > 0.03:
                s.add("warn", f"{labels[d]}: the wheel travels {abs(v / med - 1) * 100:.1f}% {'more' if v > med else 'less'} per motor "
                              f"revolution than the other modules (wheel wear or a different diameter, or a wrong gear ratio)")
    avgs = {d: st["avg_sup_moving"] for d, st in stats.items() if st.get("avg_sup_moving") is not None}
    if len(avgs) >= 3:
        for d, v in avgs.items():
            others = [x for k, x in avgs.items() if k != d]
            if v > 1.5 * float(np.median(others)) and v > 3.0:
                s.add("warn", f"{labels[d]}: draws {v:.1f} A on average while moving, {v / float(np.median(others)):.1f}x the other "
                              f"modules (dragging, misaligned or a bad bearing?)")
    stalls = {d: st["stall_s"] for d, st in stats.items() if st.get("stall_s") is not None}
    if stalls and max(stalls.values()) > 2.0:
        d = max(stalls, key=stalls.get)
        s.add("info", f"{labels[d]}: {stalls[d]:.1f} s pushing against something (stator current over 40 A, wheel not turning)")
    # share of the battery current
    na = _num(tel.by_key.get("pdh_a"), a, b)
    if na is not None and stats:
        t, x = na
        tp = float(t[int(np.argmax(x))])
        drv = sum(hoot.sig(d, "SupplyCurrent").at(tp, 0.0) for d in stats if hoot.sig(d, "SupplyCurrent") is not None)
        s.add("info", f"at the peak of {float(x.max()):.0f} A ({rel(tp):+.0f} s) the four swerve drive motors drew {drv:.0f} A "
                      f"({drv / float(x.max()) * 100:.0f}%)", rel(tp))
    nv = _num(tel.by_key.get("pdh_v"), a, b)
    if nv is not None and stats:
        t, x = nv
        tm = float(t[int(np.argmin(x))])
        drv = sum(hoot.sig(d, "SupplyCurrent").at(tm, 0.0) for d in stats if hoot.sig(d, "SupplyCurrent") is not None)
        pa = tel.by_key.get("pdh_a")
        tot = pa.at(tm, None) if pa is not None else None
        s.add("info", f"when the battery was lowest ({float(x.min()):.1f} V, {rel(tm):+.0f} s) the swerve drive motors drew {drv:.0f} A"
                      + (f" of {tot:.0f} A total" if tot else ""), rel(tm))
    if stats:
        peaks = [st["peak_sup"] for st in stats.values() if "peak_sup" in st]
        if peaks:
            s.kpi("Drive peak", f"{max(peaks):.0f} A")
        s.kpi("Device faults", str(n_faults) if n_faults else "none", "bad" if n_faults else "ok")
        if scales:
            s.kpi("Wheel scale spread", f"{spread:.1f}%", "warn" if spread > 3 else "ok")
        if stalls:
            s.kpi("Longest stall", f"{max(stalls.values()):.1f} s")
    s.note = ("From the CTRE hoot (swerve drive Krakens only; the turn motors are REV and not in it), lined up to the robot log "
              "by the robot-enable edges. 'm/s per rps' = module speed from NetworkTables divided by the motor's rotations per "
              "second: it should be the same for all four modules. Full voltage = motor voltage at 95% or more of the supply voltage.")
    return s


def _ff_section(tel, ml, win, rel, hoot):
    """Drive motor feedforward (kS / kV / kA) fitted to this match's hoot: see sysid.py for what it can and cannot say."""
    import hoot as H
    import sysid
    s = Section("Drive motor feedforward (hoot)")
    wins = sysid.enabled_windows(hoot, win)
    fits = sysid.fit_hoot(hoot, wins)
    if not fits:
        s.add("info", "the hoot has no Velocity + MotorVoltage for the drive motors, so no feedforward fit")
        return s
    s.columns = ["Motor", "kS V", "kV V/rps", "kV V/(m/s)", "kA V/(rps/s)", "kA V/(m/s^2)", "samples", "rms V", "reaches rps"]
    f = lambda x, fmt: "--" if x is None else fmt.format(x)
    kvs = {}
    for dev, fit in fits.items():
        lab = H.DEVICE_NAMES.get(dev, dev)
        if fit.ks is None:
            s.add("info", f"{lab}: {fit.note}")
            continue
        m = fit.per_ms(sysid.module_scale(tel, hoot, dev, win))
        kvs[dev] = fit.kv
        s.rows.append([lab, f(fit.ks, "{:.2f}"), f(fit.kv, "{:.3f}"), f(m and m[1], "{:.2f}"),
                       f(fit.ka, "{:.4f}") + ("" if fit.ka_ok else " ?"), f(m and m[2], "{:.3f}") + ("" if fit.ka_ok else " ?"),
                       fit.n, f(fit.rms, "{:.2f}"), f(fit.v_max, "{:.1f}")])
    if len(kvs) >= 3:
        med = float(np.median(list(kvs.values())))
        for dev, kv in kvs.items():
            if abs(kv / med - 1) > 0.06:
                s.add("warn", f"{H.DEVICE_NAMES.get(dev, dev)}: kV is {abs(kv / med - 1) * 100:.0f}% {'above' if kv > med else 'below'} "
                              f"the other modules (a dragging wheel, a worn tread or a different gearbox?)")
        s.kpi("Drive kV", f"{med:.3f} V/rps")
    s.note = ("volts = kS*sign(v) + kV*v + kA*a fitted to the motor's MotorVoltage and Velocity while enabled (battery 9 V or "
              "more, output not clipped). kV is reliable (about 2% between matches); kS is rough and kA is biased by the closed "
              "loop and wheel slip ('?' = the data does not pin it down); this robot's hoot logs MotorVoltage at only 4 Hz. "
              "Pool several matches with `python sysid.py <logs folder>` and confirm with a shop test (SYSID_SHOP_GUIDE.md).")
    return s


# ------------------------------------------------------------------------------------------ correlation
def _console(tel, ml, win, rel):
    """What the robot printed (console_*.log saved next to the wpilog by comp_system_core/helpers/console_log.py): the
    warnings and errors, with the time they appeared.  Native warnings (loop overruns, watchdog, Phoenix / REV errors) are
    otherwise only ever seen on the Driver Station console, which is gone after the match."""
    import console_file as CF
    sec = Section("Robot console")
    path = CF.find_for_log(ml.path)
    if path is None:
        sec.add("info", "no console_*.log from this boot was found next to the log (the robot only saves one since "
                        "2026-10-25; it is written in the same folder as the .wpilog)")
        return sec
    lines = CF.parse(path)
    a, b = win
    sec.note = (f"{os.path.basename(path)}: {len(lines)} lines. Times are on the wpilog clock (robot time), so they line up "
                f"with every plot; lines before the match start (the boot, up to 30 s before) show as negative times.")
    groups = {}
    for ln in lines:
        sev = CF.severity(ln.text)
        if sev is None or not (a - 30.0 <= ln.t <= b + 5.0):
            continue
        key = (sev, ln.text.strip()[:120])
        g = groups.setdefault(key, [])
        g.append(ln.t)
    n_bad = sum(len(v) for (sev, _), v in groups.items() if sev == "bad")
    n_warn = sum(len(v) for (sev, _), v in groups.items() if sev == "warn")
    for (sev, text), ts in sorted(groups.items(), key=lambda kv: kv[1][0])[:60]:
        more = f"  (x{len(ts)}, last at {rel(ts[-1]):.1f} s)" if len(ts) > 1 else ""
        sec.add(sev, f"{text}{more}", rel(ts[0]), event=(sev == "bad"))
    if not groups:
        sec.add("ok", "no warnings or errors on the robot console during the match")
    sec.kpi("console errors", str(n_bad), "bad" if n_bad else "ok")
    sec.kpi("console warnings", str(n_warn), "warn" if n_warn else "ok")
    return sec


def _correlate(rep, window=2.0):
    """A section of the events (faults, jumps, dropouts) from DIFFERENT sections that happened within `window` seconds of
    each other: several things going wrong at once usually have one cause (a brownout, a reboot, a hard hit)."""
    evs = sorted(((it.t, sec.title.split(":")[0].split(" (")[0], it) for sec in rep.sections for it in sec.items
                  if it.event and it.t is not None), key=lambda x: x[0])
    groups, cur = [], []
    for e in evs:
        if cur and e[0] - cur[-1][0] > window:
            groups.append(cur)
            cur = []
        cur.append(e)
    groups.append(cur)
    out = Section("Events that line up")
    for g in groups:
        secs = {x[1] for x in g}
        if len(secs) < 2:
            continue
        sev = "bad" if any(x[2].sev == "bad" for x in g) else "warn"
        for x in g:
            x[2].grouped = True
        short = lambda m: m if len(m) <= 62 else m[:60].rstrip() + "..."
        parts = "; ".join(f"{short(x[2].msg)} [{x[1]}]" for x in g[:4]) + (f"; and {len(g) - 4} more" if len(g) > 4 else "")
        out.add(sev, f"{len(g)} events within {g[-1][0] - g[0][0] + 0.001:.1f} s, from {len(secs)} sources, probably one cause: {parts}",
                g[0][0])
    if not out.items:
        out.add("ok", "no events from different sources happened at the same time")
    out.note = (f"Events are faults, jumps, dropouts and disconnects that are within {window:g} s of each other. A reboot of a "
                f"drive motor, a brownout, a Quest tracking loss and a camera dropout at the same instant point to one power or "
                f"wiring event.")
    return out


# ------------------------------------------------------------------------------------------ entry point
def build_report(tel: "T.Telemetry", ml: "core.MatchLog", thr: dict | None = None, hoot=None) -> Report:
    thr = {"rpm": 500.0, "speed": 0.3, "angle": 10.0, **(thr or {})}
    win = _window(ml)
    a0 = tel.a0
    rel = lambda x: float(x - a0)
    title = f"Match report: {ml.match_label}" + (f"  ({ml.alliance_text})" if ml.alliance_text != "?" else "")
    rep = Report(title, win)
    if hoot is not None:                      # the CTRE section goes after the swerve section
        pass
    for fn in (lambda: _health(tel, ml, win, rel, hoot), lambda: _can(tel, ml, win, rel), lambda: _swerve(tel, ml, win, rel, thr),
               lambda: _odometry(tel, ml, win, rel), lambda: _cycles(tel, ml, win, rel),
               *([lambda: _hoot_section(tel, ml, win, rel, hoot), lambda: _ff_section(tel, ml, win, rel, hoot)] if hoot is not None else []),
               lambda: _console(tel, ml, win, rel)):
        try:
            rep.sections.append(fn())
        except Exception as e:  # noqa - one broken section must not hide the others
            rep.sections.append(Section("Error", [Item(None, "bad", f"a section of the report failed: {e!r}")]))
    try:
        rep.sections.insert(0, _correlate(rep))              # first: it is the most useful thing to read
    except Exception:  # noqa
        pass
    return rep
