"""
Season report generator: reads every robot .wpilog (and the CTRE .hoot files next to them, when owlet is installed) in a folder
and writes two self-contained HTML pages, no network needed:

    Season_Report.html        per-event scorecard, logging matrix, battery / brownout, camera coprocessor and Quest connectivity
    Compare_Robots.html       the newest event ("new robot") against the earlier ones: current draw, battery sag, one row per
                              match with camera / Quest / brownout events, one-match detail, match table

    python season_report.py "C:/Users/<you>/Documents/WPILOG_2026"
    python season_report.py LOGS --new-event CAEL --names CAEL=CASS --team 2429 --season 2026 --out reports

Logs are named  <anything>_<YYYYMMDD>_<HHMMSS>_<EVENT>_<P|Q|E><n>.wpilog  (the match type and number are taken from that).  The
judgement text of the season report (resolved / remaining / unknown) is report_templates/season_notes.html: edit it, or pass
--notes your_file.html.  The pages load Barlow / JetBrains Mono from Google Fonts when online and fall back to system fonts
when offline.  Needs numpy; the hoot part needs owlet (see hoot.py) and is skipped with a note when it is missing.
"""
from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import os
import re
import statistics as st
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "helpers"))     # core, telemetry, hoot ... live in helpers/
sys.path.insert(0, HERE)
import core  # noqa: E402

N = "NT:/SmartDashboard/RobotStatus/"
Q = "NT:/QuestNav/"
CAMS = (("R", "NT:/Cameras/ArducamRight/_frames"), ("B", "NT:/Cameras/ArducamBack/_frames"))
WANT = {"systemTime", "DS:enabled", "DS:controlWord", N + "_pdh_voltage", N + "_pdh_current", N + "_rio_browned_out",
        "NTConnection", Q + "quest_connected"} | {n for _, n in CAMS}
NAME_RE = re.compile(r"_(\d{8})_(\d{6})_([A-Za-z0-9]+)_([PQE])(\d+)\.wpilog$")
BIN, T0, T1 = 0.5, -5.0, 170.0
NB = int((T1 - T0) / BIN)
CUR_X = np.arange(0, 452, 2)
VOLT_X = np.arange(5.0, 12.01, 0.05)
FEATURES = [  # (label, regex on entry names)
    ("Battery voltage and total current", r"_pdh_voltage$"),
    ("roboRIO brownout flag", r"_rio_browned_out"),
    ("Camera coprocessor / Quest connection history", r"^NTConnection$"),
    ("Quest-vs-drive error", r"quest_error_pose"),
    ("Shooter speeds", r"Shooter/.*(rpm|speed)"),
    ("Per-motor currents", r"Shooter/current_|Intake/current_"),
    ("Swerve module states", r"Swerve/module_states$"),
    ("Swerve temperatures", r"Swerve/drive_temp"),
    ("Control word (match phases)", r"^DS:controlWord$"),
    ("Camera frame counters", r"Cameras/Arducam\w+/_frames"),
    ("URCL raw REV data", r"URCL/Raw/Periodic"),
]


# ------------------------------------------------------------------------------------------------ per-log analysis
def _d(recs, name):
    return [(t / 1e6, struct.unpack("<d", p)[0]) for t, p in recs.get(name, []) if len(p) == 8]


def _windows(recs):
    cw = recs.get("DS:controlWord")
    if cw:
        pts = [(t / 1e6, bool(core._decode_controlword(p)[1])) for t, p in cw]
    else:
        pts = [(t / 1e6, bool(p[0])) for t, p in recs.get("DS:enabled", []) if p]
    out, cur = [], None
    for t, v in pts:
        if v and cur is None:
            cur = t
        if not v and cur is not None:
            out.append((cur, t))
            cur = None
    if cur is not None and pts:
        out.append((cur, pts[-1][0]))
    return out


def _hold(t, a, b):
    t = np.asarray(t)
    nxt = np.append(t[1:], b)
    return np.maximum(np.clip(nxt, a, b) - np.clip(t, a, b), 0)


def analyze(path):
    m = NAME_RE.search(path.replace("\\", "/"))
    date, tm, ev, typ, num = m.groups()
    ent, recs = core.read_wpilog(path, lambda n: n in WANT)
    names = {e[0] for e in ent.values()}
    wins = _windows(recs)
    d = dict(file=os.path.basename(path), path=path, event=ev, typ=typ, num=int(num), date=date, time=tm, names=names, ok=False)
    if not wins:
        return d
    a0 = wins[0][0]
    d["enabled"] = sum(b - a for a, b in wins)
    d["windows"] = [(round(a - a0, 1), round(b - a0, 1)) for a, b in wins]
    d["a0"] = a0
    cur, vol = np.array(_d(recs, N + "_pdh_current")), np.array(_d(recs, N + "_pdh_voltage"))
    if len(cur) < 2 or len(vol) < 2:
        return d
    cw, vw = [], []
    for a, b in wins:
        w = _hold(cur[:, 0], a, b); k = w > 0
        cw.append((cur[k, 1], w[k]))
        w = _hold(vol[:, 0], a, b); k = w > 0
        vw.append((vol[k, 1], w[k]))
    cv, cwt = np.concatenate([x for x, _ in cw]), np.concatenate([x for _, x in cw])
    vv, vwt = np.concatenate([x for x, _ in vw]), np.concatenate([x for _, x in vw])
    if cwt.sum() <= 0 or vwt.sum() <= 0:
        return d
    o = np.argsort(cv)
    cum = np.cumsum(cwt[o]) / cwt.sum()
    d["cur_samples"], d["vol_samples"] = (cv, cwt), (vv, vwt)
    d["stats"] = dict(avg=float((cv * cwt).sum() / cwt.sum()), p95=float(cv[o][np.searchsorted(cum, 0.95)]),
                      peak=float(cv.max()), vmin=float(vv.min()), lt8=float(vwt[vv < 8].sum() / vwt.sum() * d["enabled"]))
    cb, vb = [None] * NB, [None] * NB
    for t, v in cur:
        i = int((t - a0 - T0) / BIN)
        if 0 <= i < NB:
            cb[i] = v if cb[i] is None else max(cb[i], v)
    for t, v in vol:
        i = int((t - a0 - T0) / BIN)
        if 0 <= i < NB:
            vb[i] = v if vb[i] is None else min(vb[i], v)
    d["cur"] = [None if x is None else round(x) for x in cb]
    d["vol"] = [None if x is None else round(x, 1) for x in vb]
    rio = [t / 1e6 for t, p in recs.get(N + "_rio_browned_out", []) if p and p[0]]
    d["rio_brownouts"] = len([t for t in rio if any(a <= t <= b for a, b in wins)])
    d["rio_logged"] = N + "_rio_browned_out" in names
    # NetworkTables connection history
    cop = []
    for ts, p in recs.get("NTConnection", []):
        try:
            j = json.loads(p.decode("utf-8", "replace"))
        except ValueError:
            continue
        t = ts / 1e6
        if j.get("connected") is False and re.search(r"arducam|launcher|limelight|photon|coproc", str(j.get("remote_id", "")), re.I) \
                and any(a <= t <= b for a, b in wins):
            if not cop or t - cop[-1] > 1.0:
                cop.append(t)
    d["cop_abs"] = cop
    d["cop"] = [round(t - a0, 1) for t in cop]
    qc = [(t / 1e6, bool(p[0])) for t, p in recs.get(Q + "quest_connected", []) if p]
    qiv, start = [], None
    for t, v in qc:
        if not v and start is None:
            start = t
        if v and start is not None:
            qiv.append((start, t)); start = None
    if start is not None:
        qiv.append((start, wins[-1][1]))
    d["quest"] = [(round(max(s, a) - a0, 1), round(min(e, b) - a0, 1)) for s, e in qiv for a, b in wins if min(e, b) - max(s, a) > 0.3]
    stalls, restarts, outage = [], [], 0.0
    for cam, name in CAMS:
        r = recs.get(name)
        if not r:
            continue
        ts = [t / 1e6 for t, _ in r]
        vs = [int.from_bytes(p, "little", signed=True) for _, p in r]
        iv = []
        for a, b in wins:
            pts = [a] + [t for t in ts if a < t < b] + [b]
            iv += [(t0, t1) for t0, t1 in zip(pts, pts[1:]) if t1 - t0 > 2.5]
        for t0, t1 in iv:
            stalls.append((cam, round(t0 - a0, 1), round(t1 - a0, 1)))
        outage = max(outage, sum(t1 - t0 for t0, t1 in iv))
        restarts += [(cam, round(ts[i] - a0, 1)) for i in range(1, len(vs)) if vs[i] < vs[i - 1] - 5
                     and any(a <= ts[i] <= b for a, b in wins)]
    d["stalls"], d["restarts"], d["cam_out"] = stalls, restarts, outage
    qd = 0.0
    for s_, e_ in d["quest"]:
        qd += e_ - s_
    d["q_pct"] = qd / d["enabled"] * 100
    d["early_auto"] = len(wins) >= 2 and (wins[0][1] - wins[0][0]) < 19 and typ != "P"
    d["ok"] = True
    return d


def analyze_hoot(d, hoots_ok=True):
    """Kraken numbers for one match from the CTRE hoot next to its log (None when there is none / no owlet)."""
    import hoot as H
    import telemetry as T
    hs = H.find_hoots_for_log(d["path"], os.path.dirname(d["path"]))
    if not hs:
        return None
    conv = [H.convert_hoot(h) for h in hs]
    hl = H.load_many([w for w, _ in conv], d["path"])
    a, b = d["a0"] + d["windows"][0][0], d["a0"] + d["windows"][-1][1]
    vmin, brown, other = None, [], set()
    for (dev, sg), s in hl.sigs.items():
        t = np.asarray(s.t)
        k = (t >= a) & (t <= b)
        if sg == "SupplyVoltage" and k.any():
            v = float(np.asarray(s.v, float)[k].min())
            vmin = v if vmin is None else min(vmin, v)
        elif sg.startswith("Fault_"):
            rise = T.rising_times(s, a, b)
            if sg == "Fault_BridgeBrownout":
                brown += [(round(x - d["a0"], 1)) for x in rise]
            elif rise:
                other.add(f"{H.DEVICE_NAMES.get(dev, dev)} {sg[6:]}")
    return dict(vmin=vmin, brown=sorted(set(brown)), other=sorted(other))


# ------------------------------------------------------------------------------------------------ helpers for the pages
def pct_curve(samples, xs, above):
    v = np.concatenate([s[0] for s in samples])
    w = np.concatenate([s[1] for s in samples])
    return [round(float(w[(v >= x) if above else (v <= x)].sum() / w.sum() * 100), 3) for x in xs]


def tint(x, lo, hi):
    f = (x - lo) / (hi - lo) if hi != lo else 0
    return "c3" if f > 0.66 else "c2" if f > 0.33 else "c1" if f > 0.05 else "c0"


def strip_chart(rows, order, key, lo, hi, ticks, unit, thr=None, fmt="{:.1f}"):
    W, H, ml, mr, mt, mb, gap = 920, 250, 46, 10, 14, 44, 14
    step = (W - ml - mr - gap * (len(order) - 1)) / max(len(rows), 1)
    y = lambda v: mt + (H - mt - mb) * (1 - (min(max(v, lo), hi) - lo) / (hi - lo))
    s = [f'<svg viewBox="0 0 {W} {H}" role="img" class="chart" aria-label="{key} per match">']
    for t in ticks:
        s.append(f'<line x1="{ml}" x2="{W - mr}" y1="{y(t):.1f}" y2="{y(t):.1f}" class="grid"/>'
                 f'<text x="{ml - 6}" y="{y(t) + 4:.1f}" text-anchor="end" class="tick">{fmt.format(t)}</text>')
    if thr is not None:
        s.append(f'<line x1="{ml}" x2="{W - mr}" y1="{y(thr):.1f}" y2="{y(thr):.1f}" class="thr"/>'
                 f'<text x="{W - mr - 2}" y="{y(thr) - 4:.1f}" text-anchor="end" class="thrt">{thr:g} {unit}</text>')
    x = ml
    for ev in order:
        L = [r for r in rows if r["label"] == ev]
        if not L:
            continue
        x0 = x
        for r in L:
            v = r[key]
            bad = thr is not None and key == "vmin" and v < thr
            s.append(f'<circle cx="{x + step / 2:.1f}" cy="{y(v):.1f}" r="4.2" class="dot{" bad" if bad else ""}">'
                     f'<title>{ev} {r["match"]}: {fmt.format(v)} {unit}</title></circle>')
            x += step
        mx = st.mean(r[key] for r in L)
        s.append(f'<line x1="{x0:.1f}" x2="{x:.1f}" y1="{y(mx):.1f}" y2="{y(mx):.1f}" class="mean"/>')
        s.append(f'<text x="{(x0 + x) / 2:.1f}" y="{H - 24}" text-anchor="middle" class="evl">{ev}</text>')
        s.append(f'<text x="{(x0 + x) / 2:.1f}" y="{H - 10}" text-anchor="middle" class="evs">{rows_dates[ev]}</text>')
        x += gap
    s.append("</svg>")
    return "".join(s)


rows_dates: dict = {}


def fmt_dates(ds):
    a, b = dt.datetime.strptime(min(ds), "%Y%m%d"), dt.datetime.strptime(max(ds), "%Y%m%d")
    mon = lambda x: x.strftime("%b")
    if a == b:
        return f"{mon(a)} {a.day}"
    return f"{mon(a)} {a.day}–{b.day}" if mon(a) == mon(b) else f"{mon(a)} {a.day}–{mon(b)} {b.day}"


def matrix_html(order, names_by_label):
    head = "".join(f"<th>{e}</th>" for e in order)
    rows = []
    for label, rx in FEATURES:
        cells = []
        for e in order:
            n = sum(1 for x in names_by_label[e] if re.search(rx, x))
            cells.append('<td class="y">yes</td>' if n else '<td class="n">no</td>')
        rows.append(f'<tr><th scope="row">{label}</th>{"".join(cells)}</tr>')
    return f'<table class="matrix"><thead><tr><th>Logged signal</th>{head}</tr></thead><tbody>{"".join(rows)}</tbody></table>'


# ------------------------------------------------------------------------------------------------ main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", help="folder with the .wpilog files (searched recursively)")
    ap.add_argument("--out", default=None, help="output folder (default: the log folder)")
    ap.add_argument("--new-event", default=None, help="event code of the 'new robot' in the comparison (default: the latest event)")
    ap.add_argument("--names", default="", help="display names, e.g. CAEL=CASS,CAPOH=Hueneme")
    ap.add_argument("--team", default="2429")
    ap.add_argument("--season", default=str(dt.date.today().year))
    ap.add_argument("--notes", default=os.path.join(HERE, "report_templates", "season_notes.html"))
    ap.add_argument("--no-hoot", action="store_true", help="skip the CTRE hoot files")
    a = ap.parse_args(argv)
    if not os.path.isdir(a.folder):
        sys.exit("folder not found: " + a.folder + " - give the folder that holds your .wpilog files, "
                 "e.g.  python season_report.py C:/Users/rtoda/Documents/WPILOG_2026")
    out = a.out or a.folder
    os.makedirs(out, exist_ok=True)
    disp = dict(x.split("=", 1) for x in a.names.split(",") if "=" in x)
    files = [f for f in sorted(glob.glob(os.path.join(a.folder, "**", "*.wpilog"), recursive=True))
             if os.path.getsize(f) > 1000 and NAME_RE.search(f.replace("\\", "/"))]
    if not files:
        sys.exit("no match logs (<...>_<date>_<time>_<EVENT>_<P|Q|E><n>.wpilog) found in " + a.folder)
    print(f"{len(files)} logs ...")
    logs = []
    for i, f in enumerate(files, 1):
        print(f"  [{i}/{len(files)}] {os.path.basename(f)}", flush=True)
        try:
            logs.append(analyze(f))
        except Exception as e:  # noqa - one unreadable log must not stop the report
            print("    skipped:", e)
    events = sorted({d["event"] for d in logs}, key=lambda e: min(d["date"] for d in logs if d["event"] == e))
    label = lambda e: disp.get(e, e)
    order = [label(e) for e in events]
    new_ev = a.new_event or events[-1]
    names_by = {label(e): set().union(*[d["names"] for d in logs if d["event"] == e]) for e in events}
    full = [d for d in logs if d["ok"] and d["enabled"] >= 60]
    for d in full:
        d["label"] = label(d["event"])
        d["match"] = f'{d["typ"]}{d["num"]}'
        d["id"] = f'{d["label"]} {d["match"]}'
        d["group"] = "new" if d["event"] == new_ev else "old"
    full.sort(key=lambda d: (events.index(d["event"]), d["date"], d["time"]))
    ev_dates = {label(e): fmt_dates([d["date"] for d in logs if d["event"] == e]) for e in events}
    rows_dates.update(ev_dates)
    # hoot
    hoot_by = {}
    if not a.no_hoot:
        try:
            for d in full:
                h = analyze_hoot(d)
                if h:
                    hoot_by[d["id"]] = h
                    d["brown"] = h["brown"]
        except Exception as e:  # noqa
            print("hoot part skipped:", f"{type(e).__name__}: {e}")
    new_label = label(new_ev)
    rows = []
    for d in full:
        d.setdefault("brown", [])
        rows.append(dict(label=d["label"], match=d["match"], vmin=d["stats"]["vmin"], lt8=d["stats"]["lt8"], q=d["q_pct"],
                         cop=len(d["cop"]), cam=d["cam_out"], early=d["early_auto"]))
    # ---------------- season report
    score = ""
    by = {e: [r for r in rows if r["label"] == e] for e in order}
    for e in order:
        L = by[e]
        if not L:
            continue
        avgmin, worst, s8 = st.mean(r["vmin"] for r in L), min(r["vmin"] for r in L), st.mean(r["lt8"] for r in L)
        q, cp = st.mean(r["q"] for r in L), st.mean(r["cop"] for r in L)
        cam, early = sum(r["cam"] >= 30 for r in L), sum(r["early"] for r in L)
        score += (f'<tr><th scope="row"><span class="evn">{e}</span><span class="evd">{ev_dates[e]} · {len(L)} matches</span></th>'
                  f'<td class="{tint(8.1 - avgmin, 0, 0.9)}">{avgmin:.2f}</td><td class="{tint(7.6 - worst, 0, 1.0)}">{worst:.2f}</td>'
                  f'<td class="{tint(s8, 0, 9)}">{s8:.1f}</td><td class="{tint(q, 0, 32)}">{q:.0f}%</td>'
                  f'<td class="{tint(cp, 0, 8)}">{cp:.1f}</td><td class="{tint(cam / len(L), 0, 0.45)}">{cam} of {len(L)}</td>'
                  f'<td class="{tint(early / len(L), 0, 0.4)}">{early} of {len(L)}</td></tr>')
    hoot_rows = ""
    for d in full:
        h = hoot_by.get(d["id"])
        if h is None:
            continue
        vm = h["vmin"]
        hoot_rows += (f'<tr><th scope="row">{d["id"]}</th><td class="{tint(7.2 - vm, 0, 1.1) if vm is not None else "c0"}">'
                      f'{"–" if vm is None else f"{vm:.2f}"}</td><td class="{"c3" if h["brown"] else "c0"}">{len(h["brown"]) or "–"}</td>'
                      f'<td class="{"c3" if h["other"] else "c0"}">{", ".join(h["other"]) or "–"}</td></tr>')
    if not hoot_rows:
        hoot_rows = '<tr><td colspan="4" class="mute">no CTRE hoot files were found next to the logs (or owlet is missing)</td></tr>'
    # automatic findings
    key, power, conn = [], [], []
    for e in order:
        L = by[e]
        if not L:
            continue
        under7 = [r["match"] for r in L if r["vmin"] < 7.0]
        power.append(f"<li><b>{e}</b>: lowest battery averages {st.mean(r['vmin'] for r in L):.2f} V, worst {min(r['vmin'] for r in L):.2f} V"
                     + (f"; under 7.0 V in {', '.join(under7)}" if under7 else "; never under 7.0 V") + ".</li>")
        cp = [r for r in L if r["cop"] >= 5]
        conn.append(f"<li><b>{e}</b>: coprocessor dropped {st.mean(r['cop'] for r in L):.1f} times per match"
                    + (f" (5 or more in {len(cp)} matches)" if cp else "") + f"; Quest disconnected for {st.mean(r['q'] for r in L):.0f}% of enabled time on average"
                    + (f", over half in {', '.join(r['match'] for r in L if r['q'] > 50)}" if any(r['q'] > 50 for r in L) else "") + ".</li>")
        big = [f"{r['match']} ({r['cam']:.0f} s)" for r in L if r["cam"] >= 100]
        if big:
            conn.append(f"<li><b>{e}</b>: a camera was out for 100 s or more in {', '.join(big)}.</li>")
        early = [r["match"] for r in L if r["early"]]
        if early:
            conn.append(f"<li><b>{e}</b>: autonomous ended before 19 s in {', '.join(early)} (not a brownout if the battery was normal).</li>")
    rio_logged = [d for d in full if d["rio_logged"]]
    power.append(f"<li>roboRIO brownout flag: recorded in {len(rio_logged)} of {len(full)} logs; set during the match in "
                 f"{sum(d['rio_brownouts'] > 0 for d in rio_logged)} of them.</li>")
    if hoot_by:
        nb = sum(1 for h in hoot_by.values() if h["brown"])
        vms = [h["vmin"] for h in hoot_by.values() if h["vmin"] is not None]
        power.append(f"<li>CTRE hoot: {len(hoot_by)} matches; Kraken bridge-brownout faults in {nb}; lowest Kraken supply "
                     f"{min(vms):.2f} V, under 7 V in {sum(v < 7 for v in vms)}.</li>" if vms else "<li>CTRE hoot loaded.</li>")
        for i, h in hoot_by.items():
            if h["other"]:
                power.append(f"<li><b>{i}</b>: {', '.join(h['other'])}.</li>")
    allv = [r["vmin"] for r in rows]
    key.append(f"<li>{len(full)} full-length matches from {len(events)} events ({', '.join(order)}).</li>")
    key.append(f"<li>Lowest battery voltage in any match: {min(allv):.2f} V ({min(rows, key=lambda r: r['vmin'])['label']} {min(rows, key=lambda r: r['vmin'])['match']}); "
               f"{sum(v < 7 for v in allv)} matches under 7.0 V.</li>")
    key.append(f"<li>Coprocessor NetworkTables drops: {sum(r['cop'] for r in rows)} in total, {st.mean(r['cop'] for r in rows):.1f} per match.</li>")
    key.append(f"<li>Matches with a camera out for 30 s or more: {sum(r['cam'] >= 30 for r in rows)} of {len(rows)}.</li>")
    notes = open(a.notes, encoding="utf-8").read() if os.path.exists(a.notes) else ""
    method = ("<li>Every number comes from the files in the log folder. Enabled time uses the robot's enable state (control word, or the "
              "enabled / autonomous booleans in older logs).</li><li>Battery readings are the PDH voltage the robot code logged, sampled at 5 to 18 Hz, "
              "so brief dips are missed. Camera figures come from each camera's frame counter; link drops from the NetworkTables connection log.</li>"
              "<li>Differences of a few percent are noise at 6 to 16 matches per event. Causes named in the notes are possibilities.</li>")
    tpl = open(os.path.join(HERE, "report_templates", "season.html"), encoding="utf-8").read()
    sub = {"@@TEAM@@": a.team, "@@SEASON@@": a.season,
           "@@INTRO@@": f"{len(logs)} robot logs from {len(events)} events" + (f", {len(hoot_by)} CTRE hoot files" if hoot_by else "") + ".",
           "@@KEY@@": "".join(key), "@@SCORE@@": score, "@@MATRIX@@": matrix_html(order, names_by),
           "@@CHARTV@@": strip_chart(rows, order, "vmin", 6.0, 9.5, [6, 7, 8, 9], "V", thr=7.0),
           "@@CHARTC@@": strip_chart(rows, order, "cop", 0, max(17, max(r["cop"] for r in rows) + 1), [0, 5, 10, 15], "drops", fmt="{:.0f}"),
           "@@CHARTQ@@": strip_chart(rows, order, "q", 0, 100, [0, 25, 50, 75, 100], "%", fmt="{:.0f}"),
           "@@POWER@@": "".join(power), "@@CONN@@": "".join(conn), "@@HOOT@@": hoot_rows, "@@NOTES@@": notes, "@@METHOD@@": method}
    for k, v in sub.items():
        tpl = tpl.replace(k, v)
    p1 = os.path.join(out, "Season_Report.html")
    open(p1, "w", encoding="utf-8").write(tpl)
    # ---------------- comparison
    cmp_rows = [d for d in full]
    curves = {}
    for g in ("new", "old"):
        G = [d for d in cmp_rows if d["group"] == g]
        if G:
            curves[g] = dict(cur=pct_curve([d["cur_samples"] for d in G], CUR_X, True), vol=pct_curve([d["vol_samples"] for d in G], VOLT_X, False))
        else:
            curves[g] = dict(cur=[0] * len(CUR_X), vol=[0] * len(VOLT_X))
    matches = []
    for d in cmp_rows:
        matches.append(dict(id=d["id"], event=d["label"], group=d["group"], typ=d["typ"], file=d["file"], enabled=round(d["enabled"]),
                            complete=d["enabled"] >= 155, stats={k: round(v, 2) for k, v in d["stats"].items()}, windows=d["windows"],
                            cur=d["cur"], vol=d["vol"], cop=d["cop"], quest=d["quest"], stalls=d["stalls"], restarts=d["restarts"],
                            brown=d["brown"]))
    old_events = [label(e) for e in events if e != new_ev]
    hoot_map = {}
    for i, h in hoot_by.items():
        if i.startswith(new_label + " "):
            hoot_map[i.split(" ", 1)[1]] = dict(v=h["vmin"], b=len(h["brown"]), other=", ".join(h["other"]))
    vms = [h["vmin"] for i, h in hoot_by.items() if i.startswith(new_label + " ") and h["vmin"] is not None]
    facts = dict(krakenMin=min(vms) if vms else None, hooted=len(hoot_map), brownMatches=sum(1 for h in hoot_map.values() if h["b"]),
                 otherFaults=sorted({f"{h['other']} ({m})" for m, h in hoot_map.items() if h["other"]}))
    data = dict(curX=CUR_X.tolist(), volX=[round(float(x), 2) for x in VOLT_X], curves=curves, matches=matches, bin=BIN, t0=T0, hoot=hoot_map,
                order=[o for o in order if o != new_label] + [new_label], newName=new_label, newDate=ev_dates[new_label],
                oldNames=", ".join(old_events), oldDates=(ev_dates[old_events[0]] + " to " + ev_dates[old_events[-1]]) if old_events else "", facts=facts)
    t2 = open(os.path.join(HERE, "report_templates", "compare.html"), encoding="utf-8").read()
    n_new = sum(d["group"] == "new" for d in cmp_rows)
    for k, v in {"@@NEWNAME@@": new_label, "@@NLOGS@@": str(len(cmp_rows)), "@@NOLD@@": str(len(cmp_rows) - n_new), "@@NNEW@@": str(n_new),
                 "@@SUBTITLE@@": f"WPILib data log comparison: {new_label} ({ev_dates[new_label]}) against {', '.join(old_events)}.",
                 "@@DATA@@": json.dumps(data, separators=(",", ":"))}.items():
        t2 = t2.replace(k, v)
    p2 = os.path.join(out, "Compare_Robots.html")
    open(p2, "w", encoding="utf-8").write(t2)
    print("wrote", p1)
    print("wrote", p2)


if __name__ == "__main__":
    main()
