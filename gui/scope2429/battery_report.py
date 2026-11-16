"""
Battery report: the battery numbers of many matches side by side, one self-contained HTML page (inline SVG charts, no network).

    python battery_report.py "C:/Users/<you>/Documents/WPILOG_2026/2026DAL" [out.html]      # logs with a battery picked in Scope2429

The desktop app (Battery report... button) and the web archive (/batteries) call `render(records)` with one record per match:
    {"label": "DALY Q13", "when": "2026-04-30 14:40", "battery": "2026-C", "stats": battery.BatteryStats.to_dict(), "href": optional link}

What it answers:
  - which battery is wearing out: step-response internal resistance per battery over time, against the fleet median (+3 mOhm = suspect)
  - which battery sags most for the same load: voltage at 150 A from each match's V-I fit (a hard match does not make it look worse)
  - was it the battery or the match: lowest voltage against RMS current, coloured by battery
  - charging: resting voltage before each match, and how far it fell per amp-hour drawn
"""
from __future__ import annotations

import html
import os
import statistics as stx
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "helpers"))

SUSPECT_MOHM = 3.0           # a battery this far above the fleet median resistance is suspect
PALETTE = ["#2a78d6", "#e8730c", "#1f9e6e", "#c4356b", "#7b5cd6", "#9a7a12", "#0d9bb5", "#d14b2a", "#5c7a29", "#8a5a44", "#4a6fa5", "#b0558f"]


def _med(xs):
    xs = [x for x in xs if x is not None]
    return stx.median(xs) if xs else None


def _f(x, fmt="{:.2f}"):
    return "--" if x is None else fmt.format(x)


def summarize(records):
    """Per battery: medians and a verdict against the fleet."""
    by = {}
    for r in records:
        by.setdefault(r.get("battery") or "?", []).append(r)
    fleet_r = _med([r["stats"].get("r_step_mohm") for r in records])
    fleet_fit = _med([r["stats"].get("fit_r_mohm") for r in records])
    out = []
    for b, rs in sorted(by.items()):
        s = [r["stats"] for r in rs]
        row = {"battery": b, "n": len(rs), "r_step": _med([x.get("r_step_mohm") for x in s]), "n_step": sum(1 for x in s if x.get("r_step_mohm") is not None),
               "fit_r": _med([x.get("fit_r_mohm") for x in s]), "v_at_ref": _med([x.get("v_at_ref") for x in s]),
               "v_min": _med([x.get("v_min") for x in s]), "v_min_worst": min((x["v_min"] for x in s if x.get("v_min") is not None), default=None),
               "rest_start": _med([x.get("v_rest_start") for x in s]), "below8": sum((x.get("below") or {}).get("8 V", 0) for x in s),
               "brown": sum(v for x in s for k, v in (x.get("below") or {}).items() if k.startswith("brownout")),
               "last": max((r.get("when") or "" for r in rs), default="")}
        verdict, why = "ok", ""
        if row["r_step"] is not None and fleet_r is not None and row["r_step"] - fleet_r >= SUSPECT_MOHM:
            verdict, why = "suspect", f"step resistance {row['r_step'] - fleet_r:+.1f} mOhm above the fleet median"
        elif row["r_step"] is None and row["fit_r"] is not None and fleet_fit is not None and row["fit_r"] - fleet_fit >= SUSPECT_MOHM and row["n"] >= 3:
            verdict, why = "watch", f"fitted resistance {row['fit_r'] - fleet_fit:+.1f} mOhm above the fleet median (5 Hz data: a hint, not proof)"
        if b == "?":
            verdict, why = "unknown", "matches with no battery picked"
        row["verdict"], row["why"] = verdict, why
        out.append(row)
    return out, fleet_r, fleet_fit


# ---------------------------------------------------------------------------------------------------- SVG charts
W, H, ML, MR, MT, MB = 920, 320, 58, 16, 18, 96


def _axis(y0, y1, unit, x_labels):
    """Frame, horizontal grid lines and labels; returns (svg parts, y->px)."""
    span = (y1 - y0) or 1.0
    py = lambda v: MT + (H - MT - MB) * (1 - (v - y0) / span)
    parts = []
    step = _nice(span / 5)
    v = (int(y0 / step) + (1 if y0 > 0 else 0)) * step
    while v <= y1 + 1e-9:
        y = py(v)
        parts.append(f'<line x1="{ML}" x2="{W - MR}" y1="{y:.1f}" y2="{y:.1f}" class="grid"/>'
                     f'<text x="{ML - 6}" y="{y + 4:.1f}" class="tick" text-anchor="end">{v:g}</text>')
        v += step
    parts.append(f'<text x="14" y="{MT + (H - MT - MB) / 2:.0f}" class="unit" transform="rotate(-90 14 {MT + (H - MT - MB) / 2:.0f})" '
                 f'text-anchor="middle">{html.escape(unit)}</text>')
    n = len(x_labels)
    every = max(1, n // 24)
    for i, lab in enumerate(x_labels):
        if i % every == 0:
            x = _px(i, n)
            parts.append(f'<text x="{x:.1f}" y="{H - MB + 14}" class="tick" text-anchor="end" transform="rotate(-45 {x:.1f} {H - MB + 14})">'
                         f'{html.escape(lab)}</text>')
    return parts, py


def _nice(x):
    import math
    if x <= 0:
        return 1.0
    e = 10 ** math.floor(math.log10(x))
    for m in (1, 2, 2.5, 5, 10):
        if x <= m * e:
            return m * e
    return 10 * e


def _px(i, n):
    return ML + (W - ML - MR) * ((i + 0.5) / max(n, 1))


def _range(vals, pad=0.08, lo=None, hi=None):
    vals = [v for v in vals if v is not None]
    if not vals:
        return 0.0, 1.0
    a, b = min(vals), max(vals)
    if lo is not None:
        a = min(a, lo)
    if hi is not None:
        b = max(b, hi)
    d = (b - a) or max(abs(b) * 0.1, 1.0)
    return a - d * pad, b + d * pad


def _svg(parts, title):
    return (f'<figure><figcaption>{html.escape(title)}</figcaption><svg viewBox="0 0 {W} {H}" role="img" '
            f'aria-label="{html.escape(title)}">' + "".join(parts) + "</svg></figure>")


def match_chart(records, key, unit, title, colors, ref=None, ref_label="", band=None, err=None):
    """One dot per match in time order, coloured by battery; optional reference line and shaded band."""
    vals = [r["stats"].get(key) for r in records]
    lo, hi = (band if band else (None, None))
    y0, y1 = _range(vals + ([ref] if ref is not None else []), lo=lo, hi=hi)
    parts, py = _axis(y0, y1, unit, [r["label"] for r in records])
    if band:
        parts.append(f'<rect x="{ML}" width="{W - ML - MR}" y="{py(band[1]):.1f}" height="{py(band[0]) - py(band[1]):.1f}" class="band"/>')
    if ref is not None:
        parts.append(f'<line x1="{ML}" x2="{W - MR}" y1="{py(ref):.1f}" y2="{py(ref):.1f}" class="ref"/>'
                     f'<text x="{W - MR - 4}" y="{py(ref) - 5:.1f}" class="tick" text-anchor="end">{html.escape(ref_label)}</text>')
    n = len(records)
    for i, (r, v) in enumerate(zip(records, vals)):
        if v is None:
            continue
        x, y = _px(i, n), py(v)
        c = colors.get(r.get("battery") or "?", "#888")
        if err:
            q = r["stats"].get(err)
            if q:
                parts.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{py(q[0]):.1f}" y2="{py(q[1]):.1f}" stroke="{c}" stroke-width="2" opacity=".55"/>')
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="{c}"><title>{html.escape(r["label"])}  {html.escape(r.get("battery") or "?")}: '
                     f'{v:.2f} {html.escape(unit)}</title></circle>')
    return _svg(parts, title)


def scatter(records, kx, ky, ux, uy, title, colors):
    """y against x per match (e.g. lowest voltage against RMS current), coloured by battery."""
    pts = [(r["stats"].get(kx), r["stats"].get(ky), r) for r in records]
    pts = [p for p in pts if p[0] is not None and p[1] is not None]
    if not pts:
        return ""
    x0, x1 = _range([p[0] for p in pts])
    y0, y1 = _range([p[1] for p in pts])
    parts, py = _axis(y0, y1, uy, [])
    px = lambda v: ML + (W - ML - MR) * (v - x0) / ((x1 - x0) or 1)
    step = _nice((x1 - x0) / 6)
    v = (int(x0 / step) + 1) * step
    while v <= x1:
        parts.append(f'<text x="{px(v):.1f}" y="{H - MB + 16}" class="tick" text-anchor="middle">{v:g}</text>')
        v += step
    parts.append(f'<text x="{(ML + W - MR) / 2:.0f}" y="{H - MB + 36}" class="unit" text-anchor="middle">{html.escape(ux)}</text>')
    for x, y, r in pts:
        c = colors.get(r.get("battery") or "?", "#888")
        parts.append(f'<circle cx="{px(x):.1f}" cy="{py(y):.1f}" r="6" fill="{c}" opacity=".85"><title>{html.escape(r["label"])}  '
                     f'{html.escape(r.get("battery") or "?")}: {x:.0f} {html.escape(ux)}, {y:.2f} {html.escape(uy)}</title></circle>')
    return _svg(parts, title)


def battery_bars(rows, key, unit, title, colors, ref=None, ref_label=""):
    """One bar per battery (its median), with the fleet reference line."""
    rows = [r for r in rows if r.get(key) is not None]
    if not rows:
        return ""
    vals = [r[key] for r in rows]
    y0, y1 = _range(vals + ([ref] if ref is not None else []), pad=0.15)
    y0 = 0.0                                     # a bar is read by its length: always from zero
    parts, py = _axis(y0, y1, unit, [])
    n = len(rows)
    bw = min(60, (W - ML - MR) / n * 0.6)
    for i, r in enumerate(rows):
        x = _px(i, n)
        c = colors.get(r["battery"], "#888")
        top, base = py(r[key]), py(max(y0, 0) if y0 <= 0 else y0)
        parts.append(f'<rect x="{x - bw / 2:.1f}" y="{min(top, base):.1f}" width="{bw:.1f}" height="{abs(base - top):.1f}" fill="{c}" rx="3">'
                     f'<title>{html.escape(r["battery"])}: {r[key]:.2f} {html.escape(unit)} ({r["n"]} matches)</title></rect>'
                     f'<text x="{x:.1f}" y="{H - MB + 16}" class="tick" text-anchor="middle">{html.escape(r["battery"])}</text>'
                     f'<text x="{x:.1f}" y="{min(top, base) - 5:.1f}" class="val" text-anchor="middle">{r[key]:.1f}</text>')
    if ref is not None:
        parts.append(f'<line x1="{ML}" x2="{W - MR}" y1="{py(ref):.1f}" y2="{py(ref):.1f}" class="ref"/>'
                     f'<text x="{W - MR - 4}" y="{py(ref) - 5:.1f}" class="tick" text-anchor="end">{html.escape(ref_label)}</text>')
    return _svg(parts, title)


# ---------------------------------------------------------------------------------------------------- page
def render(records, title="Battery report") -> str:
    import battery as B
    records = sorted(records, key=lambda r: r.get("when") or "")
    e = html.escape
    rows, fleet_r, fleet_fit = summarize(records)
    names = [r["battery"] for r in rows]
    colors = {b: PALETTE[i % len(PALETTE)] for i, b in enumerate(n for n in names if n != "?")}
    colors["?"] = "#9aa0a6"
    legend = '<div class="legend">' + "".join(f'<span><i style="background:{colors[b]}"></i>{e(b)}</span>' for b in names) + "</div>"
    n_step = sum(1 for r in records if r["stats"].get("r_step_mohm") is not None)
    h = ['<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">',
         f"<title>{e(title)}</title><style>{CSS}</style></head><body><main>",
         f"<h1>{e(title)}</h1>",
         f'<p class="sub">{len(records)} matches, {len([b for b in names if b != "?"])} batteries. Made {time.strftime("%Y-%m-%d %H:%M")} by Scope2429. '
         f"Hover a dot for the match.</p>"]
    if not records:
        h.append('<p class="note">No matches with battery data yet. Pick the battery for each log (Battery selector) and come back.</p>')
        return "\n".join(h + ["</main></body></html>"])
    # verdicts first
    known = [b for b in names if b != "?"]
    flagged = [r for r in rows if r["verdict"] in ("suspect", "watch")]
    n_unknown = next((r["n"] for r in rows if r["battery"] == "?"), 0)
    if not known:
        h.append('<div class="cards"><div class="card">No battery is recorded for these logs, so batteries cannot be compared yet: the charts '
                 "show each match's numbers. Pick the battery for each new log (Battery selector) and they group by battery.</div></div>")
    elif flagged:
        h.append('<div class="cards">' + "".join(f'<div class="card {r["verdict"]}"><b>{e(r["battery"])}</b> {e(r["verdict"])}<br>{e(r["why"])}</div>'
                                                 for r in flagged) + "</div>")
    else:
        h.append('<div class="cards"><div class="card ok">No battery stands out from the fleet' +
                 ("" if n_step else " (step resistance needs the 100 Hz hoot logs, from 2026-10-25 on; the fitted numbers are the guide until then)") +
                 ".</div></div>")
    if known and n_unknown:
        h.append(f'<p class="note">{n_unknown} match(es) have no battery recorded; they are listed as "?" and left out of the fleet verdicts\' '
                 "grouping, never guessed.</p>")
    h.append("<h2>Batteries</h2><table><thead><tr><th>Battery</th><th>Matches</th><th>Step R mOhm</th><th>Fit R mOhm</th><th>V @ 150 A</th>"
             "<th>Lowest V (median / worst)</th><th>Rest V before</th><th>s below 8 V</th><th>s below brownout</th><th>Last used</th><th>Verdict</th></tr></thead><tbody>")
    for r in rows:
        n_note = f" ({r['n_step']})" if r["n_step"] else ""
        h.append(f'<tr><td><i class="sw" style="background:{colors.get(r["battery"], "#888")}"></i>{e(r["battery"])}</td><td>{r["n"]}</td>'
                 f'<td>{_f(r["r_step"], "{:.1f}")}{n_note}</td><td>{_f(r["fit_r"], "{:.1f}")}</td>'
                 f'<td>{_f(r["v_at_ref"])}</td><td>{_f(r["v_min"])} / {_f(r["v_min_worst"])}</td><td>{_f(r["rest_start"])}</td>'
                 f'<td>{r["below8"]:.0f}</td><td>{r["brown"]:.1f}</td><td>{e(r["last"][:16])}</td><td class="{r["verdict"]}">{e(r["verdict"])}</td></tr>')
    h.append("</tbody></table>")
    h.append(f'<p class="note">Fleet median: step resistance {_f(fleet_r, "{:.1f}")} mOhm, fitted resistance {_f(fleet_fit, "{:.1f}")} mOhm. '
             f"A battery {SUSPECT_MOHM:g} mOhm or more above the fleet median is suspect; every battery rising at once points to the robot "
             f"(a loose lug, a bad breaker), not the batteries.</p>")
    h.append("<h2>Is a battery wearing out?</h2>" + legend)
    if n_step:
        h.append(match_chart(records, "r_step_mohm", "mOhm", "Internal resistance per match (step response; bar = middle half of the steps)",
                             colors, fleet_r, "fleet median", band=B.R_ROBOT_PATH, err="r_step_q"))
    many = len(known) >= 2                       # bars per battery only say something with two or more known batteries
    if many:
        h.append(battery_bars(rows, "r_step" if n_step else "fit_r", "mOhm",
                              ("Median step resistance per battery" if n_step else "Median fitted resistance per battery (5 Hz V-I fit)"),
                              colors, fleet_r if n_step else fleet_fit, "fleet median"))
    h.append(match_chart(records, "fit_r_mohm", "mOhm", "Fitted resistance per match (slope of voltage against current)", colors, fleet_fit, "fleet median"))
    h.append("<h2>Which battery holds up under the same load?</h2>" + legend)
    h.append(match_chart(records, "v_at_ref", "volts", "Voltage at 150 A per match (from the fit: a hard match does not make it look worse)", colors))
    if many:
        h.append(battery_bars(rows, "v_at_ref", "volts", "Median voltage at 150 A per battery", colors))
    h.append("<h2>The battery or the match?</h2>" + legend)
    h.append(scatter(records, "i_rms", "v_min", "A RMS (how hard the match was)", "volts", "Lowest voltage against RMS current: low and to the left = the battery", colors))
    h.append(match_chart(records, "v_min", "volts", "Lowest voltage per match", colors, 8.0, "8 V"))
    h.append("<h2>Charging</h2>" + legend)
    h.append(match_chart(records, "v_rest_start", "volts", "Resting voltage before the match (a battery put in not fully charged shows low)", colors, 12.6, "12.6 V"))
    h.append(scatter(records, "ah", "rest_drop", "Ah drawn in the match", "volts", "Resting voltage lost against charge drawn: above the others = less capacity", colors))
    h.append("<h2>Matches</h2><table><thead><tr><th>Match</th><th>When</th><th>Battery</th><th>Lowest V</th><th>s &lt; 9 V</th><th>s &lt; 8 V</th>"
             "<th>s &lt; brownout</th><th>V @ 150 A</th><th>Step R</th><th>Fit R</th><th>Rest before / after</th><th>Peak / RMS A</th></tr></thead><tbody>")
    for r in reversed(records):
        s = r["stats"]
        bl = s.get("below") or {}
        brown = next((v for k, v in bl.items() if k.startswith("brownout")), None)
        lab = f'<a href="{e(r["href"])}">{e(r["label"])}</a>' if r.get("href") else e(r["label"])
        h.append(f'<tr><td>{lab}</td><td>{e((r.get("when") or "")[:16])}</td><td><i class="sw" style="background:{colors.get(r.get("battery") or "?", "#888")}"></i>'
                 f'{e(r.get("battery") or "?")}</td><td>{_f(s.get("v_min"))}</td><td>{_f(bl.get("9 V"), "{:.1f}")}</td><td>{_f(bl.get("8 V"), "{:.1f}")}</td>'
                 f'<td>{_f(brown, "{:.1f}")}</td><td>{_f(s.get("v_at_ref"))}</td><td>{_f(s.get("r_step_mohm"), "{:.1f}")}</td><td>{_f(s.get("fit_r_mohm"), "{:.1f}")}</td>'
                 f'<td>{_f(s.get("v_rest_start"))} / {_f(s.get("v_rest_end"))}</td><td>{_f(s.get("i_peak"), "{:.0f}")} / {_f(s.get("i_rms"), "{:.0f}")}</td></tr>')
    h.append("</tbody></table>")
    h.append('<p class="note">Voltage and current: the PDH topics in the robot log (about 5 Hz, so short dips are missed). Step resistance: the drive Krakens\' '
             "supply voltage and current in the CTRE hoot at 100 Hz (logged since 2026-10-25), the median over steps of 45 A or more within 20 ms; it "
             "includes the breaker, leads, SB50 and the wiring to the Krakens, so compare batteries on the same robot. "
             f"A new battery alone is {B.R_NEW_BATTERY[0]}-{B.R_NEW_BATTERY[1]} mOhm; the whole path is often {B.R_ROBOT_PATH[0]}-{B.R_ROBOT_PATH[1]} mOhm "
             "(the shaded band).</p></main></body></html>")
    return "\n".join(h)


CSS = """
:root { --bg:#ffffff; --fg:#1d2127; --dim:#5f6873; --line:#d9dde3; --card:#f6f8fa; --grid:#e6e9ee; --band:rgba(31,158,110,.10);
        --bad:#b00020; --warn:#a85a00; --ok:#1b7a34; --badbg:#fdecee; --warnbg:#fff4e0; --okbg:#e9f6ec; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#14171b; --fg:#e4e7eb; --dim:#98a1ad; --line:#2c323a; --card:#1c2127; --grid:#262c34; --band:rgba(95,208,122,.10);
          --bad:#ff6b7d; --warn:#ffb454; --ok:#5fd07a; --badbg:#3a1a20; --warnbg:#3a2a12; --okbg:#173022; } }
body { background:var(--bg); color:var(--fg); font:15px/1.45 system-ui,Segoe UI,Roboto,sans-serif; margin:0; }
main { max-width:980px; margin:0 auto; padding:20px 16px 48px; }
h1 { margin:0 0 4px; font-size:26px; } h2 { margin:30px 0 8px; font-size:19px; border-bottom:1px solid var(--line); padding-bottom:4px; }
.sub, .note { color:var(--dim); } .note { font-size:13px; }
table { border-collapse:collapse; width:100%; font-size:13px; margin:8px 0; display:block; overflow-x:auto; }
th, td { text-align:left; padding:5px 8px; border-bottom:1px solid var(--line); white-space:nowrap; font-variant-numeric:tabular-nums; }
th { background:var(--card); font-weight:600; }
td.suspect { color:var(--bad); font-weight:600; } td.watch { color:var(--warn); font-weight:600; } td.ok { color:var(--ok); } td.unknown { color:var(--dim); }
.cards { display:flex; flex-wrap:wrap; gap:10px; margin:12px 0; }
.card { padding:10px 14px; border-radius:8px; background:var(--card); border-left:4px solid var(--line); }
.card.suspect { background:var(--badbg); border-color:var(--bad); } .card.watch { background:var(--warnbg); border-color:var(--warn); }
.card.ok { background:var(--okbg); border-color:var(--ok); }
figure { margin:14px 0; } figcaption { font-weight:600; font-size:14px; margin-bottom:4px; }
svg { width:100%; height:auto; display:block; }
svg .grid { stroke:var(--grid); stroke-width:1; } svg .ref { stroke:var(--dim); stroke-width:1.5; stroke-dasharray:6 4; }
svg .band { fill:var(--band); } svg text { fill:var(--dim); font-size:11px; } svg .unit { font-size:12px; } svg .val { fill:var(--fg); font-size:12px; font-weight:600; }
.legend { display:flex; flex-wrap:wrap; gap:12px; font-size:13px; margin:6px 0; }
.legend i, i.sw { display:inline-block; width:11px; height:11px; border-radius:50%; margin-right:5px; vertical-align:-1px; }
a { color:inherit; }
"""


# ---------------------------------------------------------------------------------------------------- command line / desktop
STATS_VER = 3      # bump when battery.analyze changes, so cached numbers are worked out again


def analyze_log(path, battery_id="", hoot_search=None):
    """battery.BatteryStats of one log (with its hoot, when one is found next to it)."""
    import battery
    import core
    import hoot as H
    import telemetry as T
    ml = core.load_match_log(str(path), [])
    tel = T.Telemetry(str(path), ml)
    hl = None
    try:
        hs = H.find_hoots_for_log(str(path), hoot_search or os.path.dirname(str(path)))
        if hs:
            conv = [H.convert_hoot(h) for h in hs]
            hl = H.load_many([w for w, _ in conv], str(path))
    except Exception:  # noqa - no hoot: no step resistance, everything else still works
        hl = None
    return ml, battery.analyze(tel, ml, hl, battery_id)


def record(ml, stats, href=None):
    when = ""
    if getattr(ml, "log_utc", None):
        when = time.strftime("%Y-%m-%d %H:%M", time.localtime(ml.log_utc))
    else:
        import re
        m = re.search(r"(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})", os.path.basename(ml.path))
        if m:
            when = "{}-{}-{} {}:{}".format(*m.groups()[:5])
    import re
    lab = getattr(ml, "match_label", "") or ""
    if re.fullmatch(r"[PQE]\d+", lab):
        lab = f"{getattr(ml, 'event_code', '') or ''} {lab}".strip()          # Q7 alone does not say which event
    elif not getattr(ml, "has_match", False):
        lab = f"practice {when[5:16]}" if when else os.path.basename(ml.path)
    return {"label": lab, "when": when, "battery": stats.battery, "stats": stats.to_dict(), "href": href}


def main():
    import core
    folder = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(folder, "Battery_Report.html")
    per = core.load_settings().get("per_log", {})
    recs = []
    for name in sorted(os.listdir(folder)):
        if not name.endswith(".wpilog"):
            continue
        bat = per.get(name, {}).get("battery", "")
        ml, st = analyze_log(os.path.join(folder, name), bat)
        recs.append(record(ml, st))
        print(f"{name}: {bat or '?'}  min {_f(st.v_min)} V  @150A {_f(st.v_at_ref)} V  step R {_f(st.r_step_mohm, '{:.1f}')} mOhm")
    with open(out, "w", encoding="utf-8") as f:
        f.write(render(recs))
    print("wrote", out)


if __name__ == "__main__":
    main()
