"""
Battery performance of one match, so batteries can be compared across matches (no UI, numpy only).

    st = analyze(tel, ml, hoot, battery="2026-C")        # tel: telemetry.Telemetry, ml: core.MatchLog, hoot: hoot.HootLog or None
    st.to_dict()                                           # what the web archive and the battery report keep per match

From the PDH (NT:/SmartDashboard/RobotStatus/_pdh_voltage / _pdh_current, about 5 Hz in the 2026 logs):
  - minimum voltage, and the time spent below 9 V, 8 V and the brownout level (roboRIO 2 default 6.75 V, configurable)
  - voltage at a reference current (150 A) from a straight-line fit V = V0 - R * I over the match: this takes out how hard the drivers
    pushed in that particular match
  - resting voltage before the match and after it (current under REST_A), and the difference
  - peak and RMS current, so a battery that looks bad after a brutal defence match is not blamed for it

Step-response internal resistance, R = -(V2 - V1) / (I2 - I1) over sharp current steps (|dI| >= STEP_A within STEP_S), median over the
match.  The step must be sampled within one or two frames, before the slower electrochemical polarization builds up, so it needs voltage
and current sampled together at 50 Hz or faster.  The 5 Hz PDH topics are not (and are not sampled together), so it comes from the CTRE
hoot when that logs the drive Krakens' SupplyVoltage and SupplyCurrent at 100 Hz (from 2026-10-25): voltage = mean of the drive Krakens'
input, current step = the step of their summed current (sharp steps are the drivetrain launching or stopping; the other loads change
slowly over 20 ms).  What it measures is the battery plus main breaker, the 6 AWG leads, the SB50 and the PDH-to-Kraken wiring: constant on
one robot, so fine for comparing batteries.  A new MK / Duracell ES17-12 class battery alone is about 11-15 mOhm; the whole path is often
20-25 mOhm.  A battery several mOhm above the fleet median is on its way out; all batteries jumping at once means a loose lug or a bad
breaker.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np

BROWNOUT_V = 6.75          # roboRIO 2 default brownout level (configurable on the roboRIO 2)
LEVELS = (9.0, 8.0)        # "time below" levels besides the brownout level
REF_A = 150.0              # reference current for the normalised voltage
REST_A = 10.0              # amps under which the battery counts as resting
STEP_A = 45.0              # an internal-resistance step must change the current by at least this (PDH / Kraken noise and quantization)
STEP_S = 0.02              # ... within this time (two 10 ms Kraken frames)
FAST_DT = 0.025            # median sample spacing that is fast enough for the step method
DRIVE_KRAKENS = ("TalonFX-21", "TalonFX-23", "TalonFX-25", "TalonFX-27")
DEFAULT_BATTERIES = tuple(f"2026-{c}" for c in "ABCDEFGHI")
R_NEW_BATTERY = (11, 15)   # mOhm, a new battery by itself
R_ROBOT_PATH = (20, 25)    # mOhm, battery + breaker + leads + connectors, typical


@dataclass
class BatteryStats:
    battery: str = ""
    window: tuple = (0.0, 0.0)          # log times analysed (the match, else the log)
    v_min: float | None = None
    v_min_t: float | None = None        # seconds after time zero
    v_min_fast: float | None = None     # the drive Krakens' input at 100 Hz (catches dips the 5 Hz PDH misses)
    below: dict = field(default_factory=dict)   # "9 V" / "8 V" / "brownout (6.75 V)" -> seconds
    brownout_v: float = BROWNOUT_V
    v_rest_start: float | None = None
    v_rest_end: float | None = None
    i_peak: float | None = None
    i_rms: float | None = None
    i_mean: float | None = None
    ah: float | None = None             # charge drawn in the window
    fit_v0: float | None = None         # V = V0 - R * I
    fit_r_mohm: float | None = None
    fit_n: int = 0
    v_at_ref: float | None = None       # fitted voltage at REF_A
    ref_a: float = REF_A
    r_step_mohm: float | None = None    # median step-response resistance
    r_step_q: tuple | None = None       # (25th, 75th percentile) mOhm
    r_step_n: int = 0
    r_step_why: str = ""                # why there is no step resistance
    notes: list = field(default_factory=list)

    @property
    def rest_drop(self):
        if self.v_rest_start is None or self.v_rest_end is None:
            return None
        return self.v_rest_start - self.v_rest_end

    def to_dict(self) -> dict:
        d = asdict(self)
        d["rest_drop"] = self.rest_drop
        for k, v in list(d.items()):
            if isinstance(v, float):
                d[k] = round(v, 4)
        d["window"] = [round(x, 3) for x in self.window]
        return d

    def overlay_text(self) -> str:
        """One line for the dashboard video's PDH panel."""
        f = lambda x, fmt: "--" if x is None else fmt.format(x)
        parts = [f"BATTERY {self.battery or '?'}", f"min {f(self.v_min, '{:.1f}')} V",
                 f"@{self.ref_a:g}A {f(self.v_at_ref, '{:.1f}')} V",
                 f"R {f(self.r_step_mohm, '{:.0f}')} mOhm" if self.r_step_mohm is not None else
                 (f"R(fit) {f(self.fit_r_mohm, '{:.0f}')} mOhm" if self.fit_r_mohm is not None else "R --"),
                 f"rest {f(self.v_rest_start, '{:.1f}')}->{f(self.v_rest_end, '{:.1f}')} V",
                 f"peak {f(self.i_peak, '{:.0f}')} A  rms {f(self.i_rms, '{:.0f}')} A"]
        return "  |  ".join(parts)


def _series(sig):
    if sig is None or not len(sig):
        return None
    try:
        v = np.asarray(sig.numeric(), float)
    except (AttributeError, TypeError, ValueError):
        v = np.asarray(sig.v, float)
    t = np.asarray(sig.t, float)
    ok = np.isfinite(v)
    return (t[ok], v[ok]) if ok.any() else None


def _held(t, v, a, b):
    """Samples inside [a, b] with the value held at a, and the time each one lasts (sample-and-hold)."""
    i0 = max(int(np.searchsorted(t, a, side="right")) - 1, 0)
    i1 = int(np.searchsorted(t, b, side="right"))
    tt, vv = np.clip(t[i0:i1], a, b), v[i0:i1]
    if not len(tt):
        return None
    d = np.diff(np.append(tt, b))
    return tt, vv, d


def _hold(tv, x):
    """Values of a series at times x, sample-and-hold: NetworkTables logs a value only when it changes, so the last one holds until the
    next (a straight line between two far-apart samples would invent a load that was never there)."""
    t, v = tv
    i = np.clip(np.searchsorted(t, x, side="right") - 1, 0, len(t) - 1)
    return v[i]


def _rest(tv, ti, a, b):
    """Median voltage in [a, b] while the current is under REST_A (the battery resting)."""
    if tv is None:
        return None
    t, v = tv
    m = (t >= a) & (t <= b)
    if m.sum() < 2:
        return None
    if ti is not None and len(ti[0]):
        cur = _hold(ti, t[m])
        m2 = np.abs(cur) < REST_A
        return float(np.median(v[m][m2])) if m2.sum() >= 2 else None      # under load it is not a resting voltage
    return float(np.median(v[m]))


def analyze(tel, ml, hoot=None, battery: str = "", brownout_v: float = BROWNOUT_V, ref_a: float = REF_A) -> BatteryStats:
    st = BatteryStats(battery=battery or "", brownout_v=float(brownout_v), ref_a=float(ref_a))
    a = float(ml.t_ref)
    b = a + float(ml.match_len) if ml.has_match else float(ml.t_last)
    b = max(b, a + 1.0)
    st.window = (a, b)
    a0 = float(getattr(tel, "a0", a))
    sv, si = _series(tel.by_key.get("pdh_v")), _series(tel.by_key.get("pdh_a"))

    # ---------------------------------------------------------------- voltage over the match
    if sv is not None:
        h = _held(*sv, a, b)
        if h is not None:
            tt, vv, d = h
            i = int(np.argmin(vv))
            st.v_min, st.v_min_t = float(vv[i]), float(tt[i] - a0)
            for lvl in LEVELS:
                st.below[f"{lvl:g} V"] = round(float(d[vv < lvl].sum()), 2)
            st.below[f"brownout {brownout_v:g} V"] = round(float(d[vv < brownout_v].sum()), 2)
        st.v_rest_start = _rest(sv, si, a - 15.0, a - 0.3)
        if st.v_rest_start is None:                       # never the log's first second: a log that restarted mid-match starts under load
            st.notes.append("no resting voltage before the match (the log starts at or after the match start, or the robot was drawing current)")
        st.v_rest_end = _rest(sv, si, b + 0.5, b + 12.0)
        if st.v_rest_end is None:
            st.v_rest_end = _rest(sv, si, b - 2.0, b)
            if st.v_rest_end is not None:
                st.notes.append("end voltage from the last 2 s of the window (the log stops at the end of the match)")
    else:
        st.notes.append("battery voltage (_pdh_voltage) is not in this log")

    # ---------------------------------------------------------------- current
    if si is not None:
        h = _held(*si, a, b)
        if h is not None:
            tt, ii, d = h
            ii = np.abs(ii)
            st.i_peak = float(ii.max())
            tot = float(d.sum())
            if tot > 0:
                st.i_rms = float(np.sqrt((ii ** 2 * d).sum() / tot))
                st.i_mean = float((ii * d).sum() / tot)
                st.ah = float((ii * d).sum() / 3600.0)
    else:
        st.notes.append("total current (_pdh_current) is not in this log")

    # ---------------------------------------------------------------- V = V0 - R * I over the match, at REF_A
    if sv is not None and si is not None:
        t, v = sv
        m = (t >= a) & (t <= b)
        t, v = t[m], v[m]
        if len(t) >= 20:
            ci = np.searchsorted(si[0], t)
            near = np.minimum(np.abs(si[0][np.clip(ci, 0, len(si[0]) - 1)] - t),
                              np.abs(si[0][np.clip(ci - 1, 0, len(si[0]) - 1)] - t))
            ok = near <= 0.15                                  # a current sample close enough to pair with
            if ok.sum() >= 20:
                I = np.abs(np.interp(t[ok], si[0], si[1]))
                V = v[ok]
                if np.ptp(I) >= 60.0:
                    w = np.ones_like(I)
                    for _ in range(3):                         # Huber-style reweighting: a few unpaired spikes must not tilt the line
                        A = np.vstack([np.ones_like(I), -I]).T
                        coef, *_ = np.linalg.lstsq(A * w[:, None], V * w, rcond=None)
                        res = V - A @ coef
                        s = 1.4826 * np.median(np.abs(res)) + 1e-6
                        w = np.minimum(1.0, 1.5 * s / np.maximum(np.abs(res), 1e-9))
                    v0, r = float(coef[0]), float(coef[1])
                    if 0.0 < r < 0.2 and 9.0 < v0 < 14.5:
                        st.fit_v0, st.fit_r_mohm, st.fit_n = v0, r * 1000.0, int(ok.sum())
                        st.v_at_ref = v0 - r * ref_a
                    else:
                        st.notes.append(f"the voltage-vs-current fit came out unphysical (V0 {v0:.1f} V, R {r * 1000:.0f} mOhm): not used")
                else:
                    st.notes.append(f"the current only spanned {np.ptp(I):.0f} A: too little to fit voltage against current")

    # ---------------------------------------------------------------- step-response internal resistance (needs fast, paired V and I)
    _step(st, hoot, a, b)
    return st


def _step(st: BatteryStats, hoot, a, b):
    if hoot is None:
        st.r_step_why = ("needs voltage and current sampled together at 50 Hz or more: the PDH topics are logged at about 5 Hz and "
                         "there is no CTRE hoot for this log")
        return
    sigs = [(hoot.sig(d, "SupplyVoltage"), hoot.sig(d, "SupplyCurrent")) for d in DRIVE_KRAKENS]
    sigs = [(v, i) for v, i in sigs if v is not None and i is not None and len(v) > 50 and len(i) > 50]
    if not sigs:
        st.r_step_why = "the hoot has no drive-Kraken SupplyVoltage / SupplyCurrent"
        return
    t_ref = np.asarray(sigs[0][0].t, float)
    m = (t_ref >= a) & (t_ref <= b)
    t_ref = t_ref[m]
    if len(t_ref) < 100 or float(np.median(np.diff(t_ref))) > FAST_DT:
        dt = float(np.median(np.diff(np.asarray(sigs[0][0].t, float)))) if len(sigs[0][0]) > 1 else 0.0
        st.r_step_why = (f"the hoot logs the Krakens' supply voltage and current every {dt * 1000:.0f} ms; the step method needs "
                         f"{FAST_DT * 1000:.0f} ms or faster (the robot code has logged them at 100 Hz since 2026-10-25)")
        return
    V = np.mean([np.interp(t_ref, np.asarray(v.t, float), np.asarray(v.v, float)) for v, _ in sigs], axis=0)
    I = np.sum([np.interp(t_ref, np.asarray(i.t, float), np.asarray(i.v, float)) for _, i in sigs], axis=0)
    st.v_min_fast = float(V.min())
    dt = float(np.median(np.diff(t_ref)))
    k = max(1, int(round(STEP_S / dt)))
    dI = I[k:] - I[:-k]
    dV = V[k:] - V[:-k]
    cand = np.flatnonzero(np.abs(dI) >= STEP_A)
    rs, used = [], []
    for j in cand[np.argsort(-np.abs(dI[cand]))]:            # the biggest steps first, one per 100 ms
        if any(abs(t_ref[j] - x) < 0.1 for x in used):
            continue
        used.append(t_ref[j])
        r = -dV[j] / dI[j]
        if 0.0 < r < 0.2:
            rs.append((t_ref[j], r))
    if len(rs) < 3:
        st.r_step_why = f"only {len(rs)} current step(s) of {STEP_A:g} A or more within {STEP_S * 1000:.0f} ms in this match"
        return
    r = np.array([x[1] for x in rs]) * 1000.0
    st.r_step_mohm = float(np.median(r))
    st.r_step_q = (float(np.percentile(r, 25)), float(np.percentile(r, 75)))
    st.r_step_n = len(r)


def severity(st: BatteryStats) -> dict:
    """bad / warn / ok per headline number (for the report's colours)."""
    out = {}
    if st.v_min is not None:
        out["v_min"] = "bad" if st.v_min < st.brownout_v else "warn" if st.v_min < 8.0 else "ok"
    if st.r_step_mohm is not None:
        out["r_step"] = "warn" if st.r_step_mohm > R_ROBOT_PATH[1] + 5 else "ok"
    if st.v_at_ref is not None:
        out["v_at_ref"] = "warn" if st.v_at_ref < 9.0 else "ok"
    d = st.rest_drop
    if d is not None:
        out["rest_drop"] = "warn" if d > 0.8 else "ok"
    return out
