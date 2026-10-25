"""
Drive-motor feedforward (kS / kV / kA) estimated from the CTRE hoot of a match, with no special test.

The model is the one WPILib SysId fits for a velocity mechanism:   volts = kS * sign(v) + kV * v + kA * a
v is the motor's Velocity (rotations per second, as the hoot logs it), a its derivative, volts the applied MotorVoltage.

What a match hoot can and cannot tell:
  * kV comes out well (14 CAEL matches: within ~2% between matches).  kS is the weakest term (a spread of 0.1-0.17 V
    between matches, so only its order of magnitude means something): the voltage is the output of a closed loop, so it
    also contains kP * error, which is not zero even in the steady part.
  * kA is stable between matches but biased the same way (the closed loop and the robot's own acceleration limit make
    acceleration and velocity error go together), and the wheels slip on carpet.  This robot's hoot also logs
    MotorVoltage only every 250 ms (Velocity every 10 ms).  Treat kA as an estimate and confirm all three with a shop
    test (SYSID_SHOP_GUIDE.md).  Logging MotorVoltage at 100 Hz or more would tighten everything.
  * The standard errors are those of the regression, which treats every sample as independent: optimistic.  The spread
    between matches (`describe`, `fit_folder`'s rows) is the honest number.

    h = hoot.HootLog.load(path, robot_log)
    fits = fit_hoot(h, windows)                 # {device: Fit}
    pooled = fit_samples(pool([s1, s2, ...]))   # several matches, one motor

Used by report.py (the "Drive motor feedforward" section) and by `python sysid.py <logs folder>` (all matches at once).
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "helpers"))     # hoot, core, telemetry live in helpers/

V_MIN_SUPPLY = 9.0          # volts: below this the battery sags, the motor does not see the voltage it was told
SAT_FRAC = 0.95             # |applied| at or above this share of the supply = the output was clipped, no information
SPEED_MIN = 0.3             # rps: sign(v) is meaningless near standstill
ACC_WIN = 0.04              # s: half width of the centered difference that gives the acceleration
ACC_STEADY = 3.0            # rps/s: "steady" samples for the kS / kV fit
ACC_MAX = 60.0              # rps/s: faster than any real wheel acceleration here = a glitch or a wheel slipping
ACC_WIDE = 0.5              # a kA whose standard error is above this share of kA is called undetermined


@dataclass
class Samples:
    """One motor's regression rows (one per MotorVoltage sample that passed the filters)."""
    device: str = ""
    v: np.ndarray = field(default_factory=lambda: np.zeros(0))       # rps
    a: np.ndarray = field(default_factory=lambda: np.zeros(0))       # rps / s
    volts: np.ndarray = field(default_factory=lambda: np.zeros(0))   # applied volts
    t: np.ndarray = field(default_factory=lambda: np.zeros(0))       # robot log time
    n_all: int = 0                                                   # MotorVoltage samples seen while enabled

    def __len__(self):
        return len(self.v)


@dataclass
class Fit:
    device: str = ""
    n: int = 0
    n_all: int = 0
    ks: float | None = None
    kv: float | None = None
    ka: float | None = None
    se_ks: float | None = None
    se_kv: float | None = None
    se_ka: float | None = None
    r2: float | None = None
    rms: float | None = None            # volts, residual of the three-term fit
    v_max: float | None = None          # rps, 95th percentile of |v| (how fast the data reaches)
    a_span: float | None = None         # rps/s, 95th percentile of |a|
    ka_ok: bool = False                 # kA is determined well enough to use
    note: str = ""

    def per_ms(self, scale):
        """(kS, kV, kA) in V, V/(m/s), V/(m/s^2) for a wheel that travels `scale` metres per motor rps."""
        if not scale or self.kv is None:
            return None
        return self.ks, self.kv / scale, (self.ka / scale if self.ka is not None else None)


# ------------------------------------------------------------------------------------------ samples
def _accel(tv, xv, t):
    """Velocity and acceleration at times t: velocity interpolated, acceleration a centered difference of it."""
    v = np.interp(t, tv, xv)
    a = (np.interp(t + ACC_WIN, tv, xv) - np.interp(t - ACC_WIN, tv, xv)) / (2 * ACC_WIN)
    return v, a


def samples(hoot, dev, windows, v_min_supply=V_MIN_SUPPLY):
    """Regression rows of one motor from a hoot.log (hoot.HootLog) inside the (start, end) robot-log windows
    (when the robot was enabled).  None when the hoot lacks Velocity or MotorVoltage for it."""
    vel, mv, sv = hoot.sig(dev, "Velocity"), hoot.sig(dev, "MotorVoltage"), hoot.sig(dev, "SupplyVoltage")
    if vel is None or mv is None or len(vel) < 20 or len(mv) < 4:
        return None
    tv, xv = np.asarray(vel.t, float), np.asarray(vel.numeric(), float)
    tm, xm = np.asarray(mv.t, float), np.asarray(mv.numeric(), float)
    s = Samples(device=dev)
    if sv is not None and len(sv):
        sup = np.array([sv.at(q, 12.0) for q in tm], float)
    else:
        sup = np.full(len(tm), 12.0)
    en = np.zeros(len(tm), bool)
    for a, b in windows:
        en |= (tm >= a) & (tm <= b)
    s.n_all = int(en.sum())
    en &= (tm > tv[0] + ACC_WIN) & (tm < tv[-1] - ACC_WIN)             # inside the velocity record
    v, a = _accel(tv, xv, tm)
    gap = np.interp(tm, tv, np.concatenate([[0.0], np.diff(tv)]))      # a hole in the 100 Hz stream makes the acceleration wrong
    ok = (en & (sup >= v_min_supply) & (np.abs(xm) < SAT_FRAC * sup) & (np.abs(v) >= SPEED_MIN)
          & (np.abs(a) <= ACC_MAX) & (gap < 0.05))
    s.v, s.a, s.volts, s.t = v[ok], a[ok], xm[ok], tm[ok]
    return s


def pool(parts):
    """One Samples out of several (same motor, different matches)."""
    parts = [p for p in parts if p is not None]
    if not parts:
        return None
    return Samples(parts[0].device, np.concatenate([p.v for p in parts]), np.concatenate([p.a for p in parts]),
                   np.concatenate([p.volts for p in parts]), np.concatenate([p.t for p in parts]), sum(p.n_all for p in parts))


# ------------------------------------------------------------------------------------------ fitting
def _huber(X, y, iters=12):
    """Least squares that gives outliers (a slipping wheel, a latched voltage) less weight.  Returns (coef, cov)."""
    w = np.ones(len(y))
    c = np.zeros(X.shape[1])
    for _ in range(iters):
        sw = np.sqrt(w)
        c = np.linalg.lstsq(X * sw[:, None], y * sw, rcond=None)[0]
        r = y - X @ c
        s = max(1.4826 * float(np.median(np.abs(r - np.median(r)))), 1e-3)
        k = 1.345 * s
        w = np.where(np.abs(r) <= k, 1.0, k / np.maximum(np.abs(r), 1e-12))
    r = y - X @ c
    dof = max(len(y) - X.shape[1], 1)
    sigma2 = float((w * r * r).sum() / dof)
    try:
        cov = sigma2 * np.linalg.inv((X * w[:, None]).T @ X)
    except np.linalg.LinAlgError:
        cov = np.full((X.shape[1],) * 2, np.nan)
    return c, cov


def fit_samples(s, min_rows=60):
    f = Fit(device=s.device if s else "", n_all=s.n_all if s else 0)
    if s is None or len(s) < min_rows:
        f.n = 0 if s is None else len(s)
        f.note = f"only {f.n} usable samples (need {min_rows})"
        return f
    f.n = len(s)
    st = np.abs(s.a) <= ACC_STEADY
    if st.sum() >= min_rows and np.ptp(s.v[st]) > 1.0:                 # kS and kV from the steady part
        X = np.c_[np.sign(s.v[st]), s.v[st]]
        c, cov = _huber(X, s.volts[st])
        f.ks, f.kv = float(c[0]), float(c[1])
        f.se_ks, f.se_kv = (float(np.sqrt(max(cov[i, i], 0))) for i in range(2))
    X = np.c_[np.sign(s.v), s.v, s.a]                                  # all three terms on everything
    c, cov = _huber(X, s.volts)
    res = s.volts - X @ c
    f.rms = float(np.sqrt(np.mean(res ** 2)))
    f.r2 = float(1 - (res ** 2).sum() / max(((s.volts - s.volts.mean()) ** 2).sum(), 1e-12))
    f.ka = float(c[2])
    f.se_ka = float(np.sqrt(max(cov[2, 2], 0)))
    if f.kv is None:
        f.ks, f.kv = float(c[0]), float(c[1])
        f.se_ks, f.se_kv = float(np.sqrt(max(cov[0, 0], 0))), float(np.sqrt(max(cov[1, 1], 0)))
    f.v_max = float(np.percentile(np.abs(s.v), 95))
    f.a_span = float(np.percentile(np.abs(s.a), 95))
    f.ka_ok = bool(f.ka > 0 and f.se_ka < ACC_WIDE * f.ka)
    if not f.ka_ok:
        f.note = "kA not determined by this data"
    return f


def fit_hoot(hoot, windows, devices=None):
    """{device: Fit} for the hoot's TalonFX devices over the enabled `windows`."""
    out = {}
    for dev in (devices or [d for d in hoot.devices if d.startswith("TalonFX")]):
        s = samples(hoot, dev, windows)
        if s is not None:
            out[dev] = fit_samples(s)
    return out


def enabled_windows(hoot, window=None):
    """[(start, end)] robot-log times the hoot says the robot was enabled, clipped to `window` (the match)."""
    import hoot as H
    if hoot.enable is None or not len(hoot.enable):
        return [window] if window else []
    iv = H._bool_intervals(hoot.enable)
    if window:
        iv = [(max(a, window[0]), min(b, window[1])) for a, b in iv if b > window[0] and a < window[1]]
    return iv


def module_scale(tel, hoot, dev, window):
    """Metres the wheel travels per motor rps, measured from the robot log's module speeds (None if unavailable)."""
    import hoot as H
    mod, vel = tel.by_key.get("mod"), hoot.sig(dev, "Velocity")
    mi = H.MODULE_OF.get(dev)
    if mod is None or not len(mod) or vel is None or mi is None:
        return None
    a, b = window
    rows = [(mod.t[i], abs(mod.v[i][mi][0])) for i in np.nonzero((mod.t >= a) & (mod.t <= b))[0] if np.shape(mod.v[i]) == (4, 2)]
    if len(rows) < 100:
        return None
    ts, sp = np.array([r[0] for r in rows]), np.array([r[1] for r in rows])
    vh = np.abs(np.interp(ts, vel.t, vel.numeric()))
    ok = (sp > 0.5) & (vh > 0.5)
    if ok.sum() < 100:
        return None
    return float((sp[ok] * vh[ok]).sum() / (vh[ok] ** 2).sum())


# ------------------------------------------------------------------------------------------ all matches
def fit_folder(folder, drive="NT:/SmartDashboard/Swerve/drive_pose", quest="NT:/QuestNav/quest_pose", progress=print):
    """Pool every match of a logs folder that has a hoot.  Returns (per-motor pooled Fit, {motor: scale}, per-match rows)."""
    import core
    import hoot as H
    import telemetry as T
    parts, scales, rows = {}, {}, []
    for f in sorted(Path(folder).rglob("WPILIB_*.wpilog")):
        hs = H.find_hoots_for_log(str(f), folder)
        if not hs:
            continue
        try:
            conv = [H.convert_hoot(h_) for h_ in hs]
            hl = H.load_many([w for w, _ in conv], str(f))
            ml = core.load_match_log(str(f), [drive, quest])
            win = (ml.t_ref, ml.t_ref + ml.match_len) if ml.has_match else (ml.t_ref, ml.t_last)
            wins = enabled_windows(hl, win)
            tel = T.Telemetry(str(f), ml, {"drive": drive, "quest": quest})
        except Exception as e:  # noqa
            progress(f"{f.name}: skipped ({e})")
            continue
        got = {}
        for dev in [d for d in hl.devices if d.startswith("TalonFX")]:
            s = samples(hl, dev, wins)
            if s is None:
                continue
            parts.setdefault(dev, []).append(s)
            sc = module_scale(tel, hl, dev, win)
            if sc:
                scales.setdefault(dev, []).append(sc)
            got[dev] = fit_samples(s)
        rows.append((f.name, got))
        progress(f"{f.name}: " + ", ".join(f"{d[-2:]} kS {x.ks:.2f} kV {x.kv:.3f}" for d, x in got.items() if x.ks is not None))
    pooled = {d: fit_samples(pool(p)) for d, p in parts.items()}
    return pooled, {d: float(np.median(v)) for d, v in scales.items()}, rows


def spread(rows, dev):
    """(std of kS, kV, kA) between the matches' own fits of one motor: the honest uncertainty of the pooled numbers."""
    fs = [r[1][dev] for r in rows if dev in r[1] and r[1][dev].ks is not None]
    return tuple(float(np.std([getattr(f, k) for f in fs if getattr(f, k) is not None])) if len(fs) > 1 else None
                 for k in ("ks", "kv", "ka"))


def describe(pooled, scales, rows=()):
    import hoot as H
    lines = ["Drive motor feedforward from match hoots  (volts = kS*sign(v) + kV*v + kA*a, v in motor rps as the hoot logs it)", ""]
    for dev, f in sorted(pooled.items()):
        name = H.DEVICE_NAMES.get(dev, dev)
        if f.ks is None:
            lines.append(f"{name}: {f.note}")
            continue
        l = (f"{name}: kS {f.ks:.3f} +/-{f.se_ks:.3f} V   kV {f.kv:.4f} +/-{f.se_kv:.4f} V/rps   "
             + (f"kA {f.ka:.4f} +/-{f.se_ka:.4f} V/(rps/s) {'' if f.ka_ok else '(NOT determined)'}" if f.ka is not None else "")
             + f"   n={f.n}  rms {f.rms:.2f} V  reaches {f.v_max:.1f} rps")
        m = f.per_ms(scales.get(dev))
        if m:
            l += f"\n      in wheel units ({scales[dev]:.4f} m per rps): kV {m[1]:.3f} V/(m/s)" + (f", kA {m[2]:.3f} V/(m/s^2)" if f.ka_ok else "")
        lines.append(l)
    return "\n".join(lines) + "\n"


def main(argv):
    if len(argv) < 2:
        print("usage: python sysid.py <logs folder> [out.json]")
        return 2
    pooled, scales, rows = fit_folder(argv[1])
    print()
    print(describe(pooled, scales, rows))
    if len(argv) > 2:
        Path(argv[2]).write_text(json.dumps({"pooled": {d: asdict(f) for d, f in pooled.items()}, "scales": scales}, indent=1),
                                 encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
