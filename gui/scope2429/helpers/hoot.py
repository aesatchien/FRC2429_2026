"""
CTRE Phoenix 6 hoot logs: conversion with CTRE's `owlet` tool, matching a hoot to a robot log, and lining the two
clocks up so the hoot's device signals can be compared with the robot's wpilog.

A hoot is a binary log written on the robot by the CTRE devices (one file per CAN bus).  It holds the devices' own
status signals (supply / stator current, voltage, velocity, position, faults ...) timestamped on the CAN bus, at a
higher rate than anything published over NetworkTables.  `owlet` turns it into a wpilog, which core.read_wpilog reads.

    path = convert_hoot("CAEL_Q13_2026-10-10_11-55-00.hoot")       # -> cached .wpilog (needs owlet, see find_owlet)
    h = HootLog.load(path, robot_log_path)                         # aligned to the robot log's clock
    h.sig("TalonFX-21", "SupplyCurrent")                           # telemetry.Sig in robot log time
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

import core
import telemetry as T

# 2026 robot: the Krakens on the bus are the swerve DRIVE motors (the turn motors are REV SparkFlex, so not in a hoot)
DEVICE_NAMES = {"TalonFX-21": "LF drive", "TalonFX-23": "LB drive", "TalonFX-25": "RF drive", "TalonFX-27": "RB drive"}
MODULE_OF = {"TalonFX-21": 0, "TalonFX-25": 1, "TalonFX-23": 2, "TalonFX-27": 3}     # index in the module_states arrays
FAULTS = {   # fault signal -> severity when it is active
    "Fault_Hardware": "bad", "Fault_BridgeBrownout": "bad", "Fault_BootDuringEnable": "bad", "Fault_Undervoltage": "warn",
    "Fault_ProcTemp": "warn", "Fault_DeviceTemp": "warn", "Fault_RemoteSensorDataInvalid": "warn",
    "Fault_UnlicensedFeatureInUse": "info", "Fault_StaticBrakeDisabled": "info", "Fault_RotorFault1": "bad",
    "Fault_RotorFault2": "bad"}


class HootError(RuntimeError):
    pass


# ------------------------------------------------------------------------------------------ owlet
def _owlet_dirs():
    dirs = [core.APP_DIR / "bin"]
    if sys.platform == "win32":
        for env in ("APPDATA", "LOCALAPPDATA"):
            if os.environ.get(env):
                dirs.append(Path(os.environ[env]) / "AdvantageScope" / "owlet")
    elif sys.platform == "darwin":
        dirs.append(Path.home() / "Library/Application Support/AdvantageScope/owlet")
    else:
        dirs.append(Path.home() / ".config/AdvantageScope/owlet")
    return dirs


def find_owlets(extra=None):
    """[(compliancy number, path)] of every owlet found, newest first.  AdvantageScope downloads them into its own
    folder (owlet-<version>-C<n>); a path the user chose (settings 'owlet_path') and any owlet on PATH count too."""
    found = {}

    def add(p):
        p = Path(p)
        if p.is_file():
            m = re.search(r"-C(\d+)", p.name)
            n = int(m[1]) if m else _compliancy_of_owlet(p)
            found[str(p)] = (n, p)
    for d in _owlet_dirs():
        if d.is_dir():
            for p in d.glob("owlet*"):
                add(p)
    for p in ([extra] if extra else []):
        add(p)
    import shutil
    w = shutil.which("owlet")
    if w:
        add(w)
    return sorted(found.values(), key=lambda x: -x[0])


def _run(args, **kw):
    return subprocess.run(args, capture_output=True, creationflags=core.NO_WINDOW, **kw)


def _compliancy_of_owlet(p):
    r = _run([str(p), "--compliancy", str(p)])          # prints "owlet: N" first even when the file is not a hoot
    m = re.search(r"owlet:\s*(\d+)", r.stdout.decode("utf-8", "replace"))
    return int(m[1]) if m else 0


def hoot_compliancy(hoot, owlet):
    """The log version number of a hoot file (as `owlet --compliancy` reports it), or None."""
    r = _run([str(owlet), str(hoot), "--compliancy"])
    m = re.search(r"hoot log:\s*(\d+)", r.stdout.decode("utf-8", "replace"))
    return int(m[1]) if m else None


def pick_owlet(hoot, extra=None):
    """The owlet that reads this hoot: the one whose compliancy number equals the hoot's, else the lowest one above it."""
    owlets = find_owlets(extra)
    if not owlets:
        raise HootError("owlet (CTRE's hoot converter) was not found.\n\n"
                        "Open AdvantageScope once with a hoot file or an internet connection (it downloads owlet into "
                        f"{_owlet_dirs()[1]}), or install Phoenix Tuner X, then choose owlet.exe when asked.")
    n = hoot_compliancy(hoot, owlets[0][1])
    if n is None:
        raise HootError(f"{Path(hoot).name} does not look like a hoot file.")
    same = [o for o in owlets if o[0] == n]
    above = sorted([o for o in owlets if o[0] > n], key=lambda o: o[0])
    pick = same[0] if same else (above[0] if above else None)
    if pick is None:
        raise HootError(f"{Path(hoot).name} is a newer hoot version ({n}) than any owlet found (newest {owlets[0][0]}).\n"
                        "Update AdvantageScope or Phoenix Tuner X so a newer owlet is downloaded.")
    return pick[1]


def convert_hoot(hoot, out=None, owlet_path=None, force=False):
    """Convert a hoot file to a wpilog with owlet.  Returns (wpilog path, info dict).  The result is cached under
    ~/.scope2429/cache/hoot unless `out` is given.  info: owlet, warnings (e.g. the hoot was cut short)."""
    hoot = Path(hoot)
    if out is None:
        d = core.CACHE_DIR / "hoot"
        d.mkdir(parents=True, exist_ok=True)
        out = d / f"{hoot.stem}_{int(hoot.stat().st_mtime)}.wpilog"
    out = Path(out)
    info = {"owlet": None, "warnings": []}
    warn_file = out.with_suffix(".txt")
    if out.exists() and not force and out.stat().st_size > 1000:
        if warn_file.exists():
            info["warnings"] = [w for w in warn_file.read_text(encoding="utf-8").splitlines() if w]
        return str(out), info
    ow = pick_owlet(hoot, owlet_path)
    info["owlet"] = str(ow)
    out.parent.mkdir(parents=True, exist_ok=True)
    r = _run([str(ow), str(hoot), str(out), "-f", "wpilog"])
    text = (r.stdout + b"\n" + r.stderr).decode("utf-8", "replace")
    if "Could not read to end" in text:
        info["warnings"].append("the hoot file was not closed cleanly (CTRE logging was not stopped before power went off, "
                                "or the file was cut); everything readable was converted")
    if "Could not write signals" in text and not out.exists():
        raise HootError("owlet could not write the output file:\n" + text[-400:])
    if not out.exists() or out.stat().st_size < 1000:
        raise HootError("owlet did not produce a log:\n" + text.strip()[-600:])
    warn_file.write_text("\n".join(info["warnings"]), encoding="utf-8")
    return str(out), info


# ------------------------------------------------------------------------------------------ matching
def match_token(log_path):
    """'CAEL_Q13' from 'WPILIB_20261010_185501_CAEL_Q13.wpilog' (event + match), or None."""
    m = re.search(r"_([A-Za-z0-9]+_[PQE]\d+)\.wpilog$", str(log_path), re.I)
    return m[1].upper() if m else None


def find_hoots_for_log(log_path, search_root=None, depth=3):
    """Hoot files that belong to a robot log: found under the log's folder (or `search_root`) when the file or its
    folder name contains the log's event + match (CAEL_Q13).  One hoot per CAN bus, so there can be several."""
    tok = match_token(log_path)
    root = Path(search_root or Path(log_path).parent)
    if not tok or not root.is_dir():
        return []
    out = []
    base = len(root.parts)
    for dp, dn, fn in os.walk(root):
        if len(Path(dp).parts) - base >= depth:
            dn[:] = []
        for f in fn:
            if f.lower().endswith(".hoot") and (tok in f.upper() or tok in Path(dp).name.upper()):
                out.append(str(Path(dp) / f))
    return sorted(out)


# ------------------------------------------------------------------------------------------ clock alignment
def enable_intervals_robot(log_path):
    """[(start, end)] log times where the robot was enabled, from the Driver Station control word."""
    _e, recs = core.read_wpilog(log_path, {"DS:controlWord", "NT:/DriverStation/ControlWord"})
    rows = recs.get("DS:controlWord") or recs.get("NT:/DriverStation/ControlWord") or []
    out, start = [], None
    for t, p in rows:
        _mode, en, _fms = core._decode_controlword(p)
        if en and start is None:
            start = t / 1e6
        elif not en and start is not None:
            out.append((start, t / 1e6))
            start = None
    if start is not None and rows:
        out.append((start, rows[-1][0] / 1e6))
    return out


def _edges(intervals):
    return np.array(sorted([a for a, _ in intervals] + [b for _, b in intervals]))


def _grid(intervals, t0, t1, dt):
    x = np.zeros(int(np.ceil((t1 - t0) / dt)) + 1, np.float32)
    for a, b in intervals:
        x[max(0, int((a - t0) / dt)):max(0, int((b - t0) / dt))] = 1.0
    return x


def align_clocks(hoot_enable, robot_enable, dt=0.02):
    """Offset s.t. hoot_time + offset = robot log time, from the enable intervals of both.  Returns (offset, matched,
    total): how many hoot enable/disable edges land within 100 ms of a robot edge after shifting."""
    if not hoot_enable or not robot_enable:
        raise HootError("cannot line the hoot up with the robot log: one of them never shows the robot enabled")
    h0, h1 = min(a for a, _ in hoot_enable) - 5, max(b for _, b in hoot_enable) + 5
    r0, r1 = min(a for a, _ in robot_enable) - 5, max(b for _, b in robot_enable) + 5
    xh, xr = _grid(hoot_enable, h0, h1, dt) - 0.5, _grid(robot_enable, r0, r1, dt) - 0.5
    n = len(xh) + len(xr)
    size = 1 << int(np.ceil(np.log2(n)))
    corr = np.fft.irfft(np.fft.rfft(xr, size) * np.conj(np.fft.rfft(xh, size)), size)     # corr[k] = sum xr[i + k] xh[i]
    k = int(np.argmax(corr))
    if k > size // 2:
        k -= size
    offset = (r0 - h0) + k * dt                      # hoot time + offset = robot time
    # refine with the edges
    he, re_ = _edges(hoot_enable), _edges(robot_enable)
    shifted = he + offset
    diffs = []
    for e in shifted:
        j = int(np.argmin(np.abs(re_ - e)))
        if abs(re_[j] - e) < 0.5:
            diffs.append(re_[j] - e)
    if diffs:
        offset += float(np.median(diffs))
    matched = int(sum(1 for e in (he + offset) if np.min(np.abs(re_ - e)) < 0.1))
    return float(offset), matched, len(he)


# ------------------------------------------------------------------------------------------ loaded hoot
class HootLog:
    """The signals of one converted hoot, in the robot log's clock."""

    def __init__(self):
        self.path = ""
        self.sigs: dict = {}              # (device, signal) -> telemetry.Sig
        self.devices: list = []
        self.can_up = None                # Sig of CANBusIsUp
        self.enable = None                # Sig of RobotEnable (hoot's view)
        self.offset = 0.0
        self.matched = (0, 0)
        self.warnings: list = []
        self.t_last = 0.0                 # robot-clock time of the last record (to see where a cut-short hoot ends)

    def sig(self, device, name):
        return self.sigs.get((device, name))

    @classmethod
    def load(cls, wpilog, robot_log=None, warnings=None):
        ent, recs = core.read_wpilog(wpilog)
        types = {n: t for (n, t, _m) in ent.values()}
        h = cls()
        h.path = str(wpilog)
        h.warnings = list(warnings or [])
        raw: dict = {}
        for name, rows in recs.items():
            if not rows:
                continue
            ts = np.array([t for t, _ in rows], float) / 1e6
            if name == "RobotEnable":
                h.enable = T.Sig(ts, [bool(p[0]) for _, p in rows])
                continue
            parts = name.split("/")
            if name.endswith("CANBusIsUp"):
                h.can_up = T.Sig(ts, [bool(p[0]) for _, p in rows])
                continue
            if len(parts) < 4 or not parts[0] == "Phoenix6":
                continue
            dev, sg = parts[2], parts[3]
            typ = types.get(name, "")
            vals = [T._decode(typ, p) for _, p in rows]
            if any(v is None for v in vals):
                continue
            raw[(dev, sg)] = (ts, vals)
        h.devices = sorted({d for d, _ in raw})
        if robot_log is not None:
            if h.enable is None or not len(h.enable):
                raise HootError("this hoot has no RobotEnable signal, so it cannot be lined up with the robot log")
            hoot_iv = _bool_intervals(h.enable)
            h.offset, m, tot = align_clocks(hoot_iv, enable_intervals_robot(robot_log))
            h.matched = (m, tot)
        for k, (ts, vals) in raw.items():
            h.sigs[k] = T.Sig(ts + h.offset, vals)
        if h.can_up is not None:
            h.can_up = T.Sig(h.can_up.t + h.offset, h.can_up.v)
        if h.enable is not None:
            h.enable = T.Sig(h.enable.t + h.offset, h.enable.v)
            h.t_last = float(h.enable.t[-1])
        return h


def load_many(wpilogs, robot_log=None, warnings=None):
    """One HootLog out of several converted hoots (one file per CAN bus).  Each is lined up on its own."""
    logs = [HootLog.load(p, robot_log) for p in wpilogs]
    h = logs[0]
    for o in logs[1:]:
        h.sigs.update({k: v for k, v in o.sigs.items() if k not in h.sigs})
        h.devices = sorted(set(h.devices) | set(o.devices))
        h.t_last = max(h.t_last, o.t_last)
        if h.can_up is None:
            h.can_up = o.can_up
    h.warnings = list(warnings or [])
    return h


def _bool_intervals(sig):
    out, start = [], None
    for t, v in zip(sig.t, sig.v):
        if v and start is None:
            start = float(t)
        elif not v and start is not None:
            out.append((start, float(t)))
            start = None
    if start is not None:
        out.append((start, float(sig.t[-1])))
    return out
