"""
How often does each swerve drive Kraken record each signal in a .hoot?

    python hoot_rates.py D:\\logs                       # the newest .hoot under that folder
    python hoot_rates.py D:\\logs\\2026-10-25_14-23-15\\2026-10-25_14-23-20.hoot

Converts the hoot with owlet (the copy that ships with AdvantageScope, picked by the hoot's own compliancy number; cached in
~/.scope2429/cache/hoot) and prints, per TalonFX, the rate in Hz of the signals that matter for the drive feedforward fit:
(samples - 1) / seconds covered.  MotorVoltage should be about 100 Hz.  At 4 Hz the SysId fit has one voltage sample per 250 ms
(comp_system_core constants.k_drive_hoot_rate_hz is the setting).

Run it with the robo2026 environment (it uses the same OpenCV / NumPy as Scope2429).
"""
import glob
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "helpers"))
import numpy as np  # noqa: E402

import core  # noqa: E402
import hoot as H  # noqa: E402

SIGNALS = ["MotorVoltage", "Velocity", "Position", "StatorCurrent", "SupplyCurrent", "SupplyVoltage"]


def newest_hoot(folder):
    found = sorted(glob.glob(os.path.join(folder, "**", "*.hoot"), recursive=True), key=os.path.getmtime)
    return found[-1] if found else None


def rates(hoot_path):
    """{device: {signal: Hz}} for the TalonFX devices in the hoot."""
    wpl = H.convert_hoot(hoot_path)
    wpl = wpl[0] if isinstance(wpl, tuple) else wpl
    _e, recs = core.read_wpilog(wpl, wanted=lambda n: n.startswith("Phoenix6/"))
    out = defaultdict(dict)
    for name, rows in recs.items():
        parts = name.split("/")
        if len(parts) >= 4 and parts[2].startswith("TalonFX") and len(rows) > 3:
            t = np.array([ts for ts, _ in rows], float) / 1e6
            out[parts[2]][parts[3]] = (len(t) - 1) / max(t[-1] - t[0], 1e-9)
    return out


def main(argv):
    target = argv[1] if len(argv) > 1 else "."
    path = target if target.lower().endswith(".hoot") else newest_hoot(target)
    if not path or not os.path.exists(path):
        print(f"no .hoot found at {target}")
        return 1
    print(f"{path}  ({os.path.getsize(path) // 1024} KB)")
    table = rates(path)
    if not table:
        print("no TalonFX signals in it")
        return 1
    print(f"{'device':12s}" + "".join(f"{s:>15s}" for s in SIGNALS))
    for dev in sorted(table):
        print(f"{dev:12s}" + "".join(f"{table[dev][s]:15.0f}" if s in table[dev] else f"{'-':>15s}" for s in SIGNALS))
    mv = [table[d]["MotorVoltage"] for d in table if "MotorVoltage" in table[d]]
    if mv:
        verdict = "OK, about 100 Hz" if min(mv) > 80 else "TOO SLOW for the SysId fit (expected about 100 Hz)"
        print(f"MotorVoltage: {min(mv):.0f} to {max(mv):.0f} Hz  ->  {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
