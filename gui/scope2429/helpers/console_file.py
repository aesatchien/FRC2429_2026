"""
Reader for the robot console logs saved by comp_system_core/helpers/console_log.py (console_<boot UTC>.log).

Each line is `<robot time s> <UTC> <O|E> <text>`.  The robot time is wpilib.Timer.get_timestamp(), the same clock as the
.wpilog timestamps, so a console line's time IS its log time: `rel = t - ml.t_ref` puts it on every Scope2429 plot.  The
.hoot is lined up to the wpilog separately (hoot.py), which places console lines on the hoot as well.

Several boots can share one folder, and the robot clock restarts at every boot, so the right file is chosen with the wall
clock: the UTC time at robot time zero must agree between the console file and the wpilog.
"""
from __future__ import annotations

import calendar
import glob
import os
import re
import time
from dataclasses import dataclass

LINE = re.compile(r"^(\d+\.\d{3}) (\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d(?:\.\d+)?Z) ([OE]) ?(.*)$")
SAME_BOOT_S = 120.0       # the wall clock may jump a little when the Driver Station sets the time; this is how much we forgive


@dataclass
class Line:
    t: float               # robot clock = wpilog clock, seconds
    utc: str
    stream: str            # 'O' stdout, 'E' stderr
    text: str


def _epoch(utc: str) -> float | None:
    try:
        whole, _, frac = utc.rstrip("Z").partition(".")
        return calendar.timegm(time.strptime(whole, "%Y-%m-%dT%H:%M:%S")) + (float("0." + frac) if frac else 0.0)
    except ValueError:
        return None


def parse(path) -> list:
    """All the lines of a console log, in order.  Header ('#') and malformed lines are skipped."""
    out = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            m = LINE.match(raw.rstrip("\n"))
            if m:
                out.append(Line(float(m[1]), m[2], m[3], m[4]))
    return out


def epoch0(lines) -> float | None:
    """UTC seconds at robot time zero, from the console's own wall-clock stamps (median, so one clock jump does not matter)."""
    offs = sorted(e - ln.t for ln in lines if (e := _epoch(ln.utc)) is not None)
    return offs[len(offs) // 2] if offs else None


def find_for_log(log_path):
    """Path of the console log that goes with this wpilog, or None.  Looks in the log's folder (that is where the robot
    writes it) and picks the file from the same boot."""
    folder = os.path.dirname(os.path.abspath(log_path))
    candidates = sorted(glob.glob(os.path.join(folder, "console_*.log")))
    if not candidates:
        return None
    try:
        import webcast
        want = webcast.log_epoch0(log_path)
    except Exception:  # noqa
        want = None
    best, best_d = None, None
    for c in candidates:
        try:
            e = epoch0(parse(c))
        except OSError:
            continue
        if want is None:
            d = 0.0 if len(candidates) == 1 else None
        else:
            d = abs(e - want) if e is not None else None
        if d is not None and d <= SAME_BOOT_S and (best_d is None or d < best_d):
            best, best_d = c, d
    return best


# ---------------------------------------------------------------------------------------- what is worth a look
_BAD = re.compile(r"Traceback|Exception|\bERROR\b|\bError\b|FAILED|failed to|brown ?out", re.I)
# not "***": the robot code also uses it for banners ("*** VISION.PY CAMERA DICT ***", "startup complete")
_WARN = re.compile(r"\bWarning\b|overrun|Watchdog|not found|disconnect|dropped|\blost\b|time ?out|Cannot |not usable|not applied", re.I)
_SKIP = re.compile(r"Override me!|^\s+at |^\tat |^\s*$")      # the default-RobotPeriodic notice, native stack-trace lines, blanks


BATTERY_LINE = re.compile(r"^BATTERY: ([A-Za-z0-9][A-Za-z0-9 _.-]{0,23})(?:\s{2,}\(was .*\))?\s*$")


def battery(lines) -> str:
    """The battery picked on the driver dashboard for this boot ('2026-C'), from the robot's 'BATTERY: <id>' lines
    (comp_system_core/helpers/battery_id.py), or ''.  The last one counts: a wrong pick corrected before the match wins."""
    found = ""
    for ln in lines:
        m = BATTERY_LINE.match(ln.text.strip())
        if m:
            found = m[1].strip()
    return found


def battery_for_log(log_path) -> str:
    """battery() of the console log of THIS boot, or '' (no console log, no battery picked, or not sure the file is this boot's).

    A battery is never taken from another boot: each boot's battery is picked on the dashboard for that boot alone.  So the console
    file must be shown to be this log's: either its wall clock matches the log's (find_for_log), or the folder is the robot's
    one-boot folder (comp_system_core/helpers/log_start.py: one .wpilog, its .hoot and its console log together)."""
    try:
        path = find_for_log(log_path)
        if not path:
            return ""
        folder = os.path.dirname(os.path.abspath(log_path))
        one_boot_folder = len(glob.glob(os.path.join(folder, "*.wpilog"))) == 1
        if not one_boot_folder:
            try:
                import webcast
                matched = webcast.log_epoch0(log_path) is not None      # find_for_log only accepts a file whose boot time matches
            except Exception:  # noqa
                matched = False
            if not matched:
                return ""                                               # several logs and no way to tell which boot: never guess
        return battery(parse(path))
    except OSError:
        return ""


def severity(text: str):
    """'bad', 'warn' or None.  Native multi-line messages: only their first line is judged (the indented 'at ...' lines are skipped)."""
    if _SKIP.search(text):
        return None
    if _BAD.search(text):
        return "bad"
    if _WARN.search(text):
        return "warn"
    return None
