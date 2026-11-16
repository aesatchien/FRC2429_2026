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


def _log_span(log_path, ml=None):
    """(first, last) record time of the wpilog in seconds on the robot clock, or None.  From `ml` (core.MatchLog) when the caller
    has one, because finding it otherwise reads the whole log."""
    if ml is not None:
        return float(ml.t_first), float(ml.t_last)
    try:
        import core
        span = []
        core.read_wpilog(log_path, lambda n: False, span)
        return span[0] / 1e6, span[1] / 1e6
    except Exception:  # noqa - a log that cannot be read has no console log to go with
        return None


def _find(log_path, ml=None):
    """(path of the console log of this wpilog or None, True when it was matched to the log by the wall clock)."""
    folder = os.path.dirname(os.path.abspath(log_path))
    candidates = sorted(glob.glob(os.path.join(folder, "console_*.log")))
    if not candidates:
        return None, False
    try:
        import webcast
        want = webcast.log_epoch0(log_path)
    except Exception:  # noqa
        want = None
    span = False                                     # the wpilog's (first, last) record time, read only when a file gets that far
    best, best_d = None, None
    for c in candidates:
        try:
            lines = parse(c)
        except OSError:
            continue
        e = epoch0(lines)
        if want is None:
            d = 0.0 if len(candidates) == 1 else None
        else:
            d = abs(e - want) if e is not None else None
            if d is not None and d <= SAME_BOOT_S:
                # Boot times that agree are not enough: two boots can start within SAME_BOOT_S of each other (a power cycle),
                # and when this boot's own file is missing the other boot's would be taken.  The two files of one boot were
                # written at the same time, so on the wall clock (each file's own) the console's first..last line must overlap
                # the log's first..last record.  Another boot's file ended before this log began, or began after it ended.
                if span is False:
                    span = _log_span(log_path, ml)
                if span is None or e + lines[-1].t < want + span[0] or e + lines[0].t > want + span[1]:
                    d = None
        if d is not None and d <= SAME_BOOT_S and (best_d is None or d < best_d):
            best, best_d = c, d
    return best, best is not None and want is not None


def find_for_log(log_path, ml=None):
    """Path of the console log that goes with this wpilog, or None.  Looks in the log's folder (that is where the robot
    writes it) and picks the file from the same boot: its boot time agrees with the log's and the two were being written
    at the same time.  Pass `ml` (the log's core.MatchLog) when there is one; without it the log is read once more."""
    return _find(log_path, ml)[0]


# ---------------------------------------------------------------------------------------- what is worth a look
_BAD = re.compile(r"Traceback|Exception|\bERROR\b|\bError\b|FAILED|failed to|brown ?out", re.I)
# not "***": the robot code also uses it for banners ("*** VISION.PY CAMERA DICT ***", "startup complete")
_WARN = re.compile(r"\bWarning\b|overrun|Watchdog|not found|disconnect|dropped|\blost\b|time ?out|Cannot |not usable|not applied", re.I)
_SKIP = re.compile(r"Override me!|^\s+at |^\tat |^\s*$")      # the default-RobotPeriodic notice, native stack-trace lines, blanks


# An id has no space in it ('2026-C', or whatever the robot's list holds next year), so a sentence that happens to start with
# 'BATTERY:' ('BATTERY: voltage low') is not taken for one.  '   (was 2026-C)' is what the robot adds when the pick changed.
BATTERY_LINE = re.compile(r"^BATTERY: ([A-Za-z0-9][A-Za-z0-9_.-]{0,23})(?:\s{2,}\(was .*\))?\s*$")
BATTERY_TOPIC = "NT:/SmartDashboard/RobotStatus/battery/active"      # the same pick as a string in the .wpilog; '?' until picked


def battery(lines, t_end=None) -> str:
    """The battery picked on the driver dashboard for this boot ('2026-C'), from the robot's 'BATTERY: <id>' lines
    (subsystems/robot_state.py: _check_battery), or ''.  The last one counts: a wrong pick corrected before the match wins.
    Lines after `t_end` (robot time) are left out: the next battery picked in the pit with the robot still on is not the
    one the match was played on."""
    found = ""
    for ln in lines:
        if t_end is not None and ln.t > t_end:
            continue
        m = BATTERY_LINE.match(ln.text.strip())
        if m:
            found = m[1].strip()
    return found


def battery_in_wpilog(log_path, tel=None, t_end=None) -> str:
    """The last battery id the robot published on BATTERY_TOPIC at or before `t_end` (the whole log when None), or '' ('?' and
    an empty string mean not picked).  From `tel` (telemetry.Telemetry of this log, key 'battery_id') when the caller has one,
    because reading it from the file goes through the whole log."""
    if tel is not None:
        sig = tel.by_key.get("battery_id")
        rows = zip(sig.t, sig.v) if sig is not None else []
    else:
        try:
            import core
            _e, recs = core.read_wpilog(log_path, {BATTERY_TOPIC})
        except (OSError, ValueError):
            return ""
        rows = [(t / 1e6, p.decode("utf-8", "replace")) for t, p in recs.get(BATTERY_TOPIC) or []]
    found = ""
    for t, v in rows:
        v = str(v).strip()
        if (t_end is None or t <= t_end) and v and v != "?":
            found = v
    return found


def battery_for_log(log_path, ml=None, tel=None) -> str:
    """The battery the robot recorded for this log, or '' (none picked, or not sure).  First the log itself (battery_in_wpilog:
    it cannot belong to another boot, and comp_bot on a roboRIO has no console log), then, for logs from before the robot
    published the topic, battery() of the console log of THIS boot.  With a match, only what was picked up to the end of the
    match counts.  Pass `ml` (core.MatchLog) and `tel` (telemetry.Telemetry) when they are already loaded: without them the log
    is read again, seconds on a long log, so never call it bare on the Tk thread.

    A battery is never taken from another boot: each boot's battery is picked on the dashboard for that boot alone.  So the console
    file must be shown to be this log's: either its wall clock matches the log's (find_for_log), or the folder is the robot's
    one-boot folder (comp_system_core/helpers/log_start.py: one .wpilog, its .hoot and its console log together)."""
    if ml is None:
        try:
            import core
            ml = core.load_match_log(str(log_path), [])
        except Exception:  # noqa - no match times: the whole log counts
            ml = None
    t_end = ml.t_ref + ml.match_len if ml is not None and ml.has_match else None
    found = battery_in_wpilog(log_path, tel, t_end)
    if found:
        return found
    try:
        path, matched = _find(log_path, ml)                             # matched: its boot time agrees with the log's
        if not path:
            return ""
        folder = os.path.dirname(os.path.abspath(log_path))
        import hoot
        if not matched and not hoot.is_boot_folder(folder):
            return ""                                                   # several logs and no way to tell which boot: never guess
        return battery(parse(path), t_end)
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
