"""NetworkTables timestamps, converted to seconds in ONE place.

WHY THIS EXISTS
---------------
WPILib 2027a7 changed the unit of every NetworkTables timestamp from MICROSECONDS to
NANOSECONDS - ntcore._now(), and the .time on every get_atomic() / event.  Nothing
renamed, nothing warns: the numbers are simply 1000x bigger.  Code that still assumed
microseconds kept running and silently broke:

    latency_us = ntcore._now() - atomic.time
    if latency_us > 500000:          # meant 0.5 s - on a7 that is 0.5 ms
        continue                     # so every AprilTag was skipped

That is how vision went dark on the robot: the Pi was publishing targets, and
Vision.target_available() and Swerve._update_vision_measurements() both threw every one
away as "stale".  Measured, not assumed:

    ntcore._now() / wpilib.Timer.get_timestamp() == 1.000000e9

which also shows NT time and Timer.get_timestamp() share the same epoch - so a latency
worked out in NT time can be subtracted straight from a Timer timestamp.

RULE: never do arithmetic on a raw NT timestamp.  Convert with these first.
tests/test_nt_time.py fails if the unit ever changes again.
"""

import ntcore

# a7: nanoseconds.  a6 and earlier were microseconds (1_000_000).
k_nt_ticks_per_second = 1_000_000_000


def nt_to_seconds(nt_time: int) -> float:
    """A raw NT timestamp (e.g. subscriber.get_atomic().time) in seconds."""
    return nt_time / k_nt_ticks_per_second


def nt_now_seconds() -> float:
    """The current NT time in seconds - same clock as nt_to_seconds() values."""
    return ntcore._now() / k_nt_ticks_per_second


def nt_age_seconds(nt_time: int) -> float:
    """How long ago a raw NT timestamp was, in seconds."""
    return (ntcore._now() - nt_time) / k_nt_ticks_per_second
