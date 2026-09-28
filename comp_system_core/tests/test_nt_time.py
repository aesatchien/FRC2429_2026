"""
Pin the NetworkTables timestamp unit that helpers/nt_time.py assumes.

WHY THIS EXISTS
---------------
WPILib 2027a7 changed every NetworkTables timestamp from microseconds to nanoseconds with
no rename and no warning.  Code that still divided by 1e6 kept running and silently threw
away every vision target as "stale" - the Pi was publishing targets and the robot saw none.

helpers/nt_time.py now holds the unit in one constant.  If a future WPILib changes it
again, this fails here instead of on the robot.
"""

import time

import ntcore
import wpilib

from helpers.nt_time import k_nt_ticks_per_second, nt_age_seconds, nt_now_seconds


def test_nt_clock_matches_timer_in_seconds():
    """nt_now_seconds() must be Timer.get_timestamp(), same unit AND same epoch.  The
    epoch matters: Swerve subtracts an NT-measured latency straight off a Timer timestamp."""
    ratio = ntcore._now() / wpilib.Timer.get_timestamp()
    assert abs(ratio / k_nt_ticks_per_second - 1) < 1e-3, (
        f"ntcore._now() / Timer.get_timestamp() = {ratio:.4e}, but helpers/nt_time.py assumes "
        f"{k_nt_ticks_per_second:.0e} ticks per second.  The NT timestamp unit changed - fix "
        f"k_nt_ticks_per_second, and every vision latency check will follow.")
    assert abs(nt_now_seconds() - wpilib.Timer.get_timestamp()) < 0.5


def test_a_fresh_value_reads_as_fresh():
    """The failure this guards against: a value set just now must not look seconds old."""
    inst = ntcore.NetworkTableInstance.get_default()
    pub = inst.get_double_topic("/test_nt_time/value").publish()
    sub = inst.get_double_topic("/test_nt_time/value").subscribe(0)
    pub.set(1.0)
    time.sleep(0.05)
    age = nt_age_seconds(sub.get_atomic().time)
    assert 0 <= age < 0.5, f"a value set 50 ms ago reads as {age:.3f} s old"
