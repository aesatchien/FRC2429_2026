"""
Pin the battery picker in subsystems/robot_state.py: every boot starts at '?', a pick from the dashboard is taken
once and announced with the 'BATTERY: <id>' line Scope2429 parses, and anything that is not one of
constants.RobotStateConstants.k_batteries is ignored.
"""

import ntcore

import constants
from subsystems.robot_state import RobotState

PREFIX = f'{constants.status_prefix}/battery'


def _pick(text):
    """What the dashboard does: write its choice to 'selected'.  Returns the publisher - keep it alive."""
    pub = ntcore.NetworkTableInstance.getDefault().getStringTopic(f'{PREFIX}/selected').publish()
    pub.set(text)
    return pub


def _active():
    return ntcore.NetworkTableInstance.getDefault().getStringTopic(f'{PREFIX}/active').subscribe('none').get()


def _fresh(container):
    rs = container.robot_state
    rs.battery = RobotState.k_no_battery
    rs.battery_active_pub.set(RobotState.k_no_battery)
    return rs


def test_options_come_from_constants(container):
    options = ntcore.NetworkTableInstance.getDefault().getStringArrayTopic(f'{PREFIX}/options').subscribe([]).get()
    assert options == ['?'] + constants.RobotStateConstants.k_batteries


def test_pick_is_taken_once_and_announced(container, capsys):
    rs = _fresh(container)
    pub = _pick('2026-C')
    rs._check_battery()
    rs._check_battery()   # the same pick again is not news
    assert rs.battery == '2026-C' and _active() == '2026-C'
    assert capsys.readouterr().out.splitlines() == ['BATTERY: 2026-C']
    pub.set('2026-D')     # a wrong pick, corrected
    rs._check_battery()
    assert capsys.readouterr().out.splitlines() == ['BATTERY: 2026-D   (was 2026-C)']


def test_anything_else_is_ignored(container, capsys):
    rs = _fresh(container)
    for junk in ('', '?', '2026-Z', '2026-c', 'BATTERY: 2026-C'):
        pub = _pick(junk)
        rs._check_battery()
        assert rs.battery == '?' and _active() == '?'
    assert capsys.readouterr().out == ''
    del pub
