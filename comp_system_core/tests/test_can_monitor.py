"""
Pin the CAN bus monitor (helpers/can_monitor.py): every bus is read and published, a rise in bus-off / TX-full is reported,
and a bus that cannot be read never stops the others or the robot.
"""

from types import SimpleNamespace

import ntcore
import wpilib

import constants
from helpers.can_monitor import CanMonitor


def _status(util=12.5, off=0, full=0, rx=0, tx=0):
    return SimpleNamespace(percent_bus_utilization=util, bus_off_count=off, tx_full_count=full,
                           receive_error_count=rx, transmit_error_count=tx)


def _read(name, key):
    inst = ntcore.NetworkTableInstance.get_default()
    return inst.get_double_topic(f'{constants.can_prefix}/{name}/{key}').subscribe(-1.0).get()


def test_every_bus_is_published(monkeypatch):
    by_port = {constants.k_can_bus_drive: _status(40.0, 0, 0, 3, 1),
               constants.k_can_bus_turn: _status(11.0),
               constants.k_can_bus_other: _status(22.0, 0, 0, 0, 0)}
    mon = CanMonitor(status_fn=lambda port: by_port[port], phoenix_fn=lambda: 39.5)
    got = mon.sample()
    assert set(got) == {'can_s0', 'can_s1', 'can_s2'}
    assert _read('can_s0', 'utilization') == 40.0
    assert _read('can_s0', 'rx_errors') == 3.0 and _read('can_s0', 'tx_errors') == 1.0
    assert _read('can_s1', 'utilization') == 11.0
    assert _read('can_s2', 'utilization') == 22.0
    inst = ntcore.NetworkTableInstance.get_default()
    assert inst.get_double_topic(f'{constants.can_prefix}/can_s0/phoenix_utilization_pct').subscribe(-1.0).get() == 39.5


def test_a_rise_in_bus_off_or_tx_full_is_printed_once(capsys):
    state = {'off': 0, 'full': 0}
    mon = CanMonitor(status_fn=lambda port: _status(off=state['off'], full=state['full']), phoenix_fn=lambda: None)
    mon.sample()                                      # first reading is the baseline: nothing to report
    assert capsys.readouterr().out == ''
    state['off'] = 1
    mon.sample()
    out = capsys.readouterr().out
    assert out.count('bus-off') == 3, 'all three buses see the same fake counter, so three lines'
    mon.sample()                                      # unchanged: quiet
    assert capsys.readouterr().out == ''
    state['full'] = 5
    mon.sample()
    assert 'TX queue full' in capsys.readouterr().out


def test_an_unreadable_bus_does_not_stop_the_others():
    def flaky(port):
        if port == constants.k_can_bus_turn:
            raise RuntimeError('no such bus')
        return _status(7.0)
    mon = CanMonitor(status_fn=flaky, phoenix_fn=lambda: None)
    got = mon.sample()
    assert set(got) == {'can_s0', 'can_s2'}
    mon.counter = mon.period - 1
    mon.periodic()                                     # and periodic() never raises


def test_it_samples_once_per_period():
    calls = []
    mon = CanMonitor(period_loops=50, status_fn=lambda port: calls.append(port) or _status(), phoenix_fn=lambda: None)
    for _ in range(49):
        mon.periodic()
    assert calls == []
    mon.periodic()
    assert len(calls) == 3                             # the 50th loop reads the three buses


def test_the_real_wpilib_call_works_in_simulation():
    mon = CanMonitor(phoenix_fn=lambda: None)          # no injected status function: the real RobotController call
    got = mon.sample()
    assert set(got) == {'can_s0', 'can_s1', 'can_s2'}, "RobotController.get_can_status must work for all three SystemCore buses"
    assert _read('can_s1', 'utilization') >= 0.0
    assert wpilib.RobotController.get_can_status(constants.k_can_bus_drive) is not None
