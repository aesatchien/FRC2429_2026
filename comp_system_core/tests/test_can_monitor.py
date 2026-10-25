"""
Pin the CAN bus monitor (helpers/can_monitor.py): every bus is read and published, the Phoenix fallback works when WPILib's
call fails (which is what happened on the competition SystemCore), a failure is SAID rather than swallowed, a rise in bus-off /
TX-full is reported, and a bus that cannot be read never stops the others or the robot.
"""

from types import SimpleNamespace

import ntcore

import constants
from helpers.can_monitor import CanMonitor


def _status(util=12.5, off=0, full=0, rx=0, tx=0):
    return SimpleNamespace(percent_bus_utilization=util, bus_off_count=off, tx_full_count=full,
                           receive_error_count=rx, transmit_error_count=tx)


def _phoenix(util=2.5, off=0, full=0, rec=0, tec=0):
    return SimpleNamespace(bus_utilization=util, bus_off_count=off, tx_full_count=full, rec=rec, tec=tec)


def _read(name, key, kind='double'):
    inst = ntcore.NetworkTableInstance.get_default()
    topic = (inst.get_double_topic if kind == 'double' else inst.get_string_topic)(f'{constants.can_prefix}/{name}/{key}')
    return topic.subscribe(-1.0 if kind == 'double' else '?').get()


def _broken_wpilib(port):
    raise RuntimeError('HAL_CAN_GetCANStatus is not implemented')


def test_every_bus_is_published_from_wpilib():
    by_port = {constants.k_can_bus_drive: _status(40.0, 0, 0, 3, 1),
               constants.k_can_bus_turn: _status(11.0),
               constants.k_can_bus_other: _status(22.0)}
    mon = CanMonitor(status_fn=lambda port: by_port[port], phoenix_status_fn=lambda name: _phoenix(39.5))
    assert set(mon.sample()) == {'can_s0', 'can_s1', 'can_s2'}
    assert _read('can_s0', 'utilization') == 40.0
    assert _read('can_s0', 'rx_errors') == 3.0 and _read('can_s0', 'tx_errors') == 1.0
    assert _read('can_s1', 'utilization') == 11.0 and _read('can_s2', 'utilization') == 22.0
    assert _read('can_s0', 'source', 'string') == 'wpilib'
    assert _read('can_s0', 'phoenix_utilization_raw') == 39.5, "Phoenix's number is published unchanged, no x100"


def test_phoenix_is_used_when_wpilib_fails_and_the_failure_is_said(capsys):
    """The real robot: get_can_status raised for every bus, nothing was published, and nobody was told."""
    calls = {'n': 0}
    mon = CanMonitor(status_fn=_broken_wpilib, phoenix_status_fn=lambda name: _phoenix(7.0, 0, 0, 2, 1))
    got = mon.sample()
    assert set(got) == {'can_s0', 'can_s1', 'can_s2'}, 'Phoenix must fill in for all three buses'
    assert _read('can_s1', 'utilization') == 7.0 and _read('can_s1', 'rx_errors') == 2.0
    assert _read('can_s1', 'source', 'string') == 'phoenix'
    out = capsys.readouterr().out
    assert out.count('read from phoenix') == 3 and 'not implemented' in out, 'it must say which reader and why'
    mon.sample()
    assert capsys.readouterr().out == '', 'said once, not every second'


def test_when_nothing_can_read_a_bus_it_says_so_once(capsys):
    def no_phoenix(name):
        raise RuntimeError('no CAN device')
    mon = CanMonitor(status_fn=_broken_wpilib, phoenix_status_fn=no_phoenix)
    assert mon.sample() == {}
    out = capsys.readouterr().out
    assert out.count('NO reading') == 3 and 'not implemented' in out and 'no CAN device' in out
    assert _read('can_s2', 'source', 'string') == 'none'
    mon.sample()
    assert capsys.readouterr().out == ''


def test_a_rise_in_bus_off_or_tx_full_is_printed_once(capsys):
    state = {'off': 0, 'full': 0}
    mon = CanMonitor(status_fn=lambda port: _status(off=state['off'], full=state['full']), phoenix_status_fn=lambda n: _phoenix())
    mon.sample()                                      # first reading is the baseline: nothing to report except the source notes
    capsys.readouterr()
    state['off'] = 1
    mon.sample()
    assert capsys.readouterr().out.count('bus-off') == 3, 'all three buses see the same fake counter, so three lines'
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

    def no_phoenix_for_s1(name):
        if name == 'can_s1':
            raise RuntimeError('no such bus')
        return _phoenix()
    mon = CanMonitor(status_fn=flaky, phoenix_status_fn=no_phoenix_for_s1)
    assert set(mon.sample()) == {'can_s0', 'can_s2'}
    mon.counter = mon.period - 1
    mon.periodic()                                     # and periodic() never raises


def test_it_samples_once_per_period():
    calls = []
    mon = CanMonitor(period_loops=50, status_fn=lambda port: calls.append(port) or _status(), phoenix_status_fn=lambda n: _phoenix())
    for _ in range(49):
        mon.periodic()
    assert calls == []
    mon.periodic()
    assert len(calls) == 3                             # the 50th loop reads the three buses


def test_the_real_calls_do_not_raise_in_simulation():
    mon = CanMonitor()                                 # no injected functions: whatever works here, through the real fallback chain
    got = mon.sample()
    assert set(got) == {'can_s0', 'can_s1', 'can_s2'}
    assert _read('can_s1', 'utilization') >= 0.0
    assert _read('can_s1', 'source', 'string') in ('wpilib', 'phoenix')
