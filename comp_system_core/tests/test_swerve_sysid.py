"""
Pin the shop-only SysId test: invisible by default, and when switched on it publishes four commands
that actually drive the modules and stop them again.

The container is built once per session (conftest.py), with constants.k_enable_sysid at its default,
so the "switched on" cases call register_sysid() on that container's swerve instead of building a
second container.
"""

import commands2
import pytest
import wpilib.simulation as wsim

import constants
from subsystems import swerve_sysid
from subsystems.swerve_sysid import register_sysid


def test_sysid_is_off_by_default(container):
    assert constants.k_enable_sysid is False, "k_enable_sysid must stay False in the repo - it is shop-only"
    assert container.sysid is None, "nothing SysId should be built when the switch is off"


def test_register_does_nothing_when_off(container, monkeypatch):
    monkeypatch.setattr(constants, 'k_enable_sysid', False)
    assert register_sysid(container.swerve) is None


def test_four_commands_drive_and_stop_the_modules(container, monkeypatch):
    monkeypatch.setattr(constants, 'k_enable_sysid', True)
    published = []
    monkeypatch.setattr(swerve_sysid.dashboard.SmartDashboard, 'put_data',
                        staticmethod(lambda key, obj: published.append((key, obj))))
    routine = register_sysid(container.swerve)
    assert routine is not None
    assert [k for k, _ in published] == [f'{constants.command_prefix}/SysId/{n}' for n in
            ('1 QuasistaticForward', '2 QuasistaticReverse', '3 DynamicForward', '4 DynamicReverse')]

    calls = []
    modules = container.swerve.swerve_modules
    for m in modules:
        monkeypatch.setattr(m.drive_motor, 'set_voltage', lambda v, m=m: calls.append((m, v)))

    # commands only run while enabled
    wsim.DriverStationSim.set_enabled(True)
    wsim.DriverStationSim.set_ds_attached(True)
    wsim.DriverStationSim.notify_new_data()
    scheduler = commands2.CommandScheduler.get_instance()
    try:
        command = published[2][1]                          # 3 DynamicForward: a step, so voltage is nonzero at once
        command.schedule()
        for _ in range(4):
            scheduler.run()
        assert scheduler.is_scheduled(command)
        assert {m for m, _ in calls} == set(modules), "all four drive motors must get the voltage"
        assert any(v > 0 for _, v in calls), "a forward dynamic step must apply positive volts"
        command.cancel()
        scheduler.run()
        assert calls[-1][1] == 0, "cancelling must hand the motors 0 V, not leave the last step on"
    finally:
        wsim.DriverStationSim.set_enabled(False)
        wsim.DriverStationSim.notify_new_data()
