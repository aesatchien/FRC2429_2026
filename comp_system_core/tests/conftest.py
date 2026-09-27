"""
Shared fixtures.

RobotContainer is SESSION scoped and must stay that way.  It can only be constructed once
per process: REV raises "A SparkMax instance has already been created with this device ID"
on a second one, so two test modules each building their own would fail the second module.
"""

import pytest


@pytest.fixture(scope='session')
def container():
    """The one and only RobotContainer.

    Building it IS a test - it is everything robotInit does: every subsystem constructor and
    therefore every motor adapter, every button binding and the commands it binds, every
    dashboard command, and every autonomous routine in the chooser.  If this raises, the
    robot program would not have started.
    """
    from robotcontainer import RobotContainer
    return RobotContainer()
