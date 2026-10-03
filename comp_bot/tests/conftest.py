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


# ---------------------------------------------------------------------------------------
# CI only: exit as soon as pytest has reported, before native libraries tear down.
#
# On GitHub's runners this suite sometimes passes every test and THEN dies while Python is
# shutting down - "terminate called without an active exception", exit 134 (abort) or 139
# (segfault).  That is a native background thread (Phoenix / REV / ntcore) still running at
# interpreter exit, not a test failure, but it turns the job red at random.
#
# So on CI (GitHub sets CI=true) we leave with pytest's OWN exit status the moment it has
# printed its summary: failures still fail, passes pass, and the crash-prone teardown never
# runs.  Locally nothing changes, so a real shutdown bug still shows up on a laptop.
#
# NOT in the isolated-test child processes (test_simulation runs in one).  Those report
# their result to the parent over a pipe after pytest returns, and WPILib's runner already
# keeps them alive until the parent kills them - for this same "interpreter badness".
# Exiting a child early loses its result: "subprocess exited with exit code 0".
# ---------------------------------------------------------------------------------------
import multiprocessing
import os
import sys


def pytest_sessionfinish(session, exitstatus):
    session.config._ci_exit_status = int(exitstatus)


def pytest_unconfigure(config):
    status = getattr(config, '_ci_exit_status', None)
    is_child = multiprocessing.parent_process() is not None
    if os.environ.get('CI') == 'true' and status is not None and not is_child:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(status)
