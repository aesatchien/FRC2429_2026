"""
The datalog must wait for the USB stick, not race it.

On the SystemCore the robot program starts ~2-3 s before the stick mounts at /U.  PR #16
checked once at startup, so logs went to the stick or to internal storage depending on who
won that race on each boot.  helpers/log_start.py waits; these pin down its decision.
"""

from helpers.log_start import DeferredLogStart, decide

LIMIT = DeferredLogStart.k_wait_s


def test_waits_while_the_stick_is_not_mounted_yet():
    # exactly the 2026-10-10 boot: program running, /U not mounted 1 s in
    assert decide(is_real=True, mounted=False, waited_s=1.0, wait_limit_s=LIMIT) == 'wait'


def test_logs_to_the_stick_as_soon_as_it_mounts():
    assert decide(is_real=True, mounted=True, waited_s=3.0, wait_limit_s=LIMIT) == 'usb'


def test_falls_back_to_internal_only_after_the_wait():
    assert decide(is_real=True, mounted=False, waited_s=LIMIT - 0.1, wait_limit_s=LIMIT) == 'wait'
    assert decide(is_real=True, mounted=False, waited_s=LIMIT, wait_limit_s=LIMIT) == 'internal'


def test_simulation_starts_immediately():
    assert decide(is_real=False, mounted=False, waited_s=0.0, wait_limit_s=LIMIT) == 'sim'


def test_nothing_starts_logging_during_construction():
    # The whole fix depends on the log NOT being started before robot_periodic() polls.  If
    # anything calls DataLogManager.start()/get_log() during construction again, WPILib starts
    # the log right there on internal storage and the later start() is silently ignored.
    import pathlib, re
    project = pathlib.Path(__file__).resolve().parent.parent
    offenders = []
    for path in sorted(project.rglob('*.py')):
        rel = path.relative_to(project).as_posix()
        if rel.startswith(('tests/', 'commands/deprecated/')) or rel == 'helpers/log_start.py':
            continue
        for n, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
            code = line.split('#', 1)[0]
            if re.search(r'DataLogManager\.(start|get_log)\b|SignalLogger\.(start|set_path)\b', code):
                offenders.append(f'{rel}:{n}: {line.strip()}')
    assert not offenders, "start the logs only in helpers/log_start.py:\n  " + "\n  ".join(offenders)


# ---- one folder per boot for the .hoot, the .wpilog and the console log
def test_session_folder_name_is_local_date_and_time():
    import time
    from helpers.log_start import session_name
    t = time.mktime((2026, 11, 15, 14, 3, 27, 0, 0, -1))
    assert session_name(t) == '2026-11-15_14-03-27'


def test_finds_the_folder_phoenix_just_made(tmp_path):
    from helpers.log_start import new_session_dir, subdirs
    (tmp_path / '2026-11-14_10-00-00').mkdir()
    (tmp_path / 'stray.wpilog').write_text('')
    before = subdirs(str(tmp_path))
    assert new_session_dir(str(tmp_path), before) == '', 'nothing new yet: keep waiting'
    (tmp_path / '2026-11-15_14-03-27').mkdir()
    assert new_session_dir(str(tmp_path), before) == str(tmp_path / '2026-11-15_14-03-27')
    assert new_session_dir(str(tmp_path / 'missing'), set()) == '', 'a folder that is gone is not an error'
