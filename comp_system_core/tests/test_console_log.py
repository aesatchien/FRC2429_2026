"""
Pin the robot console log (helpers/console_log.py).

WHY THIS EXISTS: nothing the robot printed was being saved - WPILib's own `console` entry was empty in every SoCal
Showdown match log - so after a match there was no record of the warnings, loop overruns and errors that explain it.
These tests redirect this test process's real stdout / stderr descriptors, which is what the robot does.
"""

import os
import re
import sys

from helpers.console_log import ConsoleLog

def say(text, err=False):
    """print() the way the robot does: through the process's real stdout / stderr (fd 1 / 2).  Under pytest, sys.stdout is
    pytest's own stream and never reaches the descriptor, but sys.__stdout__ is the real one."""
    stream = sys.__stderr__ if err else sys.__stdout__
    print(text, file=stream, flush=True)


LINE = re.compile(r'^(\d+\.\d{3}) (\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3}Z) ([OE]) (.*)$')


def _lines(path):
    return [ln.rstrip('\n') for ln in open(path, encoding='utf-8')]


def _records(path):
    return [LINE.match(ln).groups() for ln in _lines(path) if not ln.startswith('#')]


def test_python_and_native_output_are_saved_with_timestamps_and_streams(tmp_path, capfd):
    log = ConsoleLog(fd_level=True)
    assert log.start()
    try:
        say('before the folder is known')                          # held in memory until open_in()
        path = log.open_in(str(tmp_path), note='test')
        say('python print')
        os.write(1, b'native stdout line\n')                       # what C++ std::cout / printf does
        os.write(2, b'native stderr line\n')                       # what C++ std::cerr does (watchdog warnings, phoenix)
        say('python stderr', err=True)
    finally:
        log.stop()

    recs = _records(path)
    text = {r[3]: r[2] for r in recs}
    assert text['before the folder is known'] == 'O'               # nothing printed during the wait is lost
    assert text['python print'] == 'O'
    assert text['native stdout line'] == 'O', "native output must be caught: that is where the warnings come from"
    assert text['native stderr line'] == 'E'
    assert text['python stderr'] == 'E'
    stamps = [float(r[0]) for r in recs]
    assert stamps == sorted(stamps), "robot-time stamps must not go backwards"

    head = [ln for ln in _lines(path) if ln.startswith('#')]
    assert any('wpilog clock' in ln for ln in head), "the header must say which clock the stamps use"

    out = capfd.readouterr()                                       # and the real console still got all of it, untouched
    assert 'python print' in out.out and 'native stdout line' in out.out
    assert 'native stderr line' in out.err and 'python stderr' in out.err


def test_console_is_given_back_on_stop(tmp_path, capfd):
    log = ConsoleLog(fd_level=True)
    assert log.start()
    log.open_in(str(tmp_path))
    log.stop()
    log.stop()                                                     # twice is fine
    say('after stop')
    assert 'after stop' in capfd.readouterr().out, "stdout must work normally again once stopped"


def test_long_waits_keep_the_newest_lines_and_say_how_many_were_dropped(tmp_path, monkeypatch):
    import helpers.console_log as cl
    monkeypatch.setattr(cl, 'MAX_HELD_LINES', 5)
    log = ConsoleLog(fd_level=True)
    assert log.start()
    try:
        for i in range(12):
            say(f'line {i}')
        # let the pump threads drain the pipe before opening the folder
        import time
        deadline = time.time() + 2
        while time.time() < deadline and len(log._held) + log._dropped < 12:
            time.sleep(0.01)
        path = log.open_in(str(tmp_path))
    finally:
        log.stop()
    lines = _lines(path)
    assert any('dropped' in ln for ln in lines if ln.startswith('#'))
    assert 'line 11' in ''.join(lines) and 'line 0' not in ''.join(lines)


def test_an_unwritable_folder_does_not_break_the_robot(tmp_path, capfd):
    blocker = tmp_path / 'a_file'
    blocker.write_text('x')
    log = ConsoleLog(fd_level=True)
    assert log.start()
    try:
        assert log.open_in(str(blocker / 'logs')) == ''            # a folder cannot be made under a file
        say('still printing')
    finally:
        log.stop()
    assert 'still printing' in capfd.readouterr().out


# ---------------------------------------------------------------------------------------------------------
# Python-level mode: what the simulator on Windows uses.  Redirecting the file descriptors there killed the robot program
# in its constructor ("OSError: [WinError 1] Incorrect function" on the next print()), because Python's console writer
# keeps the console's OS handle and the redirect closes it.  So this mode must never touch the descriptors.
# ---------------------------------------------------------------------------------------------------------
def test_python_mode_never_touches_the_file_descriptors(tmp_path, monkeypatch):
    calls = []
    for name in ('dup2', 'dup', 'pipe'):
        real = getattr(os, name)
        monkeypatch.setattr(os, name, lambda *a, _n=name, _r=real, **k: (calls.append(_n), _r(*a, **k))[1])
    log = ConsoleLog(fd_level=False)
    try:
        assert log.start()
        log.open_in(str(tmp_path))
        print('a line')
    finally:
        log.stop()
        monkeypatch.undo()                                # the guard covers only the console log's own window
    assert calls == [], f'the Python-level mode must not redirect file descriptors, but called os.{calls}'


def test_python_mode_saves_prints_stderr_and_logging_and_still_prints(tmp_path, capsys):
    import logging
    logger = logging.getLogger('console_log_test')
    logger.propagate = False
    handler = logging.StreamHandler(sys.stderr)          # made BEFORE start, like robotpy's: holds the original stderr
    logger.addHandler(handler)
    log = ConsoleLog(fd_level=False)
    try:
        assert log.start()
        print('held print')                              # before the folder is known
        path = log.open_in(str(tmp_path))
        print('plain print')
        print('two', 'parts', end='')                    # a line finished by a later write
        print(' of one line')
        sys.stderr.write('stderr line\n')
        logger.warning('logging line')
    finally:
        log.stop()
        logger.removeHandler(handler)
    text = {r[3]: r[2] for r in _records(path)}
    assert text['held print'] == 'O' and text['plain print'] == 'O'
    assert text['two parts of one line'] == 'O'
    assert text['stderr line'] == 'E'
    assert text['logging line'] == 'E', "robotpy's logging handler holds stderr: its lines must be caught too"
    out = capsys.readouterr()
    assert 'plain print' in out.out and 'stderr line' in out.err, "the real streams still receive everything"


def test_python_mode_gives_the_streams_back(tmp_path):
    before_out, before_err = sys.stdout, sys.stderr
    log = ConsoleLog(fd_level=False)
    assert log.start()
    assert sys.stdout is not before_out
    log.stop()
    assert sys.stdout is before_out and sys.stderr is before_err
