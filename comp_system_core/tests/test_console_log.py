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
    log = ConsoleLog()
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
    log = ConsoleLog()
    assert log.start()
    log.open_in(str(tmp_path))
    log.stop()
    log.stop()                                                     # twice is fine
    say('after stop')
    assert 'after stop' in capfd.readouterr().out, "stdout must work normally again once stopped"


def test_long_waits_keep_the_newest_lines_and_say_how_many_were_dropped(tmp_path, monkeypatch):
    import helpers.console_log as cl
    monkeypatch.setattr(cl, 'MAX_HELD_LINES', 5)
    log = ConsoleLog()
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
    log = ConsoleLog()
    assert log.start()
    try:
        assert log.open_in(str(blocker / 'logs')) == ''            # a folder cannot be made under a file
        say('still printing')
    finally:
        log.stop()
    assert 'still printing' in capfd.readouterr().out
