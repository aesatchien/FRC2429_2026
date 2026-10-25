"""
Save everything the robot program prints to a file, with timestamps that line up with the .wpilog and the .hoot.

WHY THIS EXISTS
---------------
The Driver Station console and the terminal show what the program says as it runs: our own prints, robotpy's INFO
lines, and the native libraries' messages (WPILib watchdog and loop-overrun warnings, Phoenix and REV errors).  None of it
was being saved.  WPILib has a `console` entry in the .wpilog, but in the SystemCore logs from the 2026 SoCal Showdown it
was declared and empty in every one of the 16 matches checked, and so was `messages`.  So after a match there was no way
to see what the robot had said, or when.

HOW
---
start() redirects the process's file descriptors 1 (stdout) and 2 (stderr) into pipes.  A thread per pipe forwards every
byte unchanged to the real console, and also timestamps each line.  Working on the descriptors, not on sys.stdout, is what
catches the native C++ output as well as Python's print().

Until the log folder is known the lines are held in memory (the first seconds of a boot, while the program waits for the
USB stick to mount, see helpers/log_start.py).  open_in(folder) then writes them out and carries on appending.  The folder
is the one the .wpilog uses (the USB stick, or the internal storage after the wait), so the console file ends up next to
the data it explains.

FILE FORMAT  (console_<boot UTC>.log, plain text, one line per console line)
    # comment / header lines start with '#'
    <robot time s> <UTC time> <O|E> <text>
    272130.412 2026-10-25T17:27:59.874Z O Binding driver buttons

  * robot time = wpilib.Timer.get_timestamp(), seconds: the clock the .wpilog timestamps use (microseconds there).  This is
    the reliable key.  Scope2429 matches console lines to wpilog times with it, and the hoot is lined up to the wpilog
    separately (helpers/hoot.py), so a console line can be placed on the hoot too.
  * UTC time = the controller's wall clock.  It can jump when the Driver Station sets the time after boot, so it only helps
    to find the right file, not to place a line precisely.  The .hoot folder names are local wall-clock times.
  * O = stdout, E = stderr.

NEVER BLOCK THE ROBOT.  If a pipe stopped being read, the next print would block once it filled (64 KB) and freeze the robot
loop.  So every thread body is wrapped, any failure puts the original descriptors straight back, and the file is only
written from the threads, never from the code that printed.
"""

import atexit
import datetime
import os
import sys
import threading
import time

import wpilib

MAX_HELD_LINES = 5000       # before the folder is known; the oldest are dropped (and counted)
FSYNC_EVERY_S = 2.0         # push the file to the stick this often: a power cut then costs seconds, not minutes


def _utc() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def format_line(t: float, utc: str, stream: str, text: str) -> str:
    return f'{t:.3f} {utc} {stream} {text}\n'


class ConsoleLog:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._held = []                  # lines waiting for a folder: (t, utc, stream, text)
        self._dropped = 0
        self._file = None
        self.path = None
        self._saved = {}                 # original fd -> its duplicate, so the console keeps working
        self._threads = []
        self._active = False
        self._boot_t = wpilib.Timer.get_timestamp()
        self._boot_utc = _utc()
        self._last_sync = time.monotonic()

    # ------------------------------------------------------------------ start / stop
    def start(self) -> bool:
        """Redirect stdout and stderr.  Returns False (and changes nothing) if that is not possible here."""
        if self._active:
            return True
        try:
            for stream in (sys.stdout, sys.stderr):
                try:
                    stream.flush()
                    stream.reconfigure(line_buffering=True)    # so a print is stamped when it happens, not at the next 8 KB
                except Exception:  # noqa
                    pass
            pipes = {}
            for fd, tag in ((1, 'O'), (2, 'E')):
                self._saved[fd] = os.dup(fd)
                r, w = os.pipe()
                os.dup2(w, fd)
                os.close(w)
                pipes[fd] = (r, tag)
        except Exception as e:  # noqa
            self._restore()
            print(f'  console log not started: {e}')
            return False
        self._active = True
        for fd, (r, tag) in pipes.items():
            t = threading.Thread(target=self._pump, args=(r, self._saved[fd], tag), name=f'console-{tag}', daemon=True)
            t.start()
            self._threads.append(t)
        threading.Thread(target=self._syncer, name='console-sync', daemon=True).start()
        atexit.register(self.stop)
        return True

    def stop(self) -> None:
        """Give the console back and write whatever is left.  Safe to call twice."""
        if not self._active:
            return
        self._active = False
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.flush()
            except Exception:  # noqa
                pass
        self._redirect_back()                             # puts the console back and drops the pipes' write ends: the threads see EOF
        for t in self._threads:
            t.join(timeout=1.0)                           # they forward and record what was still in the pipes
        self._close_saved()
        with self._lock:
            if self._file is not None:
                try:
                    self._file.flush()
                    os.fsync(self._file.fileno())
                    self._file.close()
                except Exception:  # noqa
                    pass
                self._file = None

    def _redirect_back(self) -> None:
        for fd, saved in list(self._saved.items()):
            try:
                os.dup2(saved, fd)
            except Exception:  # noqa
                pass

    def _close_saved(self) -> None:
        for saved in list(self._saved.values()):
            try:
                os.close(saved)
            except Exception:  # noqa
                pass
        self._saved.clear()

    def _restore(self) -> None:
        """Console back to normal immediately (used when something went wrong)."""
        self._redirect_back()
        self._close_saved()

    # ------------------------------------------------------------------ the folder
    def open_in(self, folder: str, note: str = '') -> str:
        """Start writing to a console_<boot UTC>.log in `folder` and write out what was held.  Returns the path, or '' on failure."""
        with self._lock:
            if self._file is not None:
                return self.path
            try:
                os.makedirs(folder, exist_ok=True)
                stamp = self._boot_utc.replace('-', '').replace(':', '').replace('T', '_')[:15]
                self.path = os.path.join(folder, f'console_{stamp}.log')
                self._file = open(self.path, 'a', encoding='utf-8', errors='replace', buffering=1)
                self._file.write('# Scope2429 robot console log v1: <robot time s> <UTC> <O stdout|E stderr> <text>\n')
                self._file.write(f'# robot time = wpilib.Timer.get_timestamp() = the .wpilog clock; boot at {self._boot_t:.3f} s, {self._boot_utc}\n')
                self._file.write(f'# folder {folder}  {note}\n'.rstrip() + '\n')
                if self._dropped:
                    self._file.write(f'# {self._dropped} earlier lines were dropped while waiting for the log folder\n')
                for t, utc, stream, text in self._held:
                    self._file.write(format_line(t, utc, stream, text))
                self._held.clear()
            except Exception as e:  # noqa
                self._file = None
                self.path = ''
                self._held.append((wpilib.Timer.get_timestamp(), _utc(), 'E', f'console log could not open in {folder}: {e}'))
            return self.path or ''

    # ------------------------------------------------------------------ threads
    def _pump(self, r: int, out_fd: int, tag: str) -> None:
        pending = b''
        try:
            while True:
                try:
                    chunk = os.read(r, 65536)
                except OSError:
                    break
                if not chunk:
                    break
                try:
                    os.write(out_fd, chunk)                       # the real console first, exactly as before
                except Exception:  # noqa
                    pass                                          # a console that went away must not stop the log
                try:
                    now_t, now_utc = wpilib.Timer.get_timestamp(), _utc()
                    pending += chunk
                    *lines, pending = pending.split(b'\n')
                    for ln in lines:
                        self._record(now_t, now_utc, tag, ln.decode('utf-8', 'replace').rstrip('\r'))
                except Exception:  # noqa
                    pending = b''
        except Exception:  # noqa
            self._restore()                                       # whatever happened, never leave the robot printing into a dead pipe
        finally:
            if pending:
                try:
                    self._record(wpilib.Timer.get_timestamp(), _utc(), tag, pending.decode('utf-8', 'replace'))
                except Exception:  # noqa
                    pass

    def _record(self, t: float, utc: str, tag: str, text: str) -> None:
        with self._lock:
            if self._file is not None:
                try:
                    self._file.write(format_line(t, utc, tag, text))
                except Exception:  # noqa
                    self._file = None                             # the stick went away: stop writing, keep the robot alive
            else:
                self._held.append((t, utc, tag, text))
                if len(self._held) > MAX_HELD_LINES:
                    del self._held[0]
                    self._dropped += 1

    def _syncer(self) -> None:
        while self._active:
            time.sleep(FSYNC_EVERY_S)
            with self._lock:
                f = self._file
            if f is not None:
                try:
                    f.flush()
                    os.fsync(f.fileno())
                except Exception:  # noqa
                    pass


_console = None


def start() -> 'ConsoleLog':
    """The one ConsoleLog for this process, started on first call."""
    global _console
    if _console is None:
        _console = ConsoleLog()
        _console.start()
    return _console


def get():
    return _console
