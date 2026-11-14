"""The dashboard's half of the Quest double-tap fix: bring the QuestNav app back with adb.

WHY THIS EXISTS
---------------
A double tap (or a bump) drops the Quest into passthrough and suspends the QuestNav app.  The
robot notices its pose stream stop and raises /QuestNav/quest_in_passthrough; this module sees
that and runs `adb shell am start ...` to put the app back in front.  It used to be spread
through ui_updater.py.

THE SWITCH
----------
A QuestNav update lets the double tap be disabled on the headset.  The robot says whether it is
still watching for one on /QuestNav/check_for_double_tap (QuestConstants.k_check_for_double_tap
in the robot code).  When that is False this module does nothing: no adb, DTAP indicator and
test button greyed out.  There is deliberately NO matching flag in config.py - one setting, on
the robot, so the two cannot disagree.  Robot code too old to publish the topic gets the
default, True, which is how it has always behaved.
"""

import os
import subprocess
import threading
import time

import ntcore

import config
from quest_address import quest_ip, ADB_PORT

STYLE_NOT_IN_USE = "border: 7px; border-radius: 7px; background-color:rgb(180, 180, 180); color:rgb(0, 0, 0);"


def _stamp():
    return time.strftime('%H:%M:%S')


class QuestDoubleTapFixer:
    def __init__(self, ui):
        self.ui = ui
        self.start_time = time.time()
        self.dtap_start_time = 0.0
        self.dtap_last_fix_time = 0.0
        self.dtap_retries = 0
        self.enabled = None  # unknown until the first update(), so the first one paints the widgets

        inst = ntcore.NetworkTableInstance.getDefault()
        self.enabled_sub = inst.getBooleanTopic(f'{config.quest_prefix}/check_for_double_tap').subscribe(True)
        # who is connected to the robot's NT server - the Quest's real IP is in here (see quest_address.py)
        self.nt_clients_sub = inst.getRawTopic("$clients").subscribe("msgpack", b"")

    def update(self):
        """Call once per GUI cycle, BEFORE the generic widget updaters."""
        dtap_props = self.ui.widget_dict.get('qlabel_questnav_dtap_indicator')
        sub = dtap_props.get('subscriber') if dtap_props else None
        if not sub:
            return

        enabled = self.enabled_sub.get()
        if enabled != self.enabled:
            self._set_enabled(enabled, dtap_props)
        if enabled:
            self._watch(sub.get())

    def _set_enabled(self, enabled, dtap_props):
        if self.enabled is not None:  # not worth a line at startup
            print(f"[{_stamp()}] Robot says check_for_double_tap = {enabled}: QuestNav adb fix {'ON' if enabled else 'OFF'}.", flush=True)
        self.enabled = enabled
        self.dtap_start_time = 0.0
        self.dtap_retries = 0
        button = getattr(self.ui, 'qt_button_quest_test_dt', None)
        if button is not None:
            button.setEnabled(enabled)
        widget = dtap_props.get('widget')
        if enabled:
            dtap_props['last_value'] = None  # make the indicator updater repaint it
        elif widget:
            # the topic stays False from here on, so the indicator updater will not repaint over this
            dtap_props['last_value'] = False
            widget.setStyleSheet(STYLE_NOT_IN_USE)

    def _watch(self, is_in_passthrough):
        """Restart the app via adb if the robot says the Quest is stuck in passthrough."""
        current_time = time.time() - self.start_time

        if not is_in_passthrough:
            self.dtap_start_time = 0.0
            if self.dtap_retries > 0:
                print(f"[{_stamp()}] QuestNav passthru resolved.", flush=True)
                self.dtap_retries = 0
            return

        # Start the timer if it just became True
        if self.dtap_start_time == 0.0:
            self.dtap_start_time = current_time
            print(f"[{_stamp()}] We have detected a QuestNav passthru condition.", flush=True)

        if (current_time - self.dtap_start_time) <= config.QUESTNAV_PASSTHRU_ADB_DELAY:
            return
        if (current_time - self.dtap_last_fix_time) <= config.QUESTNAV_ADB_COOLDOWN_S:
            return

        if self.dtap_retries < config.QUESTNAV_ADB_MAX_RETRIES:
            self.dtap_retries += 1
            print(f"[{_stamp()}] QuestNav has been in passthrough for > {config.QUESTNAV_PASSTHRU_ADB_DELAY:.2f}s. Attempting ADB fix ({self.dtap_retries}/{config.QUESTNAV_ADB_MAX_RETRIES}).", flush=True)
            address = self.adb_address()
            threading.Thread(target=self._run_adb_fix, args=(address,), daemon=True).start()
            print(f"[{_stamp()}] ADB fix sent to {address}. Waiting for cooldown...", flush=True)
        elif self.dtap_retries == config.QUESTNAV_ADB_MAX_RETRIES:
            print(f"[{_stamp()}] QuestNav ADB fix max retries ({config.QUESTNAV_ADB_MAX_RETRIES}) reached. Giving up.", flush=True)
            self.dtap_retries += 1  # Increment once more so we don't spam the 'Giving up' message

        self.dtap_last_fix_time = current_time
        self.dtap_start_time = current_time  # it has to stay True for the full delay again to re-trigger

    def adb_address(self):
        """The Quest's adb address RIGHT NOW: its IP as the robot's NT server sees it, falling back to
        config (hard-coded, or whatever the last Ping Quest found) only if the Quest is not connected.
        This used to be config alone, so a changed DHCP address meant adb went to the old one."""
        ip = quest_ip(self.nt_clients_sub.get())
        if ip is None:
            return config.QUESTNAV_ADB_ADDRESS
        address = f"{ip}:{ADB_PORT}"
        if address != config.QUESTNAV_ADB_ADDRESS:
            print(f"[{_stamp()}] Quest is at {address} (from the robot's NT client list), "
                  f"not {config.QUESTNAV_ADB_ADDRESS} - using {address}", flush=True)
            config.QUESTNAV_ADB_ADDRESS = address  # so Ping Quest and the test button agree
        return address

    @staticmethod
    def _adb_path():
        return os.path.join(os.path.dirname(__file__), "adb", "adb.exe")

    @classmethod
    def _run_adb_fix(cls, address):
        """Background thread: connect first (`adb -s` does not connect, and the adb server may only
        know the OLD address), then bring QuestNav back to the front.  Prints adb's answers so a
        failure is visible instead of a silent Popen."""
        adb_path = cls._adb_path()
        creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
        for cmd in ([adb_path, "connect", address],
                    [adb_path, "-s", address, "shell", "am", "start", "-n",
                     "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity"]):
            try:
                res = subprocess.run(cmd, capture_output=True, text=True, timeout=5, creationflags=creationflags)
                out = (res.stdout + res.stderr).strip().replace("\n", " | ")
                print(f"[{_stamp()}] adb {cmd[1]} {address}: {out or res.returncode}", flush=True)
            except Exception as e:
                print(f"[{_stamp()}] adb {cmd[1]} {address} failed: {e}", flush=True)
                return

    def test_double_tap(self):
        """Fires an ADB command to simulate/test Quest passthrough by returning to the Home screen.
        None of these do what they are supposed to - the HOME actually kills and restarts (seriously messes up the quest), and the FocusPlaceholderActivity doesn't work
        """
        print(f"[{_stamp()}] Testing QuestNav DTAP (Home Intent)...", flush=True)
        try:
            # cmd = [adb, "-s", address, "shell", "am", "start", "-a", "android.intent.action.MAIN", "-c", "android.intent.category.HOME"]
            cmd = [self._adb_path(), "-s", self.adb_address(), "shell", "am", "start", "-n", "com.oculus.vrshell/.FocusPlaceholderActivity"]
            subprocess.Popen(cmd)
        except Exception as e:
            print(f"[{_stamp()}] Failed to execute QuestNav DTAP test: {e}", flush=True)
