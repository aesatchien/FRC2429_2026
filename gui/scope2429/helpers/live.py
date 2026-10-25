"""
Live NetworkTables 4 connection: feeds a Telemetry (the same object a .wpilog fills) so the dashboard renders
a robot, a simulator or a NetworkTables server exactly as it renders a log.

    sess = LiveSession("10.24.29.2", pose_names, trends)
    sess.start()
    ...                       # every ~30 ms on the UI thread:
    n = sess.poll()           # drain the ntcore queue into sess.tel
    t = sess.now()            # session time (s) to hand to DashRenderer.render
    sess.stop()

Everything runs on the caller's thread (ntcore does its networking on its own threads, we only read its listener
queue), so there is nothing to lock.  Needs `pip install pyntcore`.
"""
from __future__ import annotations

import sys
import time

import core
import telemetry as T

PRESETS = {
    "Robot (roboRIO 10.24.29.2)": "10.24.29.2",
    "Simulator (this computer)": "localhost",
    "roboRIO over USB": "172.22.11.2",
}
NT4_PORT = 5810


def _import_nt():
    try:
        import ntcore
    except ImportError:
        raise RuntimeError("Live connections need the NetworkTables library (pyntcore), which is missing from the "
                           f"Python running Scope2429:\n{sys.executable}\n\nInstall it with:\n"
                           f'"{sys.executable}" -m pip install pyntcore\n\nThen restart Scope2429.')
    return ntcore


def parse_host(text: str):
    """'10.24.29.2' | 'localhost:5810' | 'roborio-2429-frc.local' | '2429' (a team number) -> (host, port, team)"""
    text = text.strip()
    if text.isdigit() and len(text) <= 5:
        return None, NT4_PORT, int(text)
    host, _, port = text.partition(":")
    return host, int(port) if port.isdigit() else NT4_PORT, None


class LiveSession:
    def __init__(self, host: str, pose_names: dict | None = None, trends=(), identity: str = "Scope2429"):
        self.host_text = host.strip()
        self.nt = _import_nt()
        self.ml = core.MatchLog(path="", source="LIVE " + self.host_text)
        self.tel = T.Telemetry(None, self.ml, pose_names, extra=trends)
        self.phase = core.PhaseTracker(self.ml)
        self.identity = identity
        self.inst = None
        self.events = 0
        self._t0_us = 0
        self._last_cw = None
        self._was_connected = None

    # ------------------------------------------------------------------ control
    def start(self):
        nt = self.nt
        host, port, team = parse_host(self.host_text)
        self.inst = nt.NetworkTableInstance.create()
        self.inst.startClient4(self.identity)
        if team is not None:
            self.inst.setServerTeam(team, port)
        else:
            self.inst.setServer(host, port)
        self._t0_us = nt._now()
        # sendAll: every change, not just the latest per period, so fast signals plot properly
        opts = nt.PubSubOptions(sendAll=True, periodic=0.02)
        self._sub = nt.MultiSubscriber(self.inst, ["/"], opts)
        self._poller = nt.NetworkTableListenerPoller(self.inst)
        self._poller.addListener(self._sub, nt.EventFlags.kValueAll | nt.EventFlags.kImmediate)

    def stop(self):
        if self.inst is not None:
            try:
                self.inst.stopClient()
                self.nt.NetworkTableInstance.destroy(self.inst)
            except Exception:  # noqa
                pass
            self.inst = None

    @property
    def connected(self) -> bool:
        return self.inst is not None and self.inst.isConnected()

    def server(self) -> str:
        """Address of the server we are connected to ('' if not connected)."""
        if not self.connected:
            return ""
        try:
            c = self.inst.getConnections()
            return c[0].remote_ip if c else ""
        except Exception:  # noqa
            return ""

    def now(self) -> float:
        return (self.nt._now() - self._t0_us) / 1e6

    # ------------------------------------------------------------------ data
    def set_trends(self, names):
        """Entries to record for the trend plots.  They are only kept from now on."""
        self.tel.extra = set(names)

    def poll(self) -> int:
        """Move everything ntcore has queued into the Telemetry; returns the number of updates."""
        if self.inst is None:
            return 0
        now = self.now()
        tel, ml = self.tel, self.ml
        n = 0
        for ev in self._poller.readQueue():
            d = ev.data
            vv = getattr(d, "value", None)
            if vv is None:
                continue
            topic = d.topic
            name = "NT:" + topic.getName()
            if not tel.want(name) and not name.startswith(("NT:/DriverStation/", "NT:/FMSInfo/")):
                continue
            t = (vv.time() - self._t0_us) / 1e6
            if abs(t - now) > 2.0:            # values from before we connected carry old robot timestamps
                t = now
            typ = topic.getTypeString()
            val = vv.value()
            tel.ingest(name, typ, t, val)
            n += 1
            if name == "NT:/DriverStation/ControlWord":
                self._control_word(t, val)
        self.events += n
        up = self.connected
        if up != self._was_connected:              # this program's own link to the robot (a Wi-Fi drop shows up here)
            if self._was_connected is not None or not up:
                self.tel.net_events.append((now, "self", up))
            self._was_connected = up
        ml.t_last = max(ml.t_last, now)
        self._driver_station(now)
        return n

    def _control_word(self, t, val):
        if isinstance(val, (bytes, bytearray)):
            mode, en, _fms = core._decode_controlword(bytes(val))
            self.phase.feed(t, mode, en)

    def _driver_station(self, now):
        """Alliance, match type / number and event name from the /DriverStation entries."""
        ml, sig = self.ml, self.tel.sig

        def last(name, default=None):
            s = sig.get(name)
            return default if s is None or len(s) == 0 else s.v[-1]

        ml.station_id = int(last("NT:/DriverStation/AllianceStationID", 0) or 0)
        red = last("NT:/DriverStation/IsRedAlliance")
        if red is None:
            red = last("NT:/FMSInfo/IsRedAlliance")             # robot code from before the SystemCore one
        ml.is_red = None if red is None else bool(red)
        if not ml.station_id and ml.is_red is not None:
            sn = int(last("NT:/FMSInfo/StationNumber", 0) or 0)
            ml.station_id = sn + (0 if ml.is_red else 3) if 1 <= sn <= 3 else 0
        ml.match_type = int(last("NT:/DriverStation/MatchType", 0) or last("NT:/FMSInfo/MatchType", 0) or 0)
        ml.match_number = int(last("NT:/DriverStation/MatchNumber", 0) or last("NT:/FMSInfo/MatchNumber", 0) or 0)
        ml.event_name = str(last("NT:/DriverStation/EventName", "") or last("NT:/FMSInfo/EventName", "") or "")

    def status_text(self) -> str:
        if self.inst is None:
            return "not connected"
        if not self.connected:
            return f"connecting to {self.host_text} …"
        return f"connected to {self.server() or self.host_text}   ·   {len(self.tel.sig)} entries   ·   {self.events} updates"
