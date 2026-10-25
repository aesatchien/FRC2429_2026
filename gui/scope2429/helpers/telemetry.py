"""
Everything the dashboard-style render needs, decoded into time series.  The data comes either from a
.wpilog file or, live, from a NetworkTables connection (see live.py) - the panels cannot tell the difference.

  * Sig          one NT/DS entry as (times, values) with sample-and-hold lookup (LiveSig can grow)
  * Telemetry    all the signals the 2429 dashboard / lower panels show, loaded in one pass or fed live
  * ShotModel    rough "points scored" estimate from the shooter's flywheel-rpm dips
"""
from __future__ import annotations

import json
import re
import struct
from dataclasses import dataclass

import numpy as np

import core
import scorebug

NT = "NT:/"
SD = NT + "SmartDashboard/"
MECH_PREFIX = SD + "Mech Side View/"

# every topic the panels use (anything else in the log is ignored)
SCALARS = {
    # QuestNav
    "q_connected": NT + "QuestNav/quest_connected", "q_tracking": NT + "QuestNav/quest_tracking",
    "q_accepted": NT + "QuestNav/quest_pose_accepted", "q_synched": NT + "QuestNav/questnav_synched",
    "q_in_use": NT + "QuestNav/questnav_in_use", "q_passthrough": NT + "QuestNav/quest_in_passthrough",
    "q_dtap": NT + "QuestNav/quest_dtap_count", "q_lost": NT + "QuestNav/quest_tracking_lost_count",
    "q_battery": NT + "QuestNav/quest_Battery_pct", "q_latency": NT + "QuestNav/quest_Latency",
    "q_reset_cmd": SD + "Command/QuestResetOdometry/running",
    # cameras (Pis publish _frames/_connections; the robot publishes targets_exist)
    "cam_right_frames": NT + "Cameras/ArducamRight/_frames", "cam_right_conn": NT + "Cameras/ArducamRight/_connections",
    "cam_back_frames": NT + "Cameras/ArducamBack/_frames", "cam_back_conn": NT + "Cameras/ArducamBack/_connections",
    "cam_right_target": SD + "Vision/arducam_right_targets_exist", "cam_back_target": SD + "Vision/arducam_back_targets_exist",
    # robot status
    "pdh_v": SD + "RobotStatus/_pdh_voltage", "pdh_a": SD + "RobotStatus/_pdh_current",
    "pdh_w": SD + "RobotStatus/_pdh_inst_power", "pdh_wh": SD + "RobotStatus/_pdh_tot_energy_wh",
    "fms": SD + "RobotStatus/_fms_attached", "brownout": SD + "RobotStatus/brownout_mode",
    "red": NT + "DriverStation/IsRedAlliance", "game_data": NT + "DriverStation/GameData",
    "navx": SD + "Swerve/_navx", "navx_yaw": SD + "Swerve/_navx_yaw", "shot_dist": SD + "Auto/shot_distance", "auto_delay": SD + "Auto/auto_delay", "ok_to_shoot": SD + "Auto/is_ok_to_shoot",
    # mechanisms
    "intake_rpm": SD + "Intake/intake_rpm", "intake_angle": SD + "Intake/deploy_angle",
    "intake_on": SD + "Intake/intake_on", "intake_deployed": SD + "Intake/deployed",
    "hopper_rpm": SD + "Shooter/hopper_rpm", "indexer_rpm": SD + "Shooter/indexer_rpm",
    "shooter_rpm": SD + "Shooter/measured_shooter_rpm", "shooter_target": SD + "Shooter/shooter_rpm",
    "hopper_on": SD + "Shooter/hopper_on", "indexer_on": SD + "Shooter/indexer_on",
    "shooter_on": SD + "Shooter/shooter_on", "roller_on": SD + "Shooter/roller_on",
    "at_speed": SD + "Shooter/shooter_at_speed",
    # motor speeds actually measured (the *_rpm entries above are the commanded values), for the deviation indicators
    "roller_set": SD + "Shooter/roller_rpm", "roller_a": SD + "Shooter/speed_roller",
    "fly_l": SD + "Shooter/speed_flywheel_left_leader", "fly_r": SD + "Shooter/speed_flywheel_right_follower",
    "idx_l": SD + "Shooter/speed_indexer_left_leader", "idx_r": SD + "Shooter/speed_indexer_right_follower",
    "hop_a": SD + "Shooter/speed_hopper",
    "intake_l": SD + "Intake/speed_intake_left_leader", "intake_r": SD + "Intake/speed_intake_right_follower",
    # health report (report.py)
    "rio_brownout": SD + "RobotStatus/_rio_browned_out", "pdh_maxa": SD + "RobotStatus/_pdh_max_current",
    "pdh_minv": SD + "RobotStatus/_pdh_min_voltage", "robot_state": SD + "RobotStatus/_robot_state",
    "cur_fly_l": SD + "Shooter/current_flywheel_left_leader", "cur_fly_r": SD + "Shooter/current_flywheel_right_follower",
    "cur_hop": SD + "Shooter/current_hopper", "cur_roller": SD + "Shooter/current_roller",
    "cur_idx_l": SD + "Shooter/current_indexer_left_leader", "cur_idx_r": SD + "Shooter/current_indexer_right_follower",
    "cur_int_l": SD + "Intake/current_intake_left_leader", "cur_int_r": SD + "Intake/current_intake_right_follower",
    "cur_deploy": SD + "Intake/current_deploy",
}
# CAN bus health, published once a second by comp_system_core/helpers/can_monitor.py (since 2026-10-25)
CAN_BUSES = (("can_s0", "swerve drive Krakens"), ("can_s1", "swerve turn SparkFlex"), ("can_s2", "shooter, intake, PDH"))
for _bus, _ in CAN_BUSES:
    for _key, _name in (("util", "utilization"), ("busoff", "bus_off_count"), ("txfull", "tx_full_count"),
                        ("rxerr", "rx_errors"), ("txerr", "tx_errors")):
        SCALARS[f"{_bus}_{_key}"] = SD + f"CAN/{_bus}/{_name}"
SCALARS["can_phx_util"] = SD + "CAN/can_s0/phoenix_utilization_raw"      # Phoenix's own number for can_s0, as it returns it


def can_util_scale(tel) -> float:
    """Factor that turns the logged CAN 'utilization' into percent.  The robot logs whichever reader worked (WPILib, else
    Phoenix, see comp_system_core/helpers/can_monitor.py) and the unit is not verified: Phoenix returned about 2.45 on a bus,
    which cannot be a 0-1 fraction, and WPILib's header does not say.  So: a log whose values never exceed 1.0 but are not all
    zero is read as a fraction.  A real bus is above 1 % whenever anything is driving, so a percent log is not mistaken
    for a fraction in practice."""
    top = 0.0
    for bus, _ in CAN_BUSES:
        s = tel.by_key.get(f"{bus}_util")
        if s is not None and len(s):
            top = max(top, float(np.nanmax(np.abs(s.numeric()))))
    return 100.0 if 0.0 < top <= 1.0 else 1.0


POSES = {
    "drive": SD + "Swerve/drive_pose", "quest": NT + "QuestNav/quest_pose", "delta": NT + "QuestNav/quest_error_pose",
}
ARRAYS = {
    "mod": SD + "Swerve/module_states", "mod_des": SD + "Swerve/module_states_desired",
    "temp_drive": SD + "Swerve/drive_temp_c", "temp_turn": SD + "Swerve/turn_temp_c",
}
T_GAME = "NT:/DriverStation/GameData"
ALIASES = {"red": NT + "FMSInfo/IsRedAlliance", "game_data": NT + "FMSInfo/GameSpecificMessage"}
COPROC_RE = re.compile(r"arducam|launcher|limelight|photon|orange|coproc", re.I)


def client_kind(cid: str) -> str:
    """'coproc' (camera coprocessor), 'quest', 'self' (this program's own link, live) or 'other' for a NetworkTables client id."""
    if cid == "self":
        return "self"
    if COPROC_RE.search(cid):
        return "coproc"
    return "quest" if "quest" in cid.lower() else "other"


def _win(sig, t0, t1):
    i0 = max(0, int(np.searchsorted(sig.t, t0, side="right")) - 1)
    return i0, int(np.searchsorted(sig.t, t1, side="right"))


def false_intervals(sig, t0, t1):
    """[(a, b)] inside [t0, t1] while the boolean signal was False (held between samples; the last value runs to t1)."""
    if sig is None or len(sig) == 0:
        return []
    i0, i1 = _win(sig, t0, t1)
    out, start = [], None
    for t, v in zip(sig.t[i0:i1], sig.v[i0:i1]):
        t = max(float(t), t0)
        if not v and start is None:
            start = t
        elif v and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, t1))
    return out


def stall_intervals(sig, t0, t1, gap=2.5):
    """[(a, b)] in [t0, t1] where a counter that normally keeps changing sent no new value for more than `gap` s."""
    if sig is None or len(sig) == 0:
        return []
    i0, i1 = _win(sig, t0, t1)
    ts = sig.t[i0:i1]
    if len(ts) == 0:
        return []
    out = [(float(a), float(b)) for a, b in zip(ts[:-1], ts[1:]) if b - a > gap]
    if t1 - float(ts[-1]) > gap:
        out.append((float(ts[-1]), t1))
    return [(max(a, t0), min(b, t1)) for a, b in out if b > t0 and a < t1]


def restart_times(sig, t0, t1):
    """Times in [t0, t1] where a counter went back (a camera process that restarted)."""
    if sig is None or len(sig) < 2:
        return []
    i0, i1 = _win(sig, t0, t1)
    t, v = sig.t[i0:i1], np.asarray(sig.v[i0:i1], float)
    if len(v) < 2:
        return []
    k = np.nonzero(v[1:] < v[:-1] - 5)[0] + 1
    return [float(t[i]) for i in k if t0 <= t[i] <= t1]


def rising_times(sig, t0, t1):
    if sig is None or len(sig) == 0:
        return []
    i0, i1 = _win(sig, t0, t1)
    t, v = sig.t[i0:i1], np.asarray(sig.v[i0:i1], bool)
    return [float(t[i]) for i in range(len(v)) if v[i] and i > 0 and not v[i - 1] and t0 <= t[i] <= t1]


def data_gaps(sig, t0, t1, gap=0.6):
    """Gaps longer than `gap` s between samples of a signal that arrives many times a second (a stalled link)."""
    if sig is None or len(sig) < 2:
        return []
    i0, i1 = _win(sig, t0, t1)
    ts = sig.t[i0:i1]
    return [(float(a), float(b)) for a, b in zip(ts[:-1], ts[1:]) if b - a > gap and b > t0 and a < t1]


def merge_intervals(iv, t0, t1):
    out = []
    for a, b in sorted((max(a, t0), min(b, t1)) for a, b in iv if b > t0 and a < t1):
        if out and a <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out
JOYSTICKS = [f"DS:joystick{i}/{k}" for i in range(6) for k in ("axes", "buttons")]

# DS:joystick0 is the DualSense; this is wpilib's DualSenseController button / axis numbering
PS5_BUTTONS = {0: "CROSS", 1: "CIRCLE", 2: "SQUARE", 3: "TRIANGLE", 4: "CREATE", 5: "PS", 6: "OPTIONS", 7: "L3",
               8: "R3", 9: "L1", 10: "R1", 11: "UP", 12: "DOWN", 13: "LEFT", 14: "RIGHT", 15: "MIC", 20: "PAD"}


class Sig:
    """One logged entry: sample-and-hold lookup by log time (seconds)."""

    def __init__(self, t, v):
        self.t = np.asarray(t, float)
        self.v = v

    def __len__(self):
        return len(self.t)

    def idx(self, tt):
        return int(np.searchsorted(self.t, tt, side="right")) - 1

    def at(self, tt, default=None):
        i = self.idx(tt)
        return self.v[i] if i >= 0 else default

    def age(self, tt):
        """seconds since the newest sample at or before tt (None if there is none)"""
        i = self.idx(tt)
        return (tt - self.t[i]) if i >= 0 else None

    def numeric(self):
        return np.asarray(self.v, float)


class LiveSig(Sig):
    """A Sig that grows as live data arrives.  Only the newest CAP samples are kept."""
    CAP = 200_000

    def __init__(self):
        self._buf = np.empty(1024)
        self._n = 0
        self.v = []

    @property
    def t(self):
        return self._buf[:self._n]

    def __len__(self):
        return self._n

    def append(self, t, v):
        if self._n and t < self._buf[self._n - 1]:
            t = self._buf[self._n - 1]                  # keep the times sorted (searchsorted needs it)
        if self._n == len(self._buf):
            if self._n >= self.CAP:
                drop = self._n // 2
                self._buf[:self._n - drop] = self._buf[drop:self._n]
                del self.v[:drop]
                self._n -= drop
            else:
                self._buf = np.concatenate([self._buf, np.empty(len(self._buf))])
        self._buf[self._n] = t
        self._n += 1
        self.v.append(v)


def decode_nt(typ: str, v):
    """A value as delivered by ntcore -> the same object `_decode` makes from a log record."""
    if isinstance(v, (bytes, bytearray)):
        return _decode(typ, bytes(v))
    if typ in ("double[]", "float[]", "int64[]"):
        return np.asarray(v, float)
    if typ == "boolean[]":
        return np.asarray(v, bool)
    return v


def swerve_dev(act, des, m):
    """{'spd': m/s, 'ang': deg} of swerve module m, actual minus desired (None if no data).  The angle difference is
    wrapped to +-180 deg, and a desired state that is the same wheel velocity written the other way round (speed
    negated, angle + 180) is compared in the form closest to the actual one.  'ang' is None while the module is
    not moving."""
    if act is None or des is None or len(act) <= m or len(des) <= m:
        return None
    sa, aa = float(act[m][0]), float(act[m][1])
    sd, ad = float(des[m][0]), float(des[m][1])
    dang = np.arctan2(np.sin(aa - ad), np.cos(aa - ad))
    if abs(dang) > np.pi / 2:
        sd, dang = -sd, np.arctan2(np.sin(dang + np.pi), np.cos(dang + np.pi))
    dspd = (sa - sd) * (1.0 if sd >= 0 else -1.0)
    moving = abs(sd) > 0.1 or abs(sa) > 0.1
    return {"spd": float(dspd), "ang": float(np.degrees(dang)) if moving else None}


def _decode(typ: str, p: bytes):
    try:
        if typ == "double":
            return struct.unpack_from("<d", p)[0]
        if typ == "float":
            return struct.unpack_from("<f", p)[0]
        if typ == "int64":
            return struct.unpack_from("<q", p)[0]
        if typ == "boolean":
            return bool(p[0])
        if typ == "string":
            return p.decode("utf-8", "replace")
        if typ == "double[]":
            return np.frombuffer(p, "<f8")[: len(p) // 8]
        if typ == "float[]":
            return np.frombuffer(p, "<f4")[: len(p) // 4]
        if typ == "boolean[]":
            return np.frombuffer(p, np.uint8).astype(bool)
        if typ == "int64[]":
            return np.frombuffer(p, "<i8")[: len(p) // 8]
        if typ == "struct:Pose2d":
            return struct.unpack_from("<3d", p)
        if typ == "struct:SwerveModuleVelocity[]":   # (speed m/s, angle rad) x n
            return np.frombuffer(p, "<f8")[: len(p) // 8].reshape(-1, 2)
    except (struct.error, IndexError, ValueError):
        pass
    return None


@dataclass
class MechNode:
    path: str
    attrs: dict          # attr -> Sig (values already floats / strings)
    children: list


class Telemetry:
    def __init__(self, path, ml: core.MatchLog, pose_names: dict | None = None, extra=()):
        """`path` is a .wpilog file, or None for live data that is fed in with `ingest`.  `extra` are further
        entry names (the user's trend plots) to load besides the ones the panels use."""
        self.ml = ml
        self.live = path is None
        self.poses = {**POSES, **{k: v for k, v in (pose_names or {}).items() if v}}
        self.extra = set(extra)
        self._key_names: dict[str, list] = {}
        for k, n in {**SCALARS, **self.poses, **ARRAYS}.items():
            self._key_names.setdefault(n, []).append(k)
        self._wanted = set(SCALARS.values()) | set(self.poses.values()) | set(ARRAYS.values()) | set(JOYSTICKS)
        self._wanted |= {"NTConnection"} | set(ALIASES.values())
        self.net_events: list = []         # (time, NT client id, connected): who dropped off the network when
        self._wanted_cmd = re.compile(r"^NT:/SmartDashboard/Command/\w+/running$")
        self.sig: dict[str, Sig] = {}
        self.types: dict[str, str] = {}
        if path is not None:
            self._load(path)
        self.by_key = {k: self.sig.get(n) for k, n in {**SCALARS, **self.poses, **ARRAYS}.items()}
        for k, n in ALIASES.items():             # robot code from before the SystemCore one used other names for the same data
            if (self.by_key.get(k) is None or len(self.by_key[k]) == 0) and n in self.sig:
                self.by_key[k] = self.sig[n]
        self._synth_live = None
        self._fill_delta()
        self._mech_n = -1
        self.mech = self._build_mech()

    @staticmethod
    def _delta_of(q, d):
        """Quest pose minus drive pose (x, y, heading wrapped): what the robot's own quest_error_pose holds; None while the Quest
        has no valid pose (it publishes -10, -10 until it tracks)."""
        if not (-1.0 <= q[0] <= 17.5 and -1.0 <= q[1] <= 9.0) or not all(np.isfinite(d[:2])):
            return None
        dth = float(np.arctan2(np.sin(q[2] - d[2]), np.cos(q[2] - d[2])))
        return (float(q[0] - d[0]), float(q[1] - d[1]), dth)

    def _fill_delta(self):
        """Robot code from before the CASCMP event did not log quest_error_pose: work it out from the two poses."""
        cur = self.by_key.get("delta")
        if cur is not None and len(cur):
            return
        qs, ds = self.by_key.get("quest"), self.by_key.get("drive")
        if qs is None or ds is None or not len(qs) or not len(ds) or self.live:
            return
        ts, vs = [], []
        for t, q in zip(qs.t, qs.v):
            d = ds.at(t)
            v = None if d is None else self._delta_of(q, d)
            if v is not None:
                ts.append(t)
                vs.append(v)
        if ts:
            self.by_key["delta"] = Sig(ts, vs)

    @property
    def a0(self):
        """Log time of time zero (auto start, the first enable, or the start of the log)."""
        return self.ml.t_ref

    def want(self, n):
        return (n in self._wanted or n in self.extra or n.startswith(MECH_PREFIX)
                or self._wanted_cmd.match(n) is not None)

    def _load(self, path):
        entries, recs = core.read_wpilog(path, self.want)
        self.types = {nm: tp for (nm, tp, _m) in entries.values()}
        for name, rows in recs.items():
            typ = self.types.get(name, "")
            if name == "NTConnection":
                for t, p in rows:
                    try:
                        j = json.loads(p.decode("utf-8", "replace"))
                        cid = re.sub(r"\s*\d*@\d+$|@\d+$", "", str(j.get("remote_id", "?")))
                        self.net_events.append((t / 1e6, re.sub(r"\s+\d+$", "", cid), bool(j.get("connected"))))
                    except (ValueError, AttributeError):
                        pass
                continue
            ts, vs = [], []
            for t, p in rows:
                v = _decode(typ, p)
                if v is None:
                    continue
                ts.append(t / 1e6)
                vs.append(v)
            if ts:
                self.sig[name] = Sig(ts, vs)

    def ingest(self, name, typ, t, value):
        """Live: append one NetworkTables update (t in seconds on the session clock)."""
        v = decode_nt(typ, value)
        if v is None:
            return
        s = self.sig.get(name)
        if s is None:
            s = self.sig[name] = LiveSig()
            self.types[name] = typ
            for k in self._key_names.get(name, ()):
                self.by_key[k] = s
        s.append(t, v)
        if name == self.poses.get("quest"):
            self._live_delta(t, v)

    def _live_delta(self, t, q):
        """Live: no quest_error_pose from this robot code, so work it out from the newest drive pose."""
        real = self.sig.get(self.poses["delta"])
        ds = self.by_key.get("drive")
        if (real is not None and len(real)) or ds is None or not len(ds):
            return
        v = self._delta_of(q, ds.at(t) if ds.at(t) is not None else ds.v[-1])
        if v is None:
            return
        if self._synth_live is None:
            self._synth_live = LiveSig()
            self.by_key["delta"] = self._synth_live
        self._synth_live.append(t, v)

    def net_intervals(self, kind, t0, t1):
        """[(a, b)] in [t0, t1] during which a NetworkTables client of this kind (see client_kind) was not connected."""
        by = {}
        for t, c, up in sorted(self.net_events):
            if client_kind(c) == kind:
                by.setdefault(c, []).append((t, up))
        out = []
        for lst in by.values():
            start = None
            for t, up in lst:
                if not up and start is None:
                    start = t
                elif up and start is not None:
                    out.append((start, t))
                    start = None
            if start is not None:
                out.append((start, t1))
        return merge_intervals(out, t0, t1)

    def numeric_entries(self):
        """Names of the entries that can be drawn as a line plot (numbers, or arrays of numbers)."""
        out = []
        for n, s in self.sig.items():
            if n.startswith(MECH_PREFIX) or n.startswith("DS:joystick") or len(s) == 0:
                continue
            v = s.v[-1]
            if isinstance(v, (bool, int, float, np.floating, np.integer)) or (
                    isinstance(v, np.ndarray) and v.ndim == 1 and v.dtype != bool and 0 < len(v) <= 8):
                out.append(n)
        return sorted(out)

    # ------------------------------------------------------------------ access
    def get(self, key, t, default=None):
        s = self.by_key.get(key)
        return default if s is None else s.at(t, default)

    def joy(self, port, kind, t):
        s = self.sig.get(f"DS:joystick{port}/{kind}")
        return None if s is None else s.at(t)

    def match_time(self, t):
        """The robot's continuous 160 -> 0 match clock (auto 160-140, then teleop 140-0), or None."""
        ml = self.ml
        if ml.auto_start is None or t < ml.auto_start:
            return None
        if ml.teleop_start is not None and t >= ml.teleop_start:
            return max(0.0, 140.0 - (t - ml.teleop_start))
        return max(140.0, 160.0 - (t - ml.auto_start))

    # --------------------------------------------------------------- Mechanism2d
    def refresh_mech(self):
        """Live: rebuild the Mechanism2d tree once more of its entries have shown up."""
        n = sum(1 for k in self.sig if k.startswith(MECH_PREFIX))
        if n != self._mech_n:
            self.mech = self._build_mech()

    def _build_mech(self):
        """Mechanism2d tree: {root_name: MechNode}, plus dims / background color."""
        self._mech_n = sum(1 for k in self.sig if k.startswith(MECH_PREFIX))
        info = {"dims": (1.0, 1.0), "bg": "#000020"}
        d = self.sig.get(MECH_PREFIX + "dims")
        if d is not None and len(d.v[0]) >= 2:
            info["dims"] = (float(d.v[0][0]), float(d.v[0][1]))
        b = self.sig.get(MECH_PREFIX + "backgroundColor")
        if b is not None:
            info["bg"] = b.v[0]
        nodes: dict[str, MechNode] = {}
        for name, s in self.sig.items():
            if not name.startswith(MECH_PREFIX):
                continue
            rel = name[len(MECH_PREFIX):]
            if "/" not in rel:
                continue
            node, attr = rel.rsplit("/", 1)
            if attr not in ("angle", "length", "weight", "color", "x", "y"):
                continue
            nodes.setdefault(node, MechNode(node, {}, []))
            nodes[node].attrs[attr] = s
        roots = {}
        todo = list(nodes)                             # insertion order == log declaration order; parents made below join the list
        for path in todo:
            n = nodes[path]
            parent = path.rsplit("/", 1)[0] if "/" in path else None
            if parent is None:
                roots[path] = n
            else:
                if parent not in nodes:
                    nodes[parent] = MechNode(parent, {}, [])
                    todo.append(parent)
                nodes[parent].children.append(n)
        # a root can appear only as a parent of ligaments (it has x/y attrs) - make sure they are all roots
        for path, n in nodes.items():
            if "/" not in path:
                roots.setdefault(path, n)
        return {"roots": roots, **info}


# ----------------------------------------------------------------------------
# rough scoring estimate (see the conversation: shooter-rpm dips = balls fired)
# ----------------------------------------------------------------------------
class ShotModel:
    """Counts balls fired from the flywheel-rpm dips during indexer-on bursts and turns them
    into an estimated number of points with assumed hit rates.  Everything is relative to
    auto start (seconds)."""
    AUTO_HIT = 0.32     # assumed fraction of auto shots that score
    TELE_HIT = 0.80     # assumed fraction of teleop shots that score
    LAG = 0.7           # s from the flywheel dip until the ball reaches the hub
    DIP = 60.0          # rpm drop that counts as one ball
    ZONE_M = 158.6 * 0.0254     # alliance zone depth (m)
    BUMPER_M = 0.5              # G407: a shot scores only while the bumpers are at least partly inside the zone, i.e. the
                                # robot centre is within about this far of the zone line

    def __init__(self, tel: Telemetry):
        a0 = tel.a0
        ind, rpm, tgt, drive = (tel.by_key.get(k) for k in ("indexer_on", "shooter_rpm", "shooter_target", "drive"))
        self.times = np.zeros(0)
        self.weights = np.zeros(0)
        self.bursts = []        # counted indexer bursts: dict(a, b, dips=[match-rel dip times of shots from the alliance zone])
        self.passes = []        # match-rel times of balls fed from outside the alliance zone (G407: they cannot score)
        self.blue = tel.ml.is_blue
        self.available = ind is not None and rpm is not None and len(rpm) > 2
        if not self.available:
            return
        wins, start = [], None
        for t, v in zip(ind.t - a0, ind.v):
            if v and start is None:
                start = t
            elif not v and start is not None:
                wins.append((start, t))
                start = None
        rt, rv = rpm.t - a0, rpm.numeric()
        ev = []
        for a, b in wins:
            if b < 0:
                continue
            mid = (a + b) / 2
            m = (rt >= a) & (rt <= b)
            v, tt = rv[m], rt[m]
            w = self.AUTO_HIT if mid < 21.5 else self.TELE_HIT
            dips = []
            for i in range(1, len(v) - 1):
                if v[i] < v[i - 1] and v[i] <= v[i + 1] and v[max(0, i - 6):i].max() - v[i] >= self.DIP:
                    if not self.in_zone(drive, a0 + tt[i]):
                        self.passes.append(float(tt[i]))
                        continue
                    ev.append((tt[i] + self.LAG, w))
                    dips.append(float(tt[i]))
            if dips:
                self.bursts.append({"a": a, "b": b, "dips": dips})
        ev.sort()
        self.times = np.array([e[0] for e in ev])
        self.weights = np.cumsum([e[1] for e in ev])

    def in_zone(self, drive, t):
        """G407: is the robot (by its centre, with a bumper's reach) in its own alliance zone at log time t?
        Without a drive pose we cannot tell and assume it is."""
        p = drive.at(t) if drive is not None else None
        if p is None or p[0] != p[0]:
            return True
        reach = self.ZONE_M + self.BUMPER_M
        return p[0] <= reach if self.blue else p[0] >= core.FIELD_W_M - reach

    def fired(self, t_rel):
        return int(np.searchsorted(self.times, t_rel, side="right"))

    def passed(self, t_rel):
        """balls fed from outside the alliance zone so far (G407 passes, not scoring attempts)"""
        return int(np.searchsorted(np.sort(self.passes), t_rel, side="right"))

    def points(self, t_rel):
        i = self.fired(t_rel)
        return int(round(self.weights[i - 1])) if i else 0

    def accuracy(self, t_rel):
        k = self.fired(t_rel)
        return self.points(t_rel) / k if k else None


def active_windows(ml: core.MatchLog, game_data: str, red: bool = True):
    """Match-relative windows during which the given alliance's hub is active, extended by the 3 s
    scoring grace period.  Auto, transition and endgame count for both; the four shifts alternate
    ('R' = red won auto, so red's hub is inactive first)."""
    ts = (ml.teleop_start - ml.auto_start) if ml.teleop_start is not None else 23.2
    te = (ml.teleop_end - ml.auto_start) if ml.teleop_end is not None else ts + 140
    win = [(0.0, ts + 10.0 + 3.0), (te - 30.0, te + 3.0)]
    for k in range(4):
        a, b = ts + 10 + 25 * k, ts + 10 + 25 * (k + 1)
        if game_data in ("R", "B"):
            red_inactive = (game_data == "R") == (k % 2 == 0)
            if (red and not red_inactive) or (not red and red_inactive):
                win.append((a, b + 3.0))
        else:
            win.append((a, b + 3.0))
    return win


class VideoShotModel:
    """2429's points, taken from the broadcast's official fuel counter only (scorebug.py, "71 / 240" in the corner box).

    Nothing is counted from the picture, and no hit rates are assumed.  The counter is exact but counts the whole
    alliance, so 2429 only gets credit for it when 2429 is shooting from inside its own alliance zone (G407, which is
    what ShotModel.bursts already holds: dips from outside the zone are passes and are left out):

      1. one delay is measured from this match's data: log flywheel dip -> the counter ticking (correlation, 0.5 to 4 s).
      2. for each indexer burst, D = the counter ticks in the window that follows it (the burst start + delay to its end
         + delay, a little wider each side).
      3. 2429 scored = D, capped at the shots 2429 fired in that burst (it cannot score more than it shot).

    Ticks outside those windows (a partner shooting while 2429 is not, tower points) are not 2429's and are not counted.
    Partners shooting in the same window ARE counted, since the video cannot tell the balls apart, so this is an upper bound.
    Each burst's points are spread over the counter ticks inside its window, so the number moves with the broadcast.
    Not available (so the score is 0, never a guess) when the counter cannot be read.  Same interface as ShotModel."""
    WIN_PRE, WIN_POST = 0.4, 0.8      # counter window around the expected tick time of a burst (s)
    LAG_RANGE = (0.5, 4.0)            # s from a flywheel dip to the counter ticking that is searched
    DEFAULT_LAG = 1.5                 # used when there are too few dips or ticks to measure it

    def __init__(self, tel: Telemetry, shots: ShotModel, flux: dict, red: bool = True):
        self.available = False
        self.times = np.zeros(0)
        self.weights = np.zeros(0)
        self.summary = []
        self.shots = shots
        self.lag = self.DEFAULT_LAG
        self.mode = "none"                 # "counter" once the broadcast fuel counter was read
        self.delays = None
        ctr = flux.get("ctr") if flux else None
        if ctr is None or len(ctr) < 10 or not len(shots.bursts):
            return
        F = scorebug.clean(ctr)
        if F[-1] < 5 or float((ctr < 0).mean()) >= 0.5:      # nothing scored yet, or it was unreadable for half the match
            return
        self.available, self.mode = True, "counter"
        self._init_counter(tel, shots, flux["t"], F)

    # -------------------------------------------------------------- helpers
    @staticmethod
    def _merge(shots):
        merged = []
        for b in sorted(shots.bursts, key=lambda b: b["a"]):
            if merged and b["a"] - merged[-1]["b"] < 2.5:
                merged[-1] = {"a": merged[-1]["a"], "b": b["b"], "dips": merged[-1]["dips"] + b["dips"]}
            else:
                merged.append(dict(b))
        return merged

    @staticmethod
    def _odo(tel, a, b):
        delta = tel.by_key.get("delta")
        if delta is None or not len(delta):
            return 0.0
        xs = []
        for x in np.linspace(a, b, 5):
            p = delta.at(tel.a0 + x)
            if p is not None:
                xs.append(float(np.hypot(p[0], p[1])))
        return float(np.mean(xs)) if xs else 0.0

    # ------------------------------------------------------- counter model
    def _init_counter(self, tel, shots, t, F):
        inc = np.diff(F, prepend=F[0])
        ci = np.concatenate([[0.0], np.cumsum(inc)])

        def tick(a, b):
            return float(ci[np.searchsorted(t, b, "right")] - ci[np.searchsorted(t, a, "right")])

        # ---- the delay dip -> counter tick, from correlation on 0.1 s bins
        bw = 0.1
        edges = np.arange(t[0], t[-1], bw)
        nb = len(edges) - 1
        i0 = np.searchsorted(t, edges)
        incb = np.convolve(ci[i0[1:]] - ci[i0[:-1]], np.array([1, 2, 3, 2, 1], float) / 9.0, mode="same")
        dips = np.array([d for b in shots.bursts for d in b["dips"]])
        dipb = np.zeros(nb)
        for d in dips:
            k = int((d - edges[0]) // bw)
            if 0 <= k < nb:
                dipb[k] += 1
        dipb = np.convolve(dipb, np.array([1, 2, 3, 2, 1], float) / 9.0, mode="same")
        lo, hi = int(self.LAG_RANGE[0] / bw), int(self.LAG_RANGE[1] / bw)
        sc = []
        for k in range(lo, hi + 1):                       # the counter lags the dips by k bins
            xa, ya = incb[k:], dipb[:nb - k]
            sc.append(float(np.corrcoef(xa, ya)[0, 1]) if len(xa) > 2 and xa.std() > 0 and ya.std() > 0 else -1.0)
        if len(dips) >= 5 and sc and max(sc) > 0.1:
            L = (lo + int(np.argmax(sc))) * bw
        else:
            L = self.DEFAULT_LAG
        self.lag, self.delays = L, (L,)

        # ---- per burst: the counter ticks in its window, no more than the shots fired
        ev_t, ev_w = [], []
        for b in self._merge(shots):
            w0, w1 = b["a"] + L - self.WIN_PRE, b["b"] + L + self.WIN_POST
            D = tick(w0, w1)
            fired = len(b["dips"])
            hits = float(min(D, fired))
            self.summary.append({"a": b["a"], "b": b["b"], "fired": fired, "odo_m": self._odo(tel, b["a"], b["b"]),
                                 "q": hits / fired if fired else 0.0, "scored": hits, "counter": D})
            if D > 0 and hits > 0:
                m = (t >= w0) & (t <= w1) & (inc > 0)
                for i in np.nonzero(m)[0]:
                    ev_t.append(t[i])
                    ev_w.append(hits / D * inc[i])
        order = np.argsort(ev_t)
        self.times = np.array(ev_t)[order]
        self.weights = np.cumsum(np.array(ev_w)[order]) if len(ev_t) else np.zeros(0)

    def fired(self, t_rel):
        return self.shots.fired(t_rel)

    def points(self, t_rel):
        i = int(np.searchsorted(self.times, t_rel, side="right"))
        return int(round(self.weights[i - 1])) if i else 0

    def accuracy(self, t_rel):
        k = self.fired(t_rel)
        return min(1.0, self.points(t_rel) / k) if k else None
