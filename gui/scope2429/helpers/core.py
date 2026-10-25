"""
Core engine for Scope2429 (FRC log / live NetworkTables viewer and match-video renderer).

  * reads WPILib .wpilog files (pure Python, no robotpy needed)
  * extracts Pose2d series (quest_pose, drive_pose) and FMS match phases
  * draws an AdvantageScope-style 2D field panel (rotated so RED is on the LEFT)
  * finds the match video on The Blue Alliance, downloads it with yt-dlp
  * auto-syncs log <-> video using the field sounds (auto start, auto end,
    teleop start, match end) heard in the video's audio track
  * composes a compact H.264/AAC MP4 (plays on Windows, macOS, Slack)
"""
from __future__ import annotations

import glob
import io
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import urllib.request
import zipfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import cv2

APP_DIR = Path.home() / ".scope2429"
_OLD_APP_DIRS = [Path.home() / ".logscope", Path.home() / ".frc_match_video"]     # names used by earlier versions
if not APP_DIR.exists():
    for _old in _OLD_APP_DIRS:
        if _old.exists():
            try:
                _old.rename(APP_DIR)
            except OSError:
                APP_DIR = _old
            break
BUNDLED_FIELDS = Path(__file__).resolve().parent.parent / "fields"   # WPILib 2026 field image ships here
CACHE_DIR = APP_DIR / "cache"
SETTINGS_FILE = APP_DIR / "settings.json"

DEFAULT_QUEST = "NT:/QuestNav/quest_pose"
DEFAULT_DRIVE = "NT:/SmartDashboard/Swerve/drive_pose"

# 2026 field (REBUILT) in metres; overridden by an AdvantageScope config.json
FIELD_W_M = 16.541
FIELD_H_M = 8.069

NO_WINDOW = 0x08000000 if sys.platform == "win32" else 0  # hide console windows


# --------------------------------------------------------------------------
# settings
# --------------------------------------------------------------------------
def load_settings() -> dict:
    try:
        d = json.loads(SETTINGS_FILE.read_text(encoding="utf-8-sig"))
    except Exception:
        return {}
    for rec in (d.get("per_log") or {}).values():       # cached videos moved with the old settings folder
        v = rec.get("video")
        for old in _OLD_APP_DIRS:
            if v and str(old) in v:
                rec["video"] = v = v.replace(str(old), str(APP_DIR))
    return d


def save_settings(d: dict):
    try:
        APP_DIR.mkdir(parents=True, exist_ok=True)
        SETTINGS_FILE.write_text(json.dumps(d, indent=2, ensure_ascii=False), encoding="utf-8")
    except Exception:
        pass


# --------------------------------------------------------------------------
# ffmpeg helpers
# --------------------------------------------------------------------------
_FFMPEG = None


def ffmpeg_exe() -> str:
    """System ffmpeg if on PATH, else the one bundled with imageio-ffmpeg
    (copied to a stable name so yt-dlp can use it for merging)."""
    global _FFMPEG
    if _FFMPEG:
        return _FFMPEG
    p = shutil.which("ffmpeg")
    if not p:
        try:
            import imageio_ffmpeg
            src = imageio_ffmpeg.get_ffmpeg_exe()
            tools = APP_DIR / "bin"
            tools.mkdir(parents=True, exist_ok=True)
            dst = tools / ("ffmpeg.exe" if sys.platform == "win32" else "ffmpeg")
            if not dst.exists() or dst.stat().st_size != os.path.getsize(src):
                shutil.copy2(src, dst)
                if sys.platform != "win32":
                    os.chmod(dst, 0o755)
            p = str(dst)
        except ImportError:
            raise RuntimeError("ffmpeg not found. Run:  pip install imageio-ffmpeg")
    _FFMPEG = p
    return p


def run_ff(args, **kw):
    return subprocess.run([ffmpeg_exe(), "-hide_banner", *args], capture_output=True,
                          creationflags=NO_WINDOW, **kw)


def detect_red_left(video):
    """True if the broadcast video shows the red alliance on the left, False if blue, None if it cannot tell
    (no score bar in the sampled frames, or they disagree - e.g. a whole-day stream or a non-FIRST video).
    The FIRST broadcast lists the alliance on the left of the field camera first in its score bar (SoCal Showdown:
    red left, Daly Division at Champs: blue left); the detector and its constants live in scorebug.py."""
    import scorebug
    return scorebug.detect_red_left(video)


def video_info(path) -> dict:
    """duration (s), width, height, fps, has_audio — parsed from ffmpeg -i."""
    r = run_ff(["-i", str(path)])
    txt = r.stderr.decode("utf-8", "replace")
    info = {"duration": None, "width": None, "height": None, "fps": 30.0, "has_audio": False}
    m = re.search(r"Duration:\s*(\d+):(\d+):(\d+\.?\d*)", txt)
    if m:
        info["duration"] = int(m[1]) * 3600 + int(m[2]) * 60 + float(m[3])
    m = re.search(r"Stream #.*Video:.*?(\d{2,5})x(\d{2,5})", txt)
    if m:
        info["width"], info["height"] = int(m[1]), int(m[2])
    m = re.search(r"Video:.*?(\d+(?:\.\d+)?) fps", txt)
    if m:
        info["fps"] = float(m[1])
    info["has_audio"] = bool(re.search(r"Stream #.*Audio:", txt))
    return info


def grab_frame(path, t: float):
    """One BGR frame from a video at time t (seconds)."""
    r = run_ff(["-ss", f"{max(0.0, t):.3f}", "-i", str(path), "-frames:v", "1",
                "-f", "image2pipe", "-vcodec", "png", "-"])
    if not r.stdout:
        return None
    return cv2.imdecode(np.frombuffer(r.stdout, np.uint8), cv2.IMREAD_COLOR)


# --------------------------------------------------------------------------
# WPILOG reading
# --------------------------------------------------------------------------
def read_wpilog(path, wanted=None, span=None):
    """Return (entries {id:(name,type,meta)}, records {name:[(ts_us, payload)]}).
    If `span` is a list, it is filled with [first, last] timestamps (us) of every record in the file."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:6] != b"WPILOG":
        raise ValueError(f"{path} is not a wpilog file")
    pos = 12 + struct.unpack_from("<I", data, 8)[0]
    n = len(data)
    entries, keep_ids, recs = {}, {}, {}
    t_lo, t_hi = None, 0

    def keep(name):
        if wanted is None:
            return True
        return wanted(name) if callable(wanted) else name in wanted

    while pos < n:
        hb = data[pos]
        pos += 1
        il = (hb & 3) + 1
        sl = ((hb >> 2) & 3) + 1
        tl = ((hb >> 4) & 7) + 1
        if pos + il + sl + tl > n:
            break
        eid = int.from_bytes(data[pos:pos + il], "little"); pos += il
        size = int.from_bytes(data[pos:pos + sl], "little"); pos += sl
        ts = int.from_bytes(data[pos:pos + tl], "little"); pos += tl
        if ts >> 62:         # a negative 64-bit time: NT values set before the log started (their server stamp is older than the log). Held from 0
            ts = 0
        payload = data[pos:pos + size]; pos += size
        if eid == 0:
            if payload and payload[0] == 0:  # start record
                e = struct.unpack_from("<I", payload, 1)[0]
                p = 5
                vals = []
                for _ in range(3):
                    ln = struct.unpack_from("<I", payload, p)[0]; p += 4
                    vals.append(payload[p:p + ln].decode("utf-8", "replace")); p += ln
                entries[e] = tuple(vals)
                if keep(vals[0]):
                    keep_ids[e] = vals[0]
                    recs.setdefault(vals[0], [])
            continue
        if t_lo is None:
            t_lo = ts
        t_hi = ts if ts > t_hi else t_hi
        nm = keep_ids.get(eid)
        if nm is not None:
            recs[nm].append((ts, payload))
    if span is not None:
        span[:] = [t_lo or 0, t_hi]
    return entries, recs


@dataclass
class PoseSeries:
    name: str
    t: np.ndarray      # seconds (log time)
    x: np.ndarray
    y: np.ndarray
    th: np.ndarray

    def at(self, tt: float):
        """Sample-and-hold value at time tt, or None."""
        if len(self.t) == 0 or tt < self.t[0]:
            return None
        i = int(np.searchsorted(self.t, tt, side="right")) - 1
        return self.x[i], self.y[i], self.th[i]

    def trail(self, t0: float, t1: float):
        i0 = int(np.searchsorted(self.t, t0, side="left"))
        i1 = int(np.searchsorted(self.t, t1, side="right"))
        return self.x[i0:i1], self.y[i0:i1]


# QuestNav status indicators drawn under the field: (entry, short label, kind)
#   kind: "bool" (green/red), "bool_warn" (true = amber warning), "count" (number; flashes on change)
STATUS_ENTRIES = [
    ("NT:/QuestNav/quest_connected", "connected", "bool"),
    ("NT:/QuestNav/quest_tracking", "tracking", "bool"),
    ("NT:/QuestNav/quest_pose_accepted", "pose accepted", "bool"),
    ("NT:/QuestNav/questnav_synched", "synched", "bool"),
    ("NT:/QuestNav/questnav_in_use", "in use", "bool"),
    ("NT:/QuestNav/quest_in_passthrough", "passthrough", "bool_warn"),
    ("NT:/QuestNav/quest_tracking_lost_count", "tracking lost", "count"),
    ("NT:/QuestNav/quest_dtap_count", "dtap", "count"),
]
# Vision camera indicators (own row under the Quest block): "bool_dim" = green when true, gray when false
CAMERA_ENTRIES = [
    ("NT:/SmartDashboard/Vision/arducam_back_targets_exist", "back target", "bool_dim"),
    ("NT:/SmartDashboard/Vision/arducam_right_targets_exist", "right target", "bool_dim"),
]
STATUS_GROUPS = [("QUEST", STATUS_ENTRIES), ("ARDUCAM", CAMERA_ENTRIES)]


@dataclass
class ValueSeries:
    name: str
    t: np.ndarray
    v: np.ndarray

    def at(self, tt):
        """(value, seconds since it last changed) or (None, None)"""
        if len(self.t) == 0 or tt < self.t[0]:
            return None, None
        i = int(np.searchsorted(self.t, tt, side="right")) - 1
        j = i
        while j > 0 and self.v[j - 1] == self.v[i]:
            j -= 1
        return self.v[i], (tt - self.t[j]) if j > 0 else 1e9


def _decode_scalar(typ, p):
    try:
        if typ == "boolean":
            return float(bool(p[0]))
        if typ == "double":
            return struct.unpack_from("<d", p)[0]
        if typ == "float":
            return struct.unpack_from("<f", p)[0]
        if typ == "int64":
            return float(struct.unpack_from("<q", p)[0])
    except (struct.error, IndexError):
        pass
    return None


@dataclass
class MatchLog:
    path: str
    pose_entries: list = field(default_factory=list)
    poses: dict = field(default_factory=dict)       # name -> PoseSeries
    status: dict = field(default_factory=dict)      # name -> ValueSeries
    auto_start: float | None = None
    auto_end: float | None = None
    teleop_start: float | None = None
    teleop_end: float | None = None
    event_name: str = ""
    match_type: int = 0      # 0 none, 1 practice, 2 qual, 3 elim
    match_number: int = 0
    replay: int = 0
    station_id: int = 0      # 1-3 red, 4-6 blue
    is_red: bool | None = None   # DriverStation/IsRedAlliance, used when there is no station id
    first_enable: float | None = None   # first time the robot was enabled in any mode (no FMS needed)
    t_first: float = 0.0     # log time (s) of the first / last record in the file (or live session time)
    t_last: float = 0.0
    source: str = ""         # display name for non-file sources (live connection)

    @property
    def event_code(self):
        """The FMS event code of this log ('CAEL', 'CAPOH' ...): from the log's event name, else from the file name."""
        if self.event_name:
            return self.event_name
        m = re.search(r"_([A-Za-z0-9]+)_[PQE]\d+\.wpilog$", str(self.path))
        return m.group(1) if m else ""

    @property
    def alliance(self):
        if 1 <= self.station_id <= 3:
            return "Red", self.station_id
        if 4 <= self.station_id <= 6:
            return "Blue", self.station_id - 3
        if self.is_red is not None:
            return ("Red" if self.is_red else "Blue"), 0
        return "?", 0

    @property
    def is_blue(self):
        return self.alliance[0] == "Blue"            # unknown alliance is drawn as red

    @property
    def alliance_text(self):
        al, st = self.alliance
        return al if not st else f"{al} {st}"

    @property
    def has_match(self):
        """True when the log has a Driver Station autonomous enable (FMS match or DS practice mode)."""
        return self.auto_start is not None

    @property
    def t_ref(self):
        """Time zero of every plot and clock: auto start, else the first enable, else the start of the log."""
        if self.auto_start is not None:
            return self.auto_start
        if self.first_enable is not None:
            return self.first_enable
        return self.t_first

    @property
    def match_label(self):
        if self.source:
            return self.source
        p = {1: "P", 2: "Q", 3: "E"}.get(self.match_type, "M")
        if self.match_number:
            return f"{p}{self.match_number}"
        m = re.search(r"_([PQE]\d+)\.wpilog$", self.path, re.I)
        return m[1].upper() if m else Path(self.path).stem

    @property
    def phase_events(self):
        """Log times of the field events that make a sound."""
        ev = {}
        for k in ("auto_start", "auto_end", "teleop_start", "teleop_end"):
            v = getattr(self, k)
            if v is not None:
                ev[k] = v
        return ev

    @property
    def match_len(self):
        if self.auto_start is None:               # no match structure: the whole log from time zero
            return max(self.t_last - self.t_ref, 1.0)
        end = self.teleop_end or (self.teleop_start + 140 if self.teleop_start else self.auto_start + 160)
        return end - self.auto_start

    def clock(self, t: float):
        """('AUTO'|'TELEOP'|'PRE'|'POST'|'', seconds remaining)"""
        if self.auto_start is None:
            return "", None
        a_len = (self.auto_end - self.auto_start) if self.auto_end else 20.0
        t_len = (self.teleop_end - self.teleop_start) if (self.teleop_end and self.teleop_start) else 140.0
        if t < self.auto_start:
            return "PRE-MATCH", None
        if t < self.auto_start + a_len:
            return "AUTO", a_len - (t - self.auto_start)
        if self.teleop_start is None or t < self.teleop_start:
            return "AUTO", 0.0
        if t < self.teleop_start + t_len:
            return "TELEOP", t_len - (t - self.teleop_start)
        return "POST-MATCH", 0.0


def _decode_controlword(p: bytes):
    v = int.from_bytes(p[:8].ljust(8, b"\0"), "little")
    mode = (v >> 56) & 3
    enabled = bool((v >> 58) & 1)
    fms = bool((v >> 60) & 1)
    return mode, enabled, fms


class PhaseTracker:
    """Turns successive Driver Station control words into auto / teleop start and end times on a
    MatchLog.  Used for log files and for the live connection."""

    def __init__(self, ml: "MatchLog"):
        self.ml = ml
        self.prev = (0, False)

    def feed(self, ts: float, mode: int, en: bool):
        ml, prev = self.ml, self.prev
        if en and ml.first_enable is None:
            ml.first_enable = ts
        if en and mode == 1 and not (prev[1] and prev[0] == 1):
            ml.auto_start = ts  # last auto enable wins (handles field resets)
            ml.auto_end = ml.teleop_start = ml.teleop_end = None
        elif prev[1] and prev[0] == 1 and not (en and mode == 1) and ml.auto_start is not None and ml.auto_end is None:
            ml.auto_end = ts
        if en and mode == 2 and not (prev[1] and prev[0] == 2) and ml.auto_start is not None and ml.teleop_start is None:
            ml.teleop_start = ts
        elif prev[1] and prev[0] == 2 and not en and ml.teleop_start is not None and ml.teleop_end is None:
            ml.teleop_end = ts
        self.prev = (mode, en)


def _decode_pose(p: bytes):
    if len(p) >= 24:
        return struct.unpack_from("<3d", p, 0)
    return None


def load_match_log(path, pose_names=None) -> MatchLog:
    pose_names = set(pose_names or [])
    status_names = {e[0] for _g, ents in STATUS_GROUPS for e in ents}

    def want(n):
        return (n in pose_names or n in status_names or n.startswith("NT:/DriverStation/") or n.startswith("NT:/FMSInfo/")
                or n in ("DS:controlWord", "DS:enabled", "DS:autonomous", "DS:test"))

    span = []
    entries, recs = read_wpilog(path, wanted=want, span=span)
    ml = MatchLog(path=str(path))
    ml.t_first, ml.t_last = span[0] / 1e6, span[1] / 1e6
    ml.pose_entries = sorted(n for (n, t, m) in entries.values() if t == "struct:Pose2d")
    for n in pose_names:
        rows = recs.get(n, [])
        ts, xs, ys, ths = [], [], [], []
        for t, p in rows:
            v = _decode_pose(p)
            if v is None:
                continue
            x, y, th = v
            # drop obviously invalid samples (QuestNav publishes -10,-10 before tracking)
            if not (-1.0 <= x <= FIELD_W_M + 1.0 and -1.0 <= y <= FIELD_H_M + 1.0):
                x = y = float("nan")
            ts.append(t / 1e6); xs.append(x); ys.append(y); ths.append(th)
        ml.poses[n] = PoseSeries(n, np.array(ts), np.array(xs), np.array(ys), np.array(ths))
    types = {nm: tp for (nm, tp, _m) in entries.values()}
    for n in status_names:
        rows = recs.get(n) or []
        ts, vs = [], []
        for t, p in rows:
            v = _decode_scalar(types.get(n), p)
            if v is not None:
                ts.append(t / 1e6); vs.append(v)
        if ts:
            ml.status[n] = ValueSeries(n, np.array(ts), np.array(vs))

    # control word -> phases (prefer DS: entry, fall back to NT copy)
    cw = recs.get("DS:controlWord") or recs.get("NT:/DriverStation/ControlWord") or []
    pt = PhaseTracker(ml)
    for t, p in cw:
        pt.feed(t / 1e6, *_decode_controlword(p)[:2])
    if not cw:               # earlier robot code (before the 2026 SystemCore one) logged three booleans instead of a control word
        state = {"DS:enabled": False, "DS:autonomous": False, "DS:test": False}
        changes = sorted((t, n, bool(p[0])) for n in state for t, p in recs.get(n, []) if p)
        for t, n, v in changes:
            state[n] = v
            mode = 1 if state["DS:autonomous"] else 3 if state["DS:test"] else 2
            pt.feed(t / 1e6, mode, state["DS:enabled"])

    def last_int(name):
        rows = recs.get(name) or []
        return int.from_bytes(rows[-1][1][:8], "little", signed=True) if rows else 0

    # the 2026 SystemCore code publishes /DriverStation/*; the roboRIO code before it only had /FMSInfo/* (same data, other names)
    def first_rows(*names):
        for nm in names:
            rows = recs.get(nm) or []
            if rows:
                return rows
        return []

    ml.match_type = last_int("NT:/DriverStation/MatchType") or last_int("NT:/FMSInfo/MatchType")
    ml.match_number = last_int("NT:/DriverStation/MatchNumber") or last_int("NT:/FMSInfo/MatchNumber")
    ml.replay = last_int("NT:/DriverStation/ReplayNumber") or last_int("NT:/FMSInfo/ReplayNumber")
    ml.station_id = last_int("NT:/DriverStation/AllianceStationID")
    rows = first_rows("NT:/DriverStation/IsRedAlliance", "NT:/FMSInfo/IsRedAlliance")
    if rows:
        ml.is_red = bool(rows[-1][1][:1] != b"\0")
        if not ml.station_id:          # FMSInfo/StationNumber is 1-3 within the alliance
            sn = last_int("NT:/FMSInfo/StationNumber")
            if 1 <= sn <= 3:
                ml.station_id = sn + (0 if ml.is_red else 3)
    rows = first_rows("NT:/DriverStation/EventName", "NT:/FMSInfo/EventName")
    ml.event_name = rows[-1][1].decode("utf-8", "replace") if rows else ""
    # fall back to file name if DS info missing
    if not ml.match_number:
        m = re.search(r"_([PQE])(\d+)\.wpilog$", str(path), re.I)
        if m:
            ml.match_type = {"P": 1, "Q": 2, "E": 3}[m[1].upper()]
            ml.match_number = int(m[2])
    return ml


def list_log_entries(path):
    """(pose entry names, numeric entry names) of a log file, for the GUI's pickers."""
    entries, _ = read_wpilog(path, wanted=lambda n: False)
    poses = sorted(n for (n, t, m) in entries.values() if t == "struct:Pose2d")
    nums = sorted(n for (n, t, m) in entries.values()
                  if t in ("double", "float", "int64", "boolean", "double[]", "float[]", "int64[]")
                  and not n.startswith(("DS:joystick", "/.", "NT:/.", "NT:/SmartDashboard/Mech Side View/")))
    return poses, nums


def list_pose_entries(path):
    entries, _ = read_wpilog(path, wanted=lambda n: False)
    return sorted(n for (n, t, m) in entries.values() if t == "struct:Pose2d")


# --------------------------------------------------------------------------
# The Blue Alliance
# --------------------------------------------------------------------------
# FMS event code -> (name used in video titles, TBA event key).  Codes that are not listed are shown by their code and
# looked up on TBA as "2026" + the code in lower case; add the events of your season here.
EVENT_INFO = {"CAEL": ("SoCal Showdown", "2026cass"), "CAPOH": ("Port Hueneme", "2026capoh")}


def event_title(code: str) -> str:
    return EVENT_INFO.get(code, (code or "event",))[0]


def event_tba_key(code: str) -> str:
    return EVENT_INFO[code][1] if code in EVENT_INFO else (f"2026{code.lower()}" if code else "")


def tba_match_key(event_key: str, ml: MatchLog) -> str | None:
    n = ml.match_number
    if not n:
        return None
    if ml.match_type == 2:
        return f"{event_key}_qm{n}"
    if ml.match_type == 3:
        # 2023+ double-elimination: playoff matches 1-13 are sf{n}m1, then finals
        return f"{event_key}_sf{n}m1" if n <= 13 else f"{event_key}_f1m{n - 13}"
    return None  # practice matches are not on TBA


def _http_get(url, headers=None, timeout=20):
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 frc-match-video", **(headers or {})})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def tba_find_video(match_key: str, api_key: str = "") -> tuple[list[str], dict]:
    """Return (list of youtube urls, info dict with alliances if available)."""
    info = {}
    ids = []
    if api_key:
        d = json.loads(_http_get(f"https://www.thebluealliance.com/api/v3/match/{match_key}",
                                 {"X-TBA-Auth-Key": api_key}))
        for v in d.get("videos") or []:
            if v.get("type") == "youtube":
                ids.append(v["key"])
        al = d.get("alliances") or {}
        info = {c: [k.replace("frc", "") for k in al.get(c, {}).get("team_keys", [])] for c in ("red", "blue")}
    else:
        html = _http_get(f"https://www.thebluealliance.com/match/{match_key}").decode("utf-8", "replace")
        for m in re.finditer(r"(?:youtube\.com/(?:embed/|watch\?v=)|youtu\.be/|youtube-nocookie\.com/embed/)([\w-]{11})", html):
            ids.append(m[1])
        for c in ("red", "blue"):
            info[c] = []
    out = []
    for i in ids:
        i = i.split("?")[0].split("#")[0]
        u = f"https://www.youtube.com/watch?v={i}"
        if u not in out:
            out.append(u)
    return out, info


# --------------------------------------------------------------------------
# YouTube download
# --------------------------------------------------------------------------
def youtube_id(url: str) -> str | None:
    m = re.search(r"(?:v=|youtu\.be/|embed/|shorts/)([\w-]{11})", url)
    return m[1] if m else (url if re.fullmatch(r"[\w-]{11}", url or "") else None)


def js_runtime_opts() -> dict:
    """yt-dlp options that point it at a JavaScript runtime (Deno) when one is installed but not on PATH: the DENO_PATH
    environment variable, ~/.scope2429/bin, ~/.deno/bin.  YouTube serves no audio without one."""
    if shutil.which("deno"):
        return {}
    exe = "deno.exe" if sys.platform == "win32" else "deno"
    for c in (os.environ.get("DENO_PATH"), APP_DIR / "bin" / exe, Path.home() / ".deno" / "bin" / exe):
        if c and Path(c).is_file():
            return {"js_runtimes": {"deno": {"path": str(c)}}}
    return {}


def download_youtube(url: str, progress=None, max_height=720) -> str:
    vid = youtube_id(url) or "video"
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    for f in CACHE_DIR.glob(f"{vid}.*"):
        if f.suffix in (".mp4", ".mkv", ".webm") and f.stat().st_size > 100_000:
            return str(f)
    try:
        import yt_dlp
    except ImportError:
        raise RuntimeError("yt-dlp is not installed. Run:  pip install -U yt-dlp")

    def hook(d):
        if progress and d.get("status") == "downloading":
            tot = d.get("total_bytes") or d.get("total_bytes_estimate") or 0
            if tot:
                progress(d.get("downloaded_bytes", 0) / tot, "Downloading video…")

    opts = {
        "format": (f"bv*[height<={max_height}][ext=mp4]+ba[ext=m4a]/"
                   f"bv*[height<={max_height}]+ba/b[height<={max_height}]/b"),
        "outtmpl": str(CACHE_DIR / "%(id)s.%(ext)s"),
        "merge_output_format": "mp4",
        "ffmpeg_location": ffmpeg_exe(),
        "quiet": True, "no_warnings": True, "noprogress": True,
        "progress_hooks": [hook],
        **js_runtime_opts(),
    }
    try:
        with yt_dlp.YoutubeDL(opts) as ydl:
            ydl.download([url])
    except Exception as e:
        raise RuntimeError(
            f"YouTube download failed: {e}\n\nTry updating yt-dlp:  pip install -U \"yt-dlp[default]\"\n"
            "Recent YouTube changes also need a JavaScript runtime — install Deno "
            "(Windows: winget install DenoLand.Deno, Mac: brew install deno) and restart.\n"
            "Or download the video yourself and use 'Local file…'.")
    for f in sorted(CACHE_DIR.glob(f"{vid}.*"), key=lambda p: -p.stat().st_size):
        if f.suffix in (".mp4", ".mkv", ".webm"):
            return str(f)
    raise RuntimeError("Download finished but no video file was found.")


# --------------------------------------------------------------------------
# Field image (AdvantageScope assets)
# --------------------------------------------------------------------------
def _advantagescope_asset_dirs():
    roots = []
    if sys.platform == "win32":
        for env in ("APPDATA", "LOCALAPPDATA"):
            if os.environ.get(env):
                roots.append(os.environ[env])
    elif sys.platform == "darwin":
        roots.append(str(Path.home() / "Library/Application Support"))
    else:
        roots.append(str(Path.home() / ".config"))
    dirs = []
    for r in roots:
        for d in glob.glob(os.path.join(r, "AdvantageScope*")):
            for sub in ("autoAssets", "userAssets", "bundledAssets"):
                if os.path.isdir(os.path.join(d, sub)):
                    dirs.append(os.path.join(d, sub))
    dirs.append(str(APP_DIR / "fields"))
    dirs.append(str(BUNDLED_FIELDS))
    return dirs


def find_field_configs(year="2026"):
    """All AdvantageScope 2D field config.json files for the given year."""
    found = []
    for d in _advantagescope_asset_dirs():
        for c in glob.glob(os.path.join(d, "Field2d_*", "config.json")):
            if year in os.path.basename(os.path.dirname(c)):
                found.append(c)
            else:
                try:
                    if year in _read_json(c).get("name", ""):
                        found.append(c)
                except Exception:
                    pass
    # prefer a plain (non-annotated) image
    found = list(dict.fromkeys(found))
    found.sort(key=lambda c: ("annot" in c.lower(), str(BUNDLED_FIELDS) in c, len(c)))
    return found


def download_field_asset(year="2026", progress=None):
    """Fetch the 2D field image from the AdvantageScopeAssets GitHub releases."""
    rel = json.loads(_http_get("https://api.github.com/repos/Mechanical-Advantage/AdvantageScopeAssets/releases"))
    cands = []
    for r in rel:
        for a in r.get("assets", []):
            nm = a["name"]
            if nm.startswith("Field2d_") and year in nm and nm.endswith(".zip"):
                cands.append(a)
    if not cands:
        raise RuntimeError("No 2D field asset for %s found on GitHub" % year)
    cands.sort(key=lambda a: ("annot" in a["name"].lower(), len(a["name"])))
    a = cands[0]
    if progress:
        progress(0, f"Downloading {a['name']}…")
    data = _http_get(a["browser_download_url"], timeout=120)
    dest = APP_DIR / "fields" / a["name"][:-4]
    dest.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        for m in z.namelist():
            base = os.path.basename(m)
            if base in ("config.json", "image.png", "image.jpg"):
                (dest / base).write_bytes(z.read(m))
    return str(dest / "config.json")


@dataclass
class FieldImage:
    img: np.ndarray            # BGR, full image
    tl: tuple                  # pixel of the field's top-left corner (config "topLeft")
    br: tuple                  # pixel of the field's bottom-right corner (config "bottomRight")
    w_m: float
    h_m: float
    name: str = ""
    # AdvantageScope convention ("wall-blue"): blue-origin x=0 maps to the RIGHT edge
    # and y=0 to the TOP edge, i.e. its images natively show red on the left.
    blue_left_native: bool = False

    def native_px(self, x, y):
        fw, fh = self.br[0] - self.tl[0], self.br[1] - self.tl[1]
        if self.blue_left_native:
            return self.tl[0] + x / self.w_m * fw, self.br[1] - y / self.h_m * fh
        return self.br[0] - x / self.w_m * fw, self.tl[1] + y / self.h_m * fh


def _read_json(path):
    """Read JSON as UTF-8 (Windows defaults to cp1252); tolerate a BOM."""
    with open(path, "r", encoding="utf-8-sig", errors="replace") as f:
        return json.load(f)


def field_is_frc_shaped(config_path) -> bool:
    """False for fields that are not about twice as long as they are wide, e.g. the FIRST Tech Challenge fields (144 x 144 in)
    that AdvantageScope installs next to the FRC ones.  The dashboard layout is built for a FRC field and crashes on those
    ("could not broadcast input array"), which showed up as a frozen preview with a "Preview error" in the status line."""
    try:
        cfg = _read_json(config_path)
        ratio = float(cfg.get("widthInches", 651.2)) / float(cfg.get("heightInches", 317.7))
        return 1.5 <= ratio <= 2.5
    except Exception:
        return True                                      # unreadable: load_field has its own fallback


def load_field(config_path: str | None) -> FieldImage:
    if config_path and os.path.exists(config_path):
        try:
            cfg = _read_json(config_path)
        except Exception as e:  # unreadable config -> drawn field instead of crashing
            print(f"Could not read field config {config_path}: {e}", file=sys.stderr)
            return synthetic_field()
        d = os.path.dirname(config_path)
        imgp = next((os.path.join(d, f) for f in ("image.png", "image.jpg") if os.path.exists(os.path.join(d, f))), None)
        img = cv2.imdecode(np.fromfile(imgp, np.uint8), cv2.IMREAD_COLOR) if imgp else None
        if img is not None:
            return FieldImage(img, tuple(cfg["topLeft"]), tuple(cfg["bottomRight"]),
                              cfg.get("widthInches", 651.2) * 0.0254, cfg.get("heightInches", 317.7) * 0.0254,
                              cfg.get("name", ""),
                              blue_left_native=cfg.get("coordinateSystem") == "center-rotated")
    return synthetic_field()


# 2026 REBUILT element geometry, blue-origin metres, blue side only (red = point-mirrored).
# Derived from the WPILib 2026-rebuilt-welded AprilTag layout + game manual dimensions.
IN = 0.0254
REBUILT_ELEMENTS = {
    "hub":      [(4.022, 3.431, 5.229, 4.638)],                        # 47.5 x 47.5 in
    "bump":     [(4.061, 1.577, 5.189, 3.431), (4.061, 4.638, 5.189, 6.492)],  # 73 x 44.4 in ramps
    "trench":   [(4.028, 0.0, 5.222, 1.577), (4.028, 6.492, 5.222, 8.069)],    # 47 in deep, guardrail->bump
    "tower":    [(0.0, 3.336, 1.143, 4.587)],                        # 49.25 x 45 in, on alliance wall
    "depot":    [(0.0, 5.42, 0.686, 6.49)],                          # 42 x 27 in, along alliance wall
    "outpost":  [(0.0, 0.43, 0.955, 1.34)],                          # corral at wall/guardrail corner
}
ALLIANCE_ZONE_M = 158.6 * IN


def synthetic_field() -> FieldImage:
    """Drawn 2026 REBUILT field (hub, bumps/ramps, trenches, tower, depot, outpost).
    Drawn in AdvantageScope's native orientation: red on the left, blue origin top-right."""
    ppm = 120
    m = 50
    L, W = FIELD_W_M, FIELD_H_M
    w, h = int(L * ppm), int(W * ppm)
    img = np.full((h + 2 * m, w + 2 * m, 3), (40, 40, 40), np.uint8)
    tl, br = (m, m), (m + w, m + h)
    fi = FieldImage(img, tl, br, L, W, "2026 REBUILT (drawn)")

    def P(x, y):
        px, py = fi.native_px(x, y)
        return int(round(px)), int(round(py))

    def rect(x0, y0, x1, y1, color, thick=-1, mirror=False):
        if mirror:
            x0, y0, x1, y1 = L - x1, W - y1, L - x0, W - y0
        cv2.rectangle(img, P(x0, y0), P(x1, y1), color, thick, cv2.LINE_AA)

    cv2.rectangle(img, tl, br, (92, 92, 92), -1)                     # carpet
    BLUE, RED = (200, 90, 30), (40, 40, 210)
    # re-tint zones (simple alpha blend)
    zone = np.zeros_like(img)
    for mir, col in ((False, BLUE), (True, RED)):
        x0, x1 = (0, ALLIANCE_ZONE_M) if not mir else (L - ALLIANCE_ZONE_M, L)
        a, b = P(x0, 0), P(x1, W)
        cv2.rectangle(zone, (min(a[0], b[0]), min(a[1], b[1])), (max(a[0], b[0]), max(a[1], b[1])), col, -1)
    mask = zone.any(axis=2)
    img[mask] = (0.75 * img[mask] + 0.25 * zone[mask]).astype(np.uint8)
    # lines: centre line, starting lines (alliance zone edge)
    cv2.line(img, P(L / 2, 0), P(L / 2, W), (235, 235, 235), 3, cv2.LINE_AA)
    cv2.line(img, P(ALLIANCE_ZONE_M, 0), P(ALLIANCE_ZONE_M, W), BLUE, 3, cv2.LINE_AA)
    cv2.line(img, P(L - ALLIANCE_ZONE_M, 0), P(L - ALLIANCE_ZONE_M, W), RED, 3, cv2.LINE_AA)

    for mir, col in ((False, BLUE), (True, RED)):
        dark = tuple(int(c * 0.55) for c in col)
        light = tuple(min(255, int(c * 0.6 + 100)) for c in col)
        for r in REBUILT_ELEMENTS["bump"]:                    # ramps: shaded with ramp edges
            rect(*r, light, mirror=mir)
            x0, y0, x1, y1 = r
            k = (x1 - x0) * 0.27
            rect(x0 + k, y0, x1 - k, y1, tuple(min(255, c + 25) for c in light), mirror=mir)
            rect(*r, dark, 3, mirror=mir)
        for r in REBUILT_ELEMENTS["trench"]:                  # trench: floor + overhead arm
            rect(*r, (70, 70, 70), mirror=mir)
            x0, y0, x1, y1 = r
            xc = (x0 + x1) / 2
            rect(xc - 0.10, y0, xc + 0.10, y1, col, mirror=mir)
            rect(*r, (200, 200, 200), 2, mirror=mir)
        for r in REBUILT_ELEMENTS["hub"]:                     # hub: box + hexagonal opening
            rect(*r, (55, 55, 55), mirror=mir)
            rect(*r, col, 4, mirror=mir)
            x0, y0, x1, y1 = r
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if mir:
                cx, cy = L - cx, W - cy
            rr = 41.7 * IN / 2
            hexp = np.array([P(cx + rr * math.cos(a), cy + rr * math.sin(a))
                             for a in np.arange(6) * math.pi / 3], np.int32)
            cv2.fillPoly(img, [hexp], (190, 190, 190), cv2.LINE_AA)
            cv2.polylines(img, [hexp], True, col, 3, cv2.LINE_AA)
        for r in REBUILT_ELEMENTS["tower"]:                   # tower: base + two uprights + rungs
            rect(*r, (35, 35, 35), mirror=mir)
            rect(*r, col, 3, mirror=mir)
            x0, y0, x1, y1 = r
            yc = (y0 + y1) / 2
            for dy in (-32.25 * IN / 2, 32.25 * IN / 2):
                rect(x1 - 0.12, yc + dy - 0.05, x1 - 0.02, yc + dy + 0.05, (220, 220, 220), mirror=mir)
            rect(x1 - 0.09, yc - 32.25 * IN / 2, x1 - 0.05, yc + 32.25 * IN / 2, (170, 170, 170), mirror=mir)
        for r in REBUILT_ELEMENTS["depot"]:
            rect(*r, (60, 140, 170), mirror=mir)
            rect(*r, col, 3, mirror=mir)
        for r in REBUILT_ELEMENTS["outpost"]:
            rect(*r, (30, 30, 30), mirror=mir)
            rect(*r, col, 3, mirror=mir)
    # labels (small, blue side + red side)
    fs = 0.9
    for mir in (False, True):
        for name, key in (("HUB", "hub"), ("TOWER", "tower"), ("DEPOT", "depot"), ("OUTPOST", "outpost")):
            x0, y0, x1, y1 = REBUILT_ELEMENTS[key][0]
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if mir:
                cx, cy = L - cx, W - cy
            if key in ("tower", "depot", "outpost"):
                cx = (L - 1.9) if mir else 1.9
            px, py = P(cx, cy)
            tw = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, fs, 2)[0][0]
            cv2.putText(img, name, (px - tw // 2, py + (int(1.0 * ppm) if key == "hub" else 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (235, 235, 235), 2, cv2.LINE_AA)
        x0, y0, x1, y1 = REBUILT_ELEMENTS["trench"][0]
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        if mir:
            cx, cy = L - cx, W - cy
        px, py = P(cx, cy)
        tw = cv2.getTextSize("TRENCH", cv2.FONT_HERSHEY_SIMPLEX, fs * 0.8, 2)[0][0]
        cv2.putText(img, "TRENCH", (px - tw // 2, py + 8), cv2.FONT_HERSHEY_SIMPLEX, fs * 0.8, (235, 235, 235), 2, cv2.LINE_AA)
    cv2.rectangle(img, tl, br, (230, 230, 230), 4)
    fi.img = img
    return fi


# --------------------------------------------------------------------------
# Rendering of the field panel
# --------------------------------------------------------------------------
QUEST_COLOR = (80, 220, 60)    # BGR green
DRIVE_COLOR = (0, 190, 255)    # BGR amber


@dataclass
class RenderStyle:
    robot_size_m: float = 0.90
    trail_s: float = 2.0
    red_left: bool = True        # rotate 180° so red alliance is on the left
    title: str = ""
    show_status: bool = True     # QuestNav status bar under the field (classic layout only)
    dashboard: bool = True       # 1920x1080 layout: 2429-dashboard replica + telemetry panels (see dash.py)


class FieldPanel:
    """Draws the 2D field at a fixed panel size."""
    HEADER = 0.075  # fraction of panel height for the header bar

    def __init__(self, field: FieldImage, width: int, height: int, style: RenderStyle):
        self.f = field
        self.W, self.H = width, height
        self.style = style
        self.hdr = max(24, int(height * self.HEADER))
        self.ftr = int(self.hdr * 1.9) if style.show_status else 0   # Quest: 2 rows of 4, cameras: 1 row
        fi = field.img
        # where does the red wall (x = field length) land natively?
        red_native_left = field.native_px(field.w_m, field.h_m / 2)[0] < field.native_px(0, field.h_m / 2)[0]
        self.rot = (red_native_left != style.red_left)
        ih, iw = fi.shape[:2]
        self.iw, self.ih = iw, ih
        if self.rot:
            fi = cv2.rotate(fi, cv2.ROTATE_180)
            tl = (iw - 1 - field.br[0], ih - 1 - field.br[1])
            br = (iw - 1 - field.tl[0], ih - 1 - field.tl[1])
        else:
            tl, br = field.tl, field.br
        # crop to field + small margin, then fit into the panel area
        fw, fh = br[0] - tl[0], br[1] - tl[1]
        mx, my = int(fw * 0.03), int(fh * 0.04)
        ih, iw = fi.shape[:2]
        x0, y0 = max(0, tl[0] - mx), max(0, tl[1] - my)
        x1, y1 = min(iw, br[0] + mx), min(ih, br[1] + my)
        crop = fi[y0:y1, x0:x1]
        avail_w, avail_h = width, height - self.hdr - self.ftr
        s = min(avail_w / crop.shape[1], avail_h / crop.shape[0])
        cw, ch = int(crop.shape[1] * s), int(crop.shape[0] * s)
        small = cv2.resize(crop, (cw, ch), interpolation=cv2.INTER_AREA)
        self.base = np.full((height, width, 3), (24, 24, 24), np.uint8)
        ox = (avail_w - cw) // 2
        oy = self.hdr + (avail_h - ch) // 2
        self.base[oy:oy + ch, ox:ox + cw] = small
        # image pixel (after optional rotation) -> panel pixel
        self.ox, self.oy, self.x0, self.y0, self.s = ox, oy, x0, y0, s
        self.sx = fw * s / field.w_m
        self.sy = fh * s / field.h_m

    def to_px(self, x, y):
        """blue-origin field metres -> panel pixels"""
        px, py = self.f.native_px(x, y)
        if self.rot:
            px, py = self.iw - 1 - px, self.ih - 1 - py
        return self.ox + (px - self.x0) * self.s, self.oy + (py - self.y0) * self.s

    def _pt(self, x, y):
        px, py = self.to_px(x, y)
        return int(round(px * 16)), int(round(py * 16))

    def draw_robot(self, img, pose, color, label=None):
        x, y, th = pose
        if x != x:  # NaN
            return
        r = self.style.robot_size_m / 2
        c, s = math.cos(th), math.sin(th)
        corners = [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((r, r), (-r, r), (-r, -r), (r, -r))]
        pts = np.array([self._pt(*p) for p in corners], np.int32)
        thick = max(2, int(self.sx * 0.06))
        cv2.polylines(img, [pts], True, color, thick, cv2.LINE_AA, shift=4)
        # heading arrow (AdvantageScope-like)
        tip = self._pt(x + c * r * 1.0, y + s * r * 1.0)
        bl = self._pt(x + c * (r * 0.1) - s * (r * 0.55), y + s * (r * 0.1) + c * (r * 0.55))
        brr = self._pt(x + c * (r * 0.1) + s * (r * 0.55), y + s * (r * 0.1) - c * (r * 0.55))
        cv2.polylines(img, [np.array([bl, tip, brr], np.int32)], False, color, thick, cv2.LINE_AA, shift=4)
        if label:
            px, py = self.to_px(x, y)
            cv2.putText(img, label, (int(px) + int(r * self.sx) + 4, int(py) - int(r * self.sy) - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, max(0.35, self.H / 1300), color, 1, cv2.LINE_AA)

    def draw_trail(self, img, series: PoseSeries, t, color):
        if self.style.trail_s <= 0:
            return
        xs, ys = series.trail(t - self.style.trail_s, t)
        if len(xs) < 2:
            return
        ok = ~np.isnan(xs)
        pts = [self._pt(a, b) for a, b, k in zip(xs, ys, ok) if k]
        if len(pts) >= 2:
            cv2.polylines(img, [np.array(pts, np.int32)], False, color,
                          max(1, int(self.sx * 0.035)), cv2.LINE_AA, shift=4)

    def render(self, ml: MatchLog, t: float, series: list[tuple[PoseSeries, tuple, str]]):
        img = self.base.copy()
        for s, col, lab in series:
            self.draw_trail(img, s, t, col)
        for s, col, lab in series:
            p = s.at(t)
            if p is not None:
                self.draw_robot(img, p, col)
        # header
        H = self.hdr
        fs = H / 48.0
        cv2.rectangle(img, (0, 0), (self.W, H), (16, 16, 16), -1)
        y = int(H * 0.68)
        x = 10
        title = self.style.title
        if title:
            cv2.putText(img, title, (x, y), cv2.FONT_HERSHEY_SIMPLEX, fs * 0.9, (235, 235, 235), 1, cv2.LINE_AA)
            x += cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, fs * 0.9, 1)[0][0] + 18
        for s, col, lab in series:
            cv2.rectangle(img, (x, int(H * 0.3)), (x + int(H * 0.4), int(H * 0.7)), col, -1)
            x += int(H * 0.4) + 6
            cv2.putText(img, lab, (x, y), cv2.FONT_HERSHEY_SIMPLEX, fs * 0.8, (220, 220, 220), 1, cv2.LINE_AA)
            x += cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, fs * 0.8, 1)[0][0] + 14
        phase, rem = ml.clock(t)
        if phase:
            txt = phase if rem is None else f"{phase} {int(math.ceil(rem)) // 60}:{int(math.ceil(rem)) % 60:02d}"
            tw = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, fs, 2)[0][0]
            col = (60, 220, 255) if phase == "AUTO" else (235, 235, 235)
            cv2.putText(img, txt, (self.W - tw - 10, y), cv2.FONT_HERSHEY_SIMPLEX, fs, col, 2, cv2.LINE_AA)
        if self.ftr:
            self.draw_status(img, ml, t)
        return img

    def draw_status(self, img, ml: MatchLog, t: float):
        """Labelled indicator groups (QUEST, ARDUCAM) along the bottom of the panel."""
        y0 = self.H - self.ftr
        cv2.rectangle(img, (0, y0), (self.W, self.H), (16, 16, 16), -1)
        nrows = sum(-(-len(ents) // 4) for _g, ents in STATUS_GROUPS)
        rh = self.ftr / nrows
        fs = rh / 30.0
        tagw = int(self.W * 0.125)
        cols = 4
        cw = (self.W - tagw) / cols
        GREEN, RED, AMBER, GRAY = (90, 200, 70), (60, 60, 220), (0, 180, 255), (110, 110, 110)
        row0 = 0
        for gname, ents in STATUS_GROUPS:
            grows = -(-len(ents) // cols)
            gy0, gy1 = int(y0 + row0 * rh), int(y0 + (row0 + grows) * rh)
            if row0:
                cv2.line(img, (0, gy0), (self.W, gy0), (60, 60, 60), 1)
            tfs = fs * 0.8
            (tw, th), _ = cv2.getTextSize(gname, cv2.FONT_HERSHEY_SIMPLEX, tfs, 2)
            cv2.putText(img, gname, (6, (gy0 + gy1) // 2 + th // 2), cv2.FONT_HERSHEY_SIMPLEX, tfs,
                        (60, 220, 255), 2, cv2.LINE_AA)
            for k, (name, label, kind) in enumerate(ents):
                r, c = divmod(k, cols)
                x = tagw + int(c * cw) + 6
                yc = int(y0 + (row0 + r) * rh + rh / 2)
                ser = ml.status.get(name)
                v, since = ser.at(t) if ser else (None, None)
                flash = since is not None and since < 1.0          # changed within the last second
                rad = max(4, int(rh * 0.30))
                if kind == "count":
                    txt = "-" if v is None else f"{int(v)}"
                    col = GRAY if v is None else (AMBER if (flash or v > 0) else GREEN)
                    cfs = fs * 0.95
                    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, cfs, 2)
                    box_w = max(tw + 10, int(rh * 0.9))
                    top, bot = int(yc - rh * 0.36), int(yc + rh * 0.36)
                    cv2.rectangle(img, (x, top), (x + box_w, bot), col, -1, cv2.LINE_AA)
                    if flash:
                        cv2.rectangle(img, (x - 2, top - 2), (x + box_w + 2, bot + 2), (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(img, txt, (x + (box_w - tw) // 2, yc + th // 2), cv2.FONT_HERSHEY_SIMPLEX,
                                cfs, (15, 15, 15), 2, cv2.LINE_AA)
                    lx = x + box_w + 6
                else:
                    if v is None:
                        col = GRAY
                    elif kind == "bool_warn":
                        col = AMBER if v else GREEN
                    elif kind == "bool_dim":
                        col = GREEN if v else (70, 70, 70)
                    else:
                        col = GREEN if v else RED
                    cv2.circle(img, (x + rad, yc), rad, col, -1, cv2.LINE_AA)
                    if flash:
                        cv2.circle(img, (x + rad, yc), rad + 3, (255, 255, 255), 1, cv2.LINE_AA)
                    lx = x + 2 * rad + 6
                cv2.putText(img, label, (lx, yc + int(rad * 0.6)), cv2.FONT_HERSHEY_SIMPLEX, fs * 0.8,
                            (235, 235, 235) if flash else (200, 200, 200), 1, cv2.LINE_AA)
            row0 += grows


def default_series(ml: MatchLog, quest_name, drive_name):
    out = []
    if drive_name and drive_name in ml.poses:
        out.append((ml.poses[drive_name], DRIVE_COLOR, drive_name.split("/")[-1]))
    if quest_name and quest_name in ml.poses:
        out.append((ml.poses[quest_name], QUEST_COLOR, quest_name.split("/")[-1]))
    return out


# --------------------------------------------------------------------------
# Auto-sync from audio
# --------------------------------------------------------------------------
def _audio(path, sr=8000) -> np.ndarray:
    r = run_ff(["-i", str(path), "-vn", "-ac", "1", "-ar", str(sr), "-f", "s16le", "-"])
    return np.frombuffer(r.stdout, np.int16).astype(np.float32) / 32768.0


def onset_envelope(path, sr=8000, hop_s=0.01):
    """Positive spectral flux in 400-3500 Hz (field horns, buzzers, bells)."""
    a = _audio(path, sr)
    if len(a) < sr:
        raise RuntimeError("Video has no usable audio track")
    win = int(0.064 * sr)
    hop = int(hop_s * sr)
    n = 1 + (len(a) - win) // hop
    frames = np.lib.stride_tricks.as_strided(a, (n, win), (a.strides[0] * hop, a.strides[0]))
    spec = np.abs(np.fft.rfft(frames * np.hanning(win), axis=1))
    freqs = np.fft.rfftfreq(win, 1 / sr)
    band = (freqs >= 400) & (freqs <= 3500)
    S = np.log1p(spec[:, band] * 50)
    flux = np.maximum(0, np.diff(S, axis=0, prepend=S[:1])).sum(axis=1)
    # remove slow loudness changes (crowd) and normalise robustly
    k = int(1.5 / hop_s)
    base = np.convolve(flux, np.ones(k) / k, mode="same")
    env = np.maximum(0, flux - base)
    scale = np.percentile(env, 99.0) + 1e-9   # top 1% ~ "loud onset"
    # frame i spans [i*hop, i*hop+win]; shift so values are indexed by window centre
    shift = int(round(win / 2 / hop))
    env = np.concatenate([np.zeros(shift), env])
    return np.clip(env / scale, 0, 3.0), hop_s


def auto_sync(video_path, ml: MatchLog, search=None, progress=None):
    """Find the video time at which autonomous starts.

    Returns (video_time_of_auto_start, confidence, details).  Confidence is a
    z-score of the best match vs. all other candidate offsets (>6 is solid).
    """
    ev = ml.phase_events
    if "auto_start" not in ev:
        raise RuntimeError("No FMS autonomous start found in this log")
    rel = np.array([v - ev["auto_start"] for v in ev.values()])
    w = np.array([1.5 if k in ("auto_start", "teleop_end") else 1.0 for k in ev.keys()])
    if progress:
        progress(0.1, "Analysing audio…")
    env, hop = onset_envelope(video_path)
    # tolerance window: ±0.12 s
    tol = int(0.12 / hop)
    padded = np.pad(env, (tol, tol))
    M = np.lib.stride_tricks.sliding_window_view(padded, 2 * tol + 1).max(axis=1)
    n = len(M)
    lo, hi = (search or (0.0, n * hop - rel.max()))
    cands = np.arange(max(0, int(lo / hop)), max(1, int(hi / hop)))
    if len(cands) == 0:
        raise RuntimeError("Video is shorter than the match")
    score = np.zeros(len(cands))
    for r_, wi in zip(rel, w):
        idx = cands + int(round(r_ / hop))
        valid = idx < n
        score[valid] += wi * M[idx[valid]]
    best = int(np.argmax(score))
    # local refine: exact peak (no tolerance) around the best estimate
    b = cands[best]
    fine = np.arange(max(0, b - tol), min(n, b + tol + 1))
    fscore = np.zeros(len(fine))
    for r_, wi in zip(rel, w):
        idx = fine + int(round(r_ / hop))
        valid = idx < n
        sm = np.convolve(env, np.ones(3) / 3, mode="same")
        fscore[valid] += wi * sm[idx[valid]]
    b2 = fine[int(np.argmax(fscore))]
    # confidence: compare against offsets more than 1 s away
    far = np.abs(cands - b) > int(1.0 / hop)
    other = score[far] if far.any() else score
    conf = float((score[best] - np.median(other)) / (np.std(other) + 1e-9))
    second = float(np.max(other)) if len(other) else 0
    details = {"score": float(score[best]), "runner_up": second, "ratio": float(score[best] / (second + 1e-9))}
    # the best few distinct candidates (more than 5 s apart), each refined like the best: for callers that can check them
    # against something else (webcast.py checks the broadcast's fuel counter)
    peaks, work = [], score.copy()
    sm3 = np.convolve(env, np.ones(3) / 3, mode="same")
    for _ in range(8):
        k = int(np.argmax(work))
        if work[k] <= 0:
            break
        c = cands[k]
        fi = np.arange(max(0, c - tol), min(n, c + tol + 1))
        fs = np.zeros(len(fi))
        for r_, wi in zip(rel, w):
            ix = fi + int(round(r_ / hop))
            ok = ix < n
            fs[ok] += wi * sm3[ix[ok]]
        peaks.append((float(fi[int(np.argmax(fs))] * hop), float(score[k])))
        work[np.abs(cands - c) <= int(5.0 / hop)] = 0
    details["peaks"] = peaks
    return b2 * hop, conf, details


# --------------------------------------------------------------------------
# Compose final video
# --------------------------------------------------------------------------
@dataclass
class RenderJob:
    log: MatchLog
    video: str | None             # None: no match video, the dashboard's top-right slot shows trend plots
    offset: float                 # video time (s) at which auto starts (log auto_start)
    out_path: str
    field: FieldImage
    quest_name: str = DEFAULT_QUEST
    drive_name: str = DEFAULT_DRIVE
    height: int = 540             # output height (360 / 540 / 720)
    fps: int = 30
    pre_s: float = 5.0
    post_s: float = 5.0
    crf: int = 27
    style: RenderStyle = field(default_factory=RenderStyle)
    trends: list = field(default_factory=list)     # extra entry names for the trend plots (see dash.TrendPanel)
    thresholds: dict = field(default_factory=dict)  # deviation indicator thresholds: rpm / speed / angle (see dash.DEFAULT_THR)
    hoot: object = None                            # hoot.HootLog (CTRE device signals in the robot clock), shown in the swerve / PDH panels


def compose_frame(job: RenderJob, panel: FieldPanel, video_frame, t_log):
    left = panel.render(job.log, t_log, default_series(job.log, job.quest_name, job.drive_name))
    if video_frame is None:
        return left
    h = left.shape[0]
    vf = cv2.resize(video_frame, (int(round(video_frame.shape[1] * h / video_frame.shape[0])) // 2 * 2, h),
                    interpolation=cv2.INTER_AREA)
    return np.hstack([left, vf])


_DASH_CACHE: dict = {}


def _fuel_flux(job: RenderJob, compute: bool, progress=None):
    """Fuel counted above the hub in the match video (balls.py).  Cached on disk; only computed
    (about 1.5 min) when `compute` is true, i.e. for a real render, not for GUI previews."""
    if not job.video:
        return None
    import balls
    alliance = job.log.alliance[0]
    cache = CACHE_DIR / f"{Path(job.video).stem}_{alliance}_flux.npz"
    if not compute and not cache.exists():
        return None
    try:
        return balls.measure(job.video, job.offset, alliance, cache_path=cache, progress=progress, compute=compute)
    except Exception as e:  # noqa - the estimate then falls back to the log-only model
        print(f"fuel counting failed: {e}", file=sys.stderr)
        return None


def get_dash(job: RenderJob, compute_flux: bool = False, progress=None):
    """The (expensive to build) dashboard renderer for this job's log, cached between previews."""
    flux = _fuel_flux(job, compute_flux, progress)
    key = (job.log.path, job.field.name, job.style.red_left, job.drive_name, job.quest_name, tuple(job.trends), id(job.hoot) if job.hoot else None,
           job.offset if flux is not None else None, job.video if flux is not None else None)
    d = _DASH_CACHE.get(key)
    if d is None:
        import dash
        d = dash.DashRenderer(job.log, job.field, job.style.title, job.style.red_left,
                              pose_names={"drive": job.drive_name, "quest": job.quest_name}, flux=flux,
                              trends=job.trends, hoot=job.hoot)
        _DASH_CACHE.clear()
        _DASH_CACHE[key] = d
    d.title = job.style.title            # editable without rebuilding
    if job.thresholds:
        d.thr = {**d.thr, **job.thresholds}
    return d


def preview(job: RenderJob, t_rel: float):
    """Preview at t_rel seconds after time zero (auto start, or the first enable / start of a log without a match)."""
    if job.style.dashboard or not job.video:
        fr = grab_frame(job.video, job.offset + t_rel) if job.video else None
        return get_dash(job).render(job.log.t_ref + t_rel, fr)
    vi = video_info(job.video)
    vw = int(round((vi["width"] or 16) / (vi["height"] or 9) * job.height)) // 2 * 2
    panel = FieldPanel(job.field, vw, job.height, job.style)
    fr = grab_frame(job.video, job.offset + t_rel)
    return compose_frame(job, panel, fr, job.log.t_ref + t_rel)


def render(job: RenderJob, progress=None, cancel=None):
    ml = job.log
    novideo = not job.video
    if ml.auto_start is None and not novideo:
        raise RuntimeError("No match start (FMS auto enable) in this log, so it cannot be synced to a match video. "
                           "Clear the video to render the dashboard on its own.")
    vi = video_info(job.video) if job.video else {"duration": None, "width": None, "height": None,
                                                  "fps": 30.0, "has_audio": False}
    H = job.height
    vw = int(round((vi["width"] or 16) / (vi["height"] or 9) * H)) // 2 * 2
    use_dash = job.style.dashboard or novideo
    dsh = get_dash(job, True, progress) if use_dash else None
    panel = None if dsh else FieldPanel(job.field, vw, H, job.style)

    pre, post_s = job.pre_s, job.post_s
    if not ml.has_match:                     # nothing is known about the clip's end or start: use the log's own
        pre, post_s = min(pre, max(0.0, ml.t_ref - ml.t_first)), 0.0
    v_start = job.offset - pre
    lead = 0.0
    if novideo:
        v_start = 0.0
    elif v_start < 0:        # video starts after our desired start: just start at 0
        lead = -v_start
        v_start = 0.0
    dur = pre - lead + ml.match_len + post_s
    if vi["duration"]:
        dur = min(dur, vi["duration"] - v_start)
    nframes = int(dur * job.fps)
    t_log0 = ml.t_ref - pre + lead

    os.makedirs(os.path.dirname(os.path.abspath(job.out_path)) or ".", exist_ok=True)
    W = vw * 2
    if dsh:
        # 1920x1080 canvas drawn here (video slot left black); ffmpeg overlays the match video at the top right
        out_h = H * 2
        post = "" if out_h == 1080 else f"scale=-2:{out_h}:flags=lanczos,"
        raw = [ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-y",
               "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", "1920x1080", "-r", str(job.fps), "-i", "-"]
        if novideo:                           # the trend plots fill the slot, nothing to overlay
            args = raw + ["-vf", f"{post}format=yuv420p"]
        else:
            args = raw + ["-ss", f"{v_start:.3f}", "-t", f"{dur:.3f}", "-i", job.video,
                          "-filter_complex",
                          f"[1:v]scale=960:540:force_original_aspect_ratio=decrease:flags=bicubic,"
                          f"pad=960:540:(ow-iw)/2:(oh-ih)/2,fps={job.fps},setsar=1[r];"
                          f"[0:v][r]overlay=960:0:shortest=1,{post}format=yuv420p[v]",
                          "-map", "[v]"]
    else:
        args = [ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-y",
                "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{vw}x{H}", "-r", str(job.fps), "-i", "-",
                "-ss", f"{v_start:.3f}", "-t", f"{dur:.3f}", "-i", job.video,
                "-filter_complex",
                f"[1:v]scale={vw}:{H}:flags=bicubic,fps={job.fps},setsar=1[r];[0:v][r]hstack=inputs=2,format=yuv420p[v]",
                "-map", "[v]"]
    if vi["has_audio"]:
        args += ["-map", "1:a:0?", "-c:a", "aac", "-b:a", "96k", "-ac", "2"]
    args += ["-c:v", "libx264", "-preset", "medium", "-crf", str(job.crf), "-profile:v", "high",
             "-level", "5.1" if (dsh and H * 2 > 1080) else "4.0", "-movflags", "+faststart", "-shortest", "-r", str(job.fps), job.out_path]
    proc = subprocess.Popen(args, stdin=subprocess.PIPE, stderr=subprocess.PIPE, creationflags=NO_WINDOW)
    series = default_series(ml, job.quest_name, job.drive_name)
    try:
        for i in range(nframes):
            if cancel and cancel():
                proc.kill()
                raise RuntimeError("Cancelled")
            img = dsh.render(t_log0 + i / job.fps, trends=novideo) if dsh else panel.render(ml, t_log0 + i / job.fps, series)
            proc.stdin.write(img.tobytes())
            if progress and i % 15 == 0:
                progress(i / nframes, f"Rendering {i}/{nframes} frames")
        proc.stdin.close()
    except (BrokenPipeError, OSError):
        pass
    err = proc.stderr.read().decode("utf-8", "replace")
    proc.wait()
    if proc.returncode != 0:
        raise RuntimeError("ffmpeg failed:\n" + err[-2000:])
    if progress:
        progress(1.0, "Done")
    return job.out_path
