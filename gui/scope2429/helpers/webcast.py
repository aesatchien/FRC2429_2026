"""
Whole-event webcast videos: not every match has its own YouTube video, but the event streams the whole day (SoCal
Showdown: one 10 hour video per day).  This module finds those videos, works out where in them a match is, downloads
only that stretch and lines it up with the robot log, so a match video can be made for every match 2429 played.

  1. wall clock: the robot log records the computer time (`systemTime`), so the moment auto started is known to
     within the roboRIO clock's error (seconds to a minute or two).
  2. the day video's start: yt-dlp gives the time the live stream started (`release_timestamp`).  The match is at
     (auto start - stream start) seconds into the video, give or take.
  3. only that stretch (a few minutes either side) is downloaded, and core.auto_sync, which listens for the field
     sounds, finds the exact moment inside it.  The result is an ordinary video file and offset for core.render.

    days = [day_video(url) for url in urls]
    clip = clip_match(log_path, ml, days)         # -> Clip(path, offset, confidence, ...)
"""
from __future__ import annotations

import os
import re
import shutil
import struct
import subprocess
import threading
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

import core

CLIP_DIR = core.CACHE_DIR / "clips"
WINDOW_S = 420.0            # seconds each side of the estimate that are downloaded and searched (first match of a day)
CLIP_VERSION = 3            # clips made by an older version (single-match calibration) can be of the neighbouring match
NARROW_S = 90.0             # ... once the day's clock error is known from an earlier match (downloads are slow: about real time)
MAX_HEIGHT = 720


class WebcastError(RuntimeError):
    pass


def _yt():
    try:
        import yt_dlp
        return yt_dlp
    except ImportError:
        raise WebcastError("yt-dlp is not installed. Run:  pip install -U yt-dlp")


# ------------------------------------------------------------------------------------------ finding the videos
def search_day_videos(query="SoCal Showdown Fall 2026 Day", count=15, min_hours=3.0):
    """Long videos (whole-day streams) that match a YouTube search: [{id, title, hours, channel, url}]."""
    yt = _yt()
    with yt.YoutubeDL({"quiet": True, "no_warnings": True, "extract_flat": True, "skip_download": True}) as y:
        r = y.extract_info(f"ytsearch{count}:{query}", download=False)
    out = []
    for e in r.get("entries") or []:
        dur = e.get("duration") or 0
        if dur >= min_hours * 3600:
            out.append({"id": e["id"], "title": e.get("title", ""), "hours": dur / 3600.0,
                        "channel": e.get("channel") or e.get("uploader") or "", "url": f"https://www.youtube.com/watch?v={e['id']}"})
    return out


@dataclass
class DayVideo:
    id: str
    url: str
    title: str
    duration: float                 # s
    start_epoch: float              # UTC time the stream started (video t = 0), from yt-dlp's release timestamp
    extra: dict = field(default_factory=dict)

    def contains(self, epoch, margin=0.0):
        return self.start_epoch - margin <= epoch <= self.start_epoch + self.duration + margin

    def label(self):
        t = time.strftime("%a %m-%d %H:%M", time.gmtime(self.start_epoch))
        return f"{self.title}  ({self.duration / 3600:.1f} h, starts {t} UTC)"

    def to_dict(self):
        return {"id": self.id, "url": self.url, "title": self.title, "duration": self.duration,
                "start_epoch": self.start_epoch}

    @staticmethod
    def from_dict(d):
        return DayVideo(d["id"], d["url"], d.get("title", ""), float(d["duration"]), float(d["start_epoch"]))


def day_video(url_or_id) -> DayVideo:
    """Look a video up with yt-dlp (no download): its length and the time the live stream started."""
    yt = _yt()
    vid = core.youtube_id(url_or_id) or url_or_id
    url = f"https://www.youtube.com/watch?v={vid}"
    try:
        with yt.YoutubeDL({"quiet": True, "no_warnings": True, "skip_download": True, **core.js_runtime_opts()}) as y:
            i = y.extract_info(url, download=False)
    except Exception as e:  # noqa
        raise WebcastError(f"could not read {url}: {e}")
    start = i.get("release_timestamp") or i.get("timestamp")
    if not start or not i.get("duration"):
        raise WebcastError(f"{url} has no stream start time / length, so matches cannot be located in it")
    if i.get("release_timestamp") is None:
        start = start - i["duration"]                  # only the upload time is known: assume it ended at upload
    return DayVideo(vid, url, i.get("title", ""), float(i["duration"]), float(start))


# ------------------------------------------------------------------------------------------ wall clock of a log
def log_epoch0(log_path) -> float | None:
    """UTC seconds at log time zero, from the log's `systemTime` records (the computer's clock), else from the file name
    (WPILIB_YYYYMMDD_HHMMSS = UTC time the file was created, which is where log time 0 is)."""
    try:
        _e, recs = core.read_wpilog(log_path, {"systemTime"})
        rows = recs.get("systemTime") or []
        if rows:
            offs = sorted(struct.unpack("<q", p)[0] / 1e6 - t / 1e6 for t, p in rows[:50])
            return float(offs[len(offs) // 2])
    except Exception:  # noqa
        pass
    m = re.search(r"_(\d{8})_(\d{6})_", Path(log_path).name)
    if m:
        import calendar
        t = time.strptime(m[1] + m[2], "%Y%m%d%H%M%S")
        return float(calendar.timegm(t))
    return None


def locate(log_path, ml, days):
    """(DayVideo, estimated video time of auto start in s) for the day video that covers this match, or (None, reason)."""
    if not ml.has_match:
        return None, "the log has no match start (no autonomous enable)"
    e0 = log_epoch0(log_path)
    if e0 is None:
        return None, "the log has no wall-clock time"
    auto_epoch = e0 + ml.auto_start
    for d in days:
        if d.contains(auto_epoch):
            return d, auto_epoch - d.start_epoch
    t = time.strftime("%a %m-%d %H:%M UTC", time.gmtime(auto_epoch))
    return None, f"no whole-day video covers this match (it started {t})"


# ------------------------------------------------------------------------------------------ clipping
@dataclass
class Clip:
    path: str
    offset: float            # video time (s) of auto start inside the clip
    confidence: float
    good: bool
    day: DayVideo
    est_video_t: float       # where in the whole-day video the match was expected
    found_video_t: float     # where auto-sync actually found it in the whole-day video
    note: str = ""


_FF_DONE = False


def _ffmpeg_on_path():
    """yt-dlp looks for ffmpeg only on PATH when it cuts a section (it ignores ffmpeg_location there), and it caches a
    'no ffmpeg' answer from an earlier look: put ours on PATH once and clear that cache."""
    global _FF_DONE
    if _FF_DONE:
        return
    os.environ["PATH"] = str(Path(core.ffmpeg_exe()).parent) + os.pathsep + os.environ.get("PATH", "")
    try:
        from yt_dlp.postprocessor.ffmpeg import FFmpegPostProcessor
        FFmpegPostProcessor._version_cache.clear()
        FFmpegPostProcessor._version_cache[None] = None
        FFmpegPostProcessor._features_cache.clear()
    except Exception:  # noqa
        pass
    _FF_DONE = True


# ------------------------------------------------------------------------------------------ fast section download (HLS)
# yt-dlp cuts a section out with ffmpeg, which YouTube serves at about real time (a 10 minute clip takes 10 minutes).
# YouTube's HLS version of the same stream is a list of 5 second files that download at 15x real time or more, so the
# files that cover the wanted stretch are fetched directly and remuxed (no re-encoding).
_HLS_LOCK = threading.Lock()
_HLS_CACHE: dict = {}          # day id -> (time, video playlist, audio playlist)


def _parse_playlist(text):
    """[(duration s, url)] of an HLS media playlist."""
    out, dur = [], None
    for ln in text.splitlines():
        if ln.startswith("#EXTINF:"):
            dur = float(ln[8:].split(",")[0])
        elif ln and not ln.startswith("#") and dur is not None:
            out.append((dur, ln.strip()))
            dur = None
    return out


def _fetch(url, headers, tries=4, timeout=40):
    last = None
    for k in range(tries):
        try:
            return urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=timeout).read()
        except Exception as e:  # noqa
            last = e
            time.sleep(0.5 * (k + 1))
    raise WebcastError(f"download failed: {last}")


def _hls_playlists(day):
    """(video segments, audio segments, http headers) of the day video's HLS streams (cached for a while: the links expire)."""
    yt = _yt()
    with _HLS_LOCK:
        hit = _HLS_CACHE.get(day.id)
        if hit and time.time() - hit[0] < 3000:
            return hit[1], hit[2], hit[3]
        with yt.YoutubeDL({"quiet": True, "no_warnings": True, "skip_download": True, **core.js_runtime_opts()}) as y:
            info = y.extract_info(day.url, download=False)
        fmts = [f for f in info.get("formats", []) if str(f.get("protocol", "")).startswith("m3u8")]
        vids = [f for f in fmts if f.get("vcodec") not in (None, "none") and (f.get("height") or 0) <= MAX_HEIGHT]
        auds = [f for f in fmts if f.get("vcodec") in (None, "none") and f.get("height") is None]
        if not vids or not auds:
            raise WebcastError("this video has no HLS streams to cut from")
        vid = max(vids, key=lambda f: (f.get("height") or 0, "avc1" in (f.get("vcodec") or ""), f.get("tbr") or 0))
        aud = max(auds, key=lambda f: (f.get("format_id") == "234", f.get("tbr") or 0, f.get("format_id", "")))
        hdr = dict(vid.get("http_headers") or {})
        v = _parse_playlist(_fetch(vid["url"], hdr, timeout=90).decode("utf-8", "replace"))
        a = _parse_playlist(_fetch(aud["url"], hdr, timeout=90).decode("utf-8", "replace"))
        _HLS_CACHE[day.id] = (time.time(), v, a, hdr)
        return v, a, hdr


def download_segment_hls(day: DayVideo, a: float, b: float, progress=None, workers=6):
    """Fetch the HLS files that cover [a, b] seconds of the day video and remux them into one mp4.  Returns
    (path, exact start of the clip in the day video, in s)."""
    v, au, hdr = _hls_playlists(day)

    def window(segs):
        t, first, last, start = 0.0, None, None, 0.0
        for i, (d, _u) in enumerate(segs):
            if first is None and t + d > a:
                first, start = i, t
            if t < b:
                last = i
            t += d
        return first, last, start
    f0, f1, start = window(v)
    g0, g1, astart = window(au)
    if f0 is None or g0 is None:
        raise WebcastError(f"{a:.0f}-{b:.0f} s is outside the video")
    stem = f"{day.id}_{int(start)}_{int(start + sum(d for d, _ in v[f0:f1 + 1]))}"
    CLIP_DIR.mkdir(parents=True, exist_ok=True)
    out = CLIP_DIR / (stem + ".mp4")
    if out.exists() and out.stat().st_size > 100_000:
        return str(out), start
    work = CLIP_DIR / (stem + "_parts")
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)
    jobs = [("v", i, v[i][1], f"v{k:05d}.ts") for k, i in enumerate(range(f0, f1 + 1))] + \
           [("a", i, au[i][1], f"a{k:05d}.aac") for k, i in enumerate(range(g0, g1 + 1))]
    done = [0]
    lock = threading.Lock()

    def one(j):
        data = _fetch(j[2], hdr)
        (work / j[3]).write_bytes(data)
        with lock:
            done[0] += 1
            if progress and done[0] % 10 == 0:
                progress(done[0] / len(jobs), "Downloading the match from the day video…")
    try:
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(workers) as ex:
            list(ex.map(one, jobs))
        for kind, segs, i0, i1, ext in (("v", v, f0, f1, "ts"), ("a", au, g0, g1, "aac")):
            lines = ["#EXTM3U", "#EXT-X-VERSION:3", "#EXT-X-TARGETDURATION:6", "#EXT-X-MEDIA-SEQUENCE:0"]
            for k, i in enumerate(range(i0, i1 + 1)):
                lines += [f"#EXTINF:{segs[i][0]:.3f},", f"{kind}{k:05d}.{ext}"]
            (work / f"{kind}.m3u8").write_text("\n".join(lines + ["#EXT-X-ENDLIST"]), encoding="utf-8")
        tmp = CLIP_DIR / (stem + ".tmp.mp4")
        args = [core.ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-y",
                "-allowed_extensions", "ALL", "-i", str(work / "v.m3u8"),
                "-allowed_extensions", "ALL", "-i", str(work / "a.m3u8"),
                "-map", "0:v:0", "-map", "1:a:0", "-c", "copy", "-avoid_negative_ts", "make_zero",
                "-movflags", "+faststart", str(tmp)]
        r = subprocess.run(args, capture_output=True, creationflags=core.NO_WINDOW)
        if r.returncode != 0 or not tmp.exists():
            raise WebcastError("could not join the downloaded pieces: " + r.stderr.decode("utf-8", "replace")[-400:])
        tmp.replace(out)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return str(out), start


def _download_section_ytdlp(day: DayVideo, a: float, b: float, progress=None) -> str:
    """Download only [a, b] seconds of a day video (cached).  Needs ffmpeg (imageio-ffmpeg) for the cut."""
    yt = _yt()
    from yt_dlp.utils import download_range_func
    a, b = max(0.0, a), min(day.duration, b)
    CLIP_DIR.mkdir(parents=True, exist_ok=True)
    stem = f"{day.id}_{int(a)}_{int(b)}"
    for f in CLIP_DIR.glob(stem + ".*"):
        if f.suffix in (".mp4", ".mkv", ".webm") and f.stat().st_size > 100_000:
            return str(f)

    def hook(d):
        if progress and d.get("status") == "downloading":
            tot = d.get("total_bytes") or d.get("total_bytes_estimate") or 0
            if tot:
                progress(d.get("downloaded_bytes", 0) / tot, "Downloading the match from the day video…")
    opts = {
        "format": (f"bv*[height<={MAX_HEIGHT}][ext=mp4]+ba[ext=m4a]/bv*[height<={MAX_HEIGHT}]+ba/b[height<={MAX_HEIGHT}]/b"),
        "outtmpl": str(CLIP_DIR / (stem + ".%(ext)s")),
        "merge_output_format": "mp4",
        "ffmpeg_location": core.ffmpeg_exe(),
        "download_ranges": download_range_func(None, [(a, b)]),
        "force_keyframes_at_cuts": True,
        "quiet": True, "no_warnings": True, "noprogress": True, "progress_hooks": [hook],
        **core.js_runtime_opts(),
    }
    _ffmpeg_on_path()
    try:
        with yt.YoutubeDL(opts) as y:
            y.download([day.url])
    except Exception as e:  # noqa
        raise WebcastError(f"download of {day.url} [{a:.0f}-{b:.0f} s] failed: {e}")
    for f in sorted(CLIP_DIR.glob(stem + ".*"), key=lambda p: -p.stat().st_size):
        if f.suffix in (".mp4", ".mkv", ".webm"):
            return str(f)
    raise WebcastError("the download finished but no video file was found")


def download_segment(day: DayVideo, a: float, b: float, progress=None):
    """Download only [a, b] seconds of a day video (cached).  Returns (path, start of the clip in the day video, s).
    Uses the fast HLS route; if that is not possible, yt-dlp's (slow, real-time) section download."""
    a, b = max(0.0, a), min(day.duration, b)
    try:
        return download_segment_hls(day, a, b, progress)
    except WebcastError as e:
        if "outside the video" in str(e):
            raise
        return _download_section_ytdlp(day, a, b, progress), a


def counters_at(video, t):
    """(red count, blue count) shown by the broadcast's fuel counters at video time t, None where they cannot be read."""
    import scorebug
    f = core.grab_frame(video, t)
    if f is None:
        return None, None
    out = []
    for al in ("Red", "Blue"):
        r = scorebug.read(f, al)
        out.append(None if r is None else r[0])
    return tuple(out)


def check_start(video, off, match_len):
    """Does the broadcast agree that a match starts at video time `off`?  Two checks, by what the overlay allows:
      * the fuel counters read 0 at auto start and have scored by the end (qualification graphics), or
      * the match clock stands still at 0:20 and starts counting there (also the playoff graphics).
    Returns (verdict, why): True / False, or None when neither can be read."""
    before = counters_at(video, off - 1.0)
    if before is not None and any(x is not None for x in before):
        if any(x not in (None, 0, 1, 2) for x in before):
            return False, f"counters read {before} at the start instead of 0"
        after = counters_at(video, off + match_len + 2.0)
        if after is not None and any(x is not None for x in after) and not any((x or 0) >= 10 for x in after):
            return False, f"counters read {after} after the match: nothing was scored"
        return True, f"counters 0 at start, {after} at the end"
    try:
        v = visual_start(video, off, span=5.0)
    except Exception:  # noqa
        v = None
    if v is not None and abs(v - off) <= 5.0:
        return True, "the match clock starts here"
    return None, "neither the fuel counters nor the match clock could be checked here"


def _candidates(path, ml, exp, win, length):
    """The best audio candidates for the auto start inside a clip, each checked against the broadcast picture:
    ([{'off', 'score', 'ok', 'why'}], audio confidence)."""
    lo, hi = max(0.0, exp - win + 15.0), exp + win - 15.0
    off, conf, det = core.auto_sync(path, ml, search=(lo, hi))
    out = []
    for cand, sc in det.get("peaks", [(off, 0.0)])[:8]:
        ok, why = check_start(path, cand, length)
        out.append({"off": cand, "score": sc, "ok": ok, "why": why})
    return out, conf


# Which match is it?  A day video is full of look-alike matches (Q13 and Q14 start 8.8 minutes apart, like every pair of
# neighbours), so a start found by the clock or the sound can be the next match.  In the qualification score bar the team
# numbers are on screen: 2429's must be there.  The template is the white-text mask of "2429" (19x37, from the red slot 1) and
# is looked for anywhere in the bar; the playoff graphics have no team numbers there, so playoff matches cannot be told apart this way.
TEAM_TEXT = "2429"
_TEAM_B64 = (
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAEBAQEAAAAAAAABAQAAAAEBAQEBAAAAAQEBAQAAAAAAAAAAAAAAAAAAAQAAAAAAAQEAAAAAAAAAAQAAAQAA"
    "AAABAAAAAAAAAAAAAAAAAAEAAAAAAQEBAAAAAAAAAAEAAAEAAAAAAQAAAAAAAAAAAAAAAAABAAAAAQEAAQAAAAAAAAABAAAB"
    "AAAAAAEAAAAAAAAAAAAAAAABAAAAAAEAAAEAAAAAAAABAQAAAAEBAAEBAAAAAAAAAAAAAAABAAAAAAEAAAABAAAAAAABAQAA"
    "AAAAAQEBAQAAAAAAAAAAAAABAAAAAAABAQEBAQEAAAABAAAAAAAAAAAAAAEAAAAAAAAAAAABAAAAAAAAAAAAAQEBAAABAAAA"
    "AAAAAAAAAAEAAAAAAAAAAAABAQAAAQAAAAAAAAABAAAAAQAAAAAAAAAAAAEBAAAAAAAAAAAAAAEBAQEBAAAAAAAAAAAAAAEB"
    "AQEBAAAAAQEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=="
)
_team_cache = {}


def team_visible(video, t, team_box=(290, 50, 990, 84)):
    """Score 0..1 of how well the white text '2429' is found in the score bar of the frame at video time t (>= 0.72: it is
    there; the neighbouring matches score about 0.5, the other graphics 0.3).  None if the frame cannot be read."""
    import cv2
    import numpy as np
    if "t" not in _team_cache:
        import base64
        _team_cache["t"] = np.frombuffer(base64.b64decode(_TEAM_B64), np.uint8).reshape(19, 37).astype(np.float32)
    f = core.grab_frame(video, t)
    if f is None:
        return None
    if f.shape[1] != 1280:
        f = cv2.resize(f, (1280, 720))
    x0, y0, x1, y1 = team_box
    band = (f[y0:y1, x0:x1].min(axis=2) > 170).astype(np.float32)
    return float(cv2.matchTemplate(band, _team_cache["t"], cv2.TM_CCOEFF_NORMED).max())


TEAM_MIN = 0.72


def clock_starts(video, ml=None, lo=0.0, hi=1e9, progress=None):
    """Every moment in [lo, hi] of the video at which the broadcast's match clock starts counting (0:20 -> 0:19), found from
    the picture alone, and for a qualification only those with 2429 on the scoreboard.  Fast: only the key frames (one per 5 s)
    are decoded to find where the clock stands at 0:20 before a match; the exact start is then found frame by frame.  Returns
    start times, no particular order."""
    import cv2
    import numpy as np
    t2, t1, o0, o9 = _clock_glyphs()
    x0, y0, x1, y1 = CLOCK_MIN[0], CLOCK_MIN[1], CLOCK_ONES[2], CLOCK_ONES[3]
    w, h = x1 - x0, y1 - y0
    args = [core.ffmpeg_exe(), "-hide_banner", "-nostats", "-skip_frame", "nokey", "-i", str(video), "-vf",
            f"crop={w}:{h}:{x0}:{y0},showinfo", "-vsync", "0", "-f", "rawvideo", "-pix_fmt", "gray", "-"]
    r = subprocess.run(args, capture_output=True, creationflags=core.NO_WINDOW)
    times = [float(m) for m in re.findall(rb"pts_time:([\d.]+)", r.stderr)]
    n = min(len(times), len(r.stdout) // (w * h))
    if n == 0:
        return []
    frames = np.frombuffer(r.stdout[:n * w * h], np.uint8).reshape(n, h, w).astype(np.float32)

    def ncc(a, b):
        d = float(np.sqrt((a * a).sum() * (b * b).sum()))
        return float((a * b).sum() / d) if d > 1e-6 else 0.0
    runs, cur = [], []
    for i in range(n):
        if not (lo <= times[i] <= hi):
            continue
        ink = np.clip((200.0 - frames[i]) / 150.0, 0, 1)
        gm = cv2.resize(ink[:, :CLOCK_MIN[2] - CLOCK_MIN[0]], (18, 30), interpolation=cv2.INTER_AREA)
        gt = cv2.resize(ink[:, CLOCK_TENS[0] - x0:CLOCK_TENS[2] - x0], (18, 30), interpolation=cv2.INTER_AREA)
        go = cv2.resize(ink[:, CLOCK_ONES[0] - x0:], (18, 30), interpolation=cv2.INTER_AREA)
        if ncc(gm, o0) > 0.75 and ncc(gt, t2) > 0.8 and ncc(go, o0) > 0.8:
            if cur and times[i] - cur[-1] > 8.0:
                runs.append(cur)
                cur = []
            cur.append(times[i])
        elif cur and times[i] - cur[-1] > 8.0:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    out = []
    for run in runs:
        if run[-1] - run[0] < 4.0:        # at least two key frames at 0:20: before a match it stands still for a long time, while
            continue                      # near the end of a match 0:20 lasts one second
        v = visual_start(video, run[-1] + 3.0, span=8.0)         # the match starts within one key-frame interval of the last 0:20
        if v is None:
            continue
        if ml is not None and ml.match_type == 2:                # a qualification: 2429 must be on the scoreboard
            sc = team_visible(video, v + 4.0)
            if sc is not None and sc < TEAM_MIN:
                continue
        if all(abs(v - x) > 2.0 for x in out):
            out.append(v)
    return out


def _visual_starts_near(path, cands, exp, span=25.0, tries=6, ml=None):
    """The moments the broadcast's match clock really starts, found near the audio candidates (nearest to where the match is
    expected first).  The fuel counters read 0 for the whole pre-match break, so a candidate they accept can be 20 s early;
    the clock starting is exact.  Returns the start times found, nearest to `exp` first."""
    found, covered = [], []
    for c in sorted((c for c in cands if c["ok"] is not False), key=lambda c: abs(c["off"] - exp)):
        if any(abs(c["off"] - x) <= span - 3 for x in covered):
            continue
        covered.append(c["off"])
        try:
            v = visual_start(path, c["off"], span=span)
        except Exception:  # noqa
            v = None
        if v is not None and ml is not None and ml.match_type == 2:          # a qualification: 2429 must be on the scoreboard
            sc = team_visible(path, v + 4.0)
            if sc is not None and sc < TEAM_MIN:
                v = None                                                    # that is a neighbouring match
        if v is not None and all(abs(v - x) > 2.0 for x in found):
            found.append(v)
        if len(covered) >= tries:
            break
    return sorted(found, key=lambda v: abs(v - exp))


def _wide_clip(day, est0, length, window=WINDOW_S, progress=None):
    a, b = est0 - window, est0 + length + window
    path, seg_a = download_segment(day, a, b, progress)
    return path, seg_a


def calibrate(day, items, cal, progress=None, n=5):
    """Find the clock error of a whole day from a few matches (the same for every match of the day: the roboRIO clock and the
    stream's start time).  Each match is searched +-WINDOW_S; the error that most matches agree on (within 20 s) wins, which
    also rules out the look-alike neighbouring matches that fill a day video."""
    mine = [(p, ml) for p, ml in items if locate(p, ml, [day])[0] is not None]
    if not mine:
        return None
    step = max(1, len(mine) // n)
    pick = mine[::step][:n]
    per = []
    for k, (path, ml) in enumerate(pick):
        _d, est0 = locate(path, ml, [day])
        length = ml.match_len or 160.0
        try:
            clip, seg_a = _wide_clip(day, est0, length)
            cands, _conf = _candidates(clip, ml, est0 - seg_a, WINDOW_S, length)
        except WebcastError:
            continue
        starts = sorted(clock_starts(clip, ml), key=lambda v: abs(v - (est0 - seg_a)))
        if not starts:
            starts = _visual_starts_near(clip, cands, est0 - seg_a, tries=6, ml=ml)
        ds = [(seg_a + v) - est0 for v in starts] or [(seg_a + c["off"]) - est0 for c in cands if c["ok"] is True]
        try:                                   # the wide clip (~250 MB) was only needed to find the clock error
            os.remove(clip)
        except OSError:
            pass
        if ds:
            per.append(ds)
        if progress:
            progress((k + 1) / len(pick), f"working out the day video's clock error ({k + 1}/{len(pick)})")
    if not per:
        return None
    best = None
    for d in sorted({x for ds in per for x in ds}):
        members = [min(ds, key=lambda x: abs(x - d)) for ds in per if any(abs(x - d) <= 20.0 for x in ds)]
        key = (len(members), -abs(float(sorted(members)[len(members) // 2])))        # most agree; then the smallest clock error
        if best is None or key > best[0]:
            best = (key, members)
    members = best[1]
    if len(per) > 1 and len(members) < 2:               # nothing two matches agree on: take the smallest error of the first
        members = [min(per[0], key=abs)]
    cal[day.id] = [float(x) for x in members]
    return float(sorted(members)[len(members) // 2])


def clip_match(log_path, ml, days, progress=None, cal=None, window=None) -> Clip:
    """Find this match in the whole-day videos, download that stretch and line it up with the log.  Raises WebcastError
    with the reason when it cannot (no video covers it, ...).

    `cal` ({day id: [clock errors]}, see calibrate) lets the download be short: with the day's clock error known the match is
    searched +-NARROW_S; without it +-WINDOW_S, and the match found is then used as the calibration."""
    day, est0 = locate(log_path, ml, days)
    if day is None:
        raise WebcastError(est0)
    cal = cal if cal is not None else {}
    known = cal.get(day.id) or []
    delta = float(sorted(known)[len(known) // 2]) if known else 0.0
    win = window or (NARROW_S if known else WINDOW_S)
    length = ml.match_len or 160.0
    for attempt in (0, 1):
        est = est0 + delta
        if progress:
            progress(0.05, f"Match is about {est / 3600:.2f} h into the day video (+-{win:.0f} s)…")
        path, seg_a = download_segment(day, est - win, est + length + win, lambda f, m: progress and progress(0.05 + 0.6 * f, m))
        if progress:
            progress(0.7, "Finding the start in the sound and the picture…")
        exp = est - seg_a
        cands, conf = _candidates(path, ml, exp, win, length)
        verified = [c for c in cands if c["ok"] is True]
        chosen, note = None, ""
        starts = sorted(clock_starts(path, ml), key=lambda v: abs(v - exp))
        if not starts:
            starts = _visual_starts_near(path, cands, exp, ml=ml)
        if starts:                                # the match clock itself: exact, and the picture is what is shown
            v = starts[0]
            near = min(cands, key=lambda c: abs(c["off"] - v)) if cands else None
            chosen = {"off": v, "why": "the match clock starts here"}
            note = f" ({chosen['why']})"
            if near is not None and abs(near["off"] - v) <= 8.0:
                note += f" (sound is {near['off'] - v:+.1f} s from the picture; synced to the picture)"
        elif verified:                            # no clock found: the verified audio candidate nearest to where it should be
            chosen = min(verified, key=lambda c: abs(c["off"] - exp))
            note = f" ({chosen['why']}; match clock not found, sound only)"
        elif cands and cands[0]["ok"] is None and conf > 6:
            chosen, note = cands[0], " (sound only: the overlay could not be checked)"
        if chosen is not None:
            off = chosen["off"]
            found_delta = (seg_a + off) - est0
            if known and abs(found_delta - delta) > 45.0 and attempt == 0 and win < WINDOW_S:
                win = WINDOW_S                    # far from this day's usual clock error: look at the neighbours too, then decide
                continue
            if not known:
                note += " (day not calibrated: neighbouring matches look alike, check that the scoreboard shows this match)"
            return Clip(path, off, max(conf, 6.5), True, day, est0, seg_a + off,
                        note=f"{(seg_a + off - est0):+.0f} s from the wall-clock estimate" + note)
        if attempt == 0 and known and win < WINDOW_S:          # the short window missed it: look wider once
            win = WINDOW_S                         # keep the day's clock error, only look further around it
            continue
        best = cands[0]["off"] if cands else 0.0
        return Clip(path, best, conf, False, day, est0, seg_a + best,
                    note=f"{(seg_a + best - est0):+.0f} s from the wall-clock estimate (nothing confirmed this start)")
    raise WebcastError("unreachable")


# The match clock (1280x720 frame): its tens and ones digit boxes, and 4 glyphs read from a broadcast frame: tens '2' and '1',
# ones '0' and '9' (18x30, dark ink = 1).  Only 0:20 and 0:19 matter: the clock stands at 0:20 before auto starts and shows
# 0:19 exactly one second after it does.
CLOCK_MIN, CLOCK_TENS, CLOCK_ONES = (608, 50, 628, 80), (634, 50, 652, 80), (652, 50, 672, 80)
_CLOCK_B64 = (
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAADBedF48AAAAAAAA"
    "AAAAACml8P//////54UAAAAAAAAAJf////////////+tAAAAAAAAPP///9rJ1P//////eQAAAAAAPfCADgAAACDU////5AAA"
    "AAAAGCkAAAAAAAA4////9gUAAAAAAAAAAAAAAAAH9v///xgAAAAAAAAAAAAAAAAM/////woAAAAAAAAAAAAAAABj////ygAA"
    "AAAAAAAAAAAAACru////SwAAAAAAAAAAAAAAKuz///+vAAAAAAAAAAAAAABV/P///8QAAAAAAAAAAAAAAJf/////rwAAAAAA"
    "AAAAAAAFwv////9oAAAAAAAAAAAAAADO////2iwAAAAAAAAAAAAAALb///++AAAAAAAAAAAAAAAAXv///7QAAAAAAAAAAAAA"
    "AAAA1v///34AAAAAAAAAAAAAAAAY///////w8fHz8/P64BgAAAA65v///////////////x8AAABE/////////////////yIA"
    "AAACdHJ6enp6enp6enp1aQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAADpyawAAAAAAAAAAAAAAAAA2qP///yUAAAAAAAAAAAAATq///////ykAAAAA"
    "AAAAAFDn/////////ykAAAAAAAAAAFz//////////ykAAAAAAAAAAFPw/7SD/P///yQAAAAAAAAAADOANgAA2////yAAAAAA"
    "AAAAAAAAAAAA2P///x0AAAAAAAAAAAAAAAAA2P///xgAAAAAAAAAAAAAAAAA2P///xQAAAAAAAAAAAAAAAAA2P///xQAAAAA"
    "AAAAAAAAAAAA2P///xQAAAAAAAAAAAAAAAAA2P///xQAAAAAAAAAAAAAAAAA2P///xQAAAAAAAAAAAAAAAAA2P///xQAAAAA"
    "AAAAAAAAAAAA2P///xQAAAAAAAAAAAAAAAAA2P///xQAAAAAAAAAAAAAAAAA2P///xQAAAAAAAAAAAAAAAAA2P///xQAAAAA"
    "AAAAAAAAAAAA2v///xgAAAAAAAAAAAAAAAAA7v///zYAAAAAAAAAAAAAAAAATm10XwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAEVJyUhMAAAAAAAAA"
    "AAAAAAee/P////iZAAAAAAAAAAAAAL7/////////sgAAAAAAAAAAnv///8qc8P///3QAAAAAAAA6+P//ogAAKun//8oAAAAA"
    "AACD///uFgAAAIH///w8AAAAAADA//+0AAAAADj///95AAAAABPx//9+AAAAAAzm//+cAAAAACf///91AAAAAADW//+oAAAA"
    "AET///9kAAAAAADF//+2AAAAAEz///9aAAAAAADF//+4AAAAAEv///9cAAAAAADC//+yAAAAAEb///9fAAAAAADK//+2AAAA"
    "ADr///9yAAAAAADR//+lAAAAABv2//+DAAAAAAfm//+FAAAAAADW//+xAAAAADr///9LAAAAAACs///mEwAAAIr//+sfAAAA"
    "AABj////kgAAOPj//6oAAAAAAAAAzv///9Ox/f//+EQAAAAAAAAAPe7/////////fAAAAAAAAAAAAD3R/////9teAAAAAAAA"
    "AAAAAAAALGlrOgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAIFBmQQAAAAAAAAAAAAAAADrA/////89hAAAAAAAAAAAATvz////////6dAAAAAAA"
    "AAA69v///8mb9f//6zAAAAAAAACI///8ZAAAQeb//48AAAAAAAfn//+bAAAAAIP//+IYAAAAABPz//9kAAAAADb///w2AAAA"
    "ABr9//9fAAAAACT///9tAAAAAB////+AAAAAADb///93AAAAAADK///MMwAAAJn///+DAAAAAACF////yXdwp/////+AAAAA"
    "AAAPxP///////9/r//+AAAAAAAAAMK//////g1r1//9oAAAAAAAAAABBXnIxAB/6//hTAAAAAAAAAAAAAAAAAEv//+caAAAA"
    "AAAAAAAAAAAAAI3//8IAAAAAAAAAAAAAAAAALPb//3AAAAAAAAAAGwAAAAAn1v//1A4AAAAAAAAkydagl8X////1SwAAAAAA"
    "AAAn//////////xcAAAAAAAAAAAHsv//////uSoAAAAAAAAAAAAAAB9Yem0pAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
)
_clock_cache = {}


def _clock_glyphs():
    if "g" not in _clock_cache:
        import base64
        import numpy as np
        a = np.frombuffer(base64.b64decode(_CLOCK_B64), np.uint8).reshape(4, 30, 18).astype(np.float32) / 255.0
        _clock_cache["g"] = a
    return _clock_cache["g"]


def visual_start(video, approx, span=10.0):
    """Video time at which the broadcast's match clock starts counting, found from the picture: it stands still at 0:20 before
    auto starts and shows 0:19 exactly 1 s after it does.  `approx` (from the audio) only says where to look: the window is
    approx +-span.  Returns None when the clock does not read 0:20 and then 0:19 there (not on screen, already running).  The
    picture is what the dashboard sits next to, so this is the sync that matters even if the stream's sound is off the picture."""
    import cv2
    import numpy as np
    t2, t1, o0, o9 = _clock_glyphs()
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        return None
    fps = cap.get(cv2.CAP_PROP_FPS) or 60.0
    sx, sy = cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 1280.0, cap.get(cv2.CAP_PROP_FRAME_HEIGHT) / 720.0
    sc = lambda box: (int(box[0] * sx), int(box[1] * sy), int(box[2] * sx), int(box[3] * sy))
    bt, bo, bm = sc(CLOCK_TENS), sc(CLOCK_ONES), sc(CLOCK_MIN)

    def glyph(f, box):
        g = cv2.cvtColor(f[box[1]:box[3], box[0]:box[2]], cv2.COLOR_BGR2GRAY)
        return np.clip((200.0 - cv2.resize(g, (18, 30), interpolation=cv2.INTER_AREA).astype(np.float32)) / 150.0, 0, 1)

    def ncc(a, b):
        d = float(np.sqrt((a * a).sum() * (b * b).sum()))
        return float((a * b).sum() / d) if d > 1e-6 else 0.0
    cap.set(cv2.CAP_PROP_POS_MSEC, max(0.0, approx - span) * 1000.0)
    first20, result = None, None
    for _ in range(int((2 * span + 4) * fps)):
        ok, f = cap.read()
        if not ok:
            break
        t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        gt, go, gm = glyph(f, bt), glyph(f, bo), glyph(f, bm)
        zero_min = ncc(gm, o0) > 0.75                                # minutes 0: it is 0:20 / 0:19, not 2:20 / 2:19 (teleop)
        is20 = zero_min and ncc(gt, t2) > 0.8 and ncc(go, o0) > 0.8
        is19 = zero_min and ncc(gt, t1) > 0.8 and ncc(go, o9) > 0.8
        if is20 and first20 is None:
            first20 = t
        if is19:
            if first20 is not None and t - first20 >= 3.0:          # it stood at 0:20 for a while: not the 1 s of 0:20 near the end
                result = t - 1.0
            break
    cap.release()
    return result


def clip_matches(items, days, cal=None, progress=None, workers=3, cancel=None, calib_items=None):
    """Clip many matches: [(log_path, MatchLog)] -> {log_path: Clip or WebcastError}.  Each day video is first calibrated from a
    few of its matches (wide search, majority vote), then every match is cut out with a short window, three at a time."""
    from concurrent.futures import ThreadPoolExecutor
    cal = cal if cal is not None else {}
    out = {}
    done = [0]
    for d in days:
        if cancel and cancel():
            break
        if not cal.get(d.id):
            if progress:
                progress(0.0, f"calibrating {d.title}…")
            calibrate(d, calib_items or items, cal, lambda f, m: progress and progress(f * 0.3, m))

    def one(it):
        path, ml = it
        if cancel and cancel():
            return path, WebcastError("cancelled")
        try:
            r = clip_match(path, ml, days, cal=cal)
        except WebcastError as e:
            r = e
        except Exception as e:  # noqa
            r = WebcastError(str(e))
        done[0] += 1
        if progress:
            progress(0.3 + 0.7 * done[0] / max(1, len(items)), f"clipped {done[0]}/{len(items)} matches from the day videos")
        return path, r

    with ThreadPoolExecutor(workers) as ex:
        for k, r in ex.map(one, list(items)):
            out[k] = r
    return out


def sync_video(video, ml, progress=None):
    """core.auto_sync (field sounds), corrected by the picture: when the broadcast's match clock can be read, the video time at
    which it starts counting is the sync, whatever the sound says.  The sound alone can lock on the wrong crowd / field noise
    (DALY Q13 was 35 s off).  Returns (offset, confidence, details) like core.auto_sync; details['picture'] says which was used."""
    off, conf, det = core.auto_sync(video, ml, progress=progress)
    try:
        starts = clock_starts(video, ml)
    except Exception:  # noqa - no ffmpeg / unreadable overlay: keep the sound's answer
        starts = []
    if not starts:
        return off, conf, dict(det, picture=False)
    cands = [float(p[0]) for p in det.get("peaks", [])][:8] or [float(off)]
    v = min(starts, key=lambda x: min(abs(x - c) for c in cands))
    return v, max(float(conf), 6.5), dict(det, picture=True, sound_off=float(off), ratio=max(det.get("ratio", 0), 1.2))

