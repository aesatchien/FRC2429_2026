"""
The bridge to Scope2429: this module imports the desktop app's own log reader and dashboard renderer (gui/scope2429/helpers), so the
website draws exactly what the desktop app draws.  Nothing from Tk is imported.

Building a dashboard renderer for a log takes a few seconds (it decodes the whole log), drawing a frame afterwards takes well under a
second, so renderers are kept in a small least-recently-used cache.  One renderer is not safe to use from two threads at once, so each
cache entry has a lock.
"""
from __future__ import annotations

import os
import re
import sys
import threading
from collections import OrderedDict
from pathlib import Path


def scope_dir() -> Path:
    """The folder holding scope2429.py (and its helpers/): SCOPE2429_DIR, else the sibling gui/scope2429."""
    env = os.environ.get("SCOPE2429_DIR")
    return Path(env) if env else Path(__file__).resolve().parents[2] / "scope2429"


for _p in (scope_dir() / "helpers", scope_dir()):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import cv2  # noqa: E402
import core  # noqa: E402  (Scope2429's log reader / renderer)
import dash  # noqa: E402
import report as report_mod  # noqa: E402

DRIVE, QUEST = core.DEFAULT_DRIVE, core.DEFAULT_QUEST
T_PAD = 5.0                                            # seconds shown before time zero and after the end, as in the desktop app


def default_title(ml, path) -> str:
    if ml.match_type and ml.match_number:
        return f"2429 · {core.event_title(ml.event_code)} {ml.match_label} · {ml.alliance_text}"
    return Path(path).stem


def log_date(path) -> str:
    """YYYY-MM-DD from a WPILIB_YYYYMMDD_HHMMSS_... file name (UTC), else ''."""
    m = re.search(r"(\d{4})(\d{2})(\d{2})_\d{6}", Path(path).name)
    return f"{m[1]}-{m[2]}-{m[3]}" if m else ""


def load_field(config_path: str = ""):
    return core.load_field(config_path or str(core.BUNDLED_FIELDS / "Field2d_2026_WPILib" / "config.json"))


def summarize(path) -> dict:
    """What the list page shows for a log.  Reads the whole log once."""
    ml = core.load_match_log(str(path), [DRIVE, QUEST])
    color, station = ml.alliance
    return {
        "title": default_title(ml, path),
        "match_label": ml.match_label if ml.match_type and ml.match_number else "",
        "event": core.event_title(ml.event_code) if ml.event_code else "",
        "alliance": f"{color} {station}" if station else str(color),
        "color": color,
        "has_match": bool(ml.has_match),
        "match_len": round(float(ml.match_len), 1),
        "t_min": -T_PAD,
        "t_max": round(float(ml.match_len) + T_PAD, 1),
        "log_date": log_date(path),
    }


class Entry:
    def __init__(self, renderer, ml) -> None:
        self.renderer, self.ml = renderer, ml
        self.lock = threading.Lock()


class Replay:
    def __init__(self, field_config: str = "", max_entries: int = 3) -> None:
        self.field_config, self.max_entries = field_config, max_entries
        self._field = None
        self._entries: "OrderedDict[tuple, Entry]" = OrderedDict()
        self._build_lock = threading.Lock()

    def entry(self, log_id: str, path, title: str, red_left: bool = True) -> Entry:
        key = (log_id, bool(red_left))
        with self._build_lock:                                   # two requests for a new log build it once, not twice
            e = self._entries.get(key)
            if e is None:
                if self._field is None:
                    self._field = load_field(self.field_config)
                ml = core.load_match_log(str(path), [DRIVE, QUEST])
                renderer = dash.DashRenderer(ml, self._field, title, bool(red_left), pose_names={"drive": DRIVE, "quest": QUEST},
                                             flux=None, trends=[], hoot=None)
                e = self._entries[key] = Entry(renderer, ml)
                while len(self._entries) > self.max_entries:
                    self._entries.popitem(last=False)
            self._entries.move_to_end(key)
        e.renderer.title = title                                 # editable without rebuilding
        return e

    def drop(self, log_id: str) -> None:
        with self._build_lock:
            for key in [k for k in self._entries if k[0] == log_id]:
                del self._entries[key]

    @staticmethod
    def frame_jpeg(entry: Entry, t_rel: float, width: int = 1280, quality: int = 80) -> bytes:
        """The dashboard at `t_rel` seconds after time zero (auto start, or the first enable in a log with no match)."""
        with entry.lock:
            img = entry.renderer.render(entry.ml.t_ref + t_rel, None)
        if width and width < img.shape[1]:
            img = cv2.resize(img, (width, int(round(img.shape[0] * width / img.shape[1]))), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if not ok:
            raise RuntimeError("could not encode the frame")
        return buf.tobytes()

    @staticmethod
    def report_html(entry: Entry) -> str:
        with entry.lock:
            return report_mod.build_report(entry.renderer.tel, entry.ml).to_html()
