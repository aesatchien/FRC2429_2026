"""
What the broadcast video tells us about scoring, measured in one pass over the match:

  * ctr      the alliance's fuel counter ("71 / 240" in the corner box, see scorebug.py): the official count of the
             fuel the whole alliance has scored so far, and the only thing the points estimate is built on.

This module used to also count yellow fuel flying over the hub in the field camera and the hub close-up camera.  That
depended on where the cameras were pointed, which changes from one event to the next, and it confused the estimate (2026dal
Q49), so it was removed: nothing here looks at the picture except to read the counter.
"""
from __future__ import annotations

import cv2
import numpy as np

import scorebug

VERSION = 3
OCR_EVERY = 2                                        # read the counter on every n-th frame (the others are only skipped)


def measure(video, offset, alliance="Red", t_from=-3.0, t_to=168.0, cache_path=None, progress=None, compute=True,
            left_is_red=None):
    """{'t': match-relative seconds, 'ctr': the alliance fuel counter (-1 where unreadable)}.
    Which corner holds the alliance's counter depends on the broadcast layout (red left at SoCal Showdown, blue left on
    the Daly Division); left_is_red=None reads it from the video's score bar and assumes red-left if that fails.
    Cached on disk (keyed by offset, window and layout).  With compute=False a cache that does not match this offset gives
    None instead of being recomputed (used by GUI previews)."""
    if left_is_red is None:
        left_is_red = scorebug.detect_red_left(video)          # four frame reads, so cheap enough for a preview too
    layout = 1.0 if left_is_red is None else float(bool(left_is_red))        # unknown is stored as red-left, like the reader
    key = np.array([VERSION, offset, t_from, t_to, layout], float)
    if cache_path is not None and cache_path.exists():
        try:
            d = np.load(cache_path)
            if d["key"].shape == key.shape and np.allclose(d["key"], key):
                return {k: d[k] for k in ("t", "ctr")}
        except Exception:
            pass
    if not compute:
        return None
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        return None
    fps = cap.get(cv2.CAP_PROP_FPS) or 60.0
    cap.set(cv2.CAP_PROP_POS_MSEC, (offset + t_from) * 1000.0)
    n = int((t_to - t_from) * fps)
    ts, ctr = [], []
    last = -1
    for i in range(n):
        if i % OCR_EVERY == 0:
            ok, f = cap.read()
            if not ok:
                break
            r = scorebug.read(f, alliance, left_is_red is not False)
            last = -1 if r is None else r[0]
        elif not cap.grab():                          # skipped frames are not decoded to an image, which is faster
            break
        ts.append(t_from + i / fps)
        ctr.append(last)
        if progress and i % 300 == 0:
            progress(i / n, "Reading the match video (fuel counter)…")
    cap.release()
    out = {"t": np.array(ts), "ctr": np.array(ctr)}
    if cache_path is not None:
        try:
            cache_path.parent.mkdir(parents=True, exist_ok=True)
            np.savez(cache_path, key=key, **out)
        except Exception:
            pass
    return out
