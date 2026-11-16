"""
Finding the start of a match in its video from the broadcast overlay.

When the match starts the overlay lights two yellow arrows at the top, one beside each alliance's score box: both hubs are active.  They are
a crisp on / off signal, so the first video frame where both are yellow is the start of the match to within a frame.  On the 14 SoCal
Showdown matches the desktop app had synced, they lit within 0 to 0.05 s of its offset (a frame is 1/60 s); the sound alone is far less exact.

The overlay is laid out a little differently at each event, so no fixed position is assumed.  A frame counts as "arrows lit" when the top
strip of the picture holds two small yellow shapes of about the same size, level with each other and placed symmetrically either side of the
centre.  The sound still gives a rough place to look (a few seconds either way); this then reads the video frames there.  Without a rough
place the whole video is scanned at a lower frame rate and the lit arrows are then read frame by frame.
"""
from __future__ import annotations

import cv2
import numpy as np

TOP = 0.20                    # the arrows are in the top fifth of the picture
MIN_AREA, MAX_AREA = 0.00015, 0.004      # a lit arrow, as a share of the whole frame (about 0.0007 on a 1280x720 broadcast)
MIN_LIT_S = 5.0               # the arrows stay lit through the 20 s of autonomous; a flash of yellow shorter than this is something else
LATENCY_S = 0.025             # the arrows light this long after the offset the desktop app's clock reading gave (mean over 14 matches)


def arrows_lit(frame) -> bool:
    """True if the top strip of this frame shows the pair of lit (yellow) arrows."""
    h, w = frame.shape[:2]
    band = frame[:max(8, int(TOP * h))]
    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 0] >= 18) & (hsv[..., 0] <= 36) & (hsv[..., 1] > 120) & (hsv[..., 2] > 170)).astype(np.uint8)
    if mask.sum() < MIN_AREA * w * h:
        return False
    n, _lab, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = []
    for i in range(1, n):
        bw, bh, area = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT], stats[i, cv2.CC_STAT_AREA]
        if MIN_AREA * w * h <= area <= MAX_AREA * w * h and 0.4 <= bw / max(bh, 1) <= 3.0:
            blobs.append((cent[i][0] / w, cent[i][1] / h, area))
    left = [b for b in blobs if b[0] < 0.5]
    right = [b for b in blobs if b[0] > 0.5]
    for lx, ly, la in left:
        for rx, ry, ra in right:
            if abs(ly - ry) < 0.025 and abs((lx + rx) / 2 - 0.5) < 0.07 and 0.5 <= la / ra <= 2.0:
                return True
    return False


def scan(video, t0: float, t1: float, step: int = 1):
    """(fps, [(time, lit)]) for every `step`-th frame between t0 and t1 seconds."""
    cap = cv2.VideoCapture(str(video))
    try:
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        first = max(0, int(t0 * fps))
        cap.set(cv2.CAP_PROP_POS_FRAMES, first)
        i, rows = first, []
        while i / fps <= t1:
            if (i - first) % step == 0:
                ok, frame = cap.read()
                if not ok:
                    break
                rows.append((i / fps, arrows_lit(frame)))
            elif not cap.grab():
                break
            i += 1
        return fps, rows
    finally:
        cap.release()


def lit_segments(rows, gap_s: float = 0.5):
    """[(first lit time, last lit time)], bridging dropouts shorter than gap_s."""
    segs = []
    for t, lit in rows:
        if lit:
            if segs and t - segs[-1][1] <= gap_s:
                segs[-1][1] = t
            else:
                segs.append([t, t])
    return [(a, b) for a, b in segs]


def find_start(video, estimate: float | None = None, window: float = 10.0):
    """The video time at which the arrows light for the start of the match, or None if they are never seen.
    `estimate` (seconds into the video, e.g. from the sound) narrows the search to estimate +/- window.  Returns
    {"time": first lit frame, "lit_s": how long they stayed lit, "coarse": True when the whole video had to be scanned}."""
    coarse = False
    segs = []
    if estimate is not None:
        lo = max(0.0, estimate - window)
        _fps, rows = scan(video, lo, estimate + window + 25.0)       # the 25 s after it show that they stay lit
        end = rows[-1][0] if rows else 0.0
        segs = [s for s in lit_segments(rows) if s[0] <= estimate + window and (s[1] - s[0] >= MIN_LIT_S or s[1] >= end - 0.5)]
        first = rows[0][0] if rows else 0.0
        # arrows already lit at the first frame looked at began earlier (the pair is lit again for the transition and the endgame): the
        # "start" of such a stretch is only where the scan began, so it is not an answer; the whole video is read instead
        segs = [s for s in segs if not (first > 0.5 and s[0] <= first + 0.5)]
    if not segs:
        coarse = True
        cap = cv2.VideoCapture(str(video))
        duration = (cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0) / (cap.get(cv2.CAP_PROP_FPS) or 30.0)
        cap.release()
        _fps, rows = scan(video, 0.0, duration or 1e9, step=6)          # ~10 frames a second over the whole video
        segs = [s for s in lit_segments(rows, gap_s=1.0) if s[1] - s[0] >= MIN_LIT_S]
        if segs:
            pick = min(segs, key=lambda s: abs(s[0] - estimate)) if estimate is not None else segs[0]
            _fps, rows = scan(video, max(0.0, pick[0] - 1.0), pick[0] + 1.0)              # then every frame around it
            fine = lit_segments(rows)
            segs = [((fine[0][0] if fine else pick[0]), pick[1])]
    if not segs:
        return None
    pick = min(segs, key=lambda s: abs(s[0] - estimate)) if estimate is not None else segs[0]
    return {"time": float(pick[0]), "lit_s": float(pick[1] - pick[0]), "coarse": coarse}
