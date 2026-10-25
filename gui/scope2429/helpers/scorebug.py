"""
Reads the broadcast's fuel counters ("71 / 240" in the red and blue boxes at the top corners of the
frame): the number of fuel the whole alliance has scored so far.  This is the ground truth that the
points estimate is built on (see telemetry.VideoShotModel).  Which corner belongs to which alliance follows the broadcast layout (see red_left()).

The text is white on the alliance color, always the same font, so the digits are found as connected
components and recognised by nearest-neighbour against one template per character embedded below (built from
labelled frames; see make_templates).  Layout of the text: "<count> / <tier>", where the tier is
240, 360, ... the last three glyphs are the tier, the one before is the slash.
"""
from __future__ import annotations

import base64

import cv2
import numpy as np

# counter text boxes in the 1280x720 frame (x0, y0, x1, y1), without the alliance icon.  The broadcast puts the alliance
# that is on the LEFT of the field camera on the left, and that is red at SoCal Showdown but blue on the Daly Division
# (Champs), so which box belongs to an alliance depends on the layout: see red_left().
BOX_LEFT, BOX_RIGHT = (55, 44, 152, 76), (1157, 44, 1256, 76)
# The score bar at the top centre has the same order: its left score box is the alliance on the left.  Used to find the layout.
BAR_LEFT, BAR_RIGHT = (520, 600, 45, 85), (680, 760, 45, 85)     # x0, x1, y0, y1
BAR_MARGIN = 100         # |left (R-B) - right (R-B)| needed to call it: measured +-255..310 with a bar, ~0 without
GLYPH = (14, 18)                       # w, h of the canvas every glyph is drawn on
UP = 3                                 # the crop is enlarged this much before thresholding
WHITE_MIN = 175                        # min(B,G,R) of a text pixel
# One 14x18 glyph image per character (uint8, row-major), the mean of ~90 samples read from SoCal Showdown 2026
# broadcasts.  Regenerate with make_templates() if the broadcast graphics change.
LABELS = "0123456789/"
_TEMPL_B64 = (
    "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"
)
_cache: dict = {}


def _templates():
    if "t" not in _cache:
        g = np.frombuffer(base64.b64decode(_TEMPL_B64), np.uint8).reshape(len(LABELS), GLYPH[1], GLYPH[0])
        _cache["t"] = (g.astype(np.float32) / 255.0, np.array(list(LABELS)))
    return _cache["t"]


def make_templates(samples):
    """samples: [(BGR frame, alliance, "count/tier")] with the counter text read by eye, e.g. (frame, "Red", "26/240").
    Returns the _TEMPL_B64 text to paste into this file (the mean glyph of each character)."""
    acc = {k: [] for k in LABELS}
    for frame, alliance, text in samples:
        want = list(text.replace("/", " / ").replace(" ", ""))
        gl = glyphs(frame, alliance)
        if len(gl) == len(want):
            for k, g in zip(want, gl):
                acc[k].append(g)
    u8 = np.array([np.round(np.mean(acc[k], axis=0) * 255) for k in LABELS], np.uint8)
    return base64.b64encode(u8.tobytes()).decode()


def bar_margin(frame) -> float:
    """How much redder the left score box of the score bar is than the right one (positive: red is on the left)."""
    sx, sy = frame.shape[1] / 1280.0, frame.shape[0] / 720.0

    def red_minus_blue(box):
        x0, x1, y0, y1 = box
        p = frame[int(y0 * sy):int(y1 * sy), int(x0 * sx):int(x1 * sx)].reshape(-1, 3).astype(np.float32)
        return float(p[:, 2].mean() - p[:, 0].mean()) if len(p) else 0.0           # BGR: R - B
    return red_minus_blue(BAR_LEFT) - red_minus_blue(BAR_RIGHT)


def red_left(frame):
    """True / False: red / blue is the alliance on the left of this broadcast frame, None if the score bar is not showing."""
    m = bar_margin(frame)
    return None if abs(m) < BAR_MARGIN else m > 0


def detect_red_left(video, fractions=(0.15, 0.35, 0.55, 0.75)):
    """The same for a whole video: samples a few frames and needs 2 usable ones that all agree, else None
    (a whole-day stream, a non-FIRST video, a video with no score bar)."""
    cap = cv2.VideoCapture(str(video))
    try:
        n = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        if not n or n < 2:
            return None
        votes = []
        for fr in fractions:
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(n * fr))
            ok, f = cap.read()
            if ok:
                v = red_left(f)
                if v is not None:
                    votes.append(v)
        return votes[0] if len(votes) >= 2 and all(v == votes[0] for v in votes) else None
    finally:
        cap.release()


def box_for(alliance, left_is_red):
    """The counter box of `alliance` ("Red" / "Blue") given which alliance is on the left (None: assume red, SoCal's layout)."""
    on_left = (alliance == "Red") == (True if left_is_red is None else bool(left_is_red))
    return BOX_LEFT if on_left else BOX_RIGHT


def glyphs(frame, alliance="Red", left_is_red=None):
    """The glyph images (GLYPH-sized float arrays, left to right) of the counter text, or [].
    left_is_red: True / False if the layout is known, None to read it from this frame's score bar."""
    if left_is_red is None:
        left_is_red = red_left(frame)
    sx, sy = frame.shape[1] / 1280.0, frame.shape[0] / 720.0
    x0, y0, x1, y1 = box_for(alliance, left_is_red)
    crop = frame[int(y0 * sy):int(y1 * sy), int(x0 * sx):int(x1 * sx)]
    if crop.size == 0:
        return []
    big = cv2.resize(crop, None, fx=UP / sx, fy=UP / sy, interpolation=cv2.INTER_CUBIC)
    soft = np.clip((big.min(axis=2).astype(np.float32) - 60.0) / (255.0 - 60.0), 0, 1)     # how white each pixel is
    mask = (big.min(axis=2) > WHITE_MIN).astype(np.uint8)
    n, _lab, st, _c = cv2.connectedComponentsWithStats(mask)
    comps = [st[i] for i in range(1, n) if st[i, 4] >= 12]
    if len(comps) < 5:
        return []
    hmax = max(c[3] for c in comps)
    comps = sorted((c for c in comps if c[3] >= 0.6 * hmax), key=lambda c: c[0])
    out = []
    for x, y, w, h, _a in comps:
        parts = 2 if w > 0.95 * h else 1             # two digits whose edges touch come out as one wide component
        for k in range(parts):
            xa, xb = x + w * k // parts, x + w * (k + 1) // parts
            out.append(_normalise(soft[max(0, y - 1):y + h + 1, max(0, xa - 1):xb + 1]))
    return out


def _normalise(g):
    """Scale to GLYPH height keeping the aspect ratio (a '1' stays narrow) and centre it in GLYPH width."""
    gh, gw = GLYPH[1], GLYPH[0]
    s = gh / g.shape[0]
    w = max(1, min(gw, int(round(g.shape[1] * s))))
    r = cv2.resize(g, (w, gh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((gh, gw), np.float32)
    x0 = (gw - w) // 2
    canvas[:, x0:x0 + w] = r
    return canvas


def _classify(g):
    G, L = _templates()
    d = ((G - g[None]) ** 2).reshape(len(G), -1).sum(axis=1)
    i = int(np.argmin(d))
    return str(L[i]), float(d[i])


MAX_DIST = 25.0        # glyph distance above which the reading is rejected (blurred / covered counter)


def read(frame, alliance="Red", left_is_red=None):
    """(count, tier) read from a BGR frame, or None when the counter is not readable.  Pass left_is_red when the
    layout of the whole video is known (red_left / detect_red_left); otherwise it is read from this frame."""
    gl = glyphs(frame, alliance, left_is_red)
    if len(gl) < 5 or len(gl) > 8:
        return None
    labs = []
    for g in gl:
        s, d = _classify(g)
        if d > MAX_DIST:
            return None
        labs.append(s)
    if labs[-4] != "/":
        return None
    num, tier = "".join(labs[:-4]), "".join(labs[-3:])
    if not (num.isdigit() and tier.isdigit()) or len(num) > 3:
        return None
    return int(num), int(tier)


def clean(v, small=5):
    """Raw readings (-1 / None where unreadable) -> a counter that never decreases.  Unreadable samples hold the last
    value.  A rise of at most `small` is believed at once; a bigger jump (a misread digit, or the counter was
    covered for a while) only when the following reading agrees."""
    out = np.zeros(len(v))
    cur = 0.0
    pend = None
    for i, x in enumerate(v):
        if x is None or x < 0:
            out[i] = cur
            continue
        if cur <= x <= cur + small:
            cur, pend = float(x), None
        elif x > cur + small:
            if pend is not None and abs(pend - x) <= 1:
                cur, pend = float(x), None
            else:
                pend = x
        out[i] = cur
    return out
