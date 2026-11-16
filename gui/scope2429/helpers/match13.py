"""
Match odds from match13.com for the desktop app: the predicted score, the chance each alliance wins and, once the match is played, the final
score from The Blue Alliance.  The same answer the web archive shows (gui/scope2429_web/scope_web/match13.py): the request is

    GET https://actions.match13.com/v1/matches/<TBA match key>      Authorization: Bearer m13_live_...

`pred` holds redScore, blueScore and winProb (the RED alliance's chance).  Which alliance 2429 is on comes from the log (`ml.alliance`); the
partners' numbers come from The Blue Alliance when a TBA key is given.  No UI and no third-party packages, so it can be used from scripts.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request

BASE = "https://actions.match13.com"


def _get(url: str, headers: dict, timeout: float = 15.0) -> bytes:
    req = urllib.request.Request(url, headers={"Accept": "application/json", "User-Agent": "scope2429", **headers})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def tba_result(match_key: str, tba_key: str, get=_get) -> dict | None:
    """The final score from The Blue Alliance, {"red": 404, "blue": 88}; None before the match is played, or without a key / an answer."""
    if not (match_key and tba_key):
        return None
    try:
        d = json.loads(get(f"https://www.thebluealliance.com/api/v3/match/{match_key}/simple", {"X-TBA-Auth-Key": tba_key}))
        red, blue = d["alliances"]["red"]["score"], d["alliances"]["blue"]["score"]
    except Exception:  # noqa - offline, 404, an odd body: the odds are shown without a result
        return None
    return {"red": red, "blue": blue} if red is not None and blue is not None and red >= 0 and blue >= 0 else None


def fetch(match_key: str, api_key: str, ours: str = "", result: dict | None = None, get=_get) -> dict:
    """{"status": "ready", "red": {"score", "win"}, "blue": {...}, "ours": "red"|"blue"|"", "result": {...}|None} /
    {"status": "none"|"failed", "message": why}."""
    if not match_key:
        return {"status": "none", "message": "this log is not a qualification or playoff match"}
    if not api_key:
        return {"status": "none", "message": "no match13 API key"}
    try:
        raw = json.loads(get(f"{BASE}/v1/matches/{match_key}", {"Authorization": f"Bearer {api_key}"}))
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return {"status": "none", "message": "match13 has no prediction for this match (yet)"}
        return {"status": "failed", "message": {401: "match13 rejected the key", 429: "match13 rate limit reached; try again in a minute"}
                .get(e.code, f"match13 answered {e.code}")}
    except Exception as e:  # noqa - offline, timeout, not JSON
        return {"status": "failed", "message": f"could not reach match13 ({type(e).__name__})"}
    pred = raw.get("pred") or {}
    red_win = pred.get("winProb")
    return {"status": "ready", "key": match_key, "ours": ours if ours in ("red", "blue") else "", "result": result,
            "red": {"score": pred.get("redScore"), "win": red_win},
            "blue": {"score": pred.get("blueScore"), "win": None if red_win is None else 1.0 - red_win}}


def summary(odds: dict) -> str:
    """One line for the status area: "match13: us (blue) 95 - 33 red, 5% win; final 88 - 404 L" (our side first)."""
    if not odds or odds.get("status") != "ready":
        return "match13: " + (odds or {}).get("message", "no odds") if odds else ""
    us = odds["ours"] or "red"
    them = "blue" if us == "red" else "red"
    f = lambda x: "?" if x is None else f"{x:.0f}"
    win = odds[us]["win"]
    text = f"match13: us ({us}) {f(odds[us]['score'])} - {f(odds[them]['score'])} {them} predicted"
    if win is not None:
        text += f", {win * 100:.0f}% win"
    res = odds.get("result")
    if res:
        a, b = res[us], res[them]
        text += f";  final {a} - {b} {'W' if a > b else 'L' if a < b else 'T'}"
    return text
