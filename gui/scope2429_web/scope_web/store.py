"""
Where the uploaded logs live.

    <data>/logs/<id>/<name>.wpilog       the log (id = first 16 hex of its SHA-256, so uploading the same file twice is one entry)
    <data>/logs/<id>/console_*.log       the robot's console file, if uploaded (Scope2429 finds it next to the log)
    <data>/meta/<id>.json                what the site knows about it: title, match, YouTube link, MP4 state
    <data>/videos/<id>.mp4               a rendered video
    <data>/tmp/                          uploads in progress

Nothing here trusts a client-supplied name or path: the id is a hash, file names are reduced to a safe set of characters, and every
path is checked to stay inside the data directory.
"""
from __future__ import annotations

import hashlib
import json
import re
import shutil
import threading
import time
import uuid
from pathlib import Path

LOG_MAGIC = b"WPILOG"
_SAFE = re.compile(r"[^A-Za-z0-9._-]")
_ID = re.compile(r"^[0-9a-f]{16}$")
EDITABLE = {"title": str, "youtube_url": str, "offset": float, "red_left": bool}


class UploadError(ValueError):
    pass


def safe_name(name: str, suffix: str) -> str:
    stem = _SAFE.sub("_", Path(name or "").name)[:120].strip("._") or "log"
    return stem if stem.lower().endswith(suffix) else stem + suffix


class LogStore:
    def __init__(self, data_dir, max_bytes: int = 250 * 1024 * 1024) -> None:
        self.root = Path(data_dir).resolve()
        self.max_bytes = max_bytes
        for sub in ("logs", "meta", "videos", "tmp"):
            (self.root / sub).mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ paths
    def valid_id(self, log_id: str) -> bool:
        return bool(_ID.match(log_id or ""))

    def _dir(self, log_id: str) -> Path:
        if not self.valid_id(log_id):
            raise KeyError(log_id)
        return self.root / "logs" / log_id

    def log_path(self, log_id: str) -> Path:
        d = self._dir(log_id)
        found = sorted(d.glob("*.wpilog")) if d.is_dir() else []
        if not found:
            raise KeyError(log_id)
        return found[0]

    def video_path(self, log_id: str) -> Path:
        self._dir(log_id)
        return self.root / "videos" / f"{log_id}.mp4"

    # ------------------------------------------------------------------ saving an upload
    def save_log(self, fileobj, filename: str):
        """Stream an uploaded .wpilog to disk.  Returns (id, created).  Raises UploadError."""
        tmp = self.root / "tmp" / f"{uuid.uuid4().hex}.part"
        h, size, head = hashlib.sha256(), 0, b""
        try:
            with open(tmp, "wb") as out:
                while True:
                    chunk = fileobj.read(1024 * 1024)
                    if not chunk:
                        break
                    if len(head) < len(LOG_MAGIC):
                        head = (head + chunk)[:len(LOG_MAGIC)]
                    size += len(chunk)
                    if size > self.max_bytes:
                        raise UploadError(f"file is larger than the {self.max_bytes // (1024 * 1024)} MB limit")
                    h.update(chunk)
                    out.write(chunk)
            if head != LOG_MAGIC:
                raise UploadError("that is not a .wpilog file (it does not start with WPILOG)")
            log_id = h.hexdigest()[:16]
            with self._lock:
                d = self.root / "logs" / log_id
                if d.is_dir() and any(d.glob("*.wpilog")):
                    return log_id, False
                d.mkdir(parents=True, exist_ok=True)
                shutil.move(str(tmp), str(d / safe_name(filename, ".wpilog")))
            return log_id, True
        finally:
            tmp.unlink(missing_ok=True)

    def save_console(self, log_id: str, fileobj, filename: str) -> str:
        """The robot's console_<boot>.log for this log.  It must be named console_*.log for Scope2429 to find it."""
        d = self._dir(log_id)
        if not d.is_dir():
            raise KeyError(log_id)
        name = safe_name(filename, ".log")
        if not name.startswith("console_"):
            name = "console_" + name
        data = fileobj.read(self.max_bytes + 1)
        if len(data) > self.max_bytes:
            raise UploadError("console file is too large")
        (d / name).write_bytes(data)
        return name

    # ------------------------------------------------------------------ metadata
    def _meta_path(self, log_id: str) -> Path:
        if not self.valid_id(log_id):
            raise KeyError(log_id)
        return self.root / "meta" / f"{log_id}.json"

    def get(self, log_id: str) -> dict:
        p = self._meta_path(log_id)
        if not p.exists():
            raise KeyError(log_id)
        return json.loads(p.read_text(encoding="utf-8"))

    def put(self, log_id: str, meta: dict) -> dict:
        p = self._meta_path(log_id)
        tmp = p.with_suffix(".tmp")
        tmp.write_text(json.dumps(meta, indent=1), encoding="utf-8")
        tmp.replace(p)                                       # atomic: a reader never sees half a file
        return meta

    def update(self, log_id: str, fields: dict) -> dict:
        """Change only the fields a person may edit; anything else in `fields` is ignored."""
        with self._lock:
            meta = self.get(log_id)
            for key, typ in EDITABLE.items():
                if key in fields and fields[key] is not None:
                    value = fields[key]
                    meta[key] = bool(value) if typ is bool else typ(value)
            if meta.get("youtube_url") and not re.match(r"^https://(www\.)?(youtube\.com|youtu\.be)/", meta["youtube_url"]):
                raise UploadError("the video link must be a https://youtube.com or https://youtu.be address")
            return self.put(log_id, meta)

    def patch_internal(self, log_id: str, **fields) -> dict:
        """For the server itself (MP4 state); unlike update() it may set any key."""
        with self._lock:
            meta = self.get(log_id)
            meta.update(fields)
            return self.put(log_id, meta)

    def list(self) -> list:
        out = []
        for p in (self.root / "meta").glob("*.json"):
            try:
                out.append(json.loads(p.read_text(encoding="utf-8")))
            except (OSError, ValueError):
                continue
        return sorted(out, key=lambda m: (m.get("log_date") or "", m.get("uploaded_at", 0)), reverse=True)

    def delete(self, log_id: str) -> None:
        d = self._dir(log_id)
        shutil.rmtree(d, ignore_errors=True)
        self._meta_path(log_id).unlink(missing_ok=True)
        self.video_path(log_id).unlink(missing_ok=True)

    @staticmethod
    def now() -> float:
        return time.time()
