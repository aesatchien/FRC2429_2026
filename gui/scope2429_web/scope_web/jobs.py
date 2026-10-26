"""
MP4 rendering.  A render takes minutes of CPU, so renders run one at a time on a background thread; a request just queues one and the
page polls for progress.

What is rendered is Scope2429's dashboard replay (the top-right slot shows the trend plots).  The YouTube match video is linked next to
it in the viewer instead of being baked in: a server's address is routinely refused by YouTube's downloader, and the desktop app can
still do the combined video.

The finished file is uploaded to R2 if it is configured (and kept on the server's disk unless KEEP_LOCAL_VIDEO=0).
"""
from __future__ import annotations

import queue
import threading
import time
import traceback

from .config import Settings
from .store import LogStore


class RenderQueue:
    def __init__(self, store: LogStore, settings: Settings, replay_mod, r2, start: bool = True) -> None:
        self.store, self.settings, self.replay, self.r2 = store, settings, replay_mod, r2
        self._q: "queue.Queue[str]" = queue.Queue()
        self._status: dict = {}
        self._lock = threading.Lock()
        self._cancel: set = set()
        for meta in store.list():                                # a restart interrupted these: say so instead of "rendering" forever
            if (meta.get("mp4") or {}).get("status") in ("queued", "rendering"):
                store.patch_internal(meta["id"], mp4={"status": "failed", "error": "the server restarted during the render"})
        if start:
            threading.Thread(target=self._worker, name="render-queue", daemon=True).start()

    # ------------------------------------------------------------------ API used by the web app
    def submit(self, log_id: str) -> dict:
        self.store.log_path(log_id)                              # KeyError if there is no such log
        with self._lock:
            cur = self._status.get(log_id)
            if cur and cur["status"] in ("queued", "rendering"):
                return cur
            self._status[log_id] = {"status": "queued", "progress": 0.0, "message": "waiting for the renderer"}
            self._persist(log_id, self._status[log_id])
        self._q.put(log_id)
        return self._status[log_id]

    def status(self, log_id: str) -> dict:
        with self._lock:
            cur = self._status.get(log_id)
        return dict(cur) if cur else (self.store.get(log_id).get("mp4") or {"status": "none"})

    def cancel(self, log_id: str) -> None:
        with self._lock:
            self._cancel.add(log_id)

    # ------------------------------------------------------------------ the worker
    def _persist(self, log_id: str, mp4: dict) -> None:
        self.store.patch_internal(log_id, mp4={k: v for k, v in mp4.items() if k != "progress"} | {"progress": round(mp4.get("progress", 0.0), 3)})

    def _set(self, log_id: str, **kw) -> None:
        with self._lock:
            self._status.setdefault(log_id, {}).update(kw)
            cur = dict(self._status[log_id])
        if cur.get("status") != "rendering":                      # progress ticks stay in memory; start / end states are saved
            self._persist(log_id, cur)

    def _worker(self) -> None:
        while True:
            log_id = self._q.get()
            try:
                self._run(log_id)
            except Exception as e:  # noqa - one bad log must not kill the queue
                traceback.print_exc()
                self._set(log_id, status="failed", error=str(e)[:300], message="failed")

    def run_now(self, log_id: str) -> None:
        """Render synchronously (used by the worker, and by tests)."""
        self._run(log_id)

    def _run(self, log_id: str) -> None:
        core = self.replay.core
        meta = self.store.get(log_id)
        self._set(log_id, status="rendering", progress=0.0, message="reading the log")
        ml = core.load_match_log(str(self.store.log_path(log_id)), [self.replay.DRIVE, self.replay.QUEST])
        out = self.store.video_path(log_id)
        job = core.RenderJob(
            log=ml, video=None, offset=0.0, out_path=str(out), field=self.replay.load_field(self.settings.field_config),
            quest_name=self.replay.QUEST, drive_name=self.replay.DRIVE, height=self.settings.render_height,
            style=core.RenderStyle(red_left=bool(meta.get("red_left", True)), title=meta.get("title", "")))

        def progress(frac, msg):
            self._set(log_id, status="rendering", progress=float(frac), message=str(msg)[:80])

        def cancelled() -> bool:
            with self._lock:
                return log_id in self._cancel

        t0 = time.time()
        core.render(job, progress, cancelled)
        if cancelled():
            with self._lock:
                self._cancel.discard(log_id)
            out.unlink(missing_ok=True)
            self._set(log_id, status="cancelled", message="cancelled")
            return
        size = out.stat().st_size
        key = f"videos/{log_id}.mp4"
        result = {"status": "ready", "progress": 1.0, "message": "done", "bytes": size, "height": self.settings.render_height,
                  "rendered_at": time.time(), "seconds": round(time.time() - t0, 1), "local": True, "r2_key": ""}
        if self.r2.enabled:
            self._set(log_id, status="rendering", progress=1.0, message="uploading to R2")
            self.r2.upload_file(out, key)
            result["r2_key"] = key
            if not self.settings.keep_local_video:
                out.unlink(missing_ok=True)
                result["local"] = False
        self._set(log_id, **result)
