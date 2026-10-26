"""
The Scope2429 website: upload a robot log, replay the dashboard in the browser, read the match report, render an MP4.

Run it with   uvicorn scope_web.app:create_app --factory --host 0.0.0.0 --port 8000     (see README.md and docker-compose.yml)

Every route except /healthz needs a signed-in team member (auth.py).  The pages are written to be embedded in an <iframe> on the team
site: FRAME_ANCESTORS lists the sites allowed to do that, and nothing else may.
"""
from __future__ import annotations

from pathlib import Path

from fastapi import Depends, FastAPI, File, HTTPException, Query, Request, UploadFile
from fastapi.responses import FileResponse, RedirectResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .auth import Auth
from .config import Settings
from .jobs import RenderQueue
from .r2 import R2
from .store import LogStore, UploadError

STATIC = Path(__file__).resolve().parent.parent / "static"


class MetaPatch(BaseModel):
    title: str | None = None
    youtube_url: str | None = None
    offset: float | None = None
    red_left: bool | None = None


def page_csp(s: Settings) -> str:
    """The security policy for our own pages: scripts only from us (and YouTube's player), no inline script or style."""
    media = "'self' blob:" + (f" {s.r2_endpoint}" if s.r2_enabled else "")
    return ("default-src 'self'; img-src 'self' data: blob:; style-src 'self'; script-src 'self' https://www.youtube.com https://s.ytimg.com; "
            "frame-src 'self' https://www.youtube.com https://www.youtube-nocookie.com; "
            f"media-src {media}; connect-src 'self'; base-uri 'none'; form-action 'self'; frame-ancestors {s.frame_ancestors}")


# the match report is a self-contained HTML page with inline styles and no scripts: allow the styles, forbid everything else
REPORT_CSP = "default-src 'none'; style-src 'unsafe-inline'; img-src data:; base-uri 'none'; form-action 'none'; frame-ancestors 'self'"


def create_app(settings: Settings | None = None, auth_verifier=None, r2: R2 | None = None, replay_service=None,
               start_worker: bool = True) -> FastAPI:
    settings = settings or Settings.from_env()
    store = LogStore(settings.data_dir, settings.max_upload_mb * 1024 * 1024)
    auth = Auth(settings, verifier=auth_verifier)             # raises ConfigError if nothing is configured
    r2 = r2 or R2(settings)
    from . import replay as replay_mod                         # heavy import (OpenCV, the dashboard renderer)
    replay_service = replay_service or replay_mod.Replay(settings.field_config)
    renders = RenderQueue(store, settings, replay_mod, r2, start=start_worker)

    app = FastAPI(title="Scope2429", docs_url=None, redoc_url=None, openapi_url=None)
    app.state.settings, app.state.store, app.state.auth = settings, store, auth
    app.state.replay, app.state.renders, app.state.r2 = replay_service, renders, r2
    csp = page_csp(settings)

    @app.middleware("http")
    async def security_headers(request: Request, call_next):
        resp = await call_next(request)
        resp.headers.setdefault("Content-Security-Policy", csp)
        resp.headers["X-Content-Type-Options"] = "nosniff"
        resp.headers["Referrer-Policy"] = "same-origin"
        return resp

    def user(request: Request) -> str:
        return request.app.state.auth.user(request)

    def meta_or_404(log_id: str) -> dict:
        try:
            return store.get(log_id)
        except KeyError:
            raise HTTPException(404, "no such log")

    # ------------------------------------------------------------------ pages
    @app.get("/healthz", include_in_schema=False)
    def healthz():
        return {"ok": True}

    @app.get("/", include_in_schema=False)
    def index(_: str = Depends(user)):
        return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-cache"})

    @app.get("/view/{log_id}", include_in_schema=False)
    def viewer(log_id: str, _: str = Depends(user)):
        meta_or_404(log_id)
        return FileResponse(STATIC / "viewer.html", headers={"Cache-Control": "no-cache"})

    @app.get("/static/{name:path}", include_in_schema=False)
    def static_files(name: str, _: str = Depends(user)):
        p = (STATIC / name).resolve()
        if STATIC.resolve() not in p.parents or not p.is_file():
            raise HTTPException(404)
        return FileResponse(p, headers={"Cache-Control": "no-cache"})

    # ------------------------------------------------------------------ logs
    @app.get("/api/logs")
    def list_logs(_: str = Depends(user)):
        return store.list()

    @app.get("/api/logs/{log_id}")
    def get_log(log_id: str, _: str = Depends(user)):
        return meta_or_404(log_id)

    @app.post("/api/upload")
    def upload(log: UploadFile = File(...), console: UploadFile | None = File(None), who: str = Depends(user)):
        try:
            log_id, created = store.save_log(log.file, log.filename or "log.wpilog")
        except UploadError as e:
            raise HTTPException(400, str(e))
        if created:
            try:
                summary = replay_mod.summarize(store.log_path(log_id))
            except Exception:  # noqa - a log we cannot read is not kept
                store.delete(log_id)
                raise HTTPException(400, "could not read that log (is it a complete .wpilog from the robot?)")
            store.put(log_id, {"id": log_id, "filename": store.log_path(log_id).name, "size": store.log_path(log_id).stat().st_size,
                               "uploaded_at": store.now(), "uploaded_by": who, **summary,
                               "youtube_url": "", "offset": 0.0, "red_left": True, "mp4": {"status": "none"}})
        if console is not None and console.filename:
            try:
                store.save_console(log_id, console.file, console.filename)
            except UploadError as e:
                raise HTTPException(400, str(e))
            app.state.replay.drop(log_id)                      # the report reads the console file
        return {"id": log_id, "created": created, **meta_or_404(log_id)}

    @app.patch("/api/logs/{log_id}")
    def patch_log(log_id: str, body: MetaPatch, _: str = Depends(user)):
        meta_or_404(log_id)
        try:
            return store.update(log_id, body.model_dump(exclude_none=True))
        except UploadError as e:
            raise HTTPException(400, str(e))

    @app.delete("/api/logs/{log_id}")
    def delete_log(log_id: str, _: str = Depends(user)):
        meta = meta_or_404(log_id)
        key = (meta.get("mp4") or {}).get("r2_key")
        if key and r2.enabled:
            try:
                r2.delete(key)
            except Exception:  # noqa - a missing cloud copy must not block removing the log
                pass
        app.state.replay.drop(log_id)
        store.delete(log_id)
        return {"deleted": log_id}

    # ------------------------------------------------------------------ the replay
    @app.get("/api/logs/{log_id}/frame.jpg")
    def frame(log_id: str, t: float = 0.0, w: int = Query(1280, ge=320, le=1920), _: str = Depends(user)):
        meta = meta_or_404(log_id)
        entry = app.state.replay.entry(log_id, store.log_path(log_id), meta.get("title", ""), meta.get("red_left", True))
        t = max(meta.get("t_min", -5.0), min(meta.get("t_max", 5.0), t))
        return Response(app.state.replay.frame_jpeg(entry, t, w), media_type="image/jpeg", headers={"Cache-Control": "private, max-age=600"})

    @app.get("/api/logs/{log_id}/report")
    def report(log_id: str, _: str = Depends(user)):
        meta = meta_or_404(log_id)
        entry = app.state.replay.entry(log_id, store.log_path(log_id), meta.get("title", ""), meta.get("red_left", True))
        return Response(app.state.replay.report_html(entry), media_type="text/html; charset=utf-8",
                        headers={"Content-Security-Policy": REPORT_CSP})

    # ------------------------------------------------------------------ MP4
    @app.post("/api/logs/{log_id}/render")
    def start_render(log_id: str, _: str = Depends(user)):
        meta_or_404(log_id)
        return renders.submit(log_id)

    @app.get("/api/logs/{log_id}/render")
    def render_status(log_id: str, _: str = Depends(user)):
        meta_or_404(log_id)
        return renders.status(log_id)

    @app.delete("/api/logs/{log_id}/render")
    def cancel_render(log_id: str, _: str = Depends(user)):
        meta_or_404(log_id)
        renders.cancel(log_id)
        return {"cancelling": log_id}

    @app.get("/api/logs/{log_id}/video")
    def video(log_id: str, download: int = 0, _: str = Depends(user)):
        meta = meta_or_404(log_id)
        mp4 = meta.get("mp4") or {}
        if mp4.get("status") != "ready":
            raise HTTPException(404, "no video rendered yet")
        if r2.enabled and mp4.get("r2_key"):
            return RedirectResponse(r2.presigned_url(mp4["r2_key"]), status_code=302)
        path = store.video_path(log_id)
        if not path.exists():
            raise HTTPException(404, "the video file is gone; render it again")
        name = (meta.get("title") or log_id).replace("/", "-") + ".mp4"
        return FileResponse(path, media_type="video/mp4", filename=name if download else None)

    return app
