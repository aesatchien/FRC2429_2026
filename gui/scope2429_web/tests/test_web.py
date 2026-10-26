"""
The Scope2429 website: who may use it, what an upload is allowed to be, that the replay and report come back, that the pages are safe to
embed, and that the MP4 queue and R2 hand-off work.  The robot logs themselves are synthetic (conftest.make_wpilog).
"""
import base64
import io
import json

import pytest
from fastapi.testclient import TestClient

from conftest import make_wpilog
from scope_web import replay as replay_mod
from scope_web.app import create_app
from scope_web.config import ConfigError, Settings
from scope_web.r2 import R2


# ------------------------------------------------------------------------------------------------------------------ auth
def test_refuses_to_start_without_any_authentication(tmp_path):
    with pytest.raises(ConfigError):
        create_app(Settings(data_dir=tmp_path), start_worker=False)


def test_password_mode(tmp_path):
    c = TestClient(create_app(Settings(data_dir=tmp_path, team_password="s3cret"), start_worker=False))
    assert c.get("/api/logs").status_code == 401
    assert c.get("/api/logs", headers={"Authorization": "Basic " + base64.b64encode(b"x:wrong").decode()}).status_code == 401
    assert c.get("/api/logs", headers={"Authorization": "Basic " + base64.b64encode(b"x:s3cret").decode()}).status_code == 200
    assert c.get("/healthz").status_code == 200, "the health check must work without a login"


def test_cloudflare_access_mode_checks_the_token_not_just_its_presence(tmp_path):
    def verifier(token):
        if token != "good-token":
            raise ValueError("bad signature")
        return "mentor@example.org"
    c = TestClient(create_app(Settings(data_dir=tmp_path, cf_team_domain="team", cf_aud="aud"), auth_verifier=verifier, start_worker=False))
    assert c.get("/api/logs").status_code == 401
    assert c.get("/api/logs", headers={"Cf-Access-Jwt-Assertion": "forged"}).status_code == 401, "a made-up header must not get in"
    assert c.get("/api/logs", headers={"Cf-Access-Jwt-Assertion": "good-token"}).status_code == 200


# ------------------------------------------------------------------------------------------------------------------ uploads
def test_upload_list_and_summary(uploaded):
    client, meta = uploaded
    assert meta["created"] is True and len(meta["id"]) == 16
    assert meta["event"] and meta["log_date"] == "2026-10-11"
    assert meta["t_min"] < 0 < meta["t_max"]
    listed = client.get("/api/logs").json()
    assert [m["id"] for m in listed] == [meta["id"]]


def test_the_same_log_twice_is_one_entry(uploaded, wpilog_bytes):
    client, meta = uploaded
    r = client.post("/api/upload", files={"log": ("again.wpilog", wpilog_bytes, "x")})
    assert r.json()["id"] == meta["id"] and r.json()["created"] is False
    assert len(client.get("/api/logs").json()) == 1


def test_upload_rejects_things_that_are_not_logs(client):
    r = client.post("/api/upload", files={"log": ("evil.wpilog", b"#!/bin/sh\nrm -rf /\n", "x")})
    assert r.status_code == 400 and "not a .wpilog" in r.json()["detail"]
    r = client.post("/api/upload", files={"log": ("trunc.wpilog", b"WPILOG\x00", "x")})
    assert r.status_code == 400, "a file with the right first bytes that cannot be read is refused and not kept"
    assert client.get("/api/logs").json() == []


def test_upload_size_limit(tmp_path):
    c = TestClient(create_app(Settings(data_dir=tmp_path, allow_no_auth=True, max_upload_mb=1), start_worker=False))
    r = c.post("/api/upload", files={"log": ("big.wpilog", b"WPILOG" + b"\0" * (2 * 1024 * 1024), "x")})
    assert r.status_code == 400 and "limit" in r.json()["detail"]


def test_file_names_cannot_escape_the_data_directory(client, wpilog_bytes, settings):
    r = client.post("/api/upload", files={"log": ("../../../../etc/evil.wpilog", wpilog_bytes, "x")})
    assert r.status_code == 200
    stored = list((settings.data_dir / "logs").rglob("*.wpilog"))
    assert len(stored) == 1 and settings.data_dir.resolve() in stored[0].resolve().parents


def test_console_file_is_stored_next_to_the_log(client, wpilog_bytes, settings):
    r = client.post("/api/upload", files={"log": ("a.wpilog", wpilog_bytes, "x"), "console": ("run1.log", b"1.000 2026-10-25T00:00:00Z O hi\n", "x")})
    assert r.status_code == 200
    files = sorted(p.name for p in (settings.data_dir / "logs" / r.json()["id"]).iterdir())
    assert files == ["a.wpilog", "console_run1.log"], "it must be named console_*.log for Scope2429 to find it"


# ------------------------------------------------------------------------------------------------------------------ the replay
def test_frame_is_a_jpeg_of_the_requested_width(uploaded):
    import cv2
    import numpy as np
    client, meta = uploaded
    r = client.get(f"/api/logs/{meta['id']}/frame.jpg", params={"t": 3.0, "w": 960})
    assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg"
    img = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape == (540, 960, 3), "the 1920x1080 dashboard scaled to the requested width"
    assert client.get(f"/api/logs/{meta['id']}/frame.jpg", params={"t": 1e9}).status_code == 200, "times outside the log are clamped"
    assert client.get(f"/api/logs/{meta['id']}/frame.jpg", params={"w": 10}).status_code == 422


def test_report_is_html_with_a_locked_down_policy(uploaded):
    client, meta = uploaded
    r = client.get(f"/api/logs/{meta['id']}/report")
    assert r.status_code == 200 and "<html" in r.text.lower()
    csp = r.headers["content-security-policy"]
    assert "default-src 'none'" in csp and "script-src" not in csp, "the report has no scripts, so none are allowed"


# ------------------------------------------------------------------------------------------------------------------ embedding / safety
def test_pages_can_be_embedded_only_where_allowed(uploaded):
    client, meta = uploaded
    for url in ("/", f"/view/{meta['id']}", "/static/app.css", f"/api/logs/{meta['id']}"):
        r = client.get(url)
        csp = r.headers["content-security-policy"]
        assert "frame-ancestors 'self' https://team.example.org" in csp, url
        assert "x-frame-options" not in r.headers, "X-Frame-Options would block the embedding that frame-ancestors allows"
        assert r.headers["x-content-type-options"] == "nosniff"
    page = client.get("/").headers["content-security-policy"]
    assert "script-src 'self' https://www.youtube.com" in page and "'unsafe-inline'" not in page


def test_static_path_traversal_and_bad_ids(client, settings):
    assert client.get("/static/../../../etc/passwd").status_code in (404, 400)
    assert client.get("/static/%2e%2e/%2e%2e/scope_web/config.py").status_code == 404
    for bad in ("..", "0123456789abcdeg", "x" * 40, "0" * 16):
        assert client.get(f"/api/logs/{bad}").status_code in (404, 422), bad
    assert client.get("/view/not-a-log").status_code == 404


# ------------------------------------------------------------------------------------------------------------------ metadata
def test_editing_details(uploaded):
    client, meta = uploaded
    url = f"/api/logs/{meta['id']}"
    r = client.patch(url, json={"title": "E2 replay", "youtube_url": "https://www.youtube.com/watch?v=abcdefghijk", "offset": 90.5, "red_left": False})
    assert r.status_code == 200 and r.json()["title"] == "E2 replay" and r.json()["offset"] == 90.5 and r.json()["red_left"] is False
    assert client.patch(url, json={"youtube_url": "https://evil.example/x"}).status_code == 400, "only YouTube links are accepted"
    assert client.patch(url, json={"id": "ffffffffffffffff", "mp4": {"status": "ready"}}).json()["id"] == meta["id"], "internal fields cannot be edited"
    assert client.get(url).json()["mp4"] == {"status": "none"}


def test_delete_removes_the_log(uploaded, settings):
    client, meta = uploaded
    assert client.delete(f"/api/logs/{meta['id']}").status_code == 200
    assert client.get(f"/api/logs/{meta['id']}").status_code == 404
    assert not list((settings.data_dir / "logs").rglob("*.wpilog"))


# ------------------------------------------------------------------------------------------------------------------ MP4 + R2
class FakeS3:
    def __init__(self):
        self.uploaded, self.deleted = [], []

    def upload_file(self, path, bucket, key, ExtraArgs=None):
        self.uploaded.append((bucket, key, ExtraArgs))

    def generate_presigned_url(self, op, Params, ExpiresIn):
        return f"https://acct.r2.cloudflarestorage.com/{Params['Bucket']}/{Params['Key']}?X-Amz-Expires={ExpiresIn}&sig=abc"

    def delete_object(self, Bucket, Key):
        self.deleted.append((Bucket, Key))


def _fake_render(core_module, monkeypatch, written=b"\x00\x00\x00\x18ftypmp42" + b"x" * 5000):
    def render(job, progress=None, cancel=None):
        progress(0.5, "halfway")
        with open(job.out_path, "wb") as f:
            f.write(written)
    monkeypatch.setattr(core_module, "render", render)


def test_render_without_r2_serves_the_local_file(uploaded, app, monkeypatch):
    client, meta = uploaded
    _fake_render(replay_mod.core, monkeypatch)
    assert client.get(f"/api/logs/{meta['id']}/video").status_code == 404, "no video before a render"
    app.state.renders.run_now(meta["id"])
    st = client.get(f"/api/logs/{meta['id']}/render").json()
    assert st["status"] == "ready" and st["bytes"] > 0
    assert client.get(f"/api/logs/{meta['id']}").json()["mp4"]["status"] == "ready", "the state is saved with the log"
    r = client.get(f"/api/logs/{meta['id']}/video", headers={"Range": "bytes=0-7"})
    assert r.status_code in (200, 206) and r.headers["content-type"] == "video/mp4"


def test_render_with_r2_uploads_and_hands_out_signed_links(tmp_path, wpilog_bytes, monkeypatch):
    s = Settings(data_dir=tmp_path, allow_no_auth=True, r2_account_id="acct", r2_access_key_id="k", r2_secret_access_key="s",
                 r2_bucket="scope-videos", keep_local_video=False)
    fake = FakeS3()
    app = create_app(s, r2=R2(s, client=fake), start_worker=False)
    c = TestClient(app)
    log_id = c.post("/api/upload", files={"log": ("a.wpilog", wpilog_bytes, "x")}).json()["id"]
    _fake_render(replay_mod.core, monkeypatch)
    app.state.renders.run_now(log_id)
    assert fake.uploaded == [("scope-videos", f"videos/{log_id}.mp4", {"ContentType": "video/mp4"})]
    assert not (tmp_path / "videos" / f"{log_id}.mp4").exists(), "KEEP_LOCAL_VIDEO=0 removes the local copy after the upload"
    r = c.get(f"/api/logs/{log_id}/video", follow_redirects=False)
    assert r.status_code == 302 and r.headers["location"].startswith("https://acct.r2.cloudflarestorage.com/scope-videos/videos/")
    assert "frame-src" in c.get("/").headers["content-security-policy"] and "acct.r2.cloudflarestorage.com" in c.get("/").headers["content-security-policy"], \
        "the page may play from the R2 address and nowhere else"
    c.delete(f"/api/logs/{log_id}")
    assert fake.deleted == [("scope-videos", f"videos/{log_id}.mp4")], "deleting a log removes its cloud copy too"


def test_a_failed_render_is_reported_not_swallowed(uploaded, app, monkeypatch):
    client, meta = uploaded

    def boom(job, progress=None, cancel=None):
        raise RuntimeError("ffmpeg not found")
    monkeypatch.setattr(replay_mod.core, "render", boom)
    with pytest.raises(RuntimeError):
        app.state.renders.run_now(meta["id"])                      # the worker thread catches this; here we see it directly
    app.state.renders._set(meta["id"], status="failed", error="ffmpeg not found", message="failed")
    st = client.get(f"/api/logs/{meta['id']}/render").json()
    assert st["status"] == "failed" and "ffmpeg" in st["error"]


def test_a_restart_does_not_leave_a_render_stuck(tmp_path, wpilog_bytes):
    s = Settings(data_dir=tmp_path, allow_no_auth=True)
    app = create_app(s, start_worker=False)
    c = TestClient(app)
    log_id = c.post("/api/upload", files={"log": ("a.wpilog", wpilog_bytes, "x")}).json()["id"]
    app.state.store.patch_internal(log_id, mp4={"status": "rendering", "progress": 0.4})
    app2 = create_app(s, start_worker=False)                         # the server restarts
    st = TestClient(app2).get(f"/api/logs/{log_id}/render").json()
    assert st["status"] == "failed" and "restarted" in st["error"]


# ------------------------------------------------------------------------------------------------------------------ settings
def test_frame_ancestors_setting():
    assert Settings.frame_ancestors_from("") == "'self'"
    assert Settings.frame_ancestors_from("https://www.team.org   https://sites.google.com") == "'self' https://www.team.org https://sites.google.com"
    for bad in ("https://a.org; script-src *", "https://a.org\r\nX-Evil: 1", "'unsafe-inline'"):
        with pytest.raises(ConfigError):
            Settings.frame_ancestors_from(bad)
