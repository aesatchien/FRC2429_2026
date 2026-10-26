"""
All the server's settings come from environment variables (see .env.example), so the same image runs on the TrueNAS and on a laptop.

The server refuses to start unless it knows how to authenticate: a team-only site that quietly runs open is worse than one that
does not start.  Set ALLOW_NO_AUTH=1 for local development only.
"""
from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path


class ConfigError(RuntimeError):
    pass


def _env(name: str, default: str = "") -> str:
    return os.environ.get(name, default).strip()


@dataclass
class Settings:
    data_dir: Path
    max_upload_mb: int = 250
    frame_ancestors: str = "'self'"          # who may embed the pages in an <iframe> (space-separated origins)
    cf_team_domain: str = ""                 # Cloudflare Access: "<team>" of <team>.cloudflareaccess.com
    cf_aud: str = ""                         # Cloudflare Access: the application's AUD tag
    team_password: str = ""                  # simple shared password (HTTP Basic) when not behind Cloudflare Access
    allow_no_auth: bool = False
    r2_account_id: str = ""
    r2_access_key_id: str = ""
    r2_secret_access_key: str = ""
    r2_bucket: str = ""
    render_height: int = 540                 # MP4 size: 360 / 540 / 720 (the dashboard canvas is 2x that tall)
    keep_local_video: bool = True            # keep the MP4 on the server's disk after it has been uploaded to R2
    field_config: str = ""                   # a field config.json; empty = the one bundled with Scope2429

    @property
    def auth_mode(self) -> str:
        if self.cf_team_domain and self.cf_aud:
            return "cloudflare"
        if self.team_password:
            return "password"
        if self.allow_no_auth:
            return "none"
        raise ConfigError(
            "No authentication configured.  Set CF_ACCESS_TEAM_DOMAIN and CF_ACCESS_AUD (Cloudflare Access, recommended) or "
            "TEAM_PASSWORD, or ALLOW_NO_AUTH=1 for local development only.")

    @property
    def r2_enabled(self) -> bool:
        return bool(self.r2_account_id and self.r2_access_key_id and self.r2_secret_access_key and self.r2_bucket)

    @property
    def r2_endpoint(self) -> str:
        return f"https://{self.r2_account_id}.r2.cloudflarestorage.com"

    @staticmethod
    def frame_ancestors_from(origins: str) -> str:
        """FRAME_ANCESTORS is a plain list of sites ("https://www.team.org https://sites.google.com"); 'self' is always added.
        Anything that could end the header value or add a second directive is refused."""
        if not re.fullmatch(r"[A-Za-z0-9.:/*_ -]*", origins):
            raise ConfigError("FRAME_ANCESTORS may only list sites separated by spaces, e.g. https://www.team.org https://sites.google.com")
        return "'self'" + (" " + " ".join(origins.split()) if origins.split() else "")

    @classmethod
    def from_env(cls) -> "Settings":
        return cls(
            data_dir=Path(_env("DATA_DIR", "./data")),
            max_upload_mb=int(_env("MAX_UPLOAD_MB", "250")),
            frame_ancestors=cls.frame_ancestors_from(_env("FRAME_ANCESTORS")),
            cf_team_domain=_env("CF_ACCESS_TEAM_DOMAIN"),
            cf_aud=_env("CF_ACCESS_AUD"),
            team_password=_env("TEAM_PASSWORD"),
            allow_no_auth=_env("ALLOW_NO_AUTH") == "1",
            r2_account_id=_env("R2_ACCOUNT_ID"),
            r2_access_key_id=_env("R2_ACCESS_KEY_ID"),
            r2_secret_access_key=_env("R2_SECRET_ACCESS_KEY"),
            r2_bucket=_env("R2_BUCKET"),
            render_height=int(_env("RENDER_HEIGHT", "540")),
            keep_local_video=_env("KEEP_LOCAL_VIDEO", "1") != "0",
            field_config=_env("FIELD_CONFIG"),
        )
