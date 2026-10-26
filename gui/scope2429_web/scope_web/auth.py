"""
Who may use the site.

The site is meant to sit behind a Cloudflare Tunnel with a Cloudflare Access policy (team members only).  Access signs every request
it lets through with a JWT in the `Cf-Access-Jwt-Assertion` header.  The server checks that signature itself, not just that the header
exists: anyone who could reach the container directly (another machine on the home network, say) could otherwise type the header.

Without Cloudflare Access a shared password (HTTP Basic) works, and ALLOW_NO_AUTH=1 turns checking off for local development.
"""
from __future__ import annotations

import base64
import hmac

from fastapi import HTTPException, Request

from .config import Settings


class Auth:
    def __init__(self, settings: Settings, verifier=None) -> None:
        self.mode = settings.auth_mode                       # raises ConfigError when nothing is configured
        self.settings = settings
        self._verifier = verifier                             # token -> user (tests inject one)
        self._jwks = None

    # ------------------------------------------------------------------ Cloudflare Access
    def _verify_cf(self, token: str) -> str:
        if self._verifier is not None:
            return self._verifier(token)
        import jwt                                            # PyJWT, only needed in this mode
        team = self.settings.cf_team_domain
        issuer = f"https://{team}.cloudflareaccess.com"
        if self._jwks is None:
            self._jwks = jwt.PyJWKClient(f"{issuer}/cdn-cgi/access/certs")
        key = self._jwks.get_signing_key_from_jwt(token).key
        claims = jwt.decode(token, key, algorithms=["RS256"], audience=self.settings.cf_aud, issuer=issuer)
        return claims.get("email") or claims.get("common_name") or "unknown"

    # ------------------------------------------------------------------ the check
    def user(self, request: Request) -> str:
        if self.mode == "none":
            return "dev"
        if self.mode == "cloudflare":
            token = request.headers.get("Cf-Access-Jwt-Assertion") or request.cookies.get("CF_Authorization")
            if not token:
                raise HTTPException(401, "Sign in through Cloudflare Access")
            try:
                return self._verify_cf(token)
            except Exception:  # noqa - any failure to prove who you are is a 401, never a 500
                raise HTTPException(401, "Invalid Cloudflare Access token")
        # password
        header = request.headers.get("Authorization", "")
        if header.lower().startswith("basic "):
            try:
                _user, _, password = base64.b64decode(header[6:]).decode("utf-8").partition(":")
            except Exception:  # noqa
                password = ""
            if hmac.compare_digest(password.encode(), self.settings.team_password.encode()):
                return _user or "team"
        raise HTTPException(401, "Team password required", headers={"WWW-Authenticate": 'Basic realm="Scope2429", charset="UTF-8"'})
