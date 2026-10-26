# Scope2429 Web

A team-only website for Scope2429 match logs: upload a `.wpilog`, then replay the dashboard, read the match report, render an MP4, and
link the match's YouTube video so it plays in sync. It reuses the desktop app's own log reader and dashboard renderer
(`gui/scope2429/`), so what you see here is what Scope2429 shows.

It can be embedded in the team website with an `<iframe>`.

## How it is hosted

```
browser --> Cloudflare (login via Access) --> Cloudflare Tunnel --> cloudflared container --> scope container (this app) --> /data on the NAS
                                                                                                         \--> Cloudflare R2 (rendered MP4s, optional)
```

- The server runs in Docker on the TrueNAS. The tunnel dials **out**, so the router needs no open ports.
- Cloudflare Access is the login: only the email addresses you allow get in. The app checks Cloudflare's signed token itself, so it is
  not open even if someone reaches the container directly.
- Cost: a domain name (about $10/year). Tunnel, Access (up to 50 users) and R2 (10 GB free) are free at team scale.

## Setup

1. **Domain.** Put a domain on Cloudflare (register one there or move DNS). Example: `yourteam.org`.
2. **Tunnel.** Zero Trust > Networks > Tunnels > Create (Cloudflared). Copy the token. Add a public hostname, for example
   `scope.yourteam.org`, service `HTTP`, URL `scope:8000`.
3. **Access.** Zero Trust > Access > Applications > Add > Self-hosted, domain `scope.yourteam.org`, with a policy that allows your team's
   email addresses. Copy the application's **AUD tag**, and your **team domain** (the `<team>` of `<team>.cloudflareaccess.com`).
4. **R2 (optional).** R2 > create a private bucket, then Manage API tokens > create a token with Object Read & Write on that bucket.
   Without R2 the MP4s stay on the NAS.
5. **NAS.** Create a dataset for the data. The container runs as user `1000`, so that user must be able to write to it
   (or change `user:` in `docker-compose.yml`).
6. **Configure.** `cp .env.example .env` and fill it in (`TUNNEL_TOKEN`, `CF_ACCESS_TEAM_DOMAIN`, `CF_ACCESS_AUD`, `DATA_PATH`, `R2_*`,
   `FRAME_ANCESTORS`). `.env` holds secrets and is git-ignored.
7. **Run.** From this folder: `docker compose up -d --build`. Then open `https://scope.yourteam.org`.

To update: `git pull` and `docker compose up -d --build`. To back up: copy the `DATA_PATH` dataset (logs and metadata; the MP4s can be
re-rendered).

Without Cloudflare Access you can set `TEAM_PASSWORD` instead (one shared password). The server refuses to start with no login
configured, unless `ALLOW_NO_AUTH=1` (local development only).

## Embedding in the team website

```html
<iframe src="https://scope.yourteam.org/?embed=1" width="100%" height="800" style="border:0"></iframe>
<!-- one match: -->
<iframe src="https://scope.yourteam.org/view/<match id>?embed=1" width="100%" height="800" style="border:0"></iframe>
```

- Put the site that hosts the iframe in `FRAME_ANCESTORS` (space separated). Only those sites may embed it.
- Embed from a site on the **same parent domain** (`www.yourteam.org` showing `scope.yourteam.org`). Browsers block third-party
  cookies, so a different parent domain (for example a Google Sites page) may show the Cloudflare login inside the frame and fail.
- Visitors still have to be allowed by the Access policy.

## Using it

- **Upload** a `.wpilog` (optionally its `console_*.log`) on the home page. Identical files are stored once.
- **Replay:** slider, play/pause, speed, quality; arrow keys and space work.
- **Report:** the same report Scope2429 produces.
- **MP4:** renders the dashboard in the background (one render at a time). When R2 is configured the file is copied there and served from a
  short-lived link.
- **YouTube:** paste the match video's link in the details form and set the offset (seconds into the video where the log's t=0 is). The
  video then plays in sync in the viewer. It is linked rather than baked into the MP4, because servers are often blocked from downloading
  YouTube videos.

## Development

```
pip install -r requirements.txt pytest httpx
ALLOW_NO_AUTH=1 uvicorn scope_web.app:create_app --factory --reload
python -m pytest tests -q
```

The desktop modules are found at `../scope2429` (override with `SCOPE2429_DIR`).

## Status and limits

- Tested: the server and its API with pytest (synthetic logs, fake core renderer, fake S3), and manually against a real robot log
  (replay, report and an MP4 render). **Not tested:** the Dockerfile, `docker-compose.yml`, the Cloudflare Tunnel/Access setup, real R2, and
  iframe behaviour behind Access. Expect to fix small things on first deployment.
- MP4s are the dashboard only (no match video baked in).
- No hoot (CTRE signal) analysis in the web version yet.
- Replay streams JPEG frames, so smoothness depends on the viewer's connection; lower the quality setting if it stutters.
