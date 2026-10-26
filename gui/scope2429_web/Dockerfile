# Build from the gui/ folder (docker-compose.yml does this):   docker build -f scope2429_web/Dockerfile .
FROM python:3.12-slim

# ffmpeg turns the rendered frames into an MP4 (Scope2429 finds it on the PATH)
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY scope2429_web/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# the desktop app's log reader and dashboard renderer (the website imports them), and the website itself
COPY scope2429 /app/scope2429
COPY scope2429_web /app/scope2429_web

# Scope2429 keeps its caches under $HOME/.scope2429, so HOME lives on the data volume
ENV SCOPE2429_DIR=/app/scope2429 DATA_DIR=/data HOME=/data/home PYTHONUNBUFFERED=1
RUN useradd --uid 1000 --no-create-home scope && mkdir -p /data && chown scope /data
USER scope
WORKDIR /app/scope2429_web
EXPOSE 8000
HEALTHCHECK --interval=30s --timeout=5s CMD python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8000/healthz')"

# --proxy-headers: the Cloudflare Tunnel is the only thing that talks to this container
CMD ["uvicorn", "scope_web.app:create_app", "--factory", "--host", "0.0.0.0", "--port", "8000", "--proxy-headers", "--forwarded-allow-ips", "*"]
