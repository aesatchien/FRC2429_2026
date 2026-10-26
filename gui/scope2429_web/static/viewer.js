"use strict";
const $ = (id) => document.getElementById(id);
const logId = location.pathname.split("/").filter(Boolean).pop();
const embed = new URLSearchParams(location.search).get("embed") === "1";
if (embed) document.body.classList.add("embed");
$("back").href = "/" + (embed ? "?embed=1" : "");

let meta = {}, t = 0, playing = false, loading = false, pending = false, lastWall = 0, frameWidth = 1280, ytPlayer = null, ytReady = false;
const api = (p) => `/api/logs/${logId}${p}`;

// ---------------------------------------------------------------- the replay: dashboard frames drawn by the server
function requestFrame() {
  if (loading) { pending = true; return; }
  loading = true;
  const img = $("stage");
  const done = () => { loading = false; if (pending) { pending = false; requestFrame(); } else if (playing) tick(); };
  img.onload = done; img.onerror = done;
  img.src = api(`/frame.jpg?t=${t.toFixed(2)}&w=${frameWidth}`);
}
function show(tt) {
  t = Math.max(meta.t_min, Math.min(meta.t_max, tt));
  $("slider").value = t;
  $("clock").textContent = t.toFixed(1) + " s";
  requestFrame();
}
function tick() {
  const now = performance.now();
  t += ((now - lastWall) / 1000) * parseFloat($("rate").value);
  lastWall = now;
  if (t >= meta.t_max) { stop(); show(meta.t_max); return; }
  show(t);
}
function play() {
  if (t >= meta.t_max - 0.2) t = meta.t_min;
  playing = true; lastWall = performance.now();
  $("play").innerHTML = "&#10074;&#10074; Pause";
  ytSeek(); if (ytReady) ytPlayer.playVideo();
  tick();
}
function stop() {
  playing = false; $("play").innerHTML = "&#9654; Play";
  if (ytReady) ytPlayer.pauseVideo();
}
$("play").addEventListener("click", () => (playing ? stop() : play()));
$("slider").addEventListener("input", (e) => { show(parseFloat(e.target.value)); });
$("slider").addEventListener("change", () => ytSeek());
$("quality").addEventListener("change", (e) => { frameWidth = parseInt(e.target.value, 10); show(t); });
document.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT" && e.target.type !== "range") return;
  if (e.key === " ") { e.preventDefault(); playing ? stop() : play(); }
  if (e.key === "ArrowLeft") show(t - (e.shiftKey ? 10 : 1));
  if (e.key === "ArrowRight") show(t + (e.shiftKey ? 10 : 1));
});

// ---------------------------------------------------------------- the YouTube match video, kept in step with the slider
function ytId(url) {
  const m = /(?:youtube\.com\/(?:watch\?v=|embed\/|live\/)|youtu\.be\/)([A-Za-z0-9_-]{11})/.exec(url || "");
  return m ? m[1] : "";
}
function ytSeek() { if (ytReady) ytPlayer.seekTo(Math.max(0, (meta.offset || 0) + t), true); }
function setupYoutube() {
  const id = ytId(meta.youtube_url);
  $("yt").hidden = !id;
  if (!id) return;
  const start = Math.max(0, Math.floor((meta.offset || 0) + Math.max(0, t)));
  $("yt").src = `https://www.youtube.com/embed/${id}?enablejsapi=1&start=${start}&rel=0&playsinline=1&origin=${encodeURIComponent(location.origin)}`;
  const boot = () => { ytPlayer = new YT.Player("yt", { events: { onReady: () => { ytReady = true; } } }); };
  if (window.YT && window.YT.Player) boot();
  else { window.onYouTubeIframeAPIReady = boot; const s = document.createElement("script"); s.src = "https://www.youtube.com/iframe_api"; document.head.appendChild(s); }
}

// ---------------------------------------------------------------- tabs
document.querySelectorAll(".tabs button").forEach((b) => b.addEventListener("click", () => {
  document.querySelectorAll(".tabs button").forEach((x) => { x.classList.toggle("on", x === b); x.classList.toggle("secondary", x !== b); });
  document.querySelectorAll(".pane").forEach((p) => p.classList.toggle("on", p.id === "pane-" + b.dataset.pane));
  if (b.dataset.pane === "report" && !$("reportframe").src) $("reportframe").src = api("/report");
  if (b.dataset.pane !== "replay") stop();
}));

// ---------------------------------------------------------------- the MP4
function videoPane() {
  const ready = (meta.mp4 || {}).status === "ready";
  $("videobox").innerHTML = ready
    ? `<video controls preload="metadata" src="${api("/video")}"></video><div class="row"><a class="btn" href="${api("/video?download=1")}">Download MP4</a></div>`
    : 'No MP4 yet. Use <b>Render MP4</b> on the right.';
}
async function pollRender() {
  const r = await fetch(api("/render"));
  if (!r.ok) return;
  const s = await r.json();
  const busy = s.status === "queued" || s.status === "rendering";
  $("prog").hidden = !busy; $("prog").value = s.progress || 0;
  $("cancel").hidden = !busy; $("render").disabled = busy;
  $("rendermsg").className = "msg " + (s.status === "ready" ? "ok" : s.status === "failed" ? "bad" : "dim");
  $("rendermsg").textContent = busy ? `${s.status}: ${s.message || ""} ${Math.round((s.progress || 0) * 100)}%`
    : s.status === "ready" ? `Ready (${(s.bytes / 1048576).toFixed(1)} MB, rendered in ${s.seconds}s)`
    : s.status === "failed" ? `Failed: ${s.error || ""}` : s.status === "cancelled" ? "Cancelled" : "";
  if (busy) setTimeout(pollRender, 2000);
  else if (s.status === "ready" && (meta.mp4 || {}).status !== "ready") { meta = await (await fetch(api(""))).json(); videoPane(); }
}
$("render").addEventListener("click", async () => { await fetch(api("/render"), { method: "POST" }); pollRender(); });
$("cancel").addEventListener("click", async () => { await fetch(api("/render"), { method: "DELETE" }); });

// ---------------------------------------------------------------- details
$("save").addEventListener("click", async () => {
  const body = { title: $("titlein").value, youtube_url: $("yturl").value.trim(), offset: parseFloat($("ytoffset").value) || 0, red_left: $("redleft").checked };
  const r = await fetch(api(""), { method: "PATCH", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  const j = await r.json().catch(() => ({}));
  $("savemsg").className = "msg " + (r.ok ? "ok" : "bad");
  $("savemsg").textContent = r.ok ? "Saved" : (j.detail || "Could not save");
  if (r.ok) { const redChanged = meta.red_left !== j.red_left; meta = j; $("title").textContent = j.title; setupYoutube(); if (redChanged) show(t); }
});
$("del").addEventListener("click", async () => {
  if (!confirm("Delete this log and its rendered video?")) return;
  const r = await fetch(api(""), { method: "DELETE" });
  if (r.ok) location.href = "/" + (embed ? "?embed=1" : "");
});

(async function init() {
  const r = await fetch(api(""));
  if (!r.ok) { $("title").textContent = "No such log"; return; }
  meta = await r.json();
  document.title = meta.title || "Scope2429 replay";
  $("title").textContent = meta.title || meta.filename;
  $("titlein").value = meta.title || ""; $("yturl").value = meta.youtube_url || ""; $("ytoffset").value = meta.offset || 0; $("redleft").checked = meta.red_left !== false;
  $("slider").min = meta.t_min; $("slider").max = meta.t_max;
  videoPane(); setupYoutube(); pollRender();
  show(0);
})();
