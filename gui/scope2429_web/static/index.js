"use strict";
const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
if (new URLSearchParams(location.search).get("embed") === "1") document.body.classList.add("embed");

function videoCell(m) {
  const s = (m.mp4 || {}).status || "none";
  if (s === "ready") return '<span class="ok">MP4 ready</span>';
  if (s === "rendering" || s === "queued") return '<span class="warn">rendering&hellip;</span>';
  if (s === "failed") return '<span class="bad">failed</span>';
  return '<span class="dim">&mdash;</span>';
}

async function load() {
  const r = await fetch("/api/logs");
  if (!r.ok) { $("rows").innerHTML = '<tr><td colspan="7" class="bad">Could not load the list (' + r.status + ")</td></tr>"; return; }
  const logs = await r.json();
  if (!logs.length) { $("rows").innerHTML = '<tr><td colspan="7" class="dim">No logs yet. Upload one above.</td></tr>'; return; }
  $("rows").innerHTML = logs.map((m) => {
    const color = (m.color || "").toLowerCase();
    const embed = new URLSearchParams(location.search).get("embed") === "1" ? "?embed=1" : "";
    return `<tr>
      <td><a href="/view/${esc(m.id)}${embed}"><b>${esc(m.match_label || m.title || m.filename)}</b></a></td>
      <td>${esc(m.event)}</td>
      <td><span class="pill ${color === "red" ? "red" : color === "blue" ? "blue" : ""}">${esc(m.alliance)}</span></td>
      <td>${esc(m.log_date)}</td>
      <td>${m.has_match ? esc(Math.round(m.match_len)) + " s" : '<span class="dim">no match</span>'}</td>
      <td>${videoCell(m)}</td>
      <td><a href="/view/${esc(m.id)}${embed}">Replay</a></td></tr>`;
  }).join("");
}

async function upload(file) {
  if (!file) return;
  const msg = $("upmsg");
  if (!/\.wpilog$/i.test(file.name)) { msg.className = "msg bad"; msg.textContent = "That is not a .wpilog file."; return; }
  msg.className = "msg"; msg.textContent = "Uploading " + file.name + " (" + (file.size / 1048576).toFixed(1) + " MB) and reading it…";
  const fd = new FormData();
  fd.append("log", file);
  const c = $("consolefile").files[0];
  if (c) fd.append("console", c);
  const r = await fetch("/api/upload", { method: "POST", body: fd });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { msg.className = "msg bad"; msg.textContent = j.detail || ("Upload failed (" + r.status + ")"); return; }
  msg.className = "msg ok";
  msg.innerHTML = (j.created ? "Added " : "Already had ") + esc(j.title || j.filename) + ' &mdash; <a href="/view/' + esc(j.id) + '">open the replay</a>';
  $("consolefile").value = "";
  load();
}

$("logfile").addEventListener("change", (e) => upload(e.target.files[0]));
const drop = $("drop");
["dragenter", "dragover"].forEach((ev) => drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.add("over"); }));
["dragleave", "drop"].forEach((ev) => drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.remove("over"); }));
drop.addEventListener("drop", (e) => upload(e.dataTransfer.files[0]));
load();
