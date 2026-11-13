#pragma once
#include <Arduino.h>

// Live log page served at "/". Connects to the /ws websocket, shows recent
// history on open, streams new lines, and sends commands typed in the box.
const char LOG_PAGE_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Spa Pack Log</title>
<style>
  :root { --bg:#0f1418; --panel:#182027; --text:#d8e0e6; --dim:#7d8a94; --ok:#3fb950; --bad:#f85149; --warn:#d29922; --accent:#4c9aff; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--text); font:14px/1.4 system-ui, sans-serif; display:flex; flex-direction:column; height:100vh; }
  header { display:flex; align-items:center; gap:10px; padding:10px 16px; background:var(--panel); flex-wrap:wrap; }
  h1 { font-size:16px; margin:0; flex:1; }
  .dot { width:10px; height:10px; border-radius:50%; background:var(--bad); display:inline-block; margin-right:6px; }
  .dot.on { background:var(--ok); }
  button { background:#24303a; color:var(--text); border:1px solid #33414c; border-radius:6px; padding:5px 10px; cursor:pointer; font:inherit; }
  button.active { background:var(--accent); color:#000; }
  #log { flex:1; overflow-y:auto; margin:0; padding:10px 16px; font:12.5px/1.45 ui-monospace, Consolas, monospace; white-space:pre-wrap; word-break:break-word; }
  .err { color:var(--bad); }
  .warn { color:var(--warn); }
  form { display:flex; gap:8px; padding:10px 16px; background:var(--panel); }
  input { flex:1; background:var(--bg); color:var(--text); border:1px solid #33414c; border-radius:6px; padding:7px 10px; font:inherit; min-width:0; }
  .help { color:var(--dim); font-size:12px; padding:0 16px 8px; background:var(--panel); }
</style>
</head>
<body>
<header>
  <h1>Spa Pack Log</h1>
  <span><span id="dot" class="dot"></span><span id="status">Connecting...</span></span>
  <button id="pause">Pause</button>
  <button id="clear">Clear</button>
</header>
<pre id="log"></pre>
<form id="cmd">
  <input id="input" placeholder="Command" autocomplete="off">
  <button type="submit">Send</button>
</form>
<div class="help">Commands: r = restart &middot; h8 = filter start hour 8 &middot; m30 = filter start minute 30</div>
<script>
const log = document.getElementById('log'), dot = document.getElementById('dot'), statusEl = document.getElementById('status');
const pauseBtn = document.getElementById('pause');
let ws, paused = false, held = [];
const MAX_LINES = 2000;

function addLine(text) {
  const div = document.createElement('div');
  if (/error/i.test(text)) div.className = 'err';
  else if (/lost|retry|bad|down|fail/i.test(text)) div.className = 'warn';
  div.textContent = text;
  log.appendChild(div);
  while (log.childNodes.length > MAX_LINES) log.removeChild(log.firstChild);
}
function addLines(text) {
  const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 30;
  text.split('\n').forEach(l => { if (l.length) addLine(l); });
  if (atBottom) log.scrollTop = log.scrollHeight;
}
function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => { dot.classList.add('on'); statusEl.textContent = 'Live'; };
  ws.onclose = () => { dot.classList.remove('on'); statusEl.textContent = 'Disconnected, retrying...'; setTimeout(connect, 2000); };
  ws.onmessage = e => { if (paused) held.push(e.data); else addLines(e.data); };
}
pauseBtn.onclick = () => {
  paused = !paused;
  pauseBtn.textContent = paused ? 'Resume' : 'Pause';
  pauseBtn.classList.toggle('active', paused);
  if (!paused) { held.forEach(addLines); held = []; }
};
document.getElementById('clear').onclick = () => { log.innerHTML = ''; };
document.getElementById('cmd').onsubmit = e => {
  e.preventDefault();
  const input = document.getElementById('input');
  if (input.value && ws.readyState === 1) { ws.send(input.value); input.value = ''; }
};
connect();
</script>
</body>
</html>
)rawliteral";
