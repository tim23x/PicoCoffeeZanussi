// The web interface, served at /. One self-contained page: it polls /api/status every second,
// draws the temperature graph from /api/history, and builds the settings form from
// /api/settings.

#pragma once

static const char WEB_PAGE[] = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Zanussi Controller</title>
<style>
:root {
  --bg: #f6f5f2; --card: #ffffff; --text: #1d1b18; --muted: #6b665e; --line: #e2dfd8;
  --accent: #b5541c; --temp: #c2410c; --target: #6b665e; --power: #2563eb;
  --ok: #15803d; --bad: #b91c1c; --warn: #a16207;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #161412; --card: #211e1b; --text: #ece8e1; --muted: #a39d93; --line: #35312c;
    --accent: #f08a4b; --temp: #fb923c; --target: #a39d93; --power: #60a5fa;
    --ok: #4ade80; --bad: #f87171; --warn: #facc15;
  }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--text);
  font: 15px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 960px; margin: 0 auto; padding: 16px; }
header { display: flex; align-items: baseline; justify-content: space-between; gap: 12px; margin-bottom: 12px; }
h1 { font-size: 20px; margin: 0; }
h2 { font-size: 15px; margin: 0 0 10px; color: var(--muted); font-weight: 600; }
.conn { font-size: 13px; color: var(--muted); }
.conn b { color: var(--ok); font-weight: 600; }
.conn b.off { color: var(--bad); }
.card { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 14px; margin-bottom: 14px; }
.state { font-size: 28px; font-weight: 700; letter-spacing: .02em; }
.fault { display: none; margin-top: 6px; color: var(--bad); font-weight: 600; }
.grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 10px; margin-top: 12px; }
.stat .k { font-size: 12px; color: var(--muted); }
.stat .v { font-size: 22px; font-weight: 600; font-variant-numeric: tabular-nums; }
.bar { height: 6px; background: var(--line); border-radius: 3px; margin-top: 4px; overflow: hidden; }
.bar div { height: 100%; width: 0; background: var(--power); }
.chips { display: flex; gap: 6px; flex-wrap: wrap; margin-top: 4px; }
.chip { font-size: 12px; padding: 2px 8px; border-radius: 10px; border: 1px solid var(--line); color: var(--muted); }
.chip.on { background: var(--accent); border-color: var(--accent); color: #fff; }
canvas { width: 100%; height: 260px; display: block; }
.legend { display: flex; gap: 16px; flex-wrap: wrap; font-size: 12px; color: var(--muted); margin-top: 6px; }
.legend i { display: inline-block; width: 14px; height: 3px; vertical-align: middle; margin-right: 5px; }
form { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 10px 20px; }
.group { grid-column: 1 / -1; font-size: 12px; text-transform: uppercase; letter-spacing: .06em; color: var(--muted); margin-top: 6px; }
label { display: grid; grid-template-columns: 1fr 110px 28px; align-items: center; gap: 8px; }
label small { display: block; color: var(--muted); font-size: 12px; }
input { width: 100%; padding: 6px 8px; font: inherit; color: var(--text); background: var(--bg);
  border: 1px solid var(--line); border-radius: 6px; font-variant-numeric: tabular-nums; }
input:invalid { border-color: var(--bad); }
.unit { color: var(--muted); font-size: 13px; }
.actions { display: flex; gap: 10px; align-items: center; margin-top: 14px; flex-wrap: wrap; }
button { font: inherit; padding: 7px 16px; border-radius: 6px; border: 1px solid var(--accent); cursor: pointer;
  background: var(--accent); color: #fff; }
button.secondary { background: transparent; color: var(--text); border-color: var(--line); }
#msg { font-size: 13px; color: var(--muted); }
#relaytest { display: none; border-color: var(--warn); }
#relaytest p { margin: 0 0 12px; color: var(--muted); font-size: 13px; }
.relays { display: flex; gap: 10px; flex-wrap: wrap; }
.relays button { min-width: 150px; background: transparent; color: var(--text); border-color: var(--line); }
.relays button.on { background: var(--accent); border-color: var(--accent); color: #fff; }
#msg.err { color: var(--bad); }
</style>
</head>
<body>
<main>
  <header>
    <h1>Zanussi ZCOF637X Coffee Machine Controller</h1>
    <span class="conn" id="conn">Connecting&hellip;</span>
  </header>

  <section class="card">
    <div class="state" id="state">&ndash;</div>
    <div class="fault" id="fault"></div>
    <div class="grid">
      <div class="stat"><div class="k">Temperature</div><div class="v" id="temp">&ndash;</div></div>
      <div class="stat"><div class="k">Target</div><div class="v" id="target">&ndash;</div></div>
      <div class="stat"><div class="k">Heater power</div><div class="v" id="power">&ndash;</div><div class="bar"><div id="powerbar"></div></div></div>
      <div class="stat"><div class="k">Time left</div><div class="v" id="left">&ndash;</div></div>
      <div class="stat"><div class="k">Outputs</div><div class="chips">
        <span class="chip" id="o-pump">Pump</span><span class="chip" id="o-valve">Valve</span><span class="chip" id="o-grinder">Grinder</span>
      </div></div>
      <div class="stat"><div class="k">PID terms</div><div class="v" id="pid" style="font-size:14px">&ndash;</div></div>
    </div>
  </section>

  <section class="card" id="relaytest">
    <h2>Relay test</h2>
    <p id="relaynote"></p>
    <div class="relays" id="relays"></div>
  </section>

  <section class="card">
    <h2>Last 15 minutes</h2>
    <canvas id="graph"></canvas>
    <div class="legend">
      <span><i style="background:var(--temp)"></i>Temperature</span>
      <span><i style="background:var(--target)"></i>Target</span>
      <span><i style="background:var(--power);opacity:.5"></i>Heater power (right axis)</span>
    </div>
  </section>

  <section class="card">
    <h2>Settings</h2>
    <form id="settings"></form>
    <div class="actions">
      <button type="button" id="save">Save</button>
      <button type="button" class="secondary" id="reset">Restore defaults</button>
      <span id="msg"></span>
    </div>
  </section>
</main>

<script>
const $ = id => document.getElementById(id);
const HISTORY_LEN = 900;  // 15 min at one point a second, matches the controller
let hist = { temp: [], target: [], power: [] };

async function getJSON(url, opts) {
  const r = await fetch(url, opts);
  const text = await r.text();
  if (!r.ok) throw new Error(text || r.statusText);
  return JSON.parse(text);
}

function fmt(v, unit, dp = 1) { return v == null ? '–' : v.toFixed(dp) + unit; }

function showStatus(s) {
  $('state').textContent = s.state;
  $('fault').style.display = s.fault ? 'block' : 'none';
  $('fault').textContent = 'Fault: ' + s.fault;
  $('temp').textContent = s.temp_valid ? fmt(s.temp, ' °C') : 'no reading';
  $('target').textContent = s.target > 0 ? fmt(s.target, ' °C', 0) : 'off';
  $('power').textContent = fmt(s.power, '%', 0);
  $('powerbar').style.width = s.power + '%';
  $('left').textContent = s.time_left > 0 ? fmt(s.time_left, ' s') : '–';
  $('pid').textContent = s.target > 0
    ? `P ${s.p.toFixed(2)}  I ${s.i.toFixed(2)}  ${s.rate >= 0 ? '+' : ''}${s.rate.toFixed(2)} °C/s` : '–';
  for (const k of ['pump', 'valve', 'grinder']) $('o-' + k).classList.toggle('on', s[k]);
  showRelays(s);
  $('conn').innerHTML = `<b>Online</b> &middot; signal ${s.rssi} dBm &middot; up ${uptime(s.uptime)}`;
}

// Relay test mode: one toggle per relay. The controller switches each one off after its time
// limit, and all of them as soon as this page stops polling.
function showRelays(s) {
  $('relaytest').style.display = s.test_mode ? 'block' : 'none';
  if (!s.test_mode) return;
  $('relaynote').textContent = `Each relay switches off after ${s.test_limit_s} s, ` +
    'and all of them within 3 s of closing this page. The heater stays off.';
  const box = $('relays');
  if (box.children.length !== s.relays.length) {
    box.innerHTML = '';
    for (const r of s.relays) {
      const b = document.createElement('button');
      b.type = 'button';
      b.id = 'r-' + r.key;
      b.onclick = () => switchRelay(r.key, !b.classList.contains('on'));
      box.appendChild(b);
    }
  }
  for (const r of s.relays) {
    const b = $('r-' + r.key);
    b.classList.toggle('on', r.on);
    b.textContent = r.on ? `${r.name}: ON (${Math.ceil(r.left)} s)` : `${r.name}: OFF`;
  }
}

async function switchRelay(key, on) {
  try {
    showStatus(await getJSON('/api/relay', { method: 'POST', body: new URLSearchParams({ relay: key, on: on ? 1 : 0 }) }));
  } catch (e) { alert(e.message); }
}

function uptime(sec) {
  const h = Math.floor(sec / 3600), m = Math.floor(sec / 60) % 60;
  return h ? `${h} h ${m} min` : `${m} min`;
}

function push(s) {
  hist.temp.push(s.temp_valid ? s.temp : null);
  hist.target.push(s.target > 0 ? s.target : null);
  hist.power.push(s.power);
  for (const k in hist) if (hist[k].length > HISTORY_LEN) hist[k].shift();
}

function draw() {
  const c = $('graph'), dpr = window.devicePixelRatio || 1;
  const w = c.clientWidth, h = c.clientHeight;
  c.width = w * dpr; c.height = h * dpr;
  const g = c.getContext('2d');
  g.scale(dpr, dpr);
  const css = getComputedStyle(document.documentElement), col = n => css.getPropertyValue(n).trim();
  const L = 44, R = 40, T = 8, B = 22, pw = w - L - R, ph = h - T - B;
  const n = hist.temp.length;

  const vals = hist.temp.concat(hist.target).filter(v => v != null);
  let lo = vals.length ? Math.min(...vals) : 20, hi = vals.length ? Math.max(...vals) : 100;
  lo = Math.floor((lo - 3) / 10) * 10; hi = Math.ceil((hi + 3) / 10) * 10;
  if (hi - lo < 20) hi = lo + 20;
  const step = (hi - lo) > 80 ? 20 : 10;
  const x = i => L + pw * (1 - (n - 1 - i) / (HISTORY_LEN - 1));
  const y = v => T + ph * (1 - (v - lo) / (hi - lo));
  const yp = p => T + ph * (1 - p / 100);

  g.font = '11px system-ui, sans-serif';
  g.strokeStyle = col('--line'); g.fillStyle = col('--muted'); g.lineWidth = 1;
  g.textAlign = 'right'; g.textBaseline = 'middle';
  for (let v = lo; v <= hi; v += step) {
    g.beginPath(); g.moveTo(L, y(v)); g.lineTo(L + pw, y(v)); g.stroke();
    g.fillText(v + '°', L - 6, y(v));
  }
  g.textAlign = 'left';
  for (const p of [0, 50, 100]) g.fillText(p + '%', L + pw + 6, yp(p));
  g.textAlign = 'center'; g.textBaseline = 'top';
  for (const m of [15, 10, 5, 0]) g.fillText(m ? `-${m} min` : 'now', L + pw * (1 - m / 15), T + ph + 6);
  if (!n) return;

  g.fillStyle = col('--power'); g.globalAlpha = 0.18;
  g.beginPath(); g.moveTo(x(0), yp(0));
  for (let i = 0; i < n; i++) g.lineTo(x(i), yp(hist.power[i]));
  g.lineTo(x(n - 1), yp(0)); g.closePath(); g.fill();
  g.globalAlpha = 1;

  const line = (arr, colour, width, dash) => {
    g.strokeStyle = colour; g.lineWidth = width; g.setLineDash(dash);
    g.beginPath();
    let pen = false;
    for (let i = 0; i < n; i++) {
      if (arr[i] == null) { pen = false; continue; }
      pen ? g.lineTo(x(i), y(arr[i])) : g.moveTo(x(i), y(arr[i]));
      pen = true;
    }
    g.stroke(); g.setLineDash([]);
  };
  line(hist.target, col('--target'), 1.5, [5, 4]);
  line(hist.temp, col('--temp'), 2, []);
}

async function poll() {
  try {
    const s = await getJSON('/api/status');
    showStatus(s); push(s); draw();
  } catch (e) {
    $('conn').innerHTML = '<b class="off">Offline</b>';
  }
  setTimeout(poll, 1000);
}

let fields = [];
function buildForm(data) {
  fields = data.fields;
  const form = $('settings');
  form.innerHTML = '';
  let group = '';
  for (const f of fields) {
    if (f.group !== group) {
      group = f.group;
      form.insertAdjacentHTML('beforeend', `<div class="group">${group}</div>`);
    }
    form.insertAdjacentHTML('beforeend',
      `<label><span>${f.label}<small>${f.min}–${f.max}, default ${data.defaults[f.key]}</small></span>` +
      `<input type="number" name="${f.key}" min="${f.min}" max="${f.max}" step="${f.step}" value="${data.values[f.key]}" required>` +
      `<span class="unit">${f.unit}</span></label>`);
  }
}

function message(text, err) { $('msg').textContent = text; $('msg').className = err ? 'err' : ''; }

async function save() {
  const form = $('settings');
  if (!form.reportValidity()) return;
  try {
    buildForm(await getJSON('/api/settings', { method: 'POST', body: new URLSearchParams(new FormData(form)) }));
    message('Saved');
  } catch (e) { message(e.message, true); }
}

async function reset() {
  if (!confirm('Restore all settings to their defaults?')) return;
  try {
    buildForm(await getJSON('/api/settings/reset', { method: 'POST' }));
    message('Defaults restored');
  } catch (e) { message(e.message, true); }
}

$('save').onclick = save;
$('reset').onclick = reset;
window.addEventListener('resize', draw);

(async () => {
  try {
    const h = await getJSON('/api/history');
    hist = { temp: h.temp, target: h.target, power: h.power };
  } catch (e) {}
  draw();
  poll();
  try { buildForm(await getJSON('/api/settings')); } catch (e) { message('Could not load settings', true); }
})();
</script>
</body>
</html>
)HTML";
