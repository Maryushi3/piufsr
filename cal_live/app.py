#!/usr/bin/env python3
"""Live FSR meters for the PIUFSR master (teejusb/fsr-style).

One vertical meter per FSR, grouped 5 panels x 4 sensors: current reading as
a bar, press and release threshold lines on the meter, -/+ steppers and number
fields per threshold, and click-and-drag on a meter to set the nearest
threshold. A History tab draws a scrolling line plot per panel.

Uses the same serial contract as cal_web: the master's 20 Hz 'c' stream for
readings, 't'/'q' to read press/release thresholds, 's <i> <v>' / 'e <i> <v>'
to set them, 'o' to zero offsets, bare 's' to save to EEPROM.

Binds to localhost by default. /cmd can zero and permanently save calibration
to every slave's EEPROM, so exposing it on a network means anyone who can
reach the port can overwrite the pad's calibration.
"""
import argparse
import json
import math
import random
import threading
import time

import serial
from flask import Flask, Response, jsonify, request


def _is_int(x):
    """int, but not bool — bool is an int subclass, so {"idx": true} would
    otherwise validate and put the literal 's True 5' on the wire."""
    return isinstance(x, int) and not isinstance(x, bool)


NUM_PANELS = 5
FSRS_PER_PANEL = 4
NUM_SENSORS = NUM_PANELS * FSRS_PER_PANEL
DEFAULT_BAUD = 115200
DEFAULT_HTTP_PORT = 8767
# Match the slave firmware's defaults, so a UI that cannot reach the hardware
# still shows plausible values instead of made-up ones.
FIRMWARE_DEFAULT_THRESHOLD = 125
FIRMWARE_DEFAULT_RELEASE_THRESHOLD = 20
THRESHOLD_TIMEOUT = 3.0
STREAM_PERIOD = 0.05

HTML = """\
<!doctype html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PIUFSR FSR Meters</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; background: #14161a; color: #ddd;
         font-family: system-ui, sans-serif; }
  header { display: flex; align-items: center; gap: 10px; padding: 10px 16px;
           background: #1d2025; border-bottom: 1px solid #2c3038; flex-wrap: wrap; }
  header h1 { font-size: 16px; margin: 0; }
  .tab { background: none; border: 1px solid #3a3f48; color: #ccc; padding: 4px 12px;
         cursor: pointer; border-radius: 4px; }
  .tab.active { background: #2a7; border-color: #2a7; color: #fff; }
  header button { font-size: 13px; padding: 3px 10px; }
  #link { width: 10px; height: 10px; border-radius: 50%; background: #777;
          display: inline-block; margin-left: auto; }
  #link.on { background: #3f6; }
  #link.off { background: #f55; }
  #pwrap { padding: 14px; display: grid;
           grid-template-columns: repeat(auto-fit, minmax(340px, 1fr));
           gap: 14px; }
  .panel { background: #1b1e24; border: 1px solid #2c3038; border-radius: 8px;
           padding: 10px; }
  .panel.act { border-color: #39ff14; box-shadow: 0 0 10px #39ff1433; }
  .phdr { display: flex; justify-content: space-between; align-items: center;
          font-weight: bold; }
  .badge { display: none; color: #39ff14; border: 1px solid #39ff14;
           padding: 1px 6px; border-radius: 3px; font-size: 11px; }
  .panel.act .badge { display: inline; }
  .mrow { display: flex; gap: 8px; margin-top: 8px; }
  .meter { flex: 1; text-align: center; min-width: 0; }
  .readout .big { font-size: 19px; font-weight: bold; font-variant-numeric: tabular-nums; }
  .readout .cap { font-size: 10px; color: #888; }
  canvas.m { width: 64px; height: 230px; touch-action: none; display: block;
             margin: 4px auto 0; background: #22252b; border-radius: 3px; }
  .ctrl { display: flex; justify-content: center; align-items: center; gap: 3px;
          margin-top: 3px; }
  .ctrl button { padding: 1px 6px; line-height: 1.1; }
  .ctrl input { width: 42px; font-size: 11px; text-align: center;
                background: #0e1013; color: #ddd; border: 1px solid #3a3f48; }
  .ctrl .tag { font-size: 10px; color: #888; width: 12px; }
  #hwrap { display: none; padding: 14px; }
  .plot { background: #0b0d10; border: 1px solid #2c3038; border-radius: 8px;
          margin-bottom: 14px; padding: 8px; }
  .plot h3 { margin: 0 0 4px; font-size: 13px; }
  .legend span { font-size: 11px; margin-right: 12px; }
  canvas.hp { width: 100%; height: 180px; }
</style>
</head>
<body>
<header>
  <h1>PIUFSR FSR Meters</h1>
  <button class="tab active" id="tab-meters" onclick="showView('meters')">Meters</button>
  <button class="tab" id="tab-plot" onclick="showView('plot')">History</button>
  <span style="flex:1"></span>
  <button onclick="doCmd('zero')">Zero offsets</button>
  <button onclick="doCmd('save')">Save to EEPROM</button>
  <button onclick="doCmd('refresh')">Reload thresholds</button>
  <span id="link" class="off"></span>
</header>
<div id="pwrap"></div>
<div id="hwrap"></div>
<script>
const NP = 5, N = 20, MAX = 400;
const PAL = ['#4fc3f7', '#ffb74d', '#81c784', '#e57373'];
let vals = new Array(N).fill(0);
let press = new Array(N).fill(125);
let rel = new Array(N).fill(20);
let hist = [];
let dragIdx = -1, dragKind = null, dragging = false;
let histVisible = false;

const $ = id => document.getElementById(id);

function clamp(x) { x = Math.round(x); return Math.max(0, Math.min(255, x)); }
function cur(idx, kind) { return kind === 'press' ? press[idx] : rel[idx]; }
function setCur(idx, kind, v) { if (kind === 'press') press[idx] = v; else rel[idx] = v; }
function syncField(idx, kind) {
  const f = $((kind === 'press' ? 'p' : 'r') + 'in' + idx);
  if (f && document.activeElement !== f) f.value = cur(idx, kind);
}

function doCmd(cmd, extra) {
  const body = Object.assign({ cmd: cmd }, extra || {});
  fetch('/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                  body: JSON.stringify(body) }).catch(() => {});
}
function commit(idx, kind, v) { doCmd('', { kind: kind, idx: idx, val: v }); }

function step(idx, kind, d) {
  const v = clamp(cur(idx, kind) + d);
  setCur(idx, kind, v);
  syncField(idx, kind);
  commit(idx, kind, v);
  drawMeter(idx);
}

function makeCtrl(idx, kind) {
  const d = document.createElement('div'); d.className = 'ctrl';
  const minus = document.createElement('button'); minus.textContent = '-';
  const inp = document.createElement('input');
  inp.type = 'number'; inp.min = 0; inp.max = 255;
  inp.id = (kind === 'press' ? 'p' : 'r') + 'in' + idx;
  const plus = document.createElement('button'); plus.textContent = '+';
  const tag = document.createElement('span'); tag.className = 'tag';
  tag.textContent = kind === 'press' ? 'P' : 'R';
  minus.onclick = () => step(idx, kind, -1);
  plus.onclick = () => step(idx, kind, 1);
  inp.onchange = () => {
    const v = parseInt(inp.value, 10);
    if (!isNaN(v) && v >= 0 && v <= 255) { commit(idx, kind, v); }
    else { syncField(idx, kind); }
  };
  d.appendChild(minus); d.appendChild(inp); d.appendChild(plus); d.appendChild(tag);
  return d;
}

function attachDrag(cv, idx) {
  cv.addEventListener('pointerdown', e => {
    e.preventDefault();
    const r = cv.getBoundingClientRect();
    const v = clamp(255 - (e.clientY - r.top) / r.height * 255);
    dragIdx = idx;
    dragKind = Math.abs(v - press[idx]) <= Math.abs(v - rel[idx]) ? 'press' : 'release';
    setCur(idx, dragKind, v);
    syncField(idx, dragKind);
    drawMeter(idx);
    dragging = true;
    cv.setPointerCapture(e.pointerId);
  });
  cv.addEventListener('pointermove', e => {
    if (!dragging || dragIdx !== idx) return;
    const r = cv.getBoundingClientRect();
    const v = clamp(255 - (e.clientY - r.top) / r.height * 255);
    setCur(idx, dragKind, v);
    syncField(idx, dragKind);
    drawMeter(idx);
  });
  cv.addEventListener('pointerup', e => {
    if (!dragging || dragIdx !== idx) return;
    const r = cv.getBoundingClientRect();
    const v = clamp(255 - (e.clientY - r.top) / r.height * 255);
    setCur(idx, dragKind, v);
    syncField(idx, dragKind);
    commit(idx, dragKind, v);
    dragging = false; dragIdx = -1;
  });
}

function line(x, val, color, label) {
  const W = 64, H = 230;
  const y = H - (val / 255) * H;
  x.strokeStyle = color; x.lineWidth = 2;
  x.beginPath(); x.moveTo(0, y); x.lineTo(W, y); x.stroke();
  x.fillStyle = color; x.font = '10px system-ui'; x.textAlign = 'left';
  x.fillText(label, 2, y > 12 ? y - 2 : y + 10);
}

function drawMeter(idx) {
  const cv = $('c' + idx); if (!cv) return;
  const dpr = window.devicePixelRatio || 1, W = 64, H = 230;
  if (cv.width !== W * dpr || cv.height !== H * dpr) { cv.width = W * dpr; cv.height = H * dpr; }
  const x = cv.getContext('2d'); x.setTransform(dpr, 0, 0, dpr, 0, 0);
  x.clearRect(0, 0, W, H);
  const v = clamp(vals[idx]), p = press[idx], r = rel[idx];
  x.fillStyle = v >= p ? '#1b3b57' : '#22252b';
  x.fillRect(0, 0, W, H);
  x.strokeStyle = '#33373f'; x.lineWidth = 1;
  for (let t = 1; t < 4; t++) {
    const y = H - (t * 64 / 255) * H;
    x.beginPath(); x.moveTo(0, y); x.lineTo(W, y); x.stroke();
  }
  const yv = H - (v / 255) * H;
  const g = x.createLinearGradient(0, H, 0, yv);
  g.addColorStop(0, '#ff9800'); g.addColorStop(1, '#e53935');
  x.fillStyle = g;
  x.fillRect(W * 0.25, yv, W * 0.5, H - yv);
  line(x, p, '#ffeb3b', 'P' + p);
  line(x, r, '#00e5ff', 'R' + r);
  x.fillStyle = '#fff'; x.font = 'bold 13px system-ui'; x.textAlign = 'center';
  x.fillText(String(v), W / 2, 14);
}

function drawPlots() {
  for (let p = 0; p < NP; p++) {
    const cv = $('hp' + p); if (!cv) return;
    const dpr = window.devicePixelRatio || 1;
    const W = cv.clientWidth || 600, H = 180;
    if (cv.width !== W * dpr || cv.height !== H * dpr) { cv.width = W * dpr; cv.height = H * dpr; }
    const x = cv.getContext('2d'); x.setTransform(dpr, 0, 0, dpr, 0, 0);
    x.clearRect(0, 0, W, H);
    x.strokeStyle = '#22262d'; x.lineWidth = 1;
    for (let t = 0; t <= 4; t++) {
      const y = H - (t / 4) * H;
      x.beginPath(); x.moveTo(0, y); x.lineTo(W, y); x.stroke();
    }
    for (let s = 0; s < 4; s++) {
      const idx = p * 4 + s;
      x.strokeStyle = PAL[s]; x.setLineDash([4, 4]); x.lineWidth = 1;
      const yp = H - (press[idx] / 255) * H;
      x.beginPath(); x.moveTo(0, yp); x.lineTo(W, yp); x.stroke();
      x.setLineDash([]);
      x.lineWidth = 2; x.beginPath();
      const n = hist.length;
      for (let i = 0; i < n; i++) {
        const px = n === 1 ? W : (i / (n - 1)) * W;
        const py = H - (clamp(hist[i][idx]) / 255) * H;
        if (i === 0) x.moveTo(px, py); else x.lineTo(px, py);
      }
      x.stroke();
    }
  }
}

function render() {
  for (let i = 0; i < N; i++) drawMeter(i);
  for (let p = 0; p < NP; p++) {
    let on = false;
    for (let s = 0; s < 4; s++) {
      const idx = p * 4 + s;
      if (vals[idx] >= press[idx]) on = true;
    }
    const card = $('panel' + p);
    card.className = 'panel' + (on ? ' act' : '');
  }
  if (histVisible) drawPlots();
}

function onData(msg) {
  if (Array.isArray(msg.v) && msg.v.length === N) vals = msg.v;
  if (Array.isArray(msg.p)) {
    for (let i = 0; i < N; i++) {
      if (dragIdx === i) continue;
      press[i] = msg.p[i]; syncField(i, 'press');
    }
  }
  if (Array.isArray(msg.r)) {
    for (let i = 0; i < N; i++) {
      if (dragIdx === i) continue;
      rel[i] = msg.r[i]; syncField(i, 'release');
    }
  }
  hist.push(msg.v.slice());
  if (hist.length > MAX) hist.shift();
  render();
}

function showView(v) {
  $('pwrap').style.display = v === 'meters' ? 'grid' : 'none';
  $('hwrap').style.display = v === 'plot' ? 'block' : 'none';
  $('tab-meters').className = 'tab' + (v === 'meters' ? ' active' : '');
  $('tab-plot').className = 'tab' + (v === 'plot' ? ' active' : '');
  histVisible = v === 'plot';
  if (histVisible) drawPlots();
}

(function build() {
  const pw = $('pwrap');
  for (let p = 0; p < NP; p++) {
    const card = document.createElement('div'); card.className = 'panel'; card.id = 'panel' + p;
    const hdr = document.createElement('div'); hdr.className = 'phdr';
    const t = document.createElement('span'); t.textContent = 'Panel ' + p;
    const b = document.createElement('span'); b.className = 'badge'; b.textContent = 'ACTIVE';
    hdr.appendChild(t); hdr.appendChild(b); card.appendChild(hdr);
    const row = document.createElement('div'); row.className = 'mrow';
    for (let s = 0; s < 4; s++) {
      const idx = p * 4 + s;
      const m = document.createElement('div'); m.className = 'meter';
      const ro = document.createElement('div'); ro.className = 'readout';
      const big = document.createElement('span'); big.className = 'big'; big.id = 'v' + idx;
      const cap = document.createElement('div'); cap.className = 'cap'; cap.textContent = 's' + s;
      ro.appendChild(big); ro.appendChild(cap);
      const cv = document.createElement('canvas'); cv.className = 'm'; cv.id = 'c' + idx;
      attachDrag(cv, idx);
      m.appendChild(ro); m.appendChild(cv);
      m.appendChild(makeCtrl(idx, 'press'));
      m.appendChild(makeCtrl(idx, 'release'));
      row.appendChild(m);
    }
    card.appendChild(row);
    pw.appendChild(card);
  }
  const hw = $('hwrap');
  for (let p = 0; p < NP; p++) {
    const pl = document.createElement('div'); pl.className = 'plot';
    const h = document.createElement('h3'); h.textContent = 'Panel ' + p;
    const lg = document.createElement('div'); lg.className = 'legend';
    for (let s = 0; s < 4; s++) {
      const sp = document.createElement('span'); sp.style.color = PAL[s];
      sp.textContent = 's' + s; lg.appendChild(sp);
    }
    const cv = document.createElement('canvas'); cv.className = 'hp'; cv.id = 'hp' + p;
    pl.appendChild(h); pl.appendChild(lg); pl.appendChild(cv);
    hw.appendChild(pl);
  }
  render();
})();

const evt = new EventSource('/stream');
evt.onmessage = e => { try { onData(JSON.parse(e.data)); } catch (err) {} };
evt.onopen = () => { $('link').className = 'on'; };
evt.onerror = () => { $('link').className = 'off'; };
</script>
</body>
</html>
"""

state_lock = threading.Lock()
state = {
    "values": [0] * NUM_SENSORS,
    "press": [FIRMWARE_DEFAULT_THRESHOLD] * NUM_SENSORS,
    "release": [FIRMWARE_DEFAULT_RELEASE_THRESHOLD] * NUM_SENSORS,
    # Bumped whenever thresholds change server-side, so a browser can tell a
    # new stream payload apart without being told twenty times a second.
    "seq": 0,
    "link": False,
}

ser = None
ser_lock = threading.Lock()
streaming = False
demo_mode = False
# Only the reader thread may read from the port. A request handler doing its
# own readline() would compete for incoming lines, so "re-read the
# thresholds" is passed to the reader thread as a request.
refresh_request = threading.Event()


def set_thresholds(values, kind):
    with state_lock:
        state[kind] = list(values)
        state["seq"] += 1


def read_thresholds(s, cmd, key):
    """Ask the master for live thresholds; None if it never answers.

    The master always emits NUM_SENSORS values (0 for an unreachable panel),
    so the reply can be indexed positionally.
    """
    s.write(cmd.encode())
    s.flush()
    deadline = time.time() + THRESHOLD_TIMEOUT
    while time.time() < deadline:
        line = s.readline().decode("utf-8", errors="replace").strip()
        if line.startswith(">"):
            line = line[1:].strip()
        if not line.startswith(key + " "):
            continue
        parts = line.split()
        if len(parts) != NUM_SENSORS + 1:
            continue
        try:
            return [int(p) for p in parts[1:]]
        except ValueError:
            continue
    return None


def serial_reader(port, baud):
    global ser, streaming
    try:
        s = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        s.reset_input_buffer()
    except Exception as e:
        print("Serial open failed:", e)
        return

    p = read_thresholds(s, "t\n", "t")
    if p is None:
        print("Warning: master did not report press thresholds; showing "
              f"firmware default ({FIRMWARE_DEFAULT_THRESHOLD}).")
    else:
        set_thresholds(p, "press")
        print("Press thresholds read from hardware:", p)
    r = read_thresholds(s, "q\n", "q")
    if r is None:
        print("Warning: master did not report release thresholds; showing "
              f"firmware default ({FIRMWARE_DEFAULT_RELEASE_THRESHOLD}).")
    else:
        set_thresholds(r, "release")
        print("Release thresholds read from hardware:", r)

    s.reset_input_buffer()
    s.write(b"c\n")
    time.sleep(0.2)
    s.reset_input_buffer()
    with ser_lock:
        ser = s
        streaming = True
    with state_lock:
        state["link"] = True

    while True:
        try:
            if refresh_request.is_set():
                refresh_request.clear()
                again = read_thresholds(s, "t\n", "t")
                if again is not None:
                    set_thresholds(again, "press")
                again = read_thresholds(s, "q\n", "q")
                if again is not None:
                    set_thresholds(again, "release")
                continue
            line = s.readline().decode("utf-8", errors="replace").strip()
            if not line.startswith("c "):
                continue
            parts = line.split()
            if len(parts) != NUM_SENSORS + 1:
                continue
            values = [int(p) for p in parts[1:]]
        except Exception as e:
            print("Serial read error:", e)
            break
        with state_lock:
            state["values"] = values

    with ser_lock:
        ser = None
        streaming = False
    with state_lock:
        state["link"] = False
    s.close()


def stop_streaming():
    """Toggle the master's 20 Hz stream back off on the way out."""
    global streaming
    with ser_lock:
        s = ser
        if s and s.is_open and streaming:
            try:
                s.write(b"c\n")
                s.flush()
            except Exception:
                pass
            streaming = False


def demo_reader():
    global demo_mode
    demo_mode = True
    with state_lock:
        state["link"] = True
    phase = [random.uniform(0, 2 * math.pi) for _ in range(NUM_SENSORS)]
    t = 0.0
    step_panel = 0
    while True:
        vals = []
        for i in range(NUM_SENSORS):
            pi = i // FSRS_PER_PANEL
            base = 18 + int(10 * math.sin(t * 1.3 + phase[i]))
            dist = (pi - step_panel) % NUM_PANELS
            dist = min(dist, NUM_PANELS - dist)
            press_amt = 0
            if dist == 0:
                press_amt = 150 + int(50 * math.sin(t * 4 + phase[i]))
            elif dist == 1:
                press_amt = int(35 * math.sin(t * 3 + phase[i]))
            vals.append(max(0, min(255, base + press_amt)))
        with state_lock:
            state["values"] = vals
        t += STREAM_PERIOD
        if t > 1.5:
            t = 0.0
            step_panel = (step_panel + 1) % NUM_PANELS
        time.sleep(STREAM_PERIOD)


app = Flask(__name__)


@app.route("/")
def index():
    return HTML


@app.route("/stream")
def stream():
    def generate():
        try:
            while True:
                with state_lock:
                    payload = {
                        "v": list(state["values"]),
                        "p": list(state["press"]),
                        "r": list(state["release"]),
                        "seq": state["seq"],
                        "link": state["link"],
                    }
                yield "data: " + json.dumps(payload, separators=(",", ":")) + "\n\n"
                time.sleep(STREAM_PERIOD)
        except GeneratorExit:
            return
    return Response(generate(), mimetype="text/event-stream")


@app.route("/cmd", methods=["POST"])
def cmd():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify(error="expected a JSON object"), 400

    with ser_lock:
        s = ser
        if not (s and s.is_open):
            s = None
    if s is None and not demo_mode:
        return jsonify(error="no serial link"), 503

    if data.get("cmd") == "refresh":
        if s is not None:
            refresh_request.set()
        return jsonify(ok=True), 202
    if data.get("cmd") == "zero":
        if s is not None:
            s.write(b"o\n")
            s.flush()
        return jsonify(ok=True)
    if data.get("cmd") == "save":
        if s is not None:
            s.write(b"s\n")
            s.flush()
        return jsonify(ok=True)

    kind = data.get("kind")
    if kind not in ("press", "release"):
        return jsonify(error="kind must be 'press' or 'release'"), 400
    key = "press" if kind == "press" else "release"
    prefix = "s" if kind == "press" else "e"
    all_prefix = "a" if kind == "press" else "y"

    try:
        if "all" in data:
            val = data["all"]
            if not (_is_int(val) and 0 <= val <= 255):
                return jsonify(error="all: 0-255"), 400
            if s is not None:
                s.write(f"{all_prefix} {val}\n".encode())
            with state_lock:
                state[key] = [val] * NUM_SENSORS
                state["seq"] += 1
            print(f"-> set all {key} = {val}")
        elif "idx" in data and "val" in data:
            idx, val = data["idx"], data["val"]
            if not (_is_int(idx) and 0 <= idx < NUM_SENSORS
                    and _is_int(val) and 0 <= val <= 255):
                return jsonify(error="idx 0-19, val 0-255"), 400
            if s is not None:
                s.write(f"{prefix} {idx} {val}\n".encode())
            with state_lock:
                state[key][idx] = val
                state["seq"] += 1
            print(f"-> set {key}[{idx}] = {val}")
        else:
            return jsonify(error="unrecognised command"), 400
        if s is not None:
            s.flush()
    except Exception as e:
        print("Serial write error:", e)
        return jsonify(error=str(e)), 500
    return jsonify(ok=True)


def main():
    parser = argparse.ArgumentParser(
        description="PIUFSR live FSR meters (browser)")
    parser.add_argument("serial_port", nargs="?",
                        help="Serial port of the Pro Micro master")
    parser.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--demo", action="store_true",
                        help="Synthesise readings instead of opening a port")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Interface to bind (default 127.0.0.1). /cmd can "
                             "overwrite pad calibration and has no auth, so "
                             "only widen this on a trusted network.")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT,
                        help=f"HTTP port (default {DEFAULT_HTTP_PORT})")
    args = parser.parse_args()

    if args.demo:
        print("Demo mode")
        threading.Thread(target=demo_reader, daemon=True).start()
    elif args.serial_port:
        threading.Thread(target=serial_reader,
                         args=(args.serial_port, args.baud),
                         daemon=True).start()
    else:
        parser.error("give a serial port, or --demo")

    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print(f"WARNING: binding {args.host} exposes /cmd, which can zero and "
              "permanently save this pad's calibration, to anyone who can "
              "reach this port. There is no authentication.")

    print(f"Listening on http://{args.host}:{args.http_port}")
    try:
        app.run(host=args.host, port=args.http_port, debug=False,
                use_reloader=False, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop_streaming()


if __name__ == "__main__":
    main()
