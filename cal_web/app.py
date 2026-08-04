#!/usr/bin/env python3
"""Browser calibration UI for the PIUFSR master.

Binds to localhost by default. /cmd can zero and permanently save calibration
to every slave's EEPROM, so exposing it on a network means anyone who can
reach the port can overwrite the pad's calibration. Pass --host explicitly
(and only on a trusted network) if you really want that.
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
DEFAULT_HTTP_PORT = 8765
# Matches the slave firmware's kDefaultThreshold, so a UI that cannot reach
# the hardware still shows a plausible value instead of a made-up one.
FIRMWARE_DEFAULT_THRESHOLD = 125
# Matches the slave firmware's kDefaultReleaseThreshold.
FIRMWARE_DEFAULT_RELEASE_THRESHOLD = 20
THRESHOLD_TIMEOUT = 3.0

HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PIU FSR Calibration</title>
<style>
* { box-sizing: border-box; }
body { font-family: 'Segoe UI', system-ui, sans-serif; background: #0f0f1a; color: #eee; margin: 0; min-height: 100vh; display: flex; flex-direction: column; align-items: center; gap: 10px; padding: 20px; }
h1 { color: #ff6b6b; margin: 0; font-size: 22px; letter-spacing: 2px; }

.parent {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: 14px;
  max-width: 960px;
}
.div1 { grid-column: 1; grid-row: 3; }
.div2 { grid-column: 1; grid-row: 1; }
.div3 { grid-column: 2; grid-row: 2; }
.div4 { grid-column: 3; grid-row: 1; }
.div5 { grid-column: 3; grid-row: 3; }

.panel {
  background: #1a1a2e; border: 2px solid #16213e; border-radius: 8px; padding: 6px;
  display: grid;
  grid-template-areas:
    ".    up    ."
    "l   mid    r"
    ".   down   .";
  grid-template-columns: 78px 1fr 78px;
  grid-template-rows: 36px 1fr 36px;
  gap: 2px;
}
.panel.hit { border-color: #e94560; box-shadow: 0 0 10px rgba(233,69,96,0.3); }

.edge-up   { grid-area: up;   display: flex; align-items: center; justify-content: center; gap: 4px; }
.edge-down { grid-area: down; display: flex; align-items: center; justify-content: center; gap: 4px; }
.edge-l    { grid-area: l;    display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 2px; width: 78px; }
.edge-r    { grid-area: r;    display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 2px; width: 78px; }

.mid {
  grid-area: mid;
  text-align: center;
  display: flex; align-items: center; justify-content: center;
}
.mid .label { font-size: 11px; color: #888; text-transform: uppercase; letter-spacing: 1px; }

.edge-up .side, .edge-down .side { font-size: 9px; color: #555; font-weight: 700; width: 10px; text-align: center; }
.edge-l .side, .edge-r .side { font-size: 9px; color: #555; font-weight: 700; }

.v { font-variant-numeric: tabular-nums; font-weight: 700; font-size: 12px; padding: 1px 4px; border-radius: 3px; min-width: 26px; text-align: center; }
.v.idle { color: #2d8a4e; }
.v.warn { color: #ffb700; background: #2a2000; }
.v.hit  { color: #ff3333; background: #2a0000; }

/* threshold entry fields */
.thr { width: 48px; background: #0f0f1a; border: 1px solid #333; color: #eee; border-radius: 3px; font-size: 11px; text-align: center; font-variant-numeric: tabular-nums; padding: 2px 1px; }
.thr:focus { border-color: #e94560; outline: none; }

/* release-threshold fields: the lower hysteresis edge */
.rel { width: 36px; background: #0f0f1a; border: 1px solid #2b4a4a; color: #9cc; border-radius: 3px; font-size: 11px; text-align: center; font-variant-numeric: tabular-nums; padding: 2px 1px; }
.rel:focus { border-color: #35a; outline: none; }

#bar { width: 100%; max-width: 600px; height: 4px; background: #1a1a2e; border-radius: 2px; overflow: hidden; }
#bar-fill { height: 100%; width: 0%; background: #e94560; border-radius: 2px; transition: width 0.05s; }

#toolbar { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; justify-content: center; }
#toolbar button { background: #16213e; color: #ccc; border: 1px solid #0f3460; border-radius: 4px; padding: 5px 12px; font-size: 11px; cursor: pointer; }
#toolbar button:hover { background: #0f3460; color: #fff; }
#toolbar .danger { border-color: #5a0000; color: #ff6b6b; }
#toolbar .danger:hover { background: #2a0000; }
.global-row { display: flex; align-items: center; gap: 6px; margin: 0 6px; font-size: 11px; color: #888; }

#status { color: #666; font-size: 11px; display: flex; align-items: center; gap: 6px; }
#status .dot { display: inline-block; width: 6px; height: 6px; border-radius: 50%; }
#status .dot.on { background: #2d8a4e; }
#status .dot.off { background: #555; }
</style>
</head>
<body>
<h1>PIU FSR CAL</h1>

<div class="parent">

<!-- TL = div2, sensors 4-7: L=4 R=5 U=6 D=7 -->
<div class="panel div2">
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v6">000</span><input type="number" class="thr" id="s6" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r6" min="0" max="255" step="1" value="20"></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v4">000</span><input type="number" class="thr" id="s4" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r4" min="0" max="255" step="1" value="20"></div>
  <div class="mid"><span class="label">T.LFT P1</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v5">000</span><input type="number" class="thr" id="s5" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r5" min="0" max="255" step="1" value="20"></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v7">000</span><input type="number" class="thr" id="s7" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r7" min="0" max="255" step="1" value="20"></div>
</div>

<!-- TR = div4, sensors 12-15: L=12 R=13 U=14 D=15 -->
<div class="panel div4">
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v14">000</span><input type="number" class="thr" id="s14" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r14" min="0" max="255" step="1" value="20"></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v12">000</span><input type="number" class="thr" id="s12" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r12" min="0" max="255" step="1" value="20"></div>
  <div class="mid"><span class="label">T.RGT P3</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v13">000</span><input type="number" class="thr" id="s13" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r13" min="0" max="255" step="1" value="20"></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v15">000</span><input type="number" class="thr" id="s15" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r15" min="0" max="255" step="1" value="20"></div>
</div>

<!-- C = div3, sensors 8-11: L=8 R=9 U=10 D=11 -->
<div class="panel div3">
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v10">000</span><input type="number" class="thr" id="s10" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r10" min="0" max="255" step="1" value="20"></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v8">000</span><input type="number" class="thr" id="s8" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r8" min="0" max="255" step="1" value="20"></div>
  <div class="mid"><span class="label">CENTER P2</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v9">000</span><input type="number" class="thr" id="s9" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r9" min="0" max="255" step="1" value="20"></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v11">000</span><input type="number" class="thr" id="s11" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r11" min="0" max="255" step="1" value="20"></div>
</div>

<!-- BL = div1, sensors 0-3: L=0 R=1 U=2 D=3 -->
<div class="panel div1">
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v2">000</span><input type="number" class="thr" id="s2" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r2" min="0" max="255" step="1" value="20"></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v0">000</span><input type="number" class="thr" id="s0" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r0" min="0" max="255" step="1" value="20"></div>
  <div class="mid"><span class="label">B.LFT P0</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v1">000</span><input type="number" class="thr" id="s1" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r1" min="0" max="255" step="1" value="20"></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v3">000</span><input type="number" class="thr" id="s3" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r3" min="0" max="255" step="1" value="20"></div>
</div>

<!-- BR = div5, sensors 16-19: L=16 R=17 U=18 D=19 -->
<div class="panel div5">
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v18">000</span><input type="number" class="thr" id="s18" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r18" min="0" max="255" step="1" value="20"></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v16">000</span><input type="number" class="thr" id="s16" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r16" min="0" max="255" step="1" value="20"></div>
  <div class="mid"><span class="label">B.RGT P4</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v17">000</span><input type="number" class="thr" id="s17" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r17" min="0" max="255" step="1" value="20"></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v19">000</span><input type="number" class="thr" id="s19" min="0" max="255" step="1" value="125"><input type="number" class="rel" id="r19" min="0" max="255" step="1" value="20"></div>
</div>

</div>

<div id="bar"><div id="bar-fill"></div></div>

<div id="toolbar">
  <button onclick="allThr(20)">20</button>
  <button onclick="allThr(35)">35</button>
  <button onclick="allThr(50)">50</button>
  <button onclick="allThr(75)">75</button>
  <button onclick="allThr(100)">100</button>
  <button onclick="allThr(150)">150</button>
  <span class="global-row">All <input type="number" class="thr" id="global-in" min="0" max="255" step="1" value="125"></span>
  <span class="global-row">Release <input type="number" class="rel" id="global-rel" min="0" max="255" step="1" value="20"></span>
  <button onclick="allRel(10)">rel 10</button>
  <button onclick="allRel(20)">rel 20</button>
  <button onclick="allRel(30)">rel 30</button>
  <button onclick="allRel(50)">rel 50</button>
  <button class="danger" onclick="doCmd('zero')">Zero</button>
  <button class="danger" onclick="doCmd('save')">Save</button>
  <button onclick="doCmd('refresh')">Reload</button>
</div>

<div id="status"><span class="dot off" id="dot"></span><span id="st">Waiting...</span></div>

<script>
var N = 20;
var thrs = new Array(N).fill(125);
var rels = new Array(N).fill(20);
var thrSeq = -1;
var barFill = document.getElementById('bar-fill');
var dot = document.getElementById('dot');
var st = document.getElementById('st');
var globalIn = document.getElementById('global-in');
var globalRel = document.getElementById('global-rel');

function p3(n) { return n < 100 ? (n < 10 ? '  ' + n : ' ' + n) : '' + n; }

function post(body) {
  fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'},
                 body:JSON.stringify(body)});
}

function showThr(idx, v) {
  thrs[idx] = v;
  var el = document.getElementById('s' + idx);
  if (el !== document.activeElement) el.value = v;
}

// Thresholds come from the firmware, not from the markup, so the UI can never
// claim a value the slaves are not using. Fields being actively edited are
// left alone so an SSE echo does not yank the value while typing.
function applyThresholds(t) {
  for (var i = 0; i < N; i++) showThr(i, t[i]);
  var allSame = t.every(function(x) { return x === t[0]; });
  if (allSame && globalIn !== document.activeElement) globalIn.value = t[0];
}

function commitThr(idx, el) {
  var v = parseInt(el.value, 10);
  if (isNaN(v)) { el.value = thrs[idx]; return; }
  v = Math.max(0, Math.min(255, v));
  el.value = v;
  if (v === thrs[idx]) return;
  thrs[idx] = v;
  post({idx:idx, val:v});
}

for (var i = 0; i < N; i++) {
  (function(idx) {
    var el = document.getElementById('s' + idx);
    el.addEventListener('change', function() { commitThr(idx, this); });
    el.addEventListener('keydown', function(e) {
      if (e.key === 'Enter') this.blur();
    });
  })(i);
}

function commitAll(el) {
  var v = parseInt(el.value, 10);
  if (isNaN(v)) { el.value = thrs[0]; return; }
  allThr(v);
}

globalIn.addEventListener('change', function() { commitAll(this); });
globalIn.addEventListener('keydown', function(e) {
  if (e.key === 'Enter') this.blur();
});

function allThr(v) {
  v = Math.max(0, Math.min(255, Math.round(v)));
  globalIn.value = v;
  for (var i = 0; i < N; i++) showThr(i, v);
  post({all:v});
}

function showRel(idx, v) {
  rels[idx] = v;
  var el = document.getElementById('r' + idx);
  if (el !== document.activeElement) el.value = v;
}

function applyRelease(r) {
  for (var i = 0; i < N; i++) showRel(i, r[i]);
  var allSame = r.every(function(x) { return x === r[0]; });
  if (allSame && globalRel !== document.activeElement) globalRel.value = r[0];
}

function commitRel(idx, el) {
  var v = parseInt(el.value, 10);
  if (isNaN(v)) { el.value = rels[idx]; return; }
  v = Math.max(0, Math.min(255, v));
  el.value = v;
  if (v === rels[idx]) return;
  rels[idx] = v;
  post({ridx:idx, rval:v});
}

for (var i = 0; i < N; i++) {
  (function(idx) {
    var el = document.getElementById('r' + idx);
    el.addEventListener('change', function() { commitRel(idx, this); });
    el.addEventListener('keydown', function(e) {
      if (e.key === 'Enter') this.blur();
    });
  })(i);
}

function commitAllRel(el) {
  var v = parseInt(el.value, 10);
  if (isNaN(v)) { el.value = rels[0]; return; }
  allRel(v);
}

globalRel.addEventListener('change', function() { commitAllRel(this); });
globalRel.addEventListener('keydown', function(e) {
  if (e.key === 'Enter') this.blur();
});

function allRel(v) {
  v = Math.max(0, Math.min(255, Math.round(v)));
  globalRel.value = v;
  for (var i = 0; i < N; i++) showRel(i, v);
  post({rall:v});
}

function doCmd(c) { post({cmd:c}); }

var evt = new EventSource('/stream');
evt.onmessage = function(e) {
  var msg;
  try { msg = JSON.parse(e.data); } catch (err) { return; }
  if (!msg.v || msg.v.length !== N) return;
  if (msg.seq !== thrSeq && msg.t && msg.t.length === N &&
      msg.r && msg.r.length === N) {
    thrSeq = msg.seq;
    applyThresholds(msg.t);
    applyRelease(msg.r);
  }
  var maxVal = 0;
  for (var i = 0; i < N; i++) {
    var v = msg.v[i]; if (v > maxVal) maxVal = v;
    var el = document.getElementById('v' + i);
    el.textContent = p3(v);
    el.className = 'v' + (v >= thrs[i] ? ' hit'
                          : v >= rels[i] ? ' warn' : ' idle');
  }
  var panelIds = ['div1','div2','div3','div4','div5'];
  for (var p = 0; p < 5; p++) {
    var hit = false;
    for (var s = 0; s < 4; s++) { if (msg.v[p*4+s] >= thrs[p*4+s]) { hit = true; break; } }
    document.getElementById(panelIds[p]).className = 'panel ' + panelIds[p] + (hit ? ' hit' : '');
  }
  barFill.style.width = Math.min(100, maxVal*100/255) + '%';
  dot.className = 'dot on';
  st.textContent = msg.link ? 'Streaming' : 'Streaming (no master)';
};
evt.onerror = function() { dot.className = 'dot off'; st.textContent = 'Disconnected'; };
</script>
</body>
</html>
"""

state_lock = threading.Lock()
state = {
    "values": [0] * NUM_SENSORS,
    "thresholds": [FIRMWARE_DEFAULT_THRESHOLD] * NUM_SENSORS,
    "release_thresholds": [FIRMWARE_DEFAULT_RELEASE_THRESHOLD] * NUM_SENSORS,
    # Bumped whenever thresholds change server-side, so a browser knows to
    # re-sync its threshold fields without being told twenty times a second.
    "seq": 0,
    "link": False,
}

ser = None
ser_lock = threading.Lock()
streaming = False
# Only the reader thread is allowed to read from the port. A request handler
# doing its own readline() would compete with it for incoming lines, so
# "re-read the thresholds" is passed to the reader thread as a request.
refresh_request = threading.Event()


def set_thresholds(values):
    with state_lock:
        state["thresholds"] = list(values)
        state["seq"] += 1


def set_release_thresholds(values):
    with state_lock:
        state["release_thresholds"] = list(values)
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

    thrs = read_thresholds(s, "t\n", "t")
    if thrs is None:
        print("Warning: master did not report thresholds; showing firmware "
              f"default ({FIRMWARE_DEFAULT_THRESHOLD}).")
    else:
        set_thresholds(thrs)
        print("Thresholds read from hardware:", thrs)
    rels = read_thresholds(s, "q\n", "q")
    if rels is None:
        print("Warning: master did not report release thresholds; showing "
              f"firmware default ({FIRMWARE_DEFAULT_RELEASE_THRESHOLD}).")
    else:
        set_release_thresholds(rels)
        print("Release thresholds read from hardware:", rels)

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
                # read_thresholds() consumes lines, so it must run on this
                # thread — the only one that reads the port.
                again = read_thresholds(s, "t\n", "t")
                if again is not None:
                    set_thresholds(again)
                    print("-> thresholds re-read:", again)
                again = read_thresholds(s, "q\n", "q")
                if again is not None:
                    set_release_thresholds(again)
                    print("-> release thresholds re-read:", again)
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
    phase = [random.uniform(0, 2 * math.pi) for _ in range(NUM_SENSORS)]
    t = 0.0
    pattern = [0, 1, 2, 3, 4, 2]
    pat_idx = 0
    hold = 0.0
    while True:
        vals = []
        for i in range(NUM_SENSORS):
            pi = i // FSRS_PER_PANEL
            base = 18 + int(8 * math.sin(t + phase[i]))
            target = pattern[pat_idx]
            dist = abs(pi - target)
            press = 0
            if dist == 0:
                press = 140 + int(40 * math.sin(t * 4 + phase[i]))
            elif dist == 1:
                press = int(30 * math.sin(t * 3 + phase[i]))
            vals.append(max(0, min(255, base + press)))
        with state_lock:
            state["values"] = vals
        hold += 0.05
        if hold > 1.2:
            hold = 0.0
            pat_idx = (pat_idx + 1) % len(pattern)
        t += 0.05
        time.sleep(0.05)


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
                    # Copied, not referenced: the lists are mutated in place
                    # elsewhere and json.dumps runs outside the lock.
                    payload = {
                        "v": list(state["values"]),
                        "t": list(state["thresholds"]),
                        "r": list(state["release_thresholds"]),
                        "seq": state["seq"],
                        "link": state["link"],
                    }
                yield "data: " + json.dumps(payload, separators=(",", ":")) + "\n\n"
                time.sleep(0.05)
        except GeneratorExit:
            # Browser closed the EventSource (tab closed, reload): stop the
            # generator instead of leaving it spinning until a write fails.
            return
    return Response(generate(), mimetype="text/event-stream")


@app.route("/cmd", methods=["POST"])
def cmd():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify(error="expected a JSON object"), 400

    if data.get("cmd") == "refresh":
        # The reader thread owns the port; ask it to do the round-trip.
        with ser_lock:
            if not (ser and ser.is_open):
                return jsonify(error="no serial link"), 503
        refresh_request.set()
        return jsonify(ok=True), 202

    with ser_lock:
        s = ser
        if not (s and s.is_open):
            return jsonify(error="no serial link"), 503
        try:
            if data.get("cmd") == "zero":
                s.write(b"o\n")
                print("-> zero offsets")
            elif data.get("cmd") == "save":
                s.write(b"s\n")
                print("-> save to eeprom")
            elif "idx" in data and "val" in data:
                idx, val = data["idx"], data["val"]
                if not (_is_int(idx) and _is_int(val)
                        and 0 <= idx < NUM_SENSORS and 0 <= val <= 255):
                    return jsonify(error="idx 0-19, val 0-255"), 400
                s.write(f"s {idx} {val}\n".encode())
                with state_lock:
                    state["thresholds"][idx] = val
                    state["seq"] += 1
                print(f"-> set thr[{idx}] = {val}")
            elif "ridx" in data and "rval" in data:
                idx, val = data["ridx"], data["rval"]
                if not (_is_int(idx) and _is_int(val)
                        and 0 <= idx < NUM_SENSORS and 0 <= val <= 255):
                    return jsonify(error="ridx 0-19, rval 0-255"), 400
                s.write(f"e {idx} {val}\n".encode())
                with state_lock:
                    state["release_thresholds"][idx] = val
                    state["seq"] += 1
                print(f"-> set rel[{idx}] = {val}")
            elif "all" in data:
                val = data["all"]
                if not (_is_int(val) and 0 <= val <= 255):
                    return jsonify(error="all: 0-255"), 400
                # One master command, not twenty: the master fans it out as a
                # single write per panel. Sending 20 `s i v` lines back to back
                # used to stall the master's poll loop for over a second.
                s.write(f"a {val}\n".encode())
                print(f"-> set all thrs = {val}")
            elif "rall" in data:
                val = data["rall"]
                if not (_is_int(val) and 0 <= val <= 255):
                    return jsonify(error="rall: 0-255"), 400
                s.write(f"y {val}\n".encode())
                print(f"-> set all release thrs = {val}")
            else:
                return jsonify(error="unrecognised command"), 400
            s.flush()
        except Exception as e:
            print("Serial write error:", e)
            return jsonify(error=str(e)), 500

    if "all" in data:
        set_thresholds([data["all"]] * NUM_SENSORS)
    if "rall" in data:
        set_release_thresholds([data["rall"]] * NUM_SENSORS)
    return jsonify(ok=True)


def main():
    parser = argparse.ArgumentParser(
        description="PIUFSR browser calibration UI")
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
