#!/usr/bin/env python3
import sys
import serial
import threading
import time
import random
import math
from flask import Flask, Response, request

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
  grid-template-columns: 40px 1fr 40px;
  grid-template-rows: 36px 1fr 36px;
  gap: 2px;
}
.panel.hit { border-color: #e94560; box-shadow: 0 0 10px rgba(233,69,96,0.3); }

.edge-up   { grid-area: up;   display: flex; align-items: center; justify-content: center; gap: 6px; }
.edge-down { grid-area: down; display: flex; align-items: center; justify-content: center; gap: 6px; }
.edge-l    { grid-area: l;    display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 2px; width: 40px; }
.edge-r    { grid-area: r;    display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 2px; width: 40px; }

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

.tv { font-size: 10px; color: #888; min-width: 20px; text-align: center; font-variant-numeric: tabular-nums; }

/* horizontal sliders (up/down) */
.sl-h { -webkit-appearance: none; appearance: none; height: 4px; width: 80px; background: #333; outline: none; border-radius: 2px; margin: 0; cursor: pointer; flex-shrink: 1; }
.sl-h::-webkit-slider-thumb { -webkit-appearance: none; width: 10px; height: 16px; border-radius: 3px; background: #e94560; cursor: pointer; }
.sl-h::-moz-range-thumb { width: 10px; height: 16px; border-radius: 3px; background: #e94560; cursor: pointer; border: none; }

/* vertical sliders (left/right) — native macOS Aqua */
.sl-v { -webkit-appearance: slider-vertical; width: 20px; height: 56px; margin: 0; cursor: pointer; }

#bar { width: 100%; max-width: 600px; height: 4px; background: #1a1a2e; border-radius: 2px; overflow: hidden; }
#bar-fill { height: 100%; width: 0%; background: #e94560; border-radius: 2px; transition: width 0.05s; }

#toolbar { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; justify-content: center; }
#toolbar button { background: #16213e; color: #ccc; border: 1px solid #0f3460; border-radius: 4px; padding: 5px 12px; font-size: 11px; cursor: pointer; }
#toolbar button:hover { background: #0f3460; color: #fff; }
#toolbar .danger { border-color: #5a0000; color: #ff6b6b; }
#toolbar .danger:hover { background: #2a0000; }
#global-row { display: flex; align-items: center; gap: 6px; margin: 0 6px; font-size: 11px; color: #888; }
#global-row input { width: 100px; height: 4px; -webkit-appearance: none; appearance: none; background: #333; border-radius: 2px; outline: none; cursor: pointer; }
#global-row input::-webkit-slider-thumb { -webkit-appearance: none; width: 10px; height: 16px; border-radius: 3px; background: #e94560; cursor: pointer; }
#global-row input::-moz-range-thumb { width: 10px; height: 16px; border-radius: 3px; background: #e94560; cursor: pointer; border: none; }

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
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v6">000</span><input type="range" class="sl-h" id="s6" min="0" max="255" value="50"><span class="tv" id="t6">050</span></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v4">000</span><input type="range" class="sl-v" id="s4" min="0" max="255" value="50"><span class="tv" id="t4">050</span></div>
  <div class="mid"><span class="label">T.LFT P1</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v5">000</span><input type="range" class="sl-v" id="s5" min="0" max="255" value="50"><span class="tv" id="t5">050</span></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v7">000</span><input type="range" class="sl-h" id="s7" min="0" max="255" value="50"><span class="tv" id="t7">050</span></div>
</div>

<!-- TR = div4, sensors 12-15: L=12 R=13 U=14 D=15 -->
<div class="panel div4">
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v14">000</span><input type="range" class="sl-h" id="s14" min="0" max="255" value="50"><span class="tv" id="t14">050</span></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v12">000</span><input type="range" class="sl-v" id="s12" min="0" max="255" value="50"><span class="tv" id="t12">050</span></div>
  <div class="mid"><span class="label">T.RGT P3</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v13">000</span><input type="range" class="sl-v" id="s13" min="0" max="255" value="50"><span class="tv" id="t13">050</span></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v15">000</span><input type="range" class="sl-h" id="s15" min="0" max="255" value="50"><span class="tv" id="t15">050</span></div>
</div>

<!-- C = div3, sensors 8-11: L=8 R=9 U=10 D=11 -->
<div class="panel div3">
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v10">000</span><input type="range" class="sl-h" id="s10" min="0" max="255" value="50"><span class="tv" id="t10">050</span></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v8">000</span><input type="range" class="sl-v" id="s8" min="0" max="255" value="50"><span class="tv" id="t8">050</span></div>
  <div class="mid"><span class="label">CENTER P2</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v9">000</span><input type="range" class="sl-v" id="s9" min="0" max="255" value="50"><span class="tv" id="t9">050</span></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v11">000</span><input type="range" class="sl-h" id="s11" min="0" max="255" value="50"><span class="tv" id="t11">050</span></div>
</div>

<!-- BL = div1, sensors 0-3: L=0 R=1 U=2 D=3 -->
<div class="panel div1">
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v2">000</span><input type="range" class="sl-h" id="s2" min="0" max="255" value="50"><span class="tv" id="t2">050</span></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v0">000</span><input type="range" class="sl-v" id="s0" min="0" max="255" value="50"><span class="tv" id="t0">050</span></div>
  <div class="mid"><span class="label">B.LFT P0</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v1">000</span><input type="range" class="sl-v" id="s1" min="0" max="255" value="50"><span class="tv" id="t1">050</span></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v3">000</span><input type="range" class="sl-h" id="s3" min="0" max="255" value="50"><span class="tv" id="t3">050</span></div>
</div>

<!-- BR = div5, sensors 16-19: L=16 R=17 U=18 D=19 -->
<div class="panel div5">
  <div class="edge-up">  <span class="side">U</span><span class="v" id="v18">000</span><input type="range" class="sl-h" id="s18" min="0" max="255" value="50"><span class="tv" id="t18">050</span></div>
  <div class="edge-l">  <span class="side">L</span><span class="v" id="v16">000</span><input type="range" class="sl-v" id="s16" min="0" max="255" value="50"><span class="tv" id="t16">050</span></div>
  <div class="mid"><span class="label">B.RGT P4</span></div>
  <div class="edge-r">  <span class="side">R</span><span class="v" id="v17">000</span><input type="range" class="sl-v" id="s17" min="0" max="255" value="50"><span class="tv" id="t17">050</span></div>
  <div class="edge-down"><span class="side">D</span><span class="v" id="v19">000</span><input type="range" class="sl-h" id="s19" min="0" max="255" value="50"><span class="tv" id="t19">050</span></div>
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
  <span id="global-row">All <input type="range" id="global-sl" min="0" max="255" value="50"><span id="global-val" style="color:#eee;min-width:24px;text-align:center;font-size:12px">50</span></span>
  <button class="danger" onclick="doCmd('zero')">Zero</button>
  <button class="danger" onclick="doCmd('save')">Save</button>
</div>

<div id="status"><span class="dot off" id="dot"></span><span id="st">Waiting...</span></div>

<script>
var thrs = new Array(20).fill(50);
var barFill = document.getElementById('bar-fill');
var dot = document.getElementById('dot');
var st = document.getElementById('st');
var globalSl = document.getElementById('global-sl');
var globalVal = document.getElementById('global-val');

function p3(n) { return n < 100 ? (n < 10 ? '  ' + n : ' ' + n) : '' + n; }

for (var i = 0; i < 20; i++) {
  (function(idx) {
    var sl = document.getElementById('s' + idx);
    sl.addEventListener('input', function() {
      thrs[idx] = parseInt(this.value);
      document.getElementById('t' + idx).textContent = p3(this.value);
    });
    sl.addEventListener('change', function() {
      fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({idx:idx, val:thrs[idx]})});
    });
  })(i);
}

globalSl.addEventListener('input', function() {
  var v = parseInt(this.value);
  globalVal.textContent = v;
  for (var i = 0; i < 20; i++) {
    thrs[i] = v;
    document.getElementById('t' + i).textContent = p3(v);
    document.getElementById('s' + i).value = v;
  }
});
globalSl.addEventListener('change', function() {
  fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({all:parseInt(this.value)})});
});

function allThr(v) {
  globalSl.value = v; globalVal.textContent = v;
  for (var i = 0; i < 20; i++) {
    thrs[i] = v; document.getElementById('t' + i).textContent = p3(v); document.getElementById('s' + i).value = v;
  }
  fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({all:v})});
}

function doCmd(c) {
  fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({cmd:c})});
}

var evt = new EventSource('/stream');
evt.onmessage = function(e) {
  var vals = e.data.trim().split(' ').map(Number);
  if (vals.length !== 20) return;
  var maxVal = 0;
  for (var i = 0; i < 20; i++) {
    var v = vals[i]; if (v > maxVal) maxVal = v;
    var el = document.getElementById('v' + i);
    el.textContent = p3(v);
    el.className = 'v' + (v >= thrs[i] ? ' hit' : v >= thrs[i]/2 ? ' warn' : ' idle');
  }
  var panelIds = ['div1','div2','div3','div4','div5'];
  for (var p = 0; p < 5; p++) {
    var hit = false;
    for (var s = 0; s < 4; s++) { if (vals[p*4+s] >= thrs[p*4+s]) { hit = true; break; } }
    document.getElementById(panelIds[p]).className = 'panel ' + panelIds[p] + (hit ? ' hit' : '');
  }
  barFill.style.width = Math.min(100, maxVal*100/255) + '%';
  dot.className = 'dot on'; st.textContent = 'Streaming';
};
evt.onerror = function() { dot.className = 'dot off'; st.textContent = 'Disconnected'; };
</script>
</body>
</html>
"""

latest = {"values": [0] * 20, "lock": threading.Lock()}
ser = None
ser_lock = threading.Lock()


def serial_reader(port, baud):
    global ser
    try:
        s = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        s.reset_input_buffer()
        s.write(b"c\n")
        time.sleep(0.2)
        s.reset_input_buffer()
    except Exception as e:
        print("Serial open failed:", e)
        return
    with ser_lock:
        ser = s
    while True:
        try:
            line = s.readline().decode("utf-8", errors="replace").strip()
            if not line.startswith("c "):
                continue
            parts = line.split()
            if len(parts) != 21:
                continue
            with latest["lock"]:
                latest["values"] = [int(p) for p in parts[1:]]
        except Exception as e:
            print("Serial read error:", e)
            break
    with ser_lock:
        ser = None
    s.close()


def demo_reader():
    phase = [random.uniform(0, 2 * math.pi) for _ in range(20)]
    t = 0.0
    pattern = [0, 1, 2, 3, 4, 2]
    pat_idx = 0
    hold = 0.0
    while True:
        vals = []
        for i in range(20):
            pi = i // 4
            base = 18 + int(8 * math.sin(t + phase[i]))
            target = pattern[pat_idx]
            dist = abs(pi - target)
            press = 0
            if dist == 0:
                press = 140 + int(40 * math.sin(t * 4 + phase[i]))
            elif dist == 1:
                press = int(30 * math.sin(t * 3 + phase[i]))
            vals.append(max(0, min(255, base + press)))
        with latest["lock"]:
            latest["values"] = vals
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
        while True:
            with latest["lock"]:
                vals = latest["values"]
            yield f"data: {' '.join(map(str, vals))}\n\n"
            time.sleep(0.05)
    return Response(generate(), mimetype="text/event-stream")


@app.route("/cmd", methods=["POST"])
def cmd():
    data = request.json
    if not data:
        return "bad", 400
    with ser_lock:
        s = ser
        if s and s.is_open:
            try:
                if data.get("cmd") == "zero":
                    s.write(b"o\n")
                    print("-> zero offsets")
                elif data.get("cmd") == "save":
                    s.write(b"s\n")
                    print("-> save to eeprom")
                elif "idx" in data and "val" in data:
                    s.write(f"s {data['idx']} {data['val']}\n".encode())
                    print(f"-> set thr[{data['idx']}] = {data['val']}")
                elif "all" in data:
                    for i in range(20):
                        s.write(f"s {i} {data['all']}\n".encode())
                    print(f"-> set all thrs = {data['all']}")
            except Exception as e:
                print("Serial write error:", e)
    return "ok"


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "--demo":
        print("Demo mode")
        t = threading.Thread(target=demo_reader, daemon=True)
        t.start()
        http_port = int(sys.argv[2]) if len(sys.argv) > 2 else 8765
    elif len(sys.argv) >= 2:
        port = sys.argv[1]
        baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
        t = threading.Thread(target=serial_reader, args=(port, baud), daemon=True)
        t.start()
        http_port = 8765
    else:
        print("usage: %s <serial_port> [baud]" % sys.argv[0])
        print("   or: %s --demo [http_port]" % sys.argv[0])
        sys.exit(1)
    print("Listening on http://localhost:%d" % http_port)
    app.run(host="0.0.0.0", port=http_port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
