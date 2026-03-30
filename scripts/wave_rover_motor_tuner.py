#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import threading
import time
from collections import deque
from typing import Any

try:
    import serial  # type: ignore
except Exception as exc:  # pragma: no cover
    serial = exc  # type: ignore

from flask import Flask, jsonify, request

HTML = r'''<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Wave Rover Motor Tuner</title>
  <style>
    body{font-family:Arial,Helvetica,sans-serif;background:#efefea;color:#222;margin:0;padding:24px}
    h1{margin:0 0 8px 0}
    .sub{color:#666;margin-bottom:16px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px;margin-bottom:16px}
    .card{background:#fff;border-radius:16px;padding:14px 16px;box-shadow:0 1px 0 rgba(0,0,0,.08)}
    .label{font-size:11px;color:#777;text-transform:uppercase;letter-spacing:.08em}
    .value{font-size:22px;font-weight:700;margin-top:8px}
    .panel{background:#fff;border-radius:18px;padding:18px;box-shadow:0 1px 0 rgba(0,0,0,.08);max-width:1080px}
    .row{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px;margin-bottom:10px}
    label{font-size:12px;color:#666;display:block;margin-bottom:4px}
    input,select{width:100%;padding:10px 12px;border:1px solid #ccc;border-radius:10px;font-size:15px;box-sizing:border-box}
    button{padding:10px 14px;border:0;border-radius:10px;background:#17795d;color:#fff;font-weight:700;cursor:pointer}
    button.alt{background:#666}
    button.stop{background:#b72d2d}
    button.small{padding:8px 10px;font-size:13px}
    .btns{display:flex;flex-wrap:wrap;gap:10px;margin:10px 0 14px 0}
    .mono{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;white-space:pre-wrap;word-break:break-word;background:#111;color:#d7f5d5;padding:12px;border-radius:12px;font-size:13px}
    .hint{font-size:13px;color:#666;line-height:1.45}
    .split{display:grid;grid-template-columns:minmax(300px,540px) minmax(300px,1fr);gap:16px;align-items:start}
  </style>
</head>
<body>
  <h1>Wave Rover Motor Tuner</h1>
  <div class="sub">Use this as a separate tuning tool. It sends raw left/right wheel pairs, probes chassis feedback, and helps you find the smallest effective pivot and forward-turn pairs.</div>

  <div class="grid">
    <div class="card"><div class="label">Serial</div><div class="value" id="serialStatus">checking</div></div>
    <div class="card"><div class="label">Rover Voltage</div><div class="value" id="voltage">-</div></div>
    <div class="card"><div class="label">Rover Current</div><div class="value" id="current">-</div></div>
    <div class="card"><div class="label">Battery %</div><div class="value" id="batteryPct">-</div></div>
    <div class="card"><div class="label">Feedback Age</div><div class="value" id="age">-</div></div>
    <div class="card"><div class="label">Address</div><div class="value" id="address">-</div></div>
  </div>

  <div class="split">
    <div class="panel">
      <div class="row">
        <div><label>Left wheel</label><input id="left" type="number" step="0.01" min="-0.50" max="0.50" value="0.00"></div>
        <div><label>Right wheel</label><input id="right" type="number" step="0.01" min="-0.50" max="0.50" value="0.00"></div>
        <div><label>Pulse seconds</label><input id="pulse" type="number" step="0.01" min="0.05" max="3.00" value="0.30"></div>
        <div><label>Repeats</label><input id="repeats" type="number" step="1" min="1" max="10" value="1"></div>
        <div><label>Pause seconds</label><input id="pause" type="number" step="0.01" min="0.00" max="3.00" value="0.25"></div>
      </div>

      <div class="row">
        <div><label>Builder outer</label><input id="outer" type="number" step="0.01" min="0.00" max="0.50" value="0.40"></div>
        <div><label>Builder inner</label><input id="inner" type="number" step="0.01" min="-0.50" max="0.50" value="0.00"></div>
        <div><label>Sweep target</label><select id="sweepTarget"><option value="left">left</option><option value="right">right</option></select></div>
        <div><label>Sweep start</label><input id="sweepStart" type="number" step="0.01" min="-0.50" max="0.50" value="0.20"></div>
        <div><label>Sweep end</label><input id="sweepEnd" type="number" step="0.01" min="-0.50" max="0.50" value="-0.10"></div>
        <div><label>Sweep step</label><input id="sweepStep" type="number" step="0.01" min="0.01" max="0.50" value="0.05"></div>
      </div>

      <div class="btns">
        <button id="probe">Probe Rover</button>
        <button id="arcLeft" class="small">Set Arc Left</button>
        <button id="arcRight" class="small">Set Arc Right</button>
        <button id="pivotLeft" class="small">Set Pivot Left</button>
        <button id="pivotRight" class="small">Set Pivot Right</button>
        <button id="runPair">Run Pair</button>
        <button id="runSweep">Run Sweep</button>
        <button id="stop" class="stop">Stop</button>
      </div>

      <div class="hint">
        Suggested process:<br>
        1) Find the smallest pivot pair that truly rotates.<br>
        2) Find the smallest forward-turn pair that truly changes heading.<br>
        3) Require 3/3 success before accepting a pair.<br>
        4) Watch voltage/current while the pulse runs to see whether turning load causes sag.
      </div>
    </div>

    <div class="panel">
      <div class="label">Status</div>
      <div id="status" style="font-weight:700;margin:8px 0 10px 0">starting</div>
      <div class="label">Latest result</div>
      <div id="latest" class="mono">-</div>
      <div class="label" style="margin-top:12px">Raw feedback</div>
      <div id="raw" class="mono">-</div>
    </div>
  </div>

<script>
const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,Number(v)||0));
function num(id){return Number(document.getElementById(id).value)||0;}
function tuning(){
  return {
    left: clamp(num('left'),-0.5,0.5),
    right: clamp(num('right'),-0.5,0.5),
    pulse: Math.max(0.05, num('pulse')),
    repeats: Math.max(1, Math.round(num('repeats'))),
    pause: Math.max(0, num('pause')),
    outer: clamp(num('outer'),0,0.5),
    inner: clamp(num('inner'),-0.5,0.5),
    sweepTarget: document.getElementById('sweepTarget').value,
    sweepStart: clamp(num('sweepStart'),-0.5,0.5),
    sweepEnd: clamp(num('sweepEnd'),-0.5,0.5),
    sweepStep: Math.max(0.01, Math.abs(num('sweepStep'))),
  };
}
async function fetchJson(url, options={}, timeoutMs=4000){
  const controller = new AbortController();
  const timer = setTimeout(()=>controller.abort(), timeoutMs);
  try {
    const resp = await fetch(url, {...options, signal: controller.signal});
    if(!resp.ok){
      const text = await resp.text();
      throw new Error(`${resp.status} ${text}`);
    }
    return await resp.json();
  } finally { clearTimeout(timer); }
}
function setPair(l,r,label=''){
  document.getElementById('left').value = clamp(l,-0.5,0.5).toFixed(2);
  document.getElementById('right').value = clamp(r,-0.5,0.5).toFixed(2);
  if(label) document.getElementById('status').innerText = label;
}
function renderFeedback(payload){
  const e=(payload&&payload.extracted)||{};
  document.getElementById('voltage').innerText = e.voltage_v==null?'-':`${Number(e.voltage_v).toFixed(2)} V`;
  document.getElementById('current').innerText = e.current_a==null?'-':`${Number(e.current_a).toFixed(2)} A`;
  document.getElementById('batteryPct').innerText = e.battery_percent==null?'-':`${Number(e.battery_percent).toFixed(0)} %`;
  document.getElementById('age').innerText = e.age_s==null?'-':`${Number(e.age_s).toFixed(2)} s`;
  document.getElementById('raw').innerText = JSON.stringify(payload&&payload.feedback?payload.feedback:null, null, 2);
}
async function refreshStatus(){
  try{
    const data = await fetchJson('/api/status', {}, 1500);
    document.getElementById('serialStatus').innerText = data.serial || 'unknown';
    document.getElementById('address').innerText = data.address || '-';
    renderFeedback(data.feedback || null);
  }catch(err){
    document.getElementById('serialStatus').innerText='error';
    document.getElementById('status').innerText=`status failed: ${err.message}`;
  }
}
async function probe(){
  const data = await fetchJson('/api/feedback?probe=1', {}, 2500);
  renderFeedback(data);
  document.getElementById('latest').innerText = JSON.stringify(data, null, 2);
  document.getElementById('status').innerText = data.status || 'feedback captured';
}
async function runPair(note='pair'){
  const t = tuning();
  const payload = {left:t.left,right:t.right,duration_s:t.pulse,repeats:t.repeats,pause_s:t.pause,probe_feedback:true};
  const timeoutMs = Math.max(2500, Math.round((t.pulse*t.repeats + t.pause*Math.max(0,t.repeats-1) + 1.5)*1000));
  const data = await fetchJson('/api/pulse_pair', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)}, timeoutMs);
  renderFeedback(data.feedback_after || null);
  document.getElementById('latest').innerText = JSON.stringify(data, null, 2);
  document.getElementById('status').innerText = data.status || note;
}
function sweepValues(start,end,step){
  const values=[];
  const signed = Math.abs(step) * (end>=start?1:-1);
  let cur = start; let guard=0;
  while ((signed>0 && cur<=end+1e-9) || (signed<0 && cur>=end-1e-9)) {
    values.push(Number(cur.toFixed(2)));
    cur += signed; guard += 1; if(guard>200) break;
  }
  if(!values.length || Math.abs(values[values.length-1]-end)>1e-9) values.push(Number(end.toFixed(2)));
  return values;
}
async function runSweep(){
  const t = tuning();
  const values = sweepValues(t.sweepStart, t.sweepEnd, t.sweepStep);
  const results=[];
  for(let i=0;i<values.length;i++){
    const value = values[i];
    if(t.sweepTarget==='left') document.getElementById('left').value=value.toFixed(2);
    else document.getElementById('right').value=value.toFixed(2);
    document.getElementById('status').innerText=`sweep ${i+1}/${values.length}`;
    const payload = {left: clamp(num('left'),-0.5,0.5), right: clamp(num('right'),-0.5,0.5), duration_s:t.pulse, repeats:t.repeats, pause_s:t.pause, probe_feedback:true};
    const data = await fetchJson('/api/pulse_pair', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)}, 6000);
    results.push(data);
    renderFeedback(data.feedback_after || null);
  }
  document.getElementById('latest').innerText = JSON.stringify({status:'sweep complete', results}, null, 2);
  document.getElementById('status').innerText = `sweep complete (${results.length} tests)`;
}
async function stopNow(){
  const data = await fetchJson('/api/stop', {method:'POST'}, 1500);
  document.getElementById('latest').innerText = JSON.stringify(data, null, 2);
  document.getElementById('status').innerText = data.status || 'stopped';
}

document.getElementById('probe').addEventListener('click',()=>probe().catch(err=>document.getElementById('status').innerText=`probe failed: ${err.message}`));
document.getElementById('arcLeft').addEventListener('click',()=>{const t=tuning(); setPair(t.inner,t.outer,'builder: arc left');});
document.getElementById('arcRight').addEventListener('click',()=>{const t=tuning(); setPair(t.outer,t.inner,'builder: arc right');});
document.getElementById('pivotLeft').addEventListener('click',()=>{const t=tuning(); setPair(-Math.abs(t.outer),Math.abs(t.outer),'builder: pivot left');});
document.getElementById('pivotRight').addEventListener('click',()=>{const t=tuning(); setPair(Math.abs(t.outer),-Math.abs(t.outer),'builder: pivot right');});
document.getElementById('runPair').addEventListener('click',()=>runPair().catch(err=>document.getElementById('status').innerText=`run failed: ${err.message}`));
document.getElementById('runSweep').addEventListener('click',()=>runSweep().catch(err=>document.getElementById('status').innerText=`sweep failed: ${err.message}`));
document.getElementById('stop').addEventListener('click',()=>stopNow().catch(err=>document.getElementById('status').innerText=`stop failed: ${err.message}`));
refreshStatus();
setInterval(refreshStatus, 1000);
</script>
</body>
</html>'''


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _feedback_numeric_items(data: Any, prefix: str = "") -> list[tuple[str, float]]:
    items: list[tuple[str, float]] = []
    if isinstance(data, dict):
        for key, value in data.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            items.extend(_feedback_numeric_items(value, child_prefix))
    elif isinstance(data, list):
        for idx, value in enumerate(data):
            child_prefix = f"{prefix}[{idx}]" if prefix else f"[{idx}]"
            items.extend(_feedback_numeric_items(value, child_prefix))
    else:
        try:
            items.append((prefix, float(data)))
        except Exception:
            pass
    return items


def _normalize_voltage_candidate(value: float) -> float | None:
    candidate = float(value)
    for scale in (1.0, 0.1, 0.01, 0.001):
        scaled = candidate * scale
        if 3.0 <= scaled <= 30.0:
            return float(scaled)
    return None


def _normalize_current_candidate(value: float) -> float | None:
    candidate = abs(float(value))
    for scale in (1.0, 0.1, 0.01, 0.001):
        scaled = candidate * scale
        if 0.0 <= scaled <= 100.0:
            return float(scaled)
    return None


def extract_rover_feedback_metrics(feedback: dict[str, Any] | None, timestamp_s: float | None = None) -> dict[str, Any]:
    metrics: dict[str, Any] = {"voltage_v": None, "current_a": None, "battery_percent": None, "age_s": None}
    if timestamp_s is not None:
        metrics["age_s"] = max(0.0, time.time() - float(timestamp_s))
    if not isinstance(feedback, dict):
        return metrics
    preferred_voltage_keys = ("battery_voltage", "bat_voltage", "power_voltage", "voltage", "vin", "battery.v", "voltage.v", "v")
    preferred_current_keys = ("battery_current", "bat_current", "power_current", "current", "curr", "amps", "amp", "i")
    preferred_percent_keys = ("battery_percent", "percent", "soc", "capacity", "battery")
    for key, value in _feedback_numeric_items(feedback):
        compact = key.replace("_", ".").lower()
        if metrics["voltage_v"] is None and any(token in compact for token in preferred_voltage_keys):
            norm = _normalize_voltage_candidate(value)
            if norm is not None:
                metrics["voltage_v"] = norm
                continue
        if metrics["current_a"] is None and any(token in compact for token in preferred_current_keys):
            norm = _normalize_current_candidate(value)
            if norm is not None:
                metrics["current_a"] = norm
                continue
        if metrics["battery_percent"] is None and any(token in compact for token in preferred_percent_keys):
            if 0.0 <= value <= 100.0:
                metrics["battery_percent"] = float(value)
    return metrics


def make_rover_feedback_payload(feedback: dict[str, Any] | None, timestamp_s: float | None = None) -> dict[str, Any]:
    return {"feedback": feedback, "timestamp": timestamp_s, "extracted": extract_rover_feedback_metrics(feedback, timestamp_s)}


class RoverSerial:
    def __init__(self, port: str, baudrate: int) -> None:
        if not hasattr(serial, 'Serial'):
            raise RuntimeError(f'pyserial is not installed correctly: {serial!r}')
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=0.05)
        try:
            self._ser.setRTS(False)
            self._ser.setDTR(False)
        except Exception:
            pass
        self._write_lock = threading.Lock()
        self._feedback_lock = threading.Lock()
        self._latest_feedback: dict[str, Any] | None = None
        self._latest_feedback_ts: float | None = None
        self._feedback_seq = 0
        self._feedback_event = threading.Event()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
            except Exception:
                break
            if not raw:
                continue
            try:
                message = json.loads(raw.decode('utf-8', errors='ignore').strip())
            except Exception:
                continue
            if not isinstance(message, dict):
                continue
            with self._feedback_lock:
                self._latest_feedback = message
                self._latest_feedback_ts = time.time()
                self._feedback_seq += 1
                self._feedback_event.set()

    def _write_json(self, payload: dict[str, Any]) -> dict[str, Any]:
        line = json.dumps(payload, separators=(',', ':')) + '\n'
        with self._write_lock:
            self._ser.write(line.encode('utf-8'))
            self._ser.flush()
        return payload

    def send(self, left: float, right: float) -> dict[str, Any]:
        return self._write_json({'T': 1, 'L': float(left), 'R': float(right)})

    def stop(self) -> dict[str, Any]:
        return self.send(0.0, 0.0)

    def pulse(self, left: float, right: float, duration_s: float) -> dict[str, Any]:
        duration_s = max(0.0, float(duration_s))
        refresh_s = 0.10
        drive_payload = self.send(left, right)
        deadline = time.monotonic() + duration_s
        refresh_count = 1
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(refresh_s, remaining))
            self.send(left, right)
            refresh_count += 1
        stop_payload = self.stop()
        return {'drive': drive_payload, 'stop': stop_payload, 'duration_s': duration_s, 'refresh_count': refresh_count}

    def latest_feedback_sequence(self) -> int:
        with self._feedback_lock:
            return int(self._feedback_seq)

    def feedback_payload(self) -> dict[str, Any]:
        with self._feedback_lock:
            return make_rover_feedback_payload(self._latest_feedback, self._latest_feedback_ts)

    def wait_for_feedback(self, after_seq: int | None = None, timeout_s: float = 0.6) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while True:
            with self._feedback_lock:
                current_seq = int(self._feedback_seq)
                payload = make_rover_feedback_payload(self._latest_feedback, self._latest_feedback_ts)
            if after_seq is None or current_seq > after_seq:
                return payload
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return payload
            self._feedback_event.clear()
            self._feedback_event.wait(timeout=min(0.08, remaining))

    def probe_feedback(self, timeout_s: float = 0.8) -> dict[str, Any]:
        seq = self.latest_feedback_sequence()
        try:
            self._write_json({'T': 130})
        except Exception:
            return self.feedback_payload()
        return self.wait_for_feedback(after_seq=seq, timeout_s=timeout_s)

    def close(self) -> None:
        self._stop.set()
        try:
            self.stop()
        except Exception:
            pass
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass


def create_app(port: str, baudrate: int, host: str, web_port: int) -> Flask:
    app = Flask(__name__)
    try:
        rover = RoverSerial(port, baudrate)
        rover_error: str | None = None
    except Exception as exc:
        rover = None
        rover_error = str(exc)

    @app.get('/')
    def index() -> str:
        return HTML

    @app.get('/api/status')
    def api_status():
        if rover is None:
            return jsonify({'serial': f'error: {rover_error}', 'address': f'{port} @ {baudrate}', 'feedback': make_rover_feedback_payload(None, None)})
        return jsonify({'serial': 'connected', 'address': f'{port} @ {baudrate}', 'feedback': rover.feedback_payload()})

    @app.get('/api/feedback')
    def api_feedback():
        if rover is None:
            return jsonify({'status': f'rover unavailable: {rover_error}', **make_rover_feedback_payload(None, None)}), 503
        probe = request.args.get('probe', '0') not in ('0', 'false', 'False', '')
        payload = rover.probe_feedback(timeout_s=0.8) if probe else rover.feedback_payload()
        payload['status'] = 'fresh feedback' if probe else 'latest feedback'
        return jsonify(payload)

    @app.post('/api/stop')
    def api_stop():
        if rover is None:
            return jsonify({'status': f'rover unavailable: {rover_error}'}), 503
        payload = rover.stop()
        return jsonify({'status': 'stopped', 'payload': payload})

    @app.post('/api/pulse_pair')
    def api_pulse_pair():
        if rover is None:
            return jsonify({'status': f'rover unavailable: {rover_error}'}), 503
        data = request.get_json(silent=True) or {}
        left = clamp(float(data.get('left', 0.0)), -0.5, 0.5)
        right = clamp(float(data.get('right', 0.0)), -0.5, 0.5)
        duration_s = max(0.05, float(data.get('duration_s', 0.3)))
        repeats = max(1, min(10, int(data.get('repeats', 1))))
        pause_s = max(0.0, min(3.0, float(data.get('pause_s', 0.25))))
        probe_feedback = bool(data.get('probe_feedback', True))
        before = rover.probe_feedback(timeout_s=0.8) if probe_feedback else rover.feedback_payload()
        runs = []
        for idx in range(repeats):
            pulse_result = rover.pulse(left, right, duration_s)
            after = rover.probe_feedback(timeout_s=0.8) if probe_feedback else rover.feedback_payload()
            runs.append({'index': idx + 1, 'pulse': pulse_result, 'feedback_after': after})
            if idx + 1 < repeats and pause_s > 0:
                time.sleep(pause_s)
        final_after = runs[-1]['feedback_after'] if runs else before
        return jsonify({
            'status': 'pulse pair complete',
            'left': left,
            'right': right,
            'duration_s': duration_s,
            'repeats': repeats,
            'pause_s': pause_s,
            'feedback_before': before,
            'feedback_after': final_after,
            'runs': runs,
        })

    @app.get('/healthz')
    def healthz():
        return jsonify({'ok': True, 'serial': rover is not None, 'address': f'{port} @ {baudrate}', 'host': host, 'web_port': web_port})

    @app.teardown_appcontext
    def _teardown(exc):
        return None

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description='Standalone Wave Rover motor tuning utility')
    parser.add_argument('--serial-port', default='/dev/ttyTHS1')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--web-port', type=int, default=8766)
    args = parser.parse_args()
    app = create_app(args.serial_port, args.baudrate, args.host, args.web_port)
    app.run(host=args.host, port=args.web_port, threaded=True)


if __name__ == '__main__':
    main()
