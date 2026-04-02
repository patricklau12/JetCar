#!/usr/bin/env python3
"""Dedicated motor calibration web panel for JetCar / WAVE ROVER.

Purpose:
- Let the user fine-tune raw L/R/T motor presets from a browser.
- Save the presets into a shared JSON file that later auto-drive scripts can load.

Default saved file:
    PROJECT_ROOT/.jetcar_motor_calibration.json

Preset groups:
- forward
- rotate_left
- rotate_right
- forward_nudge
"""

from __future__ import annotations

import argparse
import atexit
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
from typing import Any

from flask import Flask, jsonify, request
import serial

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetcar.hardware import resolve_serial_port

CALIBRATION_DEFAULTS = {
    "forward": {"left": 0.16, "right": 0.16, "time": 0.50, "note": "steady forward pair"},
    "rotate_left": {"left": -0.10, "right": 0.40, "time": 0.30, "note": "pure rotate left"},
    "rotate_right": {"left": 0.40, "right": -0.10, "time": 0.30, "note": "pure rotate right"},
    "forward_nudge": {"left": 0.14, "right": 0.14, "time": 0.18, "note": "short forward refresh after rotate"},
}

HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>JetCar Motor Calibration</title>
  <style>
    :root {
      --bg0: #f4efe7;
      --bg1: #e5efe6;
      --ink: #162025;
      --muted: #5e686e;
      --card: rgba(255,255,255,0.88);
      --line: rgba(22,32,37,0.12);
      --accent: #2d7a66;
      --accent2: #d8583d;
      --danger: #b82929;
      --shadow: 0 20px 50px rgba(24, 36, 41, 0.10);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      color: var(--ink);
      font-family: "Segoe UI", sans-serif;
      background:
        radial-gradient(circle at top left, rgba(216,88,61,0.14), transparent 32%),
        radial-gradient(circle at top right, rgba(45,122,102,0.14), transparent 28%),
        linear-gradient(135deg, var(--bg0), var(--bg1));
    }
    .wrap { max-width: 1280px; margin: 0 auto; padding: 24px; }
    .hero h1 { margin: 0 0 8px; font-size: 34px; }
    .hero p { margin: 0; color: var(--muted); max-width: 900px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; margin-top: 18px; }
    .stat, .card {
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 18px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(8px);
    }
    .stat { padding: 14px 16px; }
    .stat .label { font-size: 12px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; }
    .stat .value { margin-top: 6px; font-size: 15px; font-weight: 600; word-break: break-word; }
    .layout { display: grid; grid-template-columns: 1.05fr 1fr; gap: 22px; margin-top: 20px; }
    .card { padding: 20px; }
    .card h2 { margin: 0 0 10px; font-size: 22px; }
    .small { color: var(--muted); font-size: 13px; }
    .controls { display: grid; grid-template-columns: repeat(3, minmax(120px, 1fr)); gap: 12px; margin-top: 14px; }
    .controls.compact { grid-template-columns: repeat(2, minmax(120px, 1fr)); }
    label { display: block; font-size: 14px; }
    input[type=range] { width: 100%; }
    input[type=text] { width: 100%; padding: 10px 12px; border-radius: 10px; border: 1px solid var(--line); background: rgba(255,255,255,0.72); }
    button {
      padding: 11px 16px;
      border: 0;
      border-radius: 12px;
      background: var(--accent);
      color: white;
      font-weight: 700;
      cursor: pointer;
    }
    button.alt { background: var(--accent2); }
    button.danger { background: var(--danger); }
    .row-buttons { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 14px; }
    .preset-grid { display: grid; grid-template-columns: repeat(2, minmax(240px, 1fr)); gap: 14px; margin-top: 16px; }
    .preset {
      border: 1px solid var(--line);
      border-radius: 14px;
      background: rgba(255,255,255,0.72);
      padding: 14px;
    }
    .preset h3 { margin: 0 0 8px; font-size: 18px; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; white-space: pre-wrap; font-size: 13px; }
    @media (max-width: 980px) { .layout { grid-template-columns: 1fr; } .preset-grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hero">
      <h1>JetCar Motor Calibration Panel</h1>
      <p>Fine-tune raw L / R / T presets here first. Later line-follow scripts can load the saved JSON file directly and use these values as reference for forward, rotate-left, rotate-right, and forward-nudge.</p>
      <div class="grid">
        <div class="stat"><div class="label">Server</div><div class="value" id="serverState">starting</div></div>
        <div class="stat"><div class="label">Serial</div><div class="value" id="serialState">checking</div></div>
        <div class="stat"><div class="label">Port</div><div class="value" id="portState">-</div></div>
        <div class="stat"><div class="label">Calibration File</div><div class="value" id="fileState">-</div></div>
      </div>
    </div>

    <div class="layout">
      <div class="card">
        <h2>Raw Test</h2>
        <p class="small">Use this section to test any raw L / R / T combination directly. L and R are the exact motor values sent to the rover in the range -0.50 to +0.50. T is the test duration in seconds.</p>
        <div class="controls">
          <div><label>Left (L)<br><input id="rawL" type="range" min="-0.50" max="0.50" step="0.01" value="0.16"></label><div class="small" id="rawLVal">0.16</div></div>
          <div><label>Right (R)<br><input id="rawR" type="range" min="-0.50" max="0.50" step="0.01" value="0.16"></label><div class="small" id="rawRVal">0.16</div></div>
          <div><label>Time (T s)<br><input id="rawT" type="range" min="0.05" max="2.00" step="0.05" value="0.50"></label><div class="small" id="rawTVal">0.50</div></div>
        </div>
        <div class="row-buttons">
          <button id="runRaw">Run Raw Test</button>
          <button id="stopNow" class="danger">Stop</button>
          <button id="loadForward">Load Forward</button>
          <button id="loadRotateLeft">Load Rotate Left</button>
          <button id="loadRotateRight">Load Rotate Right</button>
          <button id="loadNudge">Load Forward Nudge</button>
        </div>
        <div class="row-buttons">
          <button id="saveForward">Save As Forward</button>
          <button id="saveRotateLeft">Save As Rotate Left</button>
          <button id="saveRotateRight">Save As Rotate Right</button>
          <button id="saveNudge">Save As Forward Nudge</button>
        </div>
        <div style="margin-top: 14px;"><label>Note<br><input id="rawNote" type="text" value=""></label></div>
        <p class="mono" id="rawStatus">ready</p>
      </div>

      <div class="card">
        <h2>Saved Reference Presets</h2>
        <p class="small">These values are saved to a shared JSON file and intended to be used later by autonomous scripts as motor references.</p>
        <div class="preset-grid" id="presetGrid"></div>
        <div class="row-buttons">
          <button id="runForward">Run Forward</button>
          <button id="runRotateLeft" class="alt">Run Rotate Left</button>
          <button id="runRotateRight" class="alt">Run Rotate Right</button>
          <button id="runNudge">Run Forward Nudge</button>
          <button id="resetDefaults" class="danger">Reset Defaults</button>
        </div>
        <p class="mono" id="jsonView"></p>
      </div>
    </div>
  </div>

  <script>
    const presetNames = ['forward', 'rotate_left', 'rotate_right', 'forward_nudge'];
    let state = null;

    function fmt(v) { return Number(v).toFixed(2); }

    function syncRawReadout() {
      rawLVal.innerText = fmt(rawL.value);
      rawRVal.innerText = fmt(rawR.value);
      rawTVal.innerText = fmt(rawT.value);
    }

    async function fetchJson(url, options={}) {
      const r = await fetch(url, options);
      return await r.json();
    }

    function updatePresetGrid() {
      if (!state || !state.calibration) return;
      presetGrid.innerHTML = presetNames.map((name) => {
        const item = state.calibration[name];
        const title = name.replaceAll('_', ' ');
        return `
          <div class="preset">
            <h3>${title}</h3>
            <div class="mono">L=${fmt(item.left)}\nR=${fmt(item.right)}\nT=${fmt(item.time)}\n${item.note || ''}</div>
          </div>`;
      }).join('');
      jsonView.innerText = JSON.stringify(state.calibration, null, 2);
    }

    function loadPresetToRaw(name) {
      if (!state || !state.calibration || !state.calibration[name]) return;
      const item = state.calibration[name];
      rawL.value = item.left;
      rawR.value = item.right;
      rawT.value = item.time;
      rawNote.value = item.note || '';
      syncRawReadout();
      rawStatus.innerText = `loaded ${name}`;
    }

    async function refreshStatus() {
      state = await fetchJson('/api/status');
      serverState.innerText = 'ready';
      serialState.innerText = state.serial.ok ? `ready: ${state.serial.port}` : `offline: ${state.serial.error}`;
      portState.innerText = `${state.serial.port} @ ${state.serial.baudrate}`;
      fileState.innerText = state.calibration_path;
      updatePresetGrid();
    }

    async function runRaw() {
      const payload = {
        left: parseFloat(rawL.value),
        right: parseFloat(rawR.value),
        duration_s: parseFloat(rawT.value),
      };
      const data = await fetchJson('/api/test/raw', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload)
      });
      rawStatus.innerText = JSON.stringify(data, null, 2);
    }

    async function savePreset(name) {
      const current = state.calibration[name] || {};
      const payload = {
        name,
        values: {
          left: parseFloat(rawL.value),
          right: parseFloat(rawR.value),
          time: parseFloat(rawT.value),
          note: rawNote.value || current.note || ''
        }
      };
      const data = await fetchJson('/api/calibration', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload)
      });
      state = { ...state, calibration: data };
      updatePresetGrid();
      rawStatus.innerText = `saved ${name}`;
    }

    async function runPreset(name) {
      const data = await fetchJson(`/api/test/preset/${name}`, { method: 'POST' });
      rawStatus.innerText = JSON.stringify(data, null, 2);
    }

    async function stopNowFn() {
      const data = await fetchJson('/api/stop', { method: 'POST' });
      rawStatus.innerText = JSON.stringify(data, null, 2);
    }

    async function resetDefaultsFn() {
      const data = await fetchJson('/api/calibration/reset', { method: 'POST' });
      state = { ...state, calibration: data };
      updatePresetGrid();
      rawStatus.innerText = 'reset defaults';
    }

    [rawL, rawR, rawT].forEach((el) => el.addEventListener('input', syncRawReadout));
    runRaw.onclick = runRaw;
    stopNow.onclick = stopNowFn;
    loadForward.onclick = () => loadPresetToRaw('forward');
    loadRotateLeft.onclick = () => loadPresetToRaw('rotate_left');
    loadRotateRight.onclick = () => loadPresetToRaw('rotate_right');
    loadNudge.onclick = () => loadPresetToRaw('forward_nudge');
    saveForward.onclick = () => savePreset('forward');
    saveRotateLeft.onclick = () => savePreset('rotate_left');
    saveRotateRight.onclick = () => savePreset('rotate_right');
    saveNudge.onclick = () => savePreset('forward_nudge');
    runForward.onclick = () => runPreset('forward');
    runRotateLeft.onclick = () => runPreset('rotate_left');
    runRotateRight.onclick = () => runPreset('rotate_right');
    runNudge.onclick = () => runPreset('forward_nudge');
    resetDefaults.onclick = resetDefaultsFn;

    syncRawReadout();
    refreshStatus();
    setInterval(refreshStatus, 2000);
  </script>
</body>
</html>
"""


class RoverSerial:
    def __init__(self, port: str, baudrate: int) -> None:
        if not hasattr(serial, "Serial"):
            module_path = getattr(serial, "__file__", "<unknown>")
            raise RuntimeError(
                "pyserial is not installed correctly. "
                f"Imported module: {module_path}"
            )
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=0.2)
        self._lock = threading.Lock()

    def send(self, left: float, right: float) -> dict[str, float | int]:
        payload = {"T": 1, "L": float(left), "R": float(right)}
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        with self._lock:
            self._ser.write(line.encode("utf-8"))
            self._ser.flush()
        return payload

    def pulse(self, left: float, right: float, duration_s: float, refresh_s: float = 0.06) -> dict[str, object]:
        duration_s = max(0.0, float(duration_s))
        payload = self.send(left, right)
        deadline = time.monotonic() + duration_s
        refresh_count = 1
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(refresh_s, remaining))
            self.send(left, right)
            refresh_count += 1
        stop_payload = self.send(0.0, 0.0)
        return {
            "drive": payload,
            "stop": stop_payload,
            "duration_s": duration_s,
            "refresh_s": refresh_s,
            "refresh_count": refresh_count,
        }

    def close(self) -> None:
        with self._lock:
            if self._ser.is_open:
                self._ser.close()


class MotorCalibrationStore:
    def __init__(self, path: Path = PROJECT_ROOT / ".jetcar_motor_calibration.json") -> None:
        self._path = path
        self._lock = threading.Lock()
        self._data = json.loads(json.dumps(CALIBRATION_DEFAULTS))
        self._load()

    @property
    def path(self) -> Path:
        return self._path

    def _load(self) -> None:
        try:
            if not self._path.exists():
                return
            raw = json.loads(self._path.read_text())
        except Exception:
            return
        for name, defaults in CALIBRATION_DEFAULTS.items():
            if not isinstance(raw.get(name), dict):
                continue
            item = raw[name]
            self._data[name] = {
                "left": float(item.get("left", defaults["left"])),
                "right": float(item.get("right", defaults["right"])),
                "time": float(item.get("time", defaults["time"])),
                "note": str(item.get("note", defaults.get("note", ""))),
            }

    def _save(self) -> None:
        self._path.write_text(json.dumps(self._data, indent=2))

    def get(self) -> dict[str, dict[str, float | str]]:
        with self._lock:
            return json.loads(json.dumps(self._data))

    def set_preset(self, name: str, values: dict[str, Any]) -> dict[str, dict[str, float | str]]:
        if name not in CALIBRATION_DEFAULTS:
            raise KeyError(name)
        with self._lock:
            current = self._data[name]
            current["left"] = float(values.get("left", current["left"]))
            current["right"] = float(values.get("right", current["right"]))
            current["time"] = float(values.get("time", current["time"]))
            current["note"] = str(values.get("note", current.get("note", "")))
            self._save()
            return json.loads(json.dumps(self._data))

    def reset(self) -> dict[str, dict[str, float | str]]:
        with self._lock:
            self._data = json.loads(json.dumps(CALIBRATION_DEFAULTS))
            self._save()
            return json.loads(json.dumps(self._data))


class AppState:
    def __init__(self, rover: RoverSerial | None, rover_error: str | None, port: str, baudrate: int) -> None:
        self.rover = rover
        self.rover_error = rover_error
        self.port = port
        self.baudrate = baudrate
        self.store = MotorCalibrationStore()

    def status_payload(self) -> dict[str, object]:
        return {
            "serial": {
                "ok": self.rover is not None,
                "port": self.port,
                "baudrate": self.baudrate,
                "error": self.rover_error,
            },
            "calibration": self.store.get(),
            "calibration_path": str(self.store.path),
        }

    def close(self) -> None:
        if self.rover is not None:
            self.rover.close()


def free_listening_port(port: int) -> list[int]:
    try:
        result = subprocess.run(
            ["ss", "-ltnp", f"sport = :{port}"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return []

    pids = sorted({int(pid) for pid in re.findall(r"pid=(\d+)", result.stdout)})
    current_pid = os.getpid()
    pids = [pid for pid in pids if pid != current_pid]
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
    if pids:
        time.sleep(0.6)
    return pids


def create_app(port: str, baudrate: int) -> Flask:
    resolved_port = resolve_serial_port(port, baudrate=baudrate)
    rover = None
    rover_error = None
    try:
        rover = RoverSerial(resolved_port, baudrate)
    except Exception as exc:
        rover_error = str(exc)

    state = AppState(rover=rover, rover_error=rover_error, port=resolved_port, baudrate=baudrate)
    app = Flask(__name__)
    atexit.register(state.close)

    @app.get("/")
    def index():
        return HTML

    @app.get("/api/status")
    def api_status():
        return jsonify(state.status_payload())

    @app.post("/api/test/raw")
    def api_test_raw():
        if state.rover is None:
            return jsonify({"status": "serial unavailable", "error": state.rover_error}), 503
        data = request.get_json(force=True)
        left = float(data.get("left", 0.0))
        right = float(data.get("right", 0.0))
        duration_s = float(data.get("duration_s", 0.5))
        payload = state.rover.pulse(left, right, duration_s)
        return jsonify({
            "status": "raw pulse sent",
            "left": left,
            "right": right,
            "duration_s": duration_s,
            "payload": payload,
        })

    @app.post("/api/test/preset/<name>")
    def api_test_preset(name: str):
        if state.rover is None:
            return jsonify({"status": "serial unavailable", "error": state.rover_error}), 503
        presets = state.store.get()
        if name not in presets:
            return jsonify({"status": f"unknown preset: {name}"}), 404
        item = presets[name]
        payload = state.rover.pulse(float(item["left"]), float(item["right"]), float(item["time"]))
        return jsonify({"status": f"ran preset {name}", "preset": item, "payload": payload})

    @app.post("/api/calibration")
    def api_calibration_set():
        data = request.get_json(force=True)
        name = str(data.get("name", ""))
        values = data.get("values", {})
        try:
            updated = state.store.set_preset(name, values)
        except KeyError:
            return jsonify({"status": f"unknown preset: {name}"}), 404
        return jsonify(updated)

    @app.post("/api/calibration/reset")
    def api_calibration_reset():
        return jsonify(state.store.reset())

    @app.post("/api/stop")
    def api_stop():
        if state.rover is None:
            return jsonify({"status": "serial unavailable", "error": state.rover_error}), 503
        payload = state.rover.send(0.0, 0.0)
        return jsonify({"status": "stopped", "payload": payload})

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the JetCar motor calibration web panel.")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--kill-port", action="store_true", default=True)
    parser.add_argument("--no-kill-port", action="store_false", dest="kill_port")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.kill_port:
        freed = free_listening_port(args.http_port)
        if freed:
            print(f"Freed port {args.http_port} from PIDs: {', '.join(str(pid) for pid in freed)}")
    app = create_app(args.port, args.baudrate)
    app.run(host=args.host, port=args.http_port, debug=False, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
