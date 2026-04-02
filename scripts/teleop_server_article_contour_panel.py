#!/usr/bin/env python3
"""Minimal web panel for contour-based line following, inspired by
Constantin Toporov's Medium article, but adapted to the JetCar/WAVE ROVER setup.

Keeps the project camera/serial setup simple and focuses on:
- live RGB
- grayscale / ROI view
- balanced binary mask
- contour + vector overlay
- simple auto drive using calibrated motor presets
"""

from __future__ import annotations

import argparse
import atexit
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from io import BytesIO
from typing import Any

import cv2
from flask import Flask, Response, jsonify, request
import numpy as np
from PIL import Image
import serial

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetcar.camera import open_camera, read_rgb_frame
from jetcar.hardware import resolve_camera_selection, resolve_serial_port


CALIBRATION_PATH = PROJECT_ROOT / ".jetcar_motor_calibration.json"
SETTINGS_PATH = PROJECT_ROOT / ".contour_follow_settings.json"

DEFAULT_CALIBRATION = {
    "forward": {"left": 0.14, "right": 0.14, "time": 0.20, "note": "steady forward pair"},
    "rotate_left": {"left": -0.50, "right": 0.50, "time": 0.05, "note": "pure rotate left"},
    "rotate_right": {"left": 0.50, "right": -0.50, "time": 0.05, "note": "pure rotate right"},
    "forward_nudge": {"left": 0.15, "right": 0.15, "time": 0.15, "note": "short forward refresh after rotate"},
}

SETTINGS_DEFAULTS = {
    "crop_top_ratio": 0.55,
    "threshold": 120,
    "threshold_min": 40,
    "threshold_max": 180,
    "threshold_step": 10,
    "th_iterations": 10,
    "white_min": 3.0,
    "white_max": 10.0,
    "turn_angle_deg": 45.0,
    "shift_max_px": 20.0,
    "straight_pause_s": 0.05,
    "nudge_after_turn": True,
    "turn_step_scale": 2.0,
    "erode_iterations": 1,
    "roi_bottom_width": 0.88,
    "roi_top_width": 0.34,
    "roi_top_y": 0.18,
    "contour_min_area": 120.0,
    "border_margin_ratio": 0.06,
    "corner_margin_ratio": 0.18,
    "center_weight": 70.0,
    "anchor_weight": 95.0,
    "fill_min": 0.05,
}

HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>JetCar Contour Follow Panel</title>
  <style>
    :root {
      --bg0: #f4efe7;
      --bg1: #e5efe6;
      --ink: #162025;
      --muted: #5e686e;
      --card: rgba(255,255,255,0.85);
      --line: rgba(22,32,37,0.12);
      --accent: #2d7a66;
      --danger: #b82929;
      --shadow: 0 20px 50px rgba(24, 36, 41, 0.10);
    }
    * { box-sizing: border-box; }
    body {
      font-family: "Segoe UI", sans-serif;
      margin: 0;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(216,88,61,0.14), transparent 32%),
        radial-gradient(circle at top right, rgba(45,122,102,0.14), transparent 28%),
        linear-gradient(135deg, var(--bg0), var(--bg1));
    }
    .wrap { max-width: 1280px; margin: 0 auto; }
    .hero { padding: 22px 24px 10px; }
    .hero h1 { margin: 0 0 8px; font-size: 30px; }
    .hero p { margin: 0; color: var(--muted); max-width: 900px; }
    .row { display: flex; gap: 22px; align-items: flex-start; flex-wrap: wrap; padding: 12px 24px 28px; }
    .card {
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 18px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(8px);
    }
    .panel { flex: 1 1 320px; }
    .panel.stream { flex-basis: 380px; }
    .panel.controls { flex-basis: 320px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; margin-top: 14px; }
    .stat {
      padding: 12px 14px;
      border-radius: 14px;
      background: rgba(255,255,255,0.72);
      border: 1px solid var(--line);
    }
    .label { font-size: 12px; text-transform: uppercase; letter-spacing: 0.08em; color: var(--muted); }
    .value { margin-top: 6px; font-size: 15px; font-weight: 600; word-break: break-word; }
    .big { font-size: 17px; font-weight: 700; }
    .mono { font-family: monospace; white-space: pre-wrap; font-size: 13px; }
    button {
      padding: 11px 16px;
      border: 0;
      border-radius: 12px;
      cursor: pointer;
      background: var(--accent);
      color: white;
      font-weight: 600;
    }
    .danger { background: var(--danger); }
    .row-buttons { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 12px; }
    img { width: 100%; border-radius: 14px; background: #111; aspect-ratio: 16 / 9; object-fit: cover; }
    .mini-grid { display: grid; grid-template-columns: repeat(2, minmax(120px, 1fr)); gap: 10px; margin-top: 12px; }
    input[type=range], input[type=number] { width: 100%; }
    details.drawer {
      border: 1px solid var(--line);
      border-radius: 16px;
      background: rgba(255,255,255,0.62);
      padding: 10px 12px;
    }
    details.drawer summary {
      cursor: pointer;
      font-weight: 700;
      list-style: none;
      display: flex;
      justify-content: space-between;
    }
    details.drawer summary::-webkit-details-marker { display: none; }
    details.drawer summary::after { content: 'Show'; font-size: 12px; color: var(--muted); text-transform: uppercase; }
    details.drawer[open] summary::after { content: 'Hide'; }
    .drawer-body { margin-top: 14px; }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hero">
      <h1>JetCar Contour Follow Panel</h1>
      <p>Contour-based line following inspired by the article: grayscale threshold, balanced ROI, largest contour, vector angle, and shift-based decision making.</p>
      <div class="grid">
        <div class="stat"><div class="label">Server</div><div class="value" id="serverState">starting</div></div>
        <div class="stat"><div class="label">Camera</div><div class="value" id="cameraState">checking</div></div>
        <div class="stat"><div class="label">Serial</div><div class="value" id="serialState">checking</div></div>
        <div class="stat"><div class="label">Auto</div><div class="value" id="autoState">idle</div></div>
      </div>
    </div>

    <div class="row">
      <div class="card panel controls">
        <div class="big">Controls</div>
        <div class="row-buttons">
          <button id="startAuto">Auto Start</button>
          <button id="stopAuto" class="danger">Stop Auto</button>
          <button id="stop" class="danger">Stop Motors</button>
        </div>
        <div class="row-buttons">
          <button id="runForward">Test Forward</button>
          <button id="runLeft">Test Rotate Left</button>
          <button id="runRight">Test Rotate Right</button>
          <button id="runNudge">Test Nudge</button>
        </div>
        <p class="mono" id="statusBox">waiting...</p>
        <details class="drawer">
          <summary>Detection Settings</summary>
          <div class="drawer-body">
            <div class="mini-grid">
              <div><label>Crop Top Ratio<br><input id="cropTop" type="range" min="0.20" max="0.80" step="0.01"></label></div>
              <div><label>Threshold<br><input id="threshold" type="range" min="0" max="255" step="1"></label></div>
              <div><label>White Min %<br><input id="whiteMin" type="number" min="0" max="50" step="0.5"></label></div>
              <div><label>White Max %<br><input id="whiteMax" type="number" min="0" max="50" step="0.5"></label></div>
              <div><label>Turn Angle Deg<br><input id="turnAngle" type="number" min="5" max="89" step="1"></label></div>
              <div><label>Shift Max Px<br><input id="shiftMax" type="number" min="1" max="200" step="1"></label></div>
            </div>
            <div class="row-buttons">
              <button id="saveSettings">Save Settings</button>
              <button id="resetSettings">Reset Settings</button>
            </div>
          </div>
        </details>
      </div>

      <div class="card panel stream">
        <div class="big">Live RGB</div>
        <img id="rgb" alt="Live RGB">
      </div>
      <div class="card panel stream">
        <div class="big">Grayscale ROI</div>
        <img id="gray" alt="Grayscale ROI">
      </div>
      <div class="card panel stream">
        <div class="big">Balanced Binary Mask</div>
        <img id="mask" alt="Binary mask">
      </div>
      <div class="card panel stream">
        <div class="big">Contour / Vector Debug</div>
        <img id="overlay" alt="Contour overlay">
      </div>
      <div class="card panel" style="flex-basis: 100%;">
        <div class="big">Detection Metrics</div>
        <p class="mono" id="metrics">no analysis yet</p>
      </div>
    </div>
  </div>

  <script>
    let autoEnabled = false;
    let frameRequestInFlight = false;
    let statusRequestInFlight = false;
    let frameTimer = null;
    let statusTimer = null;
    const frameObjectUrls = new Map();

    function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

    async function fetchJson(url, options = {}, timeoutMs = 1800) {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), timeoutMs);
      try {
        const response = await fetch(url, {...options, signal: controller.signal});
        return await response.json();
      } finally {
        clearTimeout(timeoutId);
      }
    }

    function formatError(err) {
      if (err && err.name === 'AbortError') return 'request timed out';
      return String(err);
    }

    async function refreshPanelImage(id, path, t) {
      const response = await fetch(`${path}?t=${t}`, {cache: 'no-store'});
      if (!response.ok) throw new Error(`${id} ${response.status}`);
      const blob = await response.blob();
      const nextUrl = URL.createObjectURL(blob);
      const previousUrl = frameObjectUrls.get(id);
      document.getElementById(id).src = nextUrl;
      frameObjectUrls.set(id, nextUrl);
      if (previousUrl) URL.revokeObjectURL(previousUrl);
    }

    async function refreshFramesOnce() {
      if (frameRequestInFlight) return;
      frameRequestInFlight = true;
      const t = Date.now();
      try {
        await Promise.all([
          refreshPanelImage('rgb', '/camera/rgb.jpg', t),
          refreshPanelImage('gray', '/camera/gray.jpg', t),
          refreshPanelImage('mask', '/camera/mask.jpg', t),
          refreshPanelImage('overlay', '/camera/overlay.jpg', t),
        ]);
      } catch (err) {
        document.getElementById('statusBox').innerText = `frame refresh failed: ${formatError(err)}`;
      } finally {
        frameRequestInFlight = false;
      }
    }

    function applySettings(settings) {
      document.getElementById('cropTop').value = settings.crop_top_ratio;
      document.getElementById('threshold').value = settings.threshold;
      document.getElementById('whiteMin').value = settings.white_min;
      document.getElementById('whiteMax').value = settings.white_max;
      document.getElementById('turnAngle').value = settings.turn_angle_deg;
      document.getElementById('shiftMax').value = settings.shift_max_px;
    }

    function settingsFromControls() {
      return {
        crop_top_ratio: parseFloat(document.getElementById('cropTop').value),
        threshold: parseInt(document.getElementById('threshold').value, 10),
        white_min: parseFloat(document.getElementById('whiteMin').value),
        white_max: parseFloat(document.getElementById('whiteMax').value),
        turn_angle_deg: parseFloat(document.getElementById('turnAngle').value),
        shift_max_px: parseFloat(document.getElementById('shiftMax').value),
      };
    }

    async function refreshStatus() {
      if (statusRequestInFlight) return;
      statusRequestInFlight = true;
      try {
        const data = await fetchJson('/api/status', {}, 1500);
        document.getElementById('serverState').innerText = 'ready';
        document.getElementById('cameraState').innerText = data.camera.ok ? 'ready' : `offline: ${data.camera.error}`;
        document.getElementById('serialState').innerText = data.serial.ok ? `ready: ${data.serial.port}` : `offline: ${data.serial.error}`;
        autoEnabled = !!(data.auto && data.auto.running);
        document.getElementById('autoState').innerText = autoEnabled ? data.auto.last_status : 'idle';
        if (data.settings) applySettings(data.settings);
        if (data.analysis) {
          const a = data.analysis;
          document.getElementById('metrics').innerText =
            `threshold_used=${a.threshold_used}\n` +
            `white_percent=${a.white_percent.toFixed(2)}\n` +
            `line_found=${a.line_found}\n` +
            `angle_deg=${a.angle_deg === null ? '-' : a.angle_deg.toFixed(2)}\n` +
            `shift_px=${a.shift_px === null ? '-' : a.shift_px.toFixed(1)}\n` +
            `turn_state=${a.turn_state}\n` +
            `shift_state=${a.shift_state}\n` +
            `turn_dir=${a.turn_dir}\n` +
            `turn_val=${a.turn_val.toFixed(3)}\n` +
            `decision=${a.decision}`;
        }
      } catch (err) {
        document.getElementById('serverState').innerText = `error: ${formatError(err)}`;
      } finally {
        statusRequestInFlight = false;
      }
    }

    function restartFrames() {
      if (frameTimer) clearInterval(frameTimer);
      frameTimer = setInterval(refreshFramesOnce, 220);
    }

    function restartStatus() {
      if (statusTimer) clearInterval(statusTimer);
      refreshStatus();
      statusTimer = setInterval(refreshStatus, 800);
    }

    document.getElementById('startAuto').addEventListener('click', async () => {
      const data = await fetchJson('/api/auto/start', {method: 'POST'});
      document.getElementById('statusBox').innerText = JSON.stringify(data, null, 2);
      refreshStatus();
    });
    document.getElementById('stopAuto').addEventListener('click', async () => {
      const data = await fetchJson('/api/auto/stop', {method: 'POST'});
      document.getElementById('statusBox').innerText = JSON.stringify(data, null, 2);
      refreshStatus();
    });
    document.getElementById('stop').addEventListener('click', async () => {
      const data = await fetchJson('/api/stop', {method: 'POST'});
      document.getElementById('statusBox').innerText = JSON.stringify(data, null, 2);
      refreshStatus();
    });
    document.getElementById('runForward').addEventListener('click', async () => {
      const data = await fetchJson('/api/calibration/run/forward', {method: 'POST'});
      document.getElementById('statusBox').innerText = JSON.stringify(data, null, 2);
    });
    document.getElementById('runLeft').addEventListener('click', async () => {
      const data = await fetchJson('/api/calibration/run/rotate_left', {method: 'POST'});
      document.getElementById('statusBox').innerText = JSON.stringify(data, null, 2);
    });
    document.getElementById('runRight').addEventListener('click', async () => {
      const data = await fetchJson('/api/calibration/run/rotate_right', {method: 'POST'});
      document.getElementById('statusBox').innerText = JSON.stringify(data, null, 2);
    });
    document.getElementById('runNudge').addEventListener('click', async () => {
      const data = await fetchJson('/api/calibration/run/forward_nudge', {method: 'POST'});
      document.getElementById('statusBox').innerText = JSON.stringify(data, null, 2);
    });
    document.getElementById('saveSettings').addEventListener('click', async () => {
      const data = await fetchJson('/api/settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(settingsFromControls()),
      });
      document.getElementById('statusBox').innerText = JSON.stringify(data, null, 2);
      refreshStatus();
    });
    document.getElementById('resetSettings').addEventListener('click', async () => {
      const data = await fetchJson('/api/settings/reset', {method: 'POST'});
      document.getElementById('statusBox').innerText = JSON.stringify(data, null, 2);
      refreshStatus();
    });

    restartFrames();
    restartStatus();
    refreshFramesOnce();
  </script>
</body>
</html>
"""


class JSONStore:
    def __init__(self, path: Path, defaults: dict[str, Any]) -> None:
        self._path = path
        self._defaults = json.loads(json.dumps(defaults))
        self._data = json.loads(json.dumps(defaults))
        self._lock = threading.Lock()
        self._load()

    def _load(self) -> None:
        try:
            if self._path.exists():
                raw = json.loads(self._path.read_text())
                self._data = self._merge(json.loads(json.dumps(self._defaults)), raw)
        except Exception:
            self._data = json.loads(json.dumps(self._defaults))

    @staticmethod
    def _merge(base: Any, override: Any) -> Any:
        if isinstance(base, dict) and isinstance(override, dict):
            out = dict(base)
            for k, v in override.items():
                out[k] = JSONStore._merge(out.get(k), v) if k in out else v
            return out
        return override

    def get(self) -> dict[str, Any]:
        with self._lock:
            return json.loads(json.dumps(self._data))

    def update(self, values: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._data = self._merge(self._data, values)
            try:
                self._path.write_text(json.dumps(self._data, indent=2))
            except Exception:
                pass
            return json.loads(json.dumps(self._data))

    def reset(self) -> dict[str, Any]:
        with self._lock:
            self._data = json.loads(json.dumps(self._defaults))
            try:
                self._path.write_text(json.dumps(self._data, indent=2))
            except Exception:
                pass
            return json.loads(json.dumps(self._data))


class RoverSerial:
    def __init__(self, port: str, baudrate: int) -> None:
        if not hasattr(serial, "Serial"):
            raise RuntimeError("pyserial is not installed correctly.")
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=0.2)
        self._lock = threading.Lock()

    def send(self, left: float, right: float) -> dict[str, float]:
        payload = {"T": 1, "L": float(np.clip(left, -0.5, 0.5)), "R": float(np.clip(right, -0.5, 0.5))}
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        with self._lock:
            self._ser.write(line.encode("utf-8"))
            self._ser.flush()
        return payload

    def pulse(self, left: float, right: float, duration_s: float, refresh_s: float = 0.04) -> dict[str, Any]:
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
        return {"drive": payload, "stop": stop_payload, "duration_s": duration_s, "refresh_count": refresh_count}

    def close(self) -> None:
        with self._lock:
            if self._ser.is_open:
                self._ser.close()


class CameraAnalyzer:
    def __init__(
        self,
        source: str,
        sensor_id: int,
        device_index: int,
        width: int,
        height: int,
        warmup_frames: int,
        settings_store: JSONStore,
    ) -> None:
        self._cap = open_camera(
            source=source,
            sensor_id=sensor_id,
            device_index=device_index,
            width=width,
            height=height,
            warmup_frames=warmup_frames,
        )
        self._settings_store = settings_store
        self._lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_analysis: dict[str, Any] = self._empty_analysis()
        self._jpeg_cache: dict[str, bytes] = {}
        self._stop = threading.Event()
        self._last_target_x: float | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @staticmethod
    def _empty_analysis() -> dict[str, Any]:
        return {
            "line_found": False,
            "threshold_used": SETTINGS_DEFAULTS["threshold"],
            "white_percent": 0.0,
            "angle_deg": None,
            "shift_px": None,
            "turn_state": 0,
            "shift_state": 0,
            "turn_dir": 0,
            "turn_val": 0.0,
            "decision": "wait",
            "roi_center_x": None,
        }

    @staticmethod
    def _encode_jpeg(image: np.ndarray) -> bytes:
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        with BytesIO() as buf:
            Image.fromarray(rgb).save(buf, format="JPEG", quality=82)
            return buf.getvalue()

    def latest_analysis(self) -> dict[str, Any]:
        with self._lock:
            return json.loads(json.dumps(self._latest_analysis))

    def jpeg_bytes(self, kind: str) -> bytes:
        with self._lock:
            payload = self._jpeg_cache.get(kind)
            if payload is not None:
                return payload
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        return self._encode_jpeg(image)

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._cap.release()

    def _balance_mask(self, gray_roi: np.ndarray, settings: dict[str, Any]) -> tuple[np.ndarray, int, float]:
        T = int(settings["threshold"])
        threshold_min = int(settings["threshold_min"])
        threshold_max = int(settings["threshold_max"])
        threshold_step = int(settings["threshold_step"])
        th_iterations = int(settings["th_iterations"])
        white_min = float(settings["white_min"])
        white_max = float(settings["white_max"])
        ret_mask = None
        used_threshold = T
        direction = 0
        roi_area = max(1, gray_roi.shape[0] * gray_roi.shape[1])
        white_percent = 0.0
        for _ in range(th_iterations):
            _, mask = cv2.threshold(gray_roi, T, 255, cv2.THRESH_BINARY)
            nwh = cv2.countNonZero(mask)
            perc = 100.0 * float(nwh) / float(roi_area)
            white_percent = perc
            used_threshold = T
            if perc > white_max:
                if T >= threshold_max:
                    ret_mask = mask
                    break
                if direction == -1:
                    ret_mask = mask
                    break
                T += threshold_step
                direction = 1
            elif perc < white_min:
                if T <= threshold_min:
                    ret_mask = mask
                    break
                if direction == 1:
                    ret_mask = mask
                    break
                T -= threshold_step
                    
                direction = -1
            else:
                ret_mask = mask
                break
        if ret_mask is None:
            _, ret_mask = cv2.threshold(gray_roi, used_threshold, 255, cv2.THRESH_BINARY)
        return ret_mask, used_threshold, white_percent

    @staticmethod
    def _get_turn(turn_state: int, shift_state: int, shift_step: float, turn_step: float) -> tuple[int, float]:
        turn_dir = 0
        turn_val = 0.0
        if shift_state != 0:
            turn_dir = shift_state
            turn_val = shift_step if shift_state != turn_state else turn_step
        elif turn_state != 0:
            turn_dir = turn_state
            turn_val = turn_step
        return int(turn_dir), float(turn_val)

    @staticmethod
    def _build_center_trapezoid(shape: tuple[int, int], settings: dict[str, Any]) -> np.ndarray:
        h, w = shape[:2]
        bottom_w = float(np.clip(settings.get("roi_bottom_width", 0.88), 0.40, 1.00))
        top_w = float(np.clip(settings.get("roi_top_width", 0.34), 0.10, bottom_w))
        top_y_ratio = float(np.clip(settings.get("roi_top_y", 0.18), 0.02, 0.70))
        y_top = int(round(h * top_y_ratio))
        y_top = max(0, min(h - 1, y_top))
        x_bl = int(round(w * (0.5 - bottom_w / 2.0)))
        x_br = int(round(w * (0.5 + bottom_w / 2.0)))
        x_tl = int(round(w * (0.5 - top_w / 2.0)))
        x_tr = int(round(w * (0.5 + top_w / 2.0)))
        poly = np.array([
            [x_bl, h - 1],
            [x_br, h - 1],
            [x_tr, y_top],
            [x_tl, y_top],
        ], dtype=np.int32)
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillConvexPoly(roi_mask, poly, 255)
        return roi_mask

    def _select_best_contour(self, contours: list[np.ndarray], roi_shape: tuple[int, int], settings: dict[str, Any]) -> tuple[np.ndarray | None, float, dict[str, Any]]:
        h, w = roi_shape[:2]
        min_area = float(settings.get("contour_min_area", 120.0))
        border_margin = int(round(w * float(settings.get("border_margin_ratio", 0.06))))
        corner_margin = float(settings.get("corner_margin_ratio", 0.18))
        center_weight = float(settings.get("center_weight", 70.0))
        anchor_weight = float(settings.get("anchor_weight", 95.0))
        fill_min = float(settings.get("fill_min", 0.05))
        anchor_x = self._last_target_x if self._last_target_x is not None else (w / 2.0)

        best_contour = None
        best_area = 0.0
        best_score = None
        stats = {
            "candidate_count": int(len(contours)),
            "filtered_count": 0,
            "anchor_x": float(anchor_x),
        }

        for c in contours:
            area = float(cv2.contourArea(c))
            if area < min_area:
                continue
            x, y, cw, ch = cv2.boundingRect(c)
            if cw <= 0 or ch <= 0:
                continue
            cx = x + cw / 2.0
            cy = y + ch / 2.0
            aspect = ch / float(cw)
            fill = area / float(max(1, cw * ch))
            if fill < fill_min:
                continue
            if ch < max(12.0, h * 0.08):
                continue

            border_touch = x <= border_margin or (x + cw) >= (w - border_margin)
            bottom_corner = (y + ch) >= h * 0.82 and (cx <= w * corner_margin or cx >= w * (1.0 - corner_margin))
            center_dist = abs(cx - (w / 2.0)) / max(1.0, w / 2.0)
            anchor_dist = abs(cx - anchor_x) / max(1.0, w / 2.0)
            lower_reach = (y + ch) / float(max(1, h))

            score = (
                area * 0.015
                + min(ch, h) * 0.9
                + min(aspect, 6.0) * 10.0
                + min(fill, 1.0) * 12.0
                + lower_reach * 14.0
                - center_dist * center_weight
                - anchor_dist * anchor_weight
            )
            if border_touch:
                score -= 45.0
            if bottom_corner:
                score -= 80.0
            if aspect < 0.45:
                score -= 28.0
            if cw > w * 0.45:
                score -= 38.0

            stats["filtered_count"] += 1
            if best_score is None or score > best_score:
                best_score = score
                best_contour = c
                best_area = area
                stats.update({
                    "best_score": float(score),
                    "best_cx": float(cx),
                    "best_cy": float(cy),
                    "best_fill": float(fill),
                    "best_aspect": float(aspect),
                })

        return best_contour, best_area, stats

    @staticmethod
    def _scanline_points(mask: np.ndarray) -> list[tuple[float, int]]:
        h, w = mask.shape[:2]
        rows = [int(round(h * 0.88)), int(round(h * 0.68)), int(round(h * 0.48))]
        pts: list[tuple[float, int]] = []
        for y in rows:
            y = max(0, min(h - 1, y))
            xs = np.where(mask[y] > 0)[0]
            if len(xs) >= max(10, int(w * 0.012)):
                pts.append((float(xs.mean()), y))
        return pts

    def _analyze(self, frame_bgr: np.ndarray, settings: dict[str, Any]) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
        h, w = frame_bgr.shape[:2]
        crop_top = int(np.clip(float(settings["crop_top_ratio"]), 0.2, 0.85) * h)
        crop_top = max(0, min(h - 1, crop_top))

        blur = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        roi_gray = gray[crop_top:, :]

        mask_roi, threshold_used, white_percent = self._balance_mask(roi_gray, settings)
        kernel = np.ones((3, 3), np.uint8)
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, kernel)
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_CLOSE, kernel)
        erode_iterations = int(np.clip(settings.get("erode_iterations", 1), 0, 3))
        if erode_iterations > 0:
            mask_roi = cv2.erode(mask_roi, kernel, iterations=erode_iterations)
        trapezoid = self._build_center_trapezoid(mask_roi.shape, settings)
        mask_roi = cv2.bitwise_and(mask_roi, trapezoid)

        contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_contour, best_area, contour_stats = self._select_best_contour(contours, mask_roi.shape, settings)

        overlay = frame_bgr.copy()
        cv2.line(overlay, (w // 2, crop_top), (w // 2, h - 1), (255, 255, 0), 2)
        gray_view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        gray_view[:crop_top, :] = (gray_view[:crop_top, :] * 0.20).astype(np.uint8)
        cv2.line(gray_view, (0, crop_top), (w - 1, crop_top), (255, 255, 0), 1)

        trap_vis = np.zeros_like(mask_roi)
        trap_vis[trapezoid > 0] = 80
        full_mask = np.zeros_like(gray)
        if best_contour is not None and best_area >= contour_stats.get("best_score", -1e9):
            selected_mask = np.zeros_like(mask_roi)
            cv2.drawContours(selected_mask, [best_contour], -1, 255, thickness=-1)
            full_mask[crop_top:, :] = selected_mask
        else:
            full_mask[crop_top:, :] = mask_roi
        mask_view = cv2.cvtColor(full_mask, cv2.COLOR_GRAY2BGR)
        mask_view[crop_top:, :, 1] = np.maximum(mask_view[crop_top:, :, 1], trap_vis)

        roi_poly = np.column_stack(np.where(trapezoid > 0))
        if roi_poly.size:
            poly_pts = np.array([
                [int(round(w * (0.5 - float(settings.get("roi_bottom_width", 0.88)) / 2.0))), h - 1],
                [int(round(w * (0.5 + float(settings.get("roi_bottom_width", 0.88)) / 2.0))), h - 1],
                [int(round(w * (0.5 + float(settings.get("roi_top_width", 0.34)) / 2.0))), crop_top + int(round((h - crop_top) * float(settings.get("roi_top_y", 0.18))))],
                [int(round(w * (0.5 - float(settings.get("roi_top_width", 0.34)) / 2.0))), crop_top + int(round((h - crop_top) * float(settings.get("roi_top_y", 0.18))))],
            ], dtype=np.int32)
            cv2.polylines(overlay, [poly_pts], True, (80, 180, 255), 1)

        analysis = {
            "line_found": False,
            "threshold_used": int(threshold_used),
            "white_percent": float(white_percent),
            "angle_deg": None,
            "shift_px": None,
            "turn_state": 0,
            "shift_state": 0,
            "turn_dir": 0,
            "turn_val": 0.0,
            "decision": "wait",
            "roi_center_x": float(w / 2.0),
            "crop_top_px": int(crop_top),
            "largest_area": float(best_area),
            **contour_stats,
        }

        if best_contour is not None and best_area >= float(settings.get("contour_min_area", 120.0)):
            analysis["line_found"] = True
            best_contour_full = best_contour.copy()
            best_contour_full[:, 0, 1] += crop_top
            cv2.drawContours(overlay, [best_contour_full], -1, (0, 255, 0), 2)
            x, y, cw, ch = cv2.boundingRect(best_contour)
            y_full = y + crop_top
            cv2.rectangle(overlay, (x, y_full), (x + cw, y_full + ch), (0, 128, 255), 2)

            selected_mask = np.zeros_like(mask_roi)
            cv2.drawContours(selected_mask, [best_contour], -1, 255, thickness=-1)
            scan_pts = self._scanline_points(selected_mask)
            full_mask[:] = 0
            full_mask[crop_top:, :] = selected_mask
            mask_view = cv2.cvtColor(full_mask, cv2.COLOR_GRAY2BGR)
            mask_view[crop_top:, :, 1] = np.maximum(mask_view[crop_top:, :, 1], trap_vis)

            x_near = x_mid = x_far = None
            if len(scan_pts) >= 1:
                x_near = scan_pts[0][0]
            if len(scan_pts) >= 2:
                x_mid = scan_pts[1][0]
                x_far = scan_pts[-1][0]
            elif len(scan_pts) == 1:
                x_far = scan_pts[0][0]
            if x_near is None:
                x_near = x + cw / 2.0
            if x_far is None:
                x_far = x + cw / 2.0
            if x_mid is None:
                x_mid = 0.5 * (x_near + x_far)

            target_x = 0.25 * float(x_near) + 0.75 * float(x_far)
            self._last_target_x = float(target_x)
            shift_px = target_x - (w / 2.0)
            analysis["shift_px"] = float(shift_px)
            analysis["target_x"] = float(target_x)
            analysis["x_near"] = float(x_near)
            analysis["x_mid"] = float(x_mid)
            analysis["x_far"] = float(x_far)

            pts = best_contour.reshape(-1, 2).astype(np.float32)
            vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            vx = float(vx)
            vy = float(vy)
            x0 = float(x0)
            y0 = float(y0)
            angle_deg = float(np.degrees(np.arctan2(vy, vx)))
            if angle_deg < 0:
                angle_deg += 180.0
            analysis["angle_deg"] = angle_deg

            draw_len = max(cw, ch, 60)
            pt1 = (int(round(x0 - vx * draw_len)), int(round(y0 - vy * draw_len + crop_top)))
            pt2 = (int(round(x0 + vx * draw_len)), int(round(y0 + vy * draw_len + crop_top)))
            cv2.line(overlay, pt1, pt2, (255, 0, 255), 2)
            cv2.circle(overlay, (int(round(target_x)), int(round(y_full + ch / 2.0))), 5, (0, 0, 255), -1)

            for idx, (scan_x, scan_y) in enumerate(scan_pts):
                y_draw = crop_top + int(scan_y)
                cv2.circle(overlay, (int(round(scan_x)), y_draw), 4, (0, 255, 255), -1)
                cv2.line(overlay, (0, y_draw), (w - 1, y_draw), (60, 60, 60), 1)
                cv2.circle(mask_view, (int(round(scan_x)), y_draw), 4, (0, 255, 255), -1)

            turn_state = 0
            turn_angle_deg = float(settings["turn_angle_deg"])
            if angle_deg < turn_angle_deg or angle_deg > 180.0 - turn_angle_deg:
                turn_state = int(np.sign(90.0 - angle_deg))
            shift_state = 0
            shift_max_px = float(settings["shift_max_px"])
            if abs(shift_px) > shift_max_px:
                shift_state = int(np.sign(shift_px))

            rotate_time = float(DEFAULT_CALIBRATION["rotate_left"]["time"])
            turn_step = rotate_time * float(settings["turn_step_scale"])
            shift_step = rotate_time
            turn_dir, turn_val = self._get_turn(turn_state, shift_state, shift_step, turn_step)
            analysis.update({
                "turn_state": int(turn_state),
                "shift_state": int(shift_state),
                "turn_dir": int(turn_dir),
                "turn_val": float(turn_val),
                "decision": "turn" if turn_dir != 0 else "straight",
            })
        else:
            self._last_target_x = None
            analysis["decision"] = "wait"

        return analysis, {
            "rgb": overlay,
            "gray": gray_view,
            "mask": mask_view,
            "overlay": overlay,
        }

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = read_rgb_frame(self._cap)
                settings = self._settings_store.get()
                analysis, debug_images = self._analyze(frame, settings)
                jpeg_cache = {name: self._encode_jpeg(img) for name, img in debug_images.items()}
                with self._lock:
                    self._latest_frame = frame
                    self._latest_analysis = analysis
                    self._jpeg_cache = jpeg_cache
            except Exception:
                time.sleep(0.05)
                continue
            time.sleep(0.03)


class AutoController:
    def __init__(self, rover: RoverSerial | None, analyzer: CameraAnalyzer | None, calibration_store: JSONStore, settings_store: JSONStore) -> None:
        self._rover = rover
        self._analyzer = analyzer
        self._calibration_store = calibration_store
        self._settings_store = settings_store
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.running = False
        self.last_status = "idle"
        self.last_error: str | None = None
        self.last_action: dict[str, Any] = {}

    def status_payload(self) -> dict[str, Any]:
        with self._lock:
            return {
                "running": bool(self.running),
                "last_status": self.last_status,
                "last_error": self.last_error,
                "last_action": json.loads(json.dumps(self.last_action)),
            }

    def start(self) -> dict[str, Any]:
        if self._rover is None:
            raise RuntimeError("Serial is unavailable.")
        if self._analyzer is None:
            raise RuntimeError("Camera is unavailable.")
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("Auto is already running.")
            self._stop.clear()
            self.running = True
            self.last_status = "starting"
            self.last_error = None
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        return self.status_payload()

    def stop(self) -> dict[str, Any]:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None
        if self._rover is not None:
            try:
                self._rover.send(0.0, 0.0)
            except Exception:
                pass
        with self._lock:
            self.running = False
            self.last_status = "stopped"
        return self.status_payload()

    def _run_preset(self, name: str) -> dict[str, Any]:
        if self._rover is None:
            raise RuntimeError("Serial unavailable.")
        preset = self._calibration_store.get().get(name)
        if not preset:
            raise RuntimeError(f"missing calibration preset: {name}")
        return self._rover.pulse(float(preset["left"]), float(preset["right"]), float(preset["time"]))

    def _run(self) -> None:
        straight_pause_s = lambda: float(self._settings_store.get().get("straight_pause_s", 0.05))
        while not self._stop.is_set():
            try:
                analysis = self._analyzer.latest_analysis() if self._analyzer is not None else {}
                if not analysis.get("line_found"):
                    if self._rover is not None:
                        self._rover.send(0.0, 0.0)
                    with self._lock:
                        self.last_status = "waiting for line"
                        self.last_action = {"decision": "wait", "reason": "no contour"}
                    time.sleep(0.06)
                    continue

                turn_dir = int(analysis.get("turn_dir", 0))
                turn_val = float(analysis.get("turn_val", 0.0))
                calibration = self._calibration_store.get()
                if turn_dir == 0:
                    forward = calibration["forward"]
                    if self._rover is not None:
                        self._rover.send(float(forward["left"]), float(forward["right"]))
                    with self._lock:
                        self.last_status = "following line (straight)"
                        self.last_action = {"decision": "straight", "preset": "forward", "analysis": analysis}
                    time.sleep(straight_pause_s())
                    continue

                preset_name = "rotate_right" if turn_dir > 0 else "rotate_left"
                preset = calibration[preset_name]
                rotate_time = max(float(preset["time"]), turn_val)
                if self._rover is not None:
                    self._rover.pulse(float(preset["left"]), float(preset["right"]), rotate_time)
                do_nudge = bool(self._settings_store.get().get("nudge_after_turn", True))
                if do_nudge and not self._stop.is_set():
                    nudge = calibration["forward_nudge"]
                    if self._rover is not None:
                        self._rover.pulse(float(nudge["left"]), float(nudge["right"]), float(nudge["time"]))
                with self._lock:
                    self.last_status = "following line (turn+nudge)"
                    self.last_action = {
                        "decision": "turn",
                        "preset": preset_name,
                        "rotate_time": rotate_time,
                        "turn_val": turn_val,
                        "analysis": analysis,
                    }
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)
                    self.last_status = "auto error"
                time.sleep(0.1)
        with self._lock:
            self.running = False


class AppState:
    def __init__(self, rover: RoverSerial | None, rover_error: str | None, analyzer: CameraAnalyzer | None, camera_error: str | None, port: str, baudrate: int, camera_source: str, width: int, height: int, calibration_store: JSONStore, settings_store: JSONStore) -> None:
        self.rover = rover
        self.rover_error = rover_error
        self.analyzer = analyzer
        self.camera_error = camera_error
        self.port = port
        self.baudrate = baudrate
        self.camera_source = camera_source
        self.width = width
        self.height = height
        self.calibration_store = calibration_store
        self.settings_store = settings_store
        self.auto = AutoController(rover, analyzer, calibration_store, settings_store)

    def status_payload(self) -> dict[str, Any]:
        return {
            "serial": {"ok": self.rover is not None, "port": self.port, "baudrate": self.baudrate, "error": self.rover_error},
            "camera": {"ok": self.analyzer is not None, "source": self.camera_source, "width": self.width, "height": self.height, "error": self.camera_error},
            "auto": self.auto.status_payload(),
            "settings": self.settings_store.get(),
            "calibration": self.calibration_store.get(),
            "analysis": self.analyzer.latest_analysis() if self.analyzer is not None else None,
        }

    def close(self) -> None:
        self.auto.stop()
        if self.rover is not None:
            self.rover.close()
        if self.analyzer is not None:
            self.analyzer.close()


def _jpeg_response(payload: bytes) -> Response:
    response = Response(payload, mimetype="image/jpeg")
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response


def free_listening_port(port: int) -> list[int]:
    try:
        result = subprocess.run(["ss", "-ltnp", f"sport = :{port}"], check=False, capture_output=True, text=True)
    except FileNotFoundError:
        return []
    pids = sorted({int(pid) for pid in __import__("re").findall(r"pid=(\d+)", result.stdout)})
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


def create_app(port: str, baudrate: int, camera_source: str, sensor_id: int, device_index: int, width: int, height: int, warmup_frames: int) -> Flask:
    calibration_store = JSONStore(CALIBRATION_PATH, DEFAULT_CALIBRATION)
    settings_store = JSONStore(SETTINGS_PATH, SETTINGS_DEFAULTS)
    resolved_port = resolve_serial_port(port, baudrate=baudrate)
    camera_selection = resolve_camera_selection(camera_source, sensor_id, device_index, width, height, warmup_frames)

    rover = None
    rover_error = None
    try:
        rover = RoverSerial(resolved_port, baudrate)
    except Exception as exc:
        rover_error = str(exc)

    analyzer = None
    camera_error = None
    try:
        analyzer = CameraAnalyzer(
            camera_selection.source,
            camera_selection.sensor_id,
            camera_selection.device_index,
            width,
            height,
            warmup_frames,
            settings_store,
        )
    except Exception as exc:
        camera_error = str(exc)

    state = AppState(
        rover,
        rover_error,
        analyzer,
        camera_error,
        resolved_port,
        baudrate,
        camera_selection.source,
        width,
        height,
        calibration_store,
        settings_store,
    )
    app = Flask(__name__)
    atexit.register(state.close)

    @app.get("/")
    def index() -> str:
        return HTML

    @app.get("/api/status")
    def api_status() -> Response:
        return jsonify(state.status_payload())

    @app.post("/api/auto/start")
    def api_auto_start() -> Response:
        try:
            payload = state.auto.start()
            return jsonify({"status": "auto started", "auto": payload})
        except Exception as exc:
            return jsonify({"status": f"error: {exc}"}), 400

    @app.post("/api/auto/stop")
    def api_auto_stop() -> Response:
        payload = state.auto.stop()
        return jsonify({"status": "auto stopped", "auto": payload})

    @app.post("/api/stop")
    def api_stop() -> Response:
        state.auto.stop()
        if state.rover is None:
            return jsonify({"status": "serial unavailable", "error": state.rover_error}), 503
        payload = state.rover.send(0.0, 0.0)
        return jsonify({"status": "stopped", "payload": payload})

    @app.post("/api/calibration/run/<name>")
    def api_run_calibration(name: str) -> Response:
        if state.rover is None:
            return jsonify({"status": "serial unavailable", "error": state.rover_error}), 503
        preset = state.calibration_store.get().get(name)
        if not preset:
            return jsonify({"status": f"unknown preset: {name}"}), 404
        payload = state.rover.pulse(float(preset["left"]), float(preset["right"]), float(preset["time"]))
        return jsonify({"status": f"ran preset {name}", "preset": preset, "payload": payload})

    @app.get("/api/settings")
    def api_get_settings() -> Response:
        return jsonify(state.settings_store.get())

    @app.post("/api/settings")
    def api_update_settings() -> Response:
        data = request.get_json(force=True)
        updated = state.settings_store.update({
            "crop_top_ratio": float(data.get("crop_top_ratio", SETTINGS_DEFAULTS["crop_top_ratio"])),
            "threshold": int(data.get("threshold", SETTINGS_DEFAULTS["threshold"])),
            "white_min": float(data.get("white_min", SETTINGS_DEFAULTS["white_min"])),
            "white_max": float(data.get("white_max", SETTINGS_DEFAULTS["white_max"])),
            "turn_angle_deg": float(data.get("turn_angle_deg", SETTINGS_DEFAULTS["turn_angle_deg"])),
            "shift_max_px": float(data.get("shift_max_px", SETTINGS_DEFAULTS["shift_max_px"])),
        })
        return jsonify(updated)

    @app.post("/api/settings/reset")
    def api_reset_settings() -> Response:
        return jsonify(state.settings_store.reset())

    @app.get("/camera/rgb.jpg")
    def camera_rgb() -> Response:
        if state.analyzer is None:
            return _jpeg_response(CameraAnalyzer._encode_jpeg(np.zeros((360, 640, 3), dtype=np.uint8)))
        return _jpeg_response(state.analyzer.jpeg_bytes("rgb"))

    @app.get("/camera/gray.jpg")
    def camera_gray() -> Response:
        if state.analyzer is None:
            return _jpeg_response(CameraAnalyzer._encode_jpeg(np.zeros((360, 640, 3), dtype=np.uint8)))
        return _jpeg_response(state.analyzer.jpeg_bytes("gray"))

    @app.get("/camera/mask.jpg")
    def camera_mask() -> Response:
        if state.analyzer is None:
            return _jpeg_response(CameraAnalyzer._encode_jpeg(np.zeros((360, 640, 3), dtype=np.uint8)))
        return _jpeg_response(state.analyzer.jpeg_bytes("mask"))

    @app.get("/camera/overlay.jpg")
    def camera_overlay() -> Response:
        if state.analyzer is None:
            return _jpeg_response(CameraAnalyzer._encode_jpeg(np.zeros((360, 640, 3), dtype=np.uint8)))
        return _jpeg_response(state.analyzer.jpeg_bytes("overlay"))

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run contour-based line-follow panel.")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--camera-source", default="auto", choices=["auto", "usb", "csi"])
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--warmup-frames", type=int, default=12)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    freed = free_listening_port(args.http_port)
    if freed:
        print(f"Freed port {args.http_port} from PIDs: {', '.join(str(pid) for pid in freed)}")
    app = create_app(args.port, args.baudrate, args.camera_source, args.sensor_id, args.device_index, args.width, args.height, args.warmup_frames)
    app.run(host=args.host, port=args.http_port, debug=False, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
