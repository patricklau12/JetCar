#!/usr/bin/env python3
"""Simple browser teleop server for Waveshare WAVE ROVER."""

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
from io import BytesIO

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
from jetcar.motion import MotionCalibration, apply_drive_calibration
from jetcar.vision import build_lane_mask


AUTO_SETTINGS_DEFAULTS = {
    "drive": 0.0,
    "max_drive": 0.055,
    "camera_center_offset": 0.0,
    "k_offset": 0.90,
    "k_heading": 0.70,
    "k_curve": 0.75,
    "turn_mix": 0.45,
    "loop_hz": 7.0,
    "center_tolerance": 0.085,
    "min_steer": 0.10,
    "forward_floor": 0.0,
    "motor_bias": 0.00,
    "step_up": 0.02,
    "stuck_shift_px": 10.0,
    "curve_slowdown": 0.24,
}

TURN_ASSIST_SCALE = 1.25
TURN_MIN_WHEEL_SCALE = 1.25
TURN_OUTER_FORCE_SCALE = 1.35
TURN_OUTER_EXTRA = 0.05
TURN_INNER_RATIO_BASE = 0.70
TURN_INNER_RATIO_MIN = 0.28
TURN_DIFF_BASE = 0.12
TURN_DIFF_MAX = 0.30
TURN_BRAKE_SEVERITY = 0.72
TURN_PIVOT_SEVERITY = 0.90

TURN_ENTER_MULTIPLIER = 1.00
TURN_EXIT_MULTIPLIER = 0.68
TURN_HEADING_ENTER = 0.11
TURN_HEADING_EXIT = 0.07
TURN_CURVE_ENTER = 0.14
TURN_CURVE_EXIT = 0.09
AUTO_OUTPUT_SCALE_FORWARD = 0.78
AUTO_OUTPUT_SCALE_TURN = 0.72
SUNLIGHT_MIN_ROW_RUN = 5
SUNLIGHT_MAX_PATH_JUMP = 52


# Empirically validated turning anchor pairs for the heavier rover payload.
# These are applied after the generic controller computes a target, then blended in
# according to turn severity so auto mode uses tested motor templates rather than
# relying only on guessed differential values.
ARC_LEFT_ANCHOR = (-0.20, 0.50)
ARC_RIGHT_ANCHOR = (0.50, -0.20)
PIVOT_LEFT_ANCHOR = (-0.50, 0.50)
PIVOT_RIGHT_ANCHOR = (0.50, -0.40)
TURN_ANCHOR_BLEND_START = 0.10
TURN_ANCHOR_BLEND_FULL = 0.42
TURN_PIVOT_BLEND_START = 0.56
TURN_PIVOT_BLEND_FULL = 0.82

# Raw forward-turn test pairs used by the manual test buttons.
# "min" is a mild but real turning arc; "max" matches the empirically
# validated stronger arc pair that clearly yaws the heavier rover.
RAW_MIN_ARC_LEFT = (-0.10, 0.40)
RAW_MIN_ARC_RIGHT = (0.40, -0.10)
RAW_MAX_ARC_LEFT = (-0.20, 0.50)
RAW_MAX_ARC_RIGHT = (0.50, -0.20)


HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>JetCar Teleop</title>
  <style>
    :root {
      --bg0: #f4efe7;
      --bg1: #e5efe6;
      --ink: #162025;
      --muted: #5e686e;
      --card: rgba(255,255,255,0.85);
      --line: rgba(22,32,37,0.12);
      --accent: #d8583d;
      --accent-2: #2d7a66;
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
    .wrap { max-width: 1200px; margin: 0 auto; }
    .hero { padding: 28px 24px 12px; }
    .hero h1 { margin: 0 0 8px; font-size: 34px; }
    .hero p { margin: 0; color: var(--muted); max-width: 800px; }
    .row { display: flex; gap: 24px; align-items: flex-start; flex-wrap: wrap; padding: 12px 24px 32px; }
    .card {
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 20px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(8px);
    }
    #pad { touch-action: none; cursor: crosshair; max-width: 100%; height: auto; }
    .big { font-size: 18px; font-weight: 700; }
    .mono { font-family: monospace; white-space: pre-wrap; }
    button {
      padding: 12px 18px;
      border: 0;
      border-radius: 12px;
      cursor: pointer;
      background: var(--accent-2);
      color: white;
      font-weight: 600;
    }
    .danger { background: var(--danger); color: white; }
    .muted { color: var(--muted); }
    .panel { flex: 1 1 320px; }
    .panel.wide { flex-basis: 420px; }
    .panel.stream { flex-basis: 360px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(190px, 1fr)); gap: 12px; margin-top: 16px; }
    .stat {
      padding: 14px 16px;
      border-radius: 14px;
      background: rgba(255,255,255,0.72);
      border: 1px solid var(--line);
    }
    .stat .label { font-size: 12px; text-transform: uppercase; letter-spacing: 0.08em; color: var(--muted); }
    .stat .value { margin-top: 6px; font-size: 15px; font-weight: 600; word-break: break-word; }
    .control-grid { display: grid; gap: 14px; }
    .mini-grid { display: grid; grid-template-columns: repeat(2, minmax(120px, 1fr)); gap: 10px; }
    input[type=range] { width: 100%; }
    img { width: 100%; max-width: 100%; border-radius: 14px; background: #111; aspect-ratio: 16 / 9; object-fit: cover; }
    .clickable { cursor: crosshair; }
    .small { font-size: 13px; color: var(--muted); }
    .row-buttons { display: flex; gap: 10px; flex-wrap: wrap; }
    .pill {
      display: inline-block;
      margin-top: 12px;
      padding: 7px 11px;
      border-radius: 999px;
      background: rgba(22,32,37,0.08);
      color: var(--ink);
      font-size: 13px;
      font-weight: 600;
    }
    .warn {
      margin-top: 14px;
      padding: 14px 16px;
      border-radius: 14px;
      background: rgba(184,41,41,0.08);
      border: 1px solid rgba(184,41,41,0.18);
      color: #842020;
    }
    .sample-bar { display: flex; gap: 12px; flex-wrap: wrap; margin-top: 12px; }
    details.drawer {
      border: 1px solid var(--line);
      border-radius: 16px;
      background: rgba(255,255,255,0.62);
      padding: 10px 12px;
    }
    details.drawer + details.drawer,
    details.drawer + .mono {
      margin-top: 12px;
    }
    details.drawer summary {
      cursor: pointer;
      font-weight: 700;
      list-style: none;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
    }
    details.drawer summary::-webkit-details-marker { display: none; }
    details.drawer summary::after {
      content: 'Show';
      font-size: 12px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }
    details.drawer[open] summary::after {
      content: 'Hide';
    }
    .drawer-body {
      margin-top: 14px;
    }
    .swatch {
      display: inline-flex;
      align-items: center;
      gap: 10px;
      padding: 8px 10px;
      border-radius: 12px;
      border: 1px solid var(--line);
      background: rgba(255,255,255,0.72);
      font-size: 13px;
    }
    .swatch-chip {
      width: 18px;
      height: 18px;
      border-radius: 999px;
      border: 1px solid rgba(22,32,37,0.25);
      background: #ddd;
      display: inline-block;
    }
    .advanced-note { margin-top: 12px; font-size: 12px; color: var(--muted); }
    @media (max-width: 720px) {
      .hero { padding: 20px 16px 8px; }
      .row { padding: 8px 16px 24px; gap: 16px; }
      .hero h1 { font-size: 28px; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hero">
      <h1>JetCar Browser Teleop</h1>
      <p>One page for preflight: check the live camera feed, confirm the lane mask, and nudge the rover with the joystick pad before we move on to data collection and training.</p>
      <div class="grid">
        <div class="stat">
          <div class="label">Server</div>
          <div class="value" id="serverState">starting</div>
        </div>
        <div class="stat">
          <div class="label">Camera</div>
          <div class="value" id="cameraState">checking</div>
        </div>
        <div class="stat">
          <div class="label">Serial</div>
          <div class="value" id="serialState">checking</div>
        </div>
        <div class="stat">
          <div class="label">Address</div>
          <div class="value" id="address">-</div>
        </div>
      </div>
      <div class="pill" id="configLine">Loading configuration...</div>
      <div id="warnings"></div>
    </div>
    <div class="row">
      <div class="card panel">
        <canvas id="pad" width="320" height="320"></canvas>
        <div class="big" id="status">Disconnected</div>
        <div id="metrics"></div>
        <div style="margin-top: 12px;">
          <button id="center">Center</button>
          <button id="startAuto">Auto Start</button>
          <button id="stopAuto" class="danger">Stop Auto</button>
          <button id="stop" class="danger">Stop</button>
        </div>
        <div class="row-buttons" style="margin-top: 12px;">
          <button id="manualForward">Forward</button>
          <button id="manualBack">Back</button>
          <button id="manualLeft">Left</button>
          <button id="manualRight">Right</button>
        </div>
        <div class="row-buttons" style="margin-top: 12px;">
          <button id="pulseForwardLeft">Pulse Fwd Left</button>
          <button id="pulseForwardRight">Pulse Fwd Right</button>
        </div>
        <div class="row-buttons" style="margin-top: 12px;">
          <button id="minForwardLeft">Min Fwd Left</button>
          <button id="minForwardRight">Min Fwd Right</button>
          <button id="maxForwardLeft">Max Fwd Left</button>
          <button id="maxForwardRight">Max Fwd Right</button>
        </div>
        <div class="row-buttons" style="margin-top: 12px;">
          <button id="calForward">Cal Fwd</button>
          <button id="calBack">Cal Back</button>
          <button id="calLeft">Cal Left</button>
          <button id="calRight">Cal Right</button>
        </div>
        <div class="row-buttons" style="margin-top: 12px;">
          <button id="calStepRun">Next Step</button>
          <button id="calStepBack">Prev Step</button>
          <button id="calSave">Save Moved</button>
          <button id="calReset">Reset Cal</button>
        </div>
        <p class="small">Saved calibration acts as the minimum speed for manual buttons, joystick drive, auto mode, and the CLI rover script.</p>
        <p class="small">Diagonal pulses keep both wheels forward. Use `Manual Speed` as the outer wheel speed, `Diag Inner` as the inner wheel ratio, and `Cal Pulse s` as the pulse length. The Min/Max Fwd Left/Right raw tests use real turning pairs, and the stronger pair slightly reverses the inner wheel on purpose.</p>
        <p class="mono" id="calStatus"></p>
      </div>
      <div class="card panel">
        <details class="drawer">
          <summary>Drive And Auto Settings</summary>
          <div class="drawer-body control-grid">
            <div><label>Steering Mix<br><input id="mix" type="range" min="0.1" max="1.2" step="0.05" value="0.45"></label></div>
            <div><label>Max Throttle<br><input id="maxThrottle" type="range" min="0.1" max="0.8" step="0.05" value="0.35"></label></div>
            <div><label>Update Hz<br><input id="hz" type="range" min="5" max="30" step="1" value="12"></label></div>
            <div><label>Manual Speed<br><input id="manualSpeed" type="range" min="0.00" max="0.60" step="0.01" value="0.00"></label></div>
            <div><label>Diag Inner<br><input id="diagInnerRatio" type="range" min="0.20" max="1.00" step="0.05" value="0.55"></label></div>
            <div><label>Cal Step<br><input id="calStep" type="range" min="0.01" max="0.10" step="0.01" value="0.01"></label></div>
            <div><label>Cal Pulse s<br><input id="calPulse" type="range" min="0.20" max="3.00" step="0.10" value="2.00"></label></div>
            <div><label>Auto Drive<br><input id="autoDrive" type="range" min="0.00" max="0.50" step="0.01" value="0.00"></label></div>
            <div><label>Auto Max Drive<br><input id="autoMaxDrive" type="range" min="0.02" max="0.60" step="0.01" value="0.06"></label></div>
            <div><label>Cam Center<br><input id="autoCenterOffset" type="range" min="-0.50" max="0.50" step="0.01" value="0.00"></label></div>
            <div><label>Auto K Offset<br><input id="autoKOffset" type="range" min="0.2" max="2.0" step="0.05" value="0.90"></label></div>
            <div><label>Auto K Heading<br><input id="autoKHeading" type="range" min="0.0" max="2.0" step="0.05" value="0.70"></label></div>
            <div><label>Auto K Curve<br><input id="autoKCurve" type="range" min="0.0" max="2.0" step="0.05" value="0.75"></label></div>
            <div><label>Auto Turn Mix<br><input id="autoTurnMix" type="range" min="0.10" max="1.20" step="0.05" value="0.45"></label></div>
            <div><label>Loop Hz<br><input id="autoHz" type="range" min="1" max="15" step="1" value="8"></label></div>
            <div><label>Center Tol<br><input id="autoCenterTol" type="range" min="0.01" max="0.25" step="0.01" value="0.06"></label></div>
            <div><label>Min Steer<br><input id="autoMinSteer" type="range" min="0.00" max="0.40" step="0.01" value="0.12"></label></div>
            <div><label>Fwd Floor<br><input id="autoForwardFloor" type="range" min="0.00" max="0.40" step="0.01" value="0.00"></label></div>
            <div><label>Curve Slow<br><input id="autoCurveSlowdown" type="range" min="0.00" max="0.30" step="0.01" value="0.18"></label></div>
            <div><label>Motor Bias<br><input id="autoMotorBias" type="range" min="-0.12" max="0.12" step="0.01" value="0.00"></label></div>
            <div><label>Step Up<br><input id="autoStepUp" type="range" min="0.00" max="0.08" step="0.01" value="0.02"></label></div>
            <div><label>Stuck Px<br><input id="autoStuckShift" type="range" min="2" max="60" step="1" value="10"></label></div>
          </div>
          <div class="row-buttons" style="margin-top: 12px;">
            <button id="saveAuto">Save Auto</button>
            <button id="resetAuto">Reset Auto</button>
          </div>
          <p class="small">Saved auto settings are written to <span class="mono">.teleop_auto_settings.json</span> and reused on the next launch.</p>
        </details>
        <p class="mono" id="log"></p>
      </div>
      <div class="card panel wide">
        <details class="drawer">
          <summary>Mask Tuning</summary>
          <div class="drawer-body">
            <p class="small">Beginner mode: pick the line color, pick the floor color, and let the mask compare color difference. If the tape is always black, use Black Tape first. Dark Line, Contrast, and HSV stay available as fallbacks. RGB picks only affect Color Difference mode.</p>
            <div class="row-buttons" style="margin: 12px 0;">
              <button id="pickLine">Pick Line</button>
              <button id="pickFloor">Pick Floor</button>
              <button id="presetBlack">Black Tape</button>
              <button id="presetGreen">Green Track</button>
              <button id="presetYellow">Yellow On Red</button>
              <button id="refreshMask">Refresh Mask</button>
            </div>
            <div class="sample-bar">
              <div class="swatch"><span class="swatch-chip" id="lineSwatch"></span><span id="lineSampleLabel">Line sample</span></div>
              <div class="swatch"><span class="swatch-chip" id="floorSwatch"></span><span id="floorSampleLabel">Floor sample</span></div>
            </div>
            <div class="mini-grid">
              <div><label>Detector<br><select id="detectorMode"><option value="black_tape">Black Tape / Adaptive</option><option value="color_difference">Color Difference</option><option value="dark_line">Dark Line / Anti-Glare</option><option value="contrast_line">Contrast / Shape</option><option value="hsv_color">HSV Color</option></select></label></div>
              <div><label>Crop Top<br><input id="cropTop" type="range" min="0.0" max="0.8" step="0.05" value="0.35"></label></div>
              <div><label>Proc Scale<br><input id="processingScale" type="range" min="0.30" max="1.00" step="0.05" value="1.00"></label></div>
              <div><label>Tolerance<br><input id="colorTolerance" type="range" min="8" max="90" step="1" value="42"></label></div>
              <div><label>Line Margin<br><input id="colorMargin" type="range" min="0" max="40" step="1" value="8"></label></div>
              <div><label>Patch px<br><input id="sampleRadius" type="range" min="4" max="36" step="1" value="14"></label></div>
              <div><label>H min<br><input id="hMin" type="range" min="0" max="179" step="1" value="35"></label></div>
              <div><label>H max<br><input id="hMax" type="range" min="0" max="179" step="1" value="95"></label></div>
              <div><label>S min<br><input id="sMin" type="range" min="0" max="255" step="1" value="40"></label></div>
              <div><label>S max<br><input id="sMax" type="range" min="0" max="255" step="1" value="255"></label></div>
              <div><label>V min<br><input id="vMin" type="range" min="0" max="255" step="1" value="40"></label></div>
              <div><label>V max<br><input id="vMax" type="range" min="0" max="255" step="1" value="255"></label></div>
              <div><label>Blur<br><input id="blurKernel" type="range" min="1" max="15" step="2" value="5"></label></div>
              <div><label>Morph<br><input id="morphKernel" type="range" min="1" max="15" step="2" value="5"></label></div>
              <div><label>H pad<br><input id="hPad" type="range" min="0" max="40" step="1" value="8"></label></div>
              <div><label>S pad<br><input id="sPad" type="range" min="0" max="120" step="1" value="35"></label></div>
              <div><label>V pad<br><input id="vPad" type="range" min="0" max="120" step="1" value="35"></label></div>
            </div>
            <p class="advanced-note">Black Tape and Dark Line use brightness and local contrast, so the RGB swatches are ignored there. Use the color picks only for Color Difference.</p>
          </div>
        </details>
        <p class="mono" id="maskConfig"></p>
      </div>
      <div class="card panel stream">
        <div class="big">Live RGB</div>
        <img id="rgb" class="clickable" alt="RGB stream">
      </div>
      <div class="card panel stream">
        <div class="big">Detector Score Heatmap</div>
        <img id="heatmap" alt="Per-pixel detector score heatmap">
        <p class="mono" id="heatmapNote">full frame detector score; black_tape uses this before thresholding</p>
      </div>
      <div class="card panel stream">
        <div class="big">Mask Input</div>
        <img id="maskInput" alt="Scaled detector input">
        <p class="mono" id="maskInputNote">exact cropped detector feed</p>
      </div>
      <div class="card panel stream">
        <div class="big">Live Mask</div>
        <img id="mask" alt="Mask stream">
        <p class="mono" id="maskDecision">cyan=detector region | white/green=tracking strip | decision=-</p>
      </div>
      <div class="card panel stream">
        <div class="big">Trajectory</div>
        <img id="trajectory" alt="Trajectory centerline">
        <p class="mono" id="trajectoryNote">centerline + ego gap</p>
      </div>
    </div>
  </div>
  <script>
    const canvas = document.getElementById('pad');
    const ctx = canvas.getContext('2d');
    const center = {x: 160, y: 160};
    const radius = 130;
    let state = {steer: 0, throttle: 0};
    let dragging = false;
    let manualHoldState = null;
    let timer = null;
    let frameTimer = null;
    let statusTimer = null;
    let maskConfig = null;
    let maskRefreshTimer = null;
    let autoEnabled = false;
    let holdUntilMs = 0;
    let driveRequestInFlight = false;
    let stopRequestInFlight = false;
    let statusRequestInFlight = false;
    let manualTransitionInFlight = false;
    let activeManualButtonId = null;
    let sampleTarget = 'line';
    let calibration = {forward: 0, back: 0, left: 0, right: 0};
    let calDirection = null;
    let calSpeed = 0;
    let autoSettingsLoaded = false;
    let frameRequestInFlight = false;
    const frameObjectUrls = new Map();

    function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

    async function fetchJson(url, options = {}, timeoutMs = 1500) {
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
      if (err && err.name === 'AbortError') {
        return 'request timed out';
      }
      return String(err);
    }

    function renderCalibration() {
      const dir = calDirection || '-';
      document.getElementById('calStatus').innerText =
        `selected=${dir} test=${calSpeed.toFixed(2)}\nsaved forward=${calibration.forward.toFixed(2)} back=${calibration.back.toFixed(2)} left=${calibration.left.toFixed(2)} right=${calibration.right.toFixed(2)}`;
    }

    function calibrationSeed(direction) {
      const saved = calibration[direction] || 0;
      if (saved > 0) return saved;
      const known = [calibration.forward, calibration.back, calibration.left, calibration.right]
        .filter((value) => value > 0)
        .sort((a, b) => a - b);
      return known.length ? known[0] : 0;
    }

    function rgbLabel(prefix, r, g, b) {
      return `${prefix} rgb(${r}, ${g}, ${b})`;
    }

    function detectorScoreNote(config) {
      const mode = config && config.detector_mode ? config.detector_mode : 'color_difference';
      return `detector score mode=${mode}${mode === 'black_tape' ? ' | used by black_tape detector before thresholding' : ' (hotter = more line-like)'}`;
    }

    function updateSampleSwatches(config) {
      const lineCss = `rgb(${config.line_r}, ${config.line_g}, ${config.line_b})`;
      const floorCss = `rgb(${config.floor_r}, ${config.floor_g}, ${config.floor_b})`;
      document.getElementById('lineSwatch').style.background = lineCss;
      document.getElementById('floorSwatch').style.background = floorCss;
      document.getElementById('lineSampleLabel').innerText = rgbLabel('Line', config.line_r, config.line_g, config.line_b);
      document.getElementById('floorSampleLabel').innerText = rgbLabel('Floor', config.floor_r, config.floor_g, config.floor_b);
    }

    function autoSettingsFromControls() {
      return {
        drive: parseFloat(document.getElementById('autoDrive').value),
        max_drive: parseFloat(document.getElementById('autoMaxDrive').value),
        camera_center_offset: parseFloat(document.getElementById('autoCenterOffset').value),
        k_offset: parseFloat(document.getElementById('autoKOffset').value),
        k_heading: parseFloat(document.getElementById('autoKHeading').value),
        k_curve: parseFloat(document.getElementById('autoKCurve').value),
        turn_mix: parseFloat(document.getElementById('autoTurnMix').value),
        loop_hz: parseFloat(document.getElementById('autoHz').value),
        center_tolerance: parseFloat(document.getElementById('autoCenterTol').value),
        min_steer: parseFloat(document.getElementById('autoMinSteer').value),
        forward_floor: parseFloat(document.getElementById('autoForwardFloor').value),
        curve_slowdown: parseFloat(document.getElementById('autoCurveSlowdown').value),
        motor_bias: parseFloat(document.getElementById('autoMotorBias').value),
        step_up: parseFloat(document.getElementById('autoStepUp').value),
        stuck_shift_px: parseFloat(document.getElementById('autoStuckShift').value),
      };
    }

    function applyAutoSettings(settings) {
      if (!settings) return;
      if (settings.drive !== undefined) document.getElementById('autoDrive').value = settings.drive;
      if (settings.max_drive !== undefined) document.getElementById('autoMaxDrive').value = settings.max_drive;
      if (settings.camera_center_offset !== undefined) document.getElementById('autoCenterOffset').value = settings.camera_center_offset;
      if (settings.k_offset !== undefined) document.getElementById('autoKOffset').value = settings.k_offset;
      if (settings.k_heading !== undefined) document.getElementById('autoKHeading').value = settings.k_heading;
      if (settings.k_curve !== undefined) document.getElementById('autoKCurve').value = settings.k_curve;
      if (settings.turn_mix !== undefined) document.getElementById('autoTurnMix').value = settings.turn_mix;
      if (settings.loop_hz !== undefined) document.getElementById('autoHz').value = settings.loop_hz;
      if (settings.center_tolerance !== undefined) document.getElementById('autoCenterTol').value = settings.center_tolerance;
      if (settings.min_steer !== undefined) document.getElementById('autoMinSteer').value = settings.min_steer;
      if (settings.forward_floor !== undefined) document.getElementById('autoForwardFloor').value = settings.forward_floor;
      if (settings.curve_slowdown !== undefined) document.getElementById('autoCurveSlowdown').value = settings.curve_slowdown;
      if (settings.motor_bias !== undefined) document.getElementById('autoMotorBias').value = settings.motor_bias;
      if (settings.step_up !== undefined) document.getElementById('autoStepUp').value = settings.step_up;
      if (settings.stuck_shift_px !== undefined) document.getElementById('autoStuckShift').value = settings.stuck_shift_px;
    }

    function draw() {
      const displayState = manualHoldState || state;
      const displayThrottle = manualHoldState ? manualHoldState.throttle_display : displayState.throttle;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = 'white';
      ctx.beginPath();
      ctx.arc(center.x, center.y, radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.lineWidth = 4;
      ctx.strokeStyle = 'black';
      ctx.stroke();
      ctx.strokeStyle = '#d0d0d0';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(center.x, center.y - radius);
      ctx.lineTo(center.x, center.y + radius);
      ctx.moveTo(center.x - radius, center.y);
      ctx.lineTo(center.x + radius, center.y);
      ctx.stroke();
      const x = center.x + displayState.steer * radius;
      const y = center.y - displayThrottle * radius;
      ctx.strokeStyle = 'red';
      ctx.lineWidth = 5;
      ctx.beginPath();
      ctx.moveTo(center.x, center.y);
      ctx.lineTo(x, y);
      ctx.stroke();
      ctx.fillStyle = 'red';
      ctx.beginPath();
      ctx.arc(x, y, 10, 0, Math.PI * 2);
      ctx.fill();
      document.getElementById('metrics').innerText =
        `steer=${displayState.steer.toFixed(2)} throttle=${displayThrottle.toFixed(2)}`;
    }

    function pointerToState(ev) {
      const rect = canvas.getBoundingClientRect();
      const px = ev.clientX - rect.left;
      const py = ev.clientY - rect.top;
      let dx = px - center.x;
      let dy = py - center.y;
      const dist = Math.hypot(dx, dy);
      if (dist > radius) {
        dx = dx * radius / dist;
        dy = dy * radius / dist;
      }
      state.steer = clamp(dx / radius, -1, 1);
      state.throttle = clamp(-dy / radius, -1, 1);
      draw();
    }

    async function sendState() {
      if (autoEnabled) return;
      if (Date.now() < holdUntilMs) return;
      if (driveRequestInFlight) return;
      const maxThrottle = parseFloat(document.getElementById('maxThrottle').value);
      const steer = manualHoldState ? manualHoldState.steer : state.steer;
      const throttle = manualHoldState ? manualHoldState.throttle : state.throttle * maxThrottle;
      const mix = manualHoldState
        ? manualHoldState.mix
        : parseFloat(document.getElementById('mix').value);
      driveRequestInFlight = true;
      try {
        const data = await fetchJson('/api/drive', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({steer, throttle, mix})
        }, 1200);
        document.getElementById('status').innerText = data.status;
        document.getElementById('log').innerText = JSON.stringify(data, null, 2);
      } catch (err) {
        document.getElementById('status').innerText = `drive failed: ${formatError(err)}`;
      } finally {
        driveRequestInFlight = false;
      }
    }

    function manualHoldDisplayThrottle(throttle) {
      const maxThrottle = parseFloat(document.getElementById('maxThrottle').value);
      if (Math.abs(throttle) < 1e-6) return 0.0;
      const denom = Math.max(maxThrottle, 0.01);
      return clamp(throttle / denom, -1, 1);
    }

    function manualFloorSeed(speed) {
      return Math.max(Math.abs(speed), 0.01);
    }

    function forwardTurnPair(direction, strength) {
      const maxPair = direction === 'left'
        ? {left: -0.50, right: 0.50}
        : {left: 0.50, right: -0.40};
      const minPair = direction === 'left'
        ? {left: -0.20, right: 0.50}
        : {left: 0.50, right: -0.20};
      return strength === 'max' ? maxPair : minPair;
    }

    function beginManualHold(steer, throttle, note, mixOverride = null) {
      const mix = mixOverride === null ? parseFloat(document.getElementById('mix').value) : mixOverride;
      manualHoldState = {
        steer,
        throttle,
        mix,
        note,
        throttle_display: manualHoldDisplayThrottle(throttle),
      };
      draw();
      document.getElementById('status').innerText = `holding: ${note}`;
      sendState().catch((err) => {
        document.getElementById('status').innerText = `manual hold failed: ${formatError(err)}`;
      });
    }

    async function endManualHold() {
      if (!manualHoldState) return;
      manualHoldState = null;
      activeManualButtonId = null;
      state = {steer: 0, throttle: 0};
      draw();
      if (stopRequestInFlight) return;
      stopRequestInFlight = true;
      try {
        const data = await fetchJson('/api/stop', {method: 'POST'}, 1500);
        document.getElementById('status').innerText = data.status;
        document.getElementById('log').innerText = JSON.stringify(data, null, 2);
      } catch (err) {
        document.getElementById('status').innerText = `stop failed: ${formatError(err)}`;
      } finally {
        stopRequestInFlight = false;
      }
    }

    function bindHoldButton(id, createCommand) {
      const button = document.getElementById(id);
      const onPress = async (ev) => {
        ev.preventDefault();
        if (manualTransitionInFlight) return;
        manualTransitionInFlight = true;
        button.setPointerCapture?.(ev.pointerId);
        try {
          if (manualHoldState && activeManualButtonId !== id) {
            await endManualHold();
            await new Promise((resolve) => setTimeout(resolve, 120));
          }
          activeManualButtonId = id;
          const command = createCommand();
          beginManualHold(command.steer, command.throttle, command.note, command.mix);
        } finally {
          manualTransitionInFlight = false;
        }
      };
      const onRelease = async (ev) => {
        if (ev) ev.preventDefault();
        if (manualTransitionInFlight) return;
        manualTransitionInFlight = true;
        try {
          await endManualHold();
        } catch (err) {
          document.getElementById('status').innerText = `stop failed: ${formatError(err)}`;
        } finally {
          manualTransitionInFlight = false;
        }
      };
      button.addEventListener('pointerdown', onPress);
      button.addEventListener('pointerup', onRelease);
      button.addEventListener('pointercancel', onRelease);
      button.addEventListener('lostpointercapture', onRelease);
      button.addEventListener('contextmenu', (ev) => ev.preventDefault());
    }

    async function sendPulseDirection(direction, speed, duration) {
      holdUntilMs = Date.now() + Math.max(0, duration) * 1000 + 150;
      let mix = parseFloat(document.getElementById('mix').value);
      let steer = 0.0;
      let throttle = speed;
      if (direction === 'back') {
        throttle = -speed;
      } else if (direction === 'left') {
        steer = -1.0;
        throttle = 0.0;
        mix = speed;
      } else if (direction === 'right') {
        steer = 1.0;
        throttle = 0.0;
        mix = speed;
      }
      const r = await fetch('/api/pulse', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({steer, throttle, mix, duration_s: duration})
      });
      const data = await r.json();
      document.getElementById('status').innerText = data.status;
      document.getElementById('log').innerText = JSON.stringify(data, null, 2);
    }

    async function sendRawPulse(left, right, duration, note, bypassCalibration = true) {
      holdUntilMs = Date.now() + Math.max(0, duration) * 1000 + 150;
      const r = await fetch('/api/pulse_raw', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({left, right, duration_s: duration, bypass_calibration: !!bypassCalibration})
      });
      const data = await r.json();
      document.getElementById('status').innerText = note || data.status;
      document.getElementById('log').innerText = JSON.stringify(data, null, 2);
    }

    function renderWarnings(warnings) {
      const node = document.getElementById('warnings');
      if (!warnings.length) {
        node.innerHTML = '';
        return;
      }
      node.innerHTML = warnings.map((warning) => `<div class="warn">${warning}</div>`).join('');
    }

    function maskControls() {
      return {
        detector_mode: document.getElementById('detectorMode').value,
        crop_top_ratio: parseFloat(document.getElementById('cropTop').value),
        processing_scale: parseFloat(document.getElementById('processingScale').value),
        color_distance_threshold: parseFloat(document.getElementById('colorTolerance').value),
        color_distance_margin: parseFloat(document.getElementById('colorMargin').value),
        h_min: parseInt(document.getElementById('hMin').value, 10),
        h_max: parseInt(document.getElementById('hMax').value, 10),
        s_min: parseInt(document.getElementById('sMin').value, 10),
        s_max: parseInt(document.getElementById('sMax').value, 10),
        v_min: parseInt(document.getElementById('vMin').value, 10),
        v_max: parseInt(document.getElementById('vMax').value, 10),
        blur_kernel: parseInt(document.getElementById('blurKernel').value, 10),
        morph_kernel: parseInt(document.getElementById('morphKernel').value, 10),
        sample_radius: parseInt(document.getElementById('sampleRadius').value, 10),
        h_margin: parseInt(document.getElementById('hPad').value, 10),
        s_margin: parseInt(document.getElementById('sPad').value, 10),
        v_margin: parseInt(document.getElementById('vPad').value, 10),
      };
    }

    function applyMaskConfig(config) {
      maskConfig = config;
      document.getElementById('detectorMode').value = config.detector_mode || 'color_difference';
      document.getElementById('cropTop').value = config.crop_top_ratio;
      document.getElementById('processingScale').value = config.processing_scale ?? 1.0;
      document.getElementById('colorTolerance').value = config.color_distance_threshold;
      document.getElementById('colorMargin').value = config.color_distance_margin;
      document.getElementById('hMin').value = config.h_min;
      document.getElementById('hMax').value = config.h_max;
      document.getElementById('sMin').value = config.s_min;
      document.getElementById('sMax').value = config.s_max;
      document.getElementById('vMin').value = config.v_min;
      document.getElementById('vMax').value = config.v_max;
      document.getElementById('blurKernel').value = config.blur_kernel;
      document.getElementById('morphKernel').value = config.morph_kernel;
      document.getElementById('sampleRadius').value = config.sample_radius;
      document.getElementById('hPad').value = config.h_margin;
      document.getElementById('sPad').value = config.s_margin;
      document.getElementById('vPad').value = config.v_margin;
      updateSampleSwatches(config);
      const usesSamples = (config.detector_mode || 'color_difference') === 'color_difference';
      document.getElementById('maskInputNote').innerText =
        `scale=${Number(config.processing_scale ?? 1.0).toFixed(2)} crop=${Number(config.crop_top_ratio).toFixed(2)} mode=${config.detector_mode || 'color_difference'}`;
      document.getElementById('heatmapNote').innerText = detectorScoreNote(config);
      document.getElementById('maskConfig').innerText =
        `mode=${config.detector_mode || 'color_difference'} | scale=${Number(config.processing_scale ?? 1.0).toFixed(2)} crop=${config.crop_top_ratio.toFixed(2)} blur=${config.blur_kernel} morph=${config.morph_kernel} | tol=${Number(config.color_distance_threshold).toFixed(0)} margin=${Number(config.color_distance_margin).toFixed(0)} | line=rgb(${config.line_r},${config.line_g},${config.line_b}) floor=rgb(${config.floor_r},${config.floor_g},${config.floor_b}) | samples=${usesSamples ? 'active' : 'ignored'}`;
    }

    async function fetchMaskConfig() {
      const r = await fetch('/api/mask/config');
      const data = await r.json();
      applyMaskConfig(data);
    }

    async function fetchCalibration() {
      const r = await fetch('/api/calibration');
      calibration = await r.json();
      renderCalibration();
    }

    async function fetchAutoSettings() {
      const r = await fetch('/api/auto/settings');
      const data = await r.json();
      applyAutoSettings(data);
      autoSettingsLoaded = true;
    }

    async function saveAutoSettings(statusMessage = 'saved auto settings') {
      const r = await fetch('/api/auto/settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(autoSettingsFromControls())
      });
      const data = await r.json();
      applyAutoSettings(data);
      autoSettingsLoaded = true;
      document.getElementById('status').innerText = statusMessage;
      document.getElementById('log').innerText = JSON.stringify(data, null, 2);
      return data;
    }

    async function pushMaskConfig() {
      const r = await fetch('/api/mask/config', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(maskControls())
      });
      const data = await r.json();
      applyMaskConfig(data);
      refreshFramesOnce();
    }

    function scheduleMaskPush() {
      if (maskRefreshTimer) clearTimeout(maskRefreshTimer);
      maskRefreshTimer = setTimeout(() => { pushMaskConfig().catch((err) => {
        document.getElementById('log').innerText = `mask update failed: ${err}`;
      }); }, 120);
    }

    async function refreshPanelImage(id, path, t) {
      const response = await fetch(`${path}?t=${t}`, {cache: 'no-store'});
      if (!response.ok) {
        throw new Error(`${id} ${response.status}`);
      }
      const blob = await response.blob();
      const nextUrl = URL.createObjectURL(blob);
      const previousUrl = frameObjectUrls.get(id);
      document.getElementById(id).src = nextUrl;
      frameObjectUrls.set(id, nextUrl);
      if (previousUrl) {
        URL.revokeObjectURL(previousUrl);
      }
    }

    async function refreshFramesOnce() {
      if (frameRequestInFlight) return;
      frameRequestInFlight = true;
      const t = Date.now();
      try {
        const results = await Promise.allSettled([
          refreshPanelImage('rgb', '/camera/rgb.jpg', t),
          refreshPanelImage('heatmap', '/camera/heatmap.jpg', t),
          refreshPanelImage('maskInput', '/camera/mask_input.jpg', t),
          refreshPanelImage('mask', '/camera/mask.jpg', t),
          refreshPanelImage('trajectory', '/camera/trajectory.jpg', t),
        ]);
        const failed = results.find((result) => result.status === 'rejected');
        if (failed && failed.status === 'rejected') {
          document.getElementById('log').innerText = `frame refresh warning: ${formatError(failed.reason)}`;
        }
      } finally {
        frameRequestInFlight = false;
      }
    }

    async function refreshStatus() {
      if (statusRequestInFlight) return;
      statusRequestInFlight = true;
      try {
        const data = await fetchJson('/api/status', {}, 1200);
        document.getElementById('serverState').innerText = 'ready';
        document.getElementById('cameraState').innerText = data.camera.ok ? 'ready' : `offline: ${data.camera.error}`;
        document.getElementById('serialState').innerText = data.serial.ok ? `ready: ${data.serial.port}` : `offline: ${data.serial.error}`;
        document.getElementById('address').innerText = `${window.location.hostname}:${window.location.port || '80'}`;
        document.getElementById('configLine').innerText =
          `${data.camera.source.toUpperCase()} camera ${data.camera.width}x${data.camera.height} | serial ${data.serial.port} @ ${data.serial.baudrate}`;
        autoEnabled = !!(data.auto && data.auto.running);
        if (!autoSettingsLoaded && data.auto && data.auto.settings) {
          applyAutoSettings(data.auto.settings);
          autoSettingsLoaded = true;
        }
        if (data.auto) {
          document.getElementById('status').innerText = data.auto.running ? `auto: ${data.auto.last_status}` : document.getElementById('status').innerText;
          if (maskConfig) {
            document.getElementById('heatmapNote').innerText = detectorScoreNote(maskConfig);
          }
          if (data.auto.last_analysis) {
            document.getElementById('maskDecision').innerText =
              `decision=${data.auto.last_analysis.decision || '-'}\n${data.auto.last_analysis.decision_reason || ''}`;
            document.getElementById('trajectoryNote').innerText =
              `bottom=${(data.auto.last_analysis.bottom_offset_norm ?? data.auto.last_analysis.offset_norm).toFixed(3)} curve=${(data.auto.last_analysis.curvature_norm ?? 0).toFixed(3)} steer=${data.auto.last_analysis.steer.toFixed(3)}`;
            document.getElementById('metrics').innerText =
              `decision=${data.auto.last_analysis.decision || '-'} bottom=${(data.auto.last_analysis.bottom_offset_norm ?? data.auto.last_analysis.offset_norm).toFixed(3)} gap=${data.auto.last_analysis.offset_norm.toFixed(3)} heading=${data.auto.last_analysis.heading_norm.toFixed(3)} curve=${(data.auto.last_analysis.curvature_norm ?? 0).toFixed(3)} recenter=${(data.auto.last_analysis.recenter_priority ?? 0).toFixed(2)} steer=${data.auto.last_analysis.steer.toFixed(3)} drive=${data.auto.last_analysis.drive.toFixed(2)} mix=${(data.auto.last_analysis.mix_used ?? 0).toFixed(2)} left=${data.auto.last_analysis.left.toFixed(2)} right=${data.auto.last_analysis.right.toFixed(2)} shift=${(data.auto.last_analysis.x_shift ?? 0).toFixed(1)} stuck=${data.auto.last_analysis.stuck_steps ?? 0} search=${(data.auto.last_analysis.search_turn ?? 0).toFixed(2)} sstep=${data.auto.last_analysis.search_steps ?? 0}`;
          }
        }
        renderWarnings(data.warnings || []);
      } catch (err) {
        document.getElementById('serverState').innerText = `error: ${err}`;
      } finally {
        statusRequestInFlight = false;
      }
    }

    function restartLoop() {
      if (timer) clearInterval(timer);
      timer = setInterval(sendState, 1000 / parseInt(document.getElementById('hz').value, 10));
    }

    function restartFrames() {
      if (frameTimer) clearInterval(frameTimer);
      frameTimer = setInterval(() => {
        refreshFramesOnce().catch((err) => {
          document.getElementById('log').innerText = `frame refresh failed: ${formatError(err)}`;
        });
      }, 250);
    }

    function restartStatusLoop() {
      if (statusTimer) clearInterval(statusTimer);
      refreshStatus();
      statusTimer = setInterval(refreshStatus, 2000);
    }

    canvas.addEventListener('pointerdown', (ev) => { dragging = true; pointerToState(ev); });
    canvas.addEventListener('pointermove', (ev) => { if (dragging) pointerToState(ev); });
    window.addEventListener('pointerup', () => {
      dragging = false;
      endManualHold().catch(() => {});
    });
    document.getElementById('center').addEventListener('click', () => {
      manualHoldState = null;
      activeManualButtonId = null;
      state = {steer: 0, throttle: 0};
      draw();
    });
    document.getElementById('stop').addEventListener('click', async () => {
      manualHoldState = null;
      activeManualButtonId = null;
      state = {steer: 0, throttle: 0};
      draw();
      await fetchJson('/api/stop', {method: 'POST'}, 1500);
    });
    bindHoldButton('manualForward', () => {
      const speed = manualFloorSeed(parseFloat(document.getElementById('manualSpeed').value));
      return {steer: 0.0, throttle: speed, note: 'manual forward', mix: null};
    });
    bindHoldButton('manualBack', () => {
      const speed = manualFloorSeed(parseFloat(document.getElementById('manualSpeed').value));
      return {steer: 0.0, throttle: -speed, note: 'manual back', mix: null};
    });
    bindHoldButton('manualLeft', () => {
      const speed = manualFloorSeed(parseFloat(document.getElementById('manualSpeed').value));
      return {steer: -1.0, throttle: 0.0, note: 'manual left', mix: speed};
    });
    bindHoldButton('manualRight', () => {
      const speed = manualFloorSeed(parseFloat(document.getElementById('manualSpeed').value));
      return {steer: 1.0, throttle: 0.0, note: 'manual right', mix: speed};
    });
    document.getElementById('pulseForwardLeft').addEventListener('click', async () => {
      const speed = manualFloorSeed(parseFloat(document.getElementById('manualSpeed').value));
      const innerRatio = clamp(parseFloat(document.getElementById('diagInnerRatio').value), 0.0, 1.0);
      await sendRawPulse(speed * innerRatio, speed, parseFloat(document.getElementById('calPulse').value), 'pulse: forward-left (raw)');
    });
    document.getElementById('pulseForwardRight').addEventListener('click', async () => {
      const speed = manualFloorSeed(parseFloat(document.getElementById('manualSpeed').value));
      const innerRatio = clamp(parseFloat(document.getElementById('diagInnerRatio').value), 0.0, 1.0);
      await sendRawPulse(speed, speed * innerRatio, parseFloat(document.getElementById('calPulse').value), 'pulse: forward-right (raw)');
    });
    document.getElementById('minForwardLeft').addEventListener('click', async () => {
      const pair = forwardTurnPair('left', 'min');
      await sendRawPulse(
        pair.left,
        pair.right,
        parseFloat(document.getElementById('calPulse').value),
        `test: min forward-left raw L=${pair.left.toFixed(2)} R=${pair.right.toFixed(2)}`
      );
    });
    document.getElementById('minForwardRight').addEventListener('click', async () => {
      const pair = forwardTurnPair('right', 'min');
      await sendRawPulse(
        pair.left,
        pair.right,
        parseFloat(document.getElementById('calPulse').value),
        `test: min forward-right raw L=${pair.left.toFixed(2)} R=${pair.right.toFixed(2)}`
      );
    });
    document.getElementById('maxForwardLeft').addEventListener('click', async () => {
      const pair = forwardTurnPair('left', 'max');
      await sendRawPulse(
        pair.left,
        pair.right,
        parseFloat(document.getElementById('calPulse').value),
        `test: max forward-left raw L=${pair.left.toFixed(2)} R=${pair.right.toFixed(2)}`
      );
    });
    document.getElementById('maxForwardRight').addEventListener('click', async () => {
      const pair = forwardTurnPair('right', 'max');
      await sendRawPulse(
        pair.left,
        pair.right,
        parseFloat(document.getElementById('calPulse').value),
        `test: max forward-right raw L=${pair.left.toFixed(2)} R=${pair.right.toFixed(2)}`
      );
    });
    document.getElementById('calForward').addEventListener('click', () => { calDirection = 'forward'; calSpeed = calibrationSeed('forward'); renderCalibration(); });
    document.getElementById('calBack').addEventListener('click', () => { calDirection = 'back'; calSpeed = calibrationSeed('back'); renderCalibration(); });
    document.getElementById('calLeft').addEventListener('click', () => { calDirection = 'left'; calSpeed = calibrationSeed('left'); renderCalibration(); });
    document.getElementById('calRight').addEventListener('click', () => { calDirection = 'right'; calSpeed = calibrationSeed('right'); renderCalibration(); });
    document.getElementById('calStepRun').addEventListener('click', async () => {
      if (!calDirection) {
        document.getElementById('status').innerText = 'pick a calibration direction first';
        return;
      }
      calSpeed = clamp(calSpeed + parseFloat(document.getElementById('calStep').value), 0.0, 1.0);
      renderCalibration();
      await sendPulseDirection(calDirection, calSpeed, parseFloat(document.getElementById('calPulse').value));
    });
    document.getElementById('calStepBack').addEventListener('click', async () => {
      if (!calDirection) {
        document.getElementById('status').innerText = 'pick a calibration direction first';
        return;
      }
      calSpeed = clamp(calSpeed - parseFloat(document.getElementById('calStep').value), 0.0, 1.0);
      renderCalibration();
      await sendPulseDirection(calDirection, calSpeed, parseFloat(document.getElementById('calPulse').value));
    });
    document.getElementById('calSave').addEventListener('click', async () => {
      if (!calDirection) {
        document.getElementById('status').innerText = 'pick a calibration direction first';
        return;
      }
      const r = await fetch('/api/calibration', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({direction: calDirection, value: calSpeed})
      });
      calibration = await r.json();
      renderCalibration();
      document.getElementById('status').innerText = `saved ${calDirection} minimum`;
    });
    document.getElementById('calReset').addEventListener('click', async () => {
      const r = await fetch('/api/calibration/reset', {method: 'POST'});
      calibration = await r.json();
      calDirection = null;
      calSpeed = 0.0;
      renderCalibration();
      document.getElementById('status').innerText = 'calibration reset';
    });
    document.getElementById('startAuto').addEventListener('click', async () => {
      const saved = await saveAutoSettings('saved auto settings, starting auto');
      const r = await fetch('/api/auto/start', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(saved)
      });
      const data = await r.json();
      autoEnabled = true;
      document.getElementById('status').innerText = data.status || 'auto started';
      document.getElementById('log').innerText = JSON.stringify(data, null, 2);
      refreshStatus();
    });
    document.getElementById('stopAuto').addEventListener('click', async () => {
      const r = await fetch('/api/auto/stop', {method: 'POST'});
      const data = await r.json();
      autoEnabled = false;
      manualHoldState = null;
      activeManualButtonId = null;
      state = {steer: 0, throttle: 0};
      draw();
      document.getElementById('status').innerText = data.status || 'auto stopped';
      document.getElementById('log').innerText = JSON.stringify(data, null, 2);
      refreshStatus();
    });
    document.getElementById('saveAuto').addEventListener('click', async () => {
      await saveAutoSettings();
    });
    document.getElementById('resetAuto').addEventListener('click', async () => {
      const r = await fetch('/api/auto/settings/reset', {method: 'POST'});
      const data = await r.json();
      applyAutoSettings(data);
      autoSettingsLoaded = true;
      document.getElementById('status').innerText = 'auto settings reset';
      document.getElementById('log').innerText = JSON.stringify(data, null, 2);
    });
    document.getElementById('hz').addEventListener('input', restartLoop);
    ['detectorMode', 'cropTop', 'processingScale', 'colorTolerance', 'colorMargin', 'hMin', 'hMax', 'sMin', 'sMax', 'vMin', 'vMax', 'blurKernel', 'morphKernel', 'sampleRadius', 'hPad', 'sPad', 'vPad']
      .forEach((id) => document.getElementById(id).addEventListener('input', scheduleMaskPush));
    document.getElementById('pickLine').addEventListener('click', () => {
      sampleTarget = 'line';
      document.getElementById('status').innerText = 'click the RGB image to sample the line color';
    });
    document.getElementById('pickFloor').addEventListener('click', () => {
      sampleTarget = 'floor';
      document.getElementById('status').innerText = 'click the RGB image to sample the floor color';
    });
    document.getElementById('presetBlack').addEventListener('click', async () => {
      const r = await fetch('/api/mask/preset/black_tape', {method: 'POST'});
      applyMaskConfig(await r.json());
      sampleTarget = 'line';
      refreshFramesOnce();
    });
    document.getElementById('presetGreen').addEventListener('click', async () => {
      const r = await fetch('/api/mask/preset/green_track', {method: 'POST'});
      applyMaskConfig(await r.json());
      sampleTarget = 'line';
      refreshFramesOnce();
    });
    document.getElementById('presetYellow').addEventListener('click', async () => {
      const r = await fetch('/api/mask/preset/yellow_on_red', {method: 'POST'});
      applyMaskConfig(await r.json());
      sampleTarget = 'line';
      refreshFramesOnce();
    });
    document.getElementById('refreshMask').addEventListener('click', refreshFramesOnce);
    document.getElementById('rgb').addEventListener('click', async (ev) => {
      const img = ev.currentTarget;
      const rect = img.getBoundingClientRect();
      const x = (ev.clientX - rect.left) / rect.width;
      const y = (ev.clientY - rect.top) / rect.height;
      const r = await fetch('/api/mask/sample', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({x, y, role: sampleTarget})
      });
      const data = await r.json();
      applyMaskConfig(data.config);
      document.getElementById('status').innerText = data.status;
      document.getElementById('log').innerText = JSON.stringify(data, null, 2);
      sampleTarget = sampleTarget === 'line' ? 'floor' : sampleTarget;
      refreshFramesOnce();
    });
    draw();
    restartLoop();
    restartFrames();
    restartStatusLoop();
    fetchMaskConfig();
    fetchCalibration();
    fetchAutoSettings();
    renderCalibration();
  </script>
</body>
</html>
"""


PRESETS = {
    "black_tape": {
        "detector_mode": "black_tape",
        "crop_top_ratio": 0.20,      # was 0.30
        "processing_scale": 1.00,    # was 0.50
        "h_min": 0,
        "h_max": 179,
        "s_min": 0,
        "s_max": 255,
        "v_min": 0,
        "v_max": 255,
        "blur_kernel": 5,            # was 3
        "morph_kernel": 7,           # was 9
        "sample_radius": 14,
        "h_margin": 8,
        "s_margin": 35,
        "v_margin": 35,
        "color_distance_threshold": 42.0,
        "color_distance_margin": 8.0,
        "line_r": 38,
        "line_g": 38,
        "line_b": 38,
        "floor_r": 170,
        "floor_g": 160,
        "floor_b": 150,
    },
    "green_track": {
        "detector_mode": "color_difference",
        "crop_top_ratio": 0.35,
        "processing_scale": 1.0,
        "h_min": 35,
        "h_max": 95,
        "s_min": 40,
        "s_max": 255,
        "v_min": 40,
        "v_max": 255,
        "blur_kernel": 5,
        "morph_kernel": 5,
        "sample_radius": 14,
        "h_margin": 8,
        "s_margin": 35,
        "v_margin": 35,
        "color_distance_threshold": 42.0,
        "color_distance_margin": 8.0,
        "line_r": 72,
        "line_g": 159,
        "line_b": 83,
        "floor_r": 143,
        "floor_g": 99,
        "floor_b": 82,
    },
    "yellow_on_red": {
        "detector_mode": "color_difference",
        "crop_top_ratio": 0.35,
        "processing_scale": 1.0,
        "h_min": 8,
        "h_max": 55,
        "s_min": 20,
        "s_max": 255,
        "v_min": 80,
        "v_max": 255,
        "blur_kernel": 5,
        "morph_kernel": 5,
        "sample_radius": 14,
        "h_margin": 8,
        "s_margin": 35,
        "v_margin": 35,
        "color_distance_threshold": 48.0,
        "color_distance_margin": 10.0,
        "line_r": 208,
        "line_g": 184,
        "line_b": 72,
        "floor_r": 148,
        "floor_g": 68,
        "floor_b": 54,
    },
}


class MaskConfig:
    def __init__(self, path: Path = PROJECT_ROOT / ".teleop_mask_config.json") -> None:
        self._lock = threading.Lock()
        self._path = path
        self._config = dict(PRESETS["green_track"])
        self._load()

    def _load(self) -> None:
        try:
            if not self._path.exists():
                return
            data = json.loads(self._path.read_text())
        except Exception:
            return
        for key in self._config:
            if key in data:
                self._config[key] = data[key]

    def _save(self) -> None:
        try:
            self._path.write_text(json.dumps(self._config, indent=2))
        except Exception:
            pass

    def get(self) -> dict[str, int | float]:
        with self._lock:
            return dict(self._config)

    def update(self, values: dict[str, int | float]) -> dict[str, int | float]:
        with self._lock:
            for key, value in values.items():
                if key not in self._config:
                    continue
                self._config[key] = value
            self._save()
            return dict(self._config)

    def apply_preset(self, name: str) -> dict[str, int | float]:
        if name not in PRESETS:
            raise KeyError(name)
        with self._lock:
            self._config = dict(PRESETS[name])
            self._save()
            return dict(self._config)


class AutoSettings:
    def __init__(self, path: Path = PROJECT_ROOT / ".teleop_auto_settings.json") -> None:
        self._lock = threading.Lock()
        self._path = path
        self._settings = dict(AUTO_SETTINGS_DEFAULTS)
        self._load()

    def _load(self) -> None:
        try:
            if not self._path.exists():
                return
            data = json.loads(self._path.read_text())
        except Exception:
            return
        for key in self._settings:
            if key in data:
                try:
                    self._settings[key] = float(data[key])
                except Exception:
                    continue

    def _save(self) -> None:
        try:
            self._path.write_text(json.dumps(self._settings, indent=2))
        except Exception:
            pass

    def get(self) -> dict[str, float]:
        with self._lock:
            return dict(self._settings)

    def update(self, values: dict[str, Any]) -> dict[str, float]:
        with self._lock:
            for key in self._settings:
                if key not in values:
                    continue
                try:
                    self._settings[key] = float(values[key])
                except Exception:
                    continue
            self._save()
            return dict(self._settings)

    def reset(self) -> dict[str, float]:
        with self._lock:
            self._settings = dict(AUTO_SETTINGS_DEFAULTS)
            self._save()
            return dict(self._settings)


def rgb_triplet_to_lab(rgb: tuple[int, int, int]) -> np.ndarray:
    swatch = np.array([[list(rgb)]], dtype=np.uint8)
    lab = cv2.cvtColor(swatch, cv2.COLOR_RGB2LAB).astype(np.float32)
    return lab[0, 0]


def normalize_processing_scale(scale: float | int | None) -> float:
    try:
        value = float(scale)
    except Exception:
        value = 1.0
    return float(np.clip(value, 0.30, 1.0))


def prepare_mask_working_image(
    image_rgb: Image.Image,
    config: dict[str, int | float | str],
) -> Image.Image:
    scale = normalize_processing_scale(config.get("processing_scale", 1.0))
    if abs(scale - 1.0) < 1e-3:
        return image_rgb.copy()

    frame = np.array(image_rgb)
    height, width = frame.shape[:2]
    scaled_width = max(96, int(round(width * scale)))
    scaled_height = max(54, int(round(height * scale)))
    resized = cv2.resize(frame, (scaled_width, scaled_height), interpolation=cv2.INTER_AREA)
    return Image.fromarray(resized)


def render_mask_input_preview(
    working_rgb: Image.Image,
    config: dict[str, int | float | str],
) -> Image.Image:
    frame = np.array(working_rgb)
    mode = str(config.get("detector_mode", "contrast_line"))
    if mode in {"black_tape", "dark_line", "contrast_line"}:
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        display = np.stack([gray] * 3, axis=-1)
    else:
        display = frame.copy()

    crop_top_ratio = float(config.get("crop_top_ratio", 0.35))
    crop_start = int(display.shape[0] * crop_top_ratio)
    crop_start = max(0, min(display.shape[0], crop_start))
    if crop_start > 0:
        display[:crop_start, :] = (display[:crop_start, :] * 0.18).astype(np.uint8)
        guide_y = max(0, crop_start - 1)
        display[guide_y : min(display.shape[0], guide_y + 2), :] = np.array([64, 196, 255], dtype=np.uint8)
    return Image.fromarray(display)


def compute_black_tape_darkness_score(
    roi_lightness: np.ndarray,
    morph_kernel: int = 7,
    loose: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    roi_f = roi_lightness.astype(np.float32)
    if roi_f.size == 0:
        empty = np.zeros_like(roi_f, dtype=np.float32)
        return empty, empty, roi_f

    background_kernel = max(25, morph_kernel * 6 + 1)
    if background_kernel % 2 == 0:
        background_kernel += 1
    local_background = cv2.GaussianBlur(roi_f, (background_kernel, background_kernel), 0)
    local_dark = np.maximum(local_background - roi_f, 0.0)

    row_floor = np.percentile(roi_f, 92 if loose else 95, axis=1, keepdims=True).astype(np.float32)
    row_dark = np.maximum(row_floor - roi_f, 0.0)

    score = 0.72 * local_dark + 0.28 * row_dark
    score_u8 = np.clip(score, 0.0, 255.0).astype(np.uint8)
    fill_kernel = np.ones((max(5, morph_kernel), max(9, morph_kernel * 3 + 1)), np.uint8)
    score_u8 = cv2.morphologyEx(score_u8, cv2.MORPH_CLOSE, fill_kernel)
    score = score_u8.astype(np.float32)
    return score, row_floor, roi_f


def render_darkness_map(
    working_rgb: Image.Image,
    config: dict[str, int | float | str],
) -> Image.Image:
    frame = np.array(working_rgb)
    height, width = frame.shape[:2]
    crop_top_ratio = float(config.get("crop_top_ratio", 0.35))
    crop_start = max(0, min(height, int(height * crop_top_ratio)))
    roi = frame[crop_start:, :]
    heat = np.zeros((height, width), dtype=np.uint8)
    if roi.size == 0:
        return Image.fromarray(cv2.cvtColor(cv2.applyColorMap(heat, cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB))

    blur_kernel = int(config.get("blur_kernel", 5))
    morph_kernel = int(config.get("morph_kernel", 7))
    loose = False
    if blur_kernel > 1:
        k = blur_kernel if blur_kernel % 2 == 1 else blur_kernel + 1
        roi = cv2.GaussianBlur(roi, (k, k), 0)

    lab = cv2.cvtColor(roi, cv2.COLOR_RGB2LAB)
    lightness = lab[:, :, 0]
    score, row_floor, roi_f = compute_black_tape_darkness_score(lightness, morph_kernel=morph_kernel, loose=loose)

    valid = roi_f <= np.minimum(row_floor - 2.0, 205.0)
    values = score[valid]
    if values.size < 32:
        values = score.reshape(-1)
    lo = float(np.percentile(values, 5)) if values.size else 0.0
    hi = float(np.percentile(values, 99)) if values.size else 1.0
    if hi <= lo + 1e-6:
        scaled = np.zeros_like(score, dtype=np.uint8)
    else:
        scaled = np.uint8(np.clip((score - lo) * (255.0 / (hi - lo)), 0.0, 255.0))

    heat[crop_start:, :] = scaled
    color = cv2.applyColorMap(heat, cv2.COLORMAP_TURBO)
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    if crop_start > 0:
        color[:crop_start, :] = (color[:crop_start, :] * 0.18).astype(np.uint8)
        guide_y = max(0, crop_start - 1)
        color[guide_y : min(height, guide_y + 2), :] = np.array([64, 196, 255], dtype=np.uint8)
    return Image.fromarray(color)


def render_color_index_heatmap(
    working_rgb: Image.Image,
    config: dict[str, int | float | str],
) -> Image.Image:
    frame = np.array(working_rgb)
    height, width = frame.shape[:2]
    crop_top_ratio = float(config.get("crop_top_ratio", 0.35))
    crop_start = max(0, min(height, int(height * crop_top_ratio)))
    roi = frame[crop_start:, :]
    heat = np.zeros((height, width), dtype=np.uint8)
    if roi.size == 0:
        return Image.fromarray(cv2.applyColorMap(heat, cv2.COLORMAP_TURBO))

    mode = str(config.get("detector_mode", "contrast_line"))
    blur_kernel = int(config.get("blur_kernel", 5))
    if blur_kernel > 1:
        k = blur_kernel if blur_kernel % 2 == 1 else blur_kernel + 1
        roi_proc = cv2.GaussianBlur(roi, (k, k), 0)
    else:
        roi_proc = roi.copy()

    gray = cv2.cvtColor(roi_proc, cv2.COLOR_RGB2GRAY)
    lab = cv2.cvtColor(roi_proc, cv2.COLOR_RGB2LAB)
    l = lab[:, :, 0].astype(np.float32)

    # Base darkness score: darker pixels are hotter.
    darkness = (255.0 - l) / 255.0

    # Local contrast score: pixels darker than their neighborhood become hotter.
    local_mean = cv2.GaussianBlur(l, (0, 0), sigmaX=9.0, sigmaY=9.0)
    local_dark = np.clip((local_mean - l) / 48.0, 0.0, 1.0)

    if mode == "color_difference":
        roi_lab = lab.astype(np.float32)
        line_rgb = (
            int(config.get("line_r", 72)),
            int(config.get("line_g", 159)),
            int(config.get("line_b", 83)),
        )
        floor_rgb = (
            int(config.get("floor_r", 143)),
            int(config.get("floor_g", 99)),
            int(config.get("floor_b", 82)),
        )
        line_lab = rgb_triplet_to_lab(line_rgb)
        floor_lab = rgb_triplet_to_lab(floor_rgb)
        dist_line = np.linalg.norm(roi_lab - line_lab, axis=2)
        dist_floor = np.linalg.norm(roi_lab - floor_lab, axis=2)
        advantage = dist_floor - dist_line
        score = np.clip((advantage + 20.0) / 60.0, 0.0, 1.0)
    elif mode == "dark_line":
        score = np.clip(0.45 * darkness + 0.55 * local_dark, 0.0, 1.0)
    else:
        # black_tape / contrast_line: visualize the same darkness-driven ribbon score used by the detector itself before thresholding.
        dark_score, row_floor, roi_f = compute_black_tape_darkness_score(
            lab[:, :, 0],
            morph_kernel=int(config.get("morph_kernel", 7)),
            loose=False,
        )
        valid = roi_f <= np.minimum(row_floor - 2.0, 205.0)
        values = dark_score[valid]
        if values.size < 32:
            values = dark_score.reshape(-1)
        lo = float(np.percentile(values, 5)) if values.size else 0.0
        hi = float(np.percentile(values, 99)) if values.size else 1.0
        if hi <= lo + 1e-6:
            score = np.zeros_like(dark_score, dtype=np.float32)
        else:
            score = np.clip((dark_score - lo) / (hi - lo), 0.0, 1.0)

    heat[crop_start:, :] = np.uint8(np.clip(score * 255.0, 0, 255))
    color = cv2.applyColorMap(heat, cv2.COLORMAP_TURBO)
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    if crop_start > 0:
        color[:crop_start, :] = (color[:crop_start, :] * 0.18).astype(np.uint8)
        guide_y = max(0, crop_start - 1)
        color[guide_y : min(height, guide_y + 2), :] = np.array([64, 196, 255], dtype=np.uint8)
    return Image.fromarray(color)


def select_target_component(
    mask: np.ndarray,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8)
    if binary.sum() == 0:
        return np.zeros_like(mask)

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return np.zeros_like(mask)

    height, width = mask.shape[:2]
    image_center_x = width / 2.0 if anchor_x is None else float(anchor_x)
    bottom_band_start = int(height * 0.68)
    bottom_touch_start = int(height * 0.92)
    hard_bottom_reach = int(height * (0.62 if loose else 0.68))
    min_bottom_reach = int(height * (0.72 if loose else 0.78))
    best_label = 0
    best_score = None

    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < 50:
            continue

        left = int(stats[label, cv2.CC_STAT_LEFT])
        top = int(stats[label, cv2.CC_STAT_TOP])
        comp_width = int(stats[label, cv2.CC_STAT_WIDTH])
        comp_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        if comp_width <= 0 or comp_height <= 0:
            continue

        ys, xs = np.where(labels == label)
        if ys.size == 0:
            continue

        bottom_y = int(ys.max())
        bottom_hits = int(np.count_nonzero(ys >= bottom_touch_start))
        bottom_band_hits = int(np.count_nonzero(ys >= bottom_band_start))

        bottom_xs = xs[ys == bottom_y]
        bottom_x = float(bottom_xs.mean()) if bottom_xs.size else float(xs.mean())
        center_x = float(centroids[label][0])
        aspect_ratio = comp_height / float(comp_width)
        fill_ratio = area / float(comp_width * comp_height)
        top_penalty = max(0, top - int(height * 0.35))
        distance = abs(bottom_x - image_center_x)
        min_area = 80 if loose else 120
        min_height = max(12, int(height * (0.08 if loose else 0.11)))
        min_bottom_hits = 2 if loose else 5
        min_bottom_band_hits = 2 if loose else 4
        min_aspect = 0.75 if loose else 1.0
        min_fill = 0.08 if loose else 0.14
        if area < min_area or comp_height < min_height:
            continue
        if bottom_y < hard_bottom_reach:
            continue
        if bottom_band_hits <= 0:
            continue
        if aspect_ratio < max(0.32, min_aspect * 0.38):
            continue
        side_penalty = 0.0
        if prefer_side != 0:
            side = 1 if center_x > width / 2.0 else -1
            if side != prefer_side:
                side_penalty = 120.0 if not loose else 60.0
        score = (
            distance * 2.0
            + (height - bottom_y) * 0.8
            + abs(center_x - image_center_x) * 0.7
            + top_penalty * 0.5
            + max(0, bottom_band_start - bottom_y) * 2.4
            + max(0, min_bottom_reach - bottom_y) * 1.5
            + max(0, min_bottom_hits - bottom_hits) * 12.0
            + max(0, min_bottom_band_hits - bottom_band_hits) * 10.0
            + max(0.0, min_aspect - aspect_ratio) * 40.0
            + max(0.0, min_fill - fill_ratio) * 120.0
            + side_penalty
            - min(area, 6000) * 0.01
            - min(comp_height, height) * 1.2
            - min(aspect_ratio, 8.0) * 18.0
            - min(fill_ratio, 1.0) * 30.0
            - min(bottom_hits, 80) * 1.2
            - min(bottom_band_hits, 140) * 0.9
        )
        if best_score is None or score < best_score:
            best_score = score
            best_label = label

    if best_label == 0:
        return np.zeros_like(mask)

    filtered = np.zeros_like(mask)
    filtered[labels == best_label] = 255
    return filtered


def select_dark_ribbon_component(
    mask: np.ndarray,
    lightness_roi: np.ndarray,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> np.ndarray:
    """
    Choose the best dark *ribbon-like* component using soft scoring, not strict
    pass/fail gates. This is more stable for glossy black tape that may be broken
    by reflections.
    """
    binary = (mask > 0).astype(np.uint8)
    if int(binary.sum()) == 0:
        return np.zeros_like(mask)

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return np.zeros_like(mask)

    height, width = mask.shape[:2]
    target_center = float(width / 2.0 if anchor_x is None else np.clip(anchor_x, 0.0, width - 1.0))
    if prefer_side != 0 and anchor_x is None:
        target_center = width * (0.64 if prefer_side > 0 else 0.36)

    bottom_band_y = max(0, height - max(10, int(height * 0.18)))
    expected_min_area = max(60, int(mask.size * (0.0008 if loose else 0.0012)))
    preferred_area = max(180, int(mask.size * (0.0030 if loose else 0.0040)))
    preferred_width = max(8, int(width * (0.025 if loose else 0.030)))
    best_label = 0
    best_score = None
    fallback_label = 0
    fallback_area = -1

    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area <= 0:
            continue
        if area > fallback_area:
            fallback_area = area
            fallback_label = label

        left = int(stats[label, cv2.CC_STAT_LEFT])
        top = int(stats[label, cv2.CC_STAT_TOP])
        comp_width = int(stats[label, cv2.CC_STAT_WIDTH])
        comp_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        center_x = float(centroids[label][0])
        component = labels == label
        ys, xs = np.where(component)
        if ys.size == 0:
            continue

        bottom_y = int(ys.max())
        top_y = int(ys.min())
        vertical_span = bottom_y - top_y + 1
        bottom_band = ys >= bottom_band_y
        bottom_touch = 1.0 if np.any(bottom_band) else 0.0
        bottom_x = float(xs[bottom_band].mean()) if np.any(bottom_band) else float(xs.mean())

        sampled_rows = np.unique(np.linspace(top_y, bottom_y, min(28, max(10, vertical_span)), dtype=int))
        row_widths: list[int] = []
        row_centers: list[float] = []
        for row in sampled_rows:
            row_xs = np.flatnonzero(component[row])
            if row_xs.size == 0:
                continue
            splits = np.where(np.diff(row_xs) > 1)[0] + 1
            runs = np.split(row_xs, splits)
            if not runs:
                continue
            best_run = max(runs, key=len)
            row_widths.append(int(len(best_run)))
            row_centers.append(float((best_run[0] + best_run[-1]) / 2.0))
        if not row_widths:
            continue

        median_width = float(np.median(row_widths))
        p80_width = float(np.percentile(row_widths, 80))
        width_std = float(np.std(row_widths))
        fill_ratio = area / float(max(1, comp_width * comp_height))
        aspect_ratio = comp_height / float(max(1, comp_width))
        curvature = float(np.std(np.diff(row_centers))) if len(row_centers) >= 3 else 0.0

        comp_lightness = lightness_roi[component].astype(np.float32)
        light_p20 = float(np.percentile(comp_lightness, 20))
        light_mean = float(comp_lightness.mean())
        darkness_score = (255.0 - light_p20) * 0.75 + (255.0 - light_mean) * 0.25

        area_score = min(area / float(preferred_area), 1.6)
        width_score = min(median_width / float(preferred_width), 2.0)
        span_score = min(vertical_span / float(max(24, int(height * 0.28))), 1.8)
        bottom_score = 1.0 - min(abs(bottom_x - target_center) / float(max(1, width * 0.45)), 1.5)
        center_score = 1.0 - min(abs(center_x - target_center) / float(max(1, width * 0.50)), 1.5)
        stability_score = 1.0 - min(width_std / float(max(2.0, median_width)), 1.2)
        ribbon_score = min(fill_ratio / 0.20, 1.4)

        penalty = 0.0
        if area < expected_min_area:
            penalty += 1.6
        if median_width < max(4.0, preferred_width * 0.45):
            penalty += 2.0
        if p80_width < max(5.0, preferred_width * 0.55):
            penalty += 1.4
        if vertical_span < max(18, int(height * 0.10)):
            penalty += 1.2
        if fill_ratio < 0.04:
            penalty += 1.0
        if aspect_ratio < 0.6:
            penalty += 0.8
        if curvature > max(2.0, median_width * 0.35):
            penalty += 0.8
        if prefer_side != 0:
            side = 1 if center_x > width / 2.0 else -1
            if side != prefer_side:
                penalty += 0.6

        score = (
            darkness_score * 0.020
            + area_score * 20.0
            + width_score * 24.0
            + span_score * 18.0
            + bottom_touch * 12.0
            + bottom_score * 14.0
            + center_score * 6.0
            + stability_score * 10.0
            + ribbon_score * 8.0
            - penalty * 12.0
        )

        if best_score is None or score > best_score:
            best_score = score
            best_label = label

    chosen = best_label if best_label != 0 else fallback_label
    if chosen == 0:
        return np.zeros_like(mask)

    selected = np.zeros_like(mask)
    selected[labels == chosen] = 255
    return selected



def solidify_component_mask(mask: np.ndarray, anchor_x: float | None = None) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8)
    if binary.sum() == 0:
        return np.zeros_like(mask)

    filled = np.zeros_like(mask)
    width = mask.shape[1]
    target_center = float(width / 2.0 if anchor_x is None else anchor_x)
    last_center: float | None = None
    last_width: int | None = None
    for row in range(mask.shape[0] - 1, -1, -1):
        xs = np.flatnonzero(binary[row] > 0)
        if xs.size == 0:
            continue

        runs: list[tuple[int, int]] = []
        run_start = int(xs[0])
        prev_x = int(xs[0])
        for x_val in xs[1:]:
            x_int = int(x_val)
            if x_int != prev_x + 1:
                runs.append((run_start, prev_x))
                run_start = x_int
            prev_x = x_int
        runs.append((run_start, prev_x))

        row_target = last_center if last_center is not None else target_center
        best_left = runs[0][0]
        best_right = runs[0][1]
        best_score = None
        for cand_left, cand_right in runs:
            cand_center = (cand_left + cand_right) / 2.0
            cand_width = cand_right - cand_left + 1
            score = abs(cand_center - row_target)
            if last_width is not None:
                score += abs(cand_width - last_width) * 0.35
            if best_score is None or score < best_score:
                best_score = score
                best_left = cand_left
                best_right = cand_right

        left = int(best_left)
        right = int(best_right)
        current_width = right - left + 1

        # Keep the ribbon width stable so sparse edge pixels do not explode into wedges.
        if last_width is not None:
            center = int(round((left + right) / 2.0))
            if current_width <= 2:
                target_width = max(4, last_width)
            else:
                target_width = min(current_width, max(6, int(round(last_width * 1.35))))
            half = max(2, int(round(target_width / 2.0)))
            left = max(0, center - half)
            right = min(mask.shape[1] - 1, center + half)

        filled[row, left : right + 1] = 255
        last_center = (left + right) / 2.0
        last_width = right - left + 1

    close_kernel = np.ones((5, 3), np.uint8)
    filled = cv2.morphologyEx(filled, cv2.MORPH_CLOSE, close_kernel)
    return filled


def build_contrast_line_mask(
    image_rgb: Image.Image,
    crop_top_ratio: float = 0.35,
    blur_kernel: int = 5,
    morph_kernel: int = 5,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> Image.Image:
    frame = np.array(image_rgb)
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    crop_start = int(gray.shape[0] * crop_top_ratio)
    roi = gray[crop_start:, :]
    if roi.size == 0:
        return Image.fromarray(np.zeros_like(gray), mode="L")

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    roi = clahe.apply(roi)

    if blur_kernel > 1:
        k = blur_kernel if blur_kernel % 2 == 1 else blur_kernel + 1
        roi = cv2.GaussianBlur(roi, (k, k), 0)

    bright_thresh = int(np.percentile(roi, 76 if loose else 82))
    bright_thresh = max(110, min(245, bright_thresh))
    _, mask = cv2.threshold(roi, bright_thresh, 255, cv2.THRESH_BINARY)

    if morph_kernel > 1:
        kernel = np.ones((morph_kernel, morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    mask = select_dark_ribbon_component(mask, roi, anchor_x=anchor_x, prefer_side=prefer_side, loose=loose)
    full_mask = np.zeros_like(gray)
    full_mask[crop_start:, :] = mask
    return Image.fromarray(full_mask, mode="L")


def build_dark_line_mask(
    image_rgb: Image.Image,
    crop_top_ratio: float = 0.35,
    blur_kernel: int = 5,
    morph_kernel: int = 5,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> Image.Image:
    frame = np.array(image_rgb)
    lab = cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)
    lightness = lab[:, :, 0]
    crop_start = int(lightness.shape[0] * crop_top_ratio)
    roi = lightness[crop_start:, :]
    if roi.size == 0:
        return Image.fromarray(np.zeros_like(lightness), mode="L")

    if blur_kernel > 1:
        k = blur_kernel if blur_kernel % 2 == 1 else blur_kernel + 1
        roi = cv2.GaussianBlur(roi, (k, k), 0)

    roi_f = roi.astype(np.float32)
    row_floor = np.percentile(roi_f, 90 if loose else 94, axis=1, keepdims=True)
    row_shadow = row_floor - roi_f
    row_noise = np.std(roi_f, axis=1, keepdims=True)

    background_kernel = max(15, morph_kernel * 4 + 1)
    if background_kernel % 2 == 0:
        background_kernel += 1
    local_background = cv2.GaussianBlur(roi_f, (background_kernel, background_kernel), 0)
    local_shadow = local_background - roi_f

    darkness = np.maximum(row_shadow, local_shadow)
    darkness_gate = np.maximum(12.0 if loose else 16.0, row_noise * (0.95 if loose else 1.20))
    lightness_cap = np.minimum(row_floor - (8.0 if loose else 12.0), 150.0 if loose else 140.0)
    mask = np.where((darkness >= darkness_gate) & (roi_f <= lightness_cap), 255, 0).astype(np.uint8)

    min_pixels = max(120, int(mask.size * 0.0012))
    if int(np.count_nonzero(mask)) < min_pixels:
        relaxed_gate = np.maximum(8.0 if loose else 10.0, row_noise * (0.70 if loose else 0.90))
        relaxed_cap = np.minimum(row_floor - 6.0, 160.0 if loose else 148.0)
        mask = np.where((darkness >= relaxed_gate) & (roi_f <= relaxed_cap), 255, 0).astype(np.uint8)

    if morph_kernel > 1:
        kernel = np.ones((morph_kernel, morph_kernel), np.uint8)
        # Close first so glare streaks inside dark tape become fillable holes.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    mask = select_target_component(mask, anchor_x=anchor_x, prefer_side=prefer_side, loose=loose)
    full_mask = np.zeros_like(lightness)
    full_mask[crop_start:, :] = mask
    return Image.fromarray(full_mask, mode="L")


def build_black_tape_region_mask(
    image_rgb: Image.Image,
    crop_top_ratio: float = 0.35,
    blur_kernel: int = 5,
    morph_kernel: int = 7,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> Image.Image:
    frame = np.array(image_rgb)
    lab = cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)
    lightness = lab[:, :, 0]
    crop_start = int(lightness.shape[0] * crop_top_ratio)
    roi = lightness[crop_start:, :]
    if roi.size == 0:
        return Image.fromarray(np.zeros_like(lightness), mode="L")

    if blur_kernel > 1:
        k = blur_kernel if blur_kernel % 2 == 1 else blur_kernel + 1
        roi = cv2.GaussianBlur(roi, (k, k), 0)

    score, row_floor, roi_f = compute_black_tape_darkness_score(roi, morph_kernel=morph_kernel, loose=loose)

    valid_dark = roi_f <= np.minimum(row_floor - (3.0 if loose else 5.0), 182.0 if loose else 170.0)
    score_values = score[valid_dark]
    if score_values.size < 64:
        score_values = score.reshape(-1)

    thresh = float(np.percentile(score_values, 72 if loose else 78))
    thresh = max(2.0, thresh)
    mask = np.where((score >= thresh) & valid_dark, 255, 0).astype(np.uint8)

    if int(np.count_nonzero(mask)) < max(120, int(mask.size * 0.0012)):
        thresh = float(np.percentile(score_values, 62 if loose else 70))
        relaxed_dark = roi_f <= np.minimum(row_floor - 1.0, 194.0 if loose else 180.0)
        mask = np.where((score >= max(1.0, thresh)) & relaxed_dark, 255, 0).astype(np.uint8)

    if morph_kernel > 1:
        close_kernel = np.ones((max(5, morph_kernel * 2 + 1), max(3, morph_kernel)), np.uint8)
        open_kernel = np.ones((max(3, morph_kernel), max(3, morph_kernel // 2 + 1)), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)

    mask = prune_mask_row_speckles(mask, min_run=max(4, morph_kernel // 2 + 2))

    # Keep the best dark ribbon candidate instead of every dark fragment.
    selected = select_dark_ribbon_component(mask, roi, anchor_x=anchor_x, prefer_side=prefer_side, loose=loose)
    if int(np.count_nonzero(selected)) == 0 and int(np.count_nonzero(mask)) > 0:
        selected = mask

    selected = solidify_component_mask(selected, anchor_x=anchor_x)
    if morph_kernel > 1:
        solid_kernel = np.ones((max(3, morph_kernel), max(5, morph_kernel * 2 + 1)), np.uint8)
        selected = cv2.morphologyEx(selected, cv2.MORPH_CLOSE, solid_kernel)

    full_mask = np.zeros_like(lightness)
    full_mask[crop_start:, :] = selected
    return Image.fromarray(full_mask, mode="L")



def build_black_tape_centerline_mask(
    image_rgb: Image.Image,
    crop_top_ratio: float = 0.35,
    blur_kernel: int = 5,
    morph_kernel: int = 7,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> Image.Image:
    frame = np.array(image_rgb)
    lab = cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)
    lightness = lab[:, :, 0]
    crop_start = int(lightness.shape[0] * crop_top_ratio)
    roi = lightness[crop_start:, :]
    if roi.size == 0:
        return Image.fromarray(np.zeros_like(lightness), mode="L")

    if blur_kernel > 1:
        k = blur_kernel if blur_kernel % 2 == 1 else blur_kernel + 1
        roi = cv2.GaussianBlur(roi, (k, k), 0)

    roi_f = roi.astype(np.float32)
    height, width = roi.shape[:2]
    target_center = float(width / 2.0 if anchor_x is None else np.clip(anchor_x, 0.0, width - 1.0))
    if prefer_side != 0 and anchor_x is None:
        target_center = width * (0.66 if prefer_side > 0 else 0.34)

    background_kernel = max(21, morph_kernel * 6 + 1)
    if background_kernel % 2 == 0:
        background_kernel += 1
    local_background = cv2.GaussianBlur(roi_f, (background_kernel, background_kernel), 0)
    dark_response = np.maximum(local_background - roi_f, 0.0)

    blackhat_kernel = max(9, morph_kernel * 2 + 1)
    if blackhat_kernel % 2 == 0:
        blackhat_kernel += 1
    blackhat = cv2.morphologyEx(
        roi,
        cv2.MORPH_BLACKHAT,
        np.ones((blackhat_kernel, blackhat_kernel), np.uint8),
    ).astype(np.float32)

    score_map = 0.65 * dark_response + 0.35 * blackhat
    score_map = cv2.GaussianBlur(score_map, (5, 1), 0)
    grad_x = cv2.Sobel(roi_f, cv2.CV_32F, 1, 0, ksize=3)

    last_center = target_center
    last_width = max(6, int(width * 0.04))
    misses = 0
    points: list[tuple[int, float, int]] = []
    min_width = 4 if loose else 6
    max_width = max(16, int(width * (0.14 if loose else 0.12)))

    for row in range(height - 1, -1, -1):
        row_score = score_map[row]
        row_grad = grad_x[row]
        row_gate = float(np.percentile(row_score, 78 if loose else 84))
        row_gate = max(4.0, row_gate)

        search_radius = int(max(18, min(width // 2, last_width * (4.0 if misses == 0 else 6.0))))
        left_bound = max(0, int(round(last_center - search_radius)))
        right_bound = min(width, int(round(last_center + search_radius + 1)))
        if right_bound - left_bound < 5:
            left_bound = max(0, int(round(last_center - 4)))
            right_bound = min(width, int(round(last_center + 5)))

        search_scores = row_score[left_bound:right_bound]
        if search_scores.size == 0:
            misses += 1
            continue

        candidate_count = min(8, search_scores.size)
        candidate_rel = np.argpartition(search_scores, -candidate_count)[-candidate_count:]
        candidate_xs = [left_bound + int(idx) for idx in candidate_rel]
        candidate_xs.sort(key=lambda x_val: row_score[x_val], reverse=True)

        best_choice: tuple[float, int, float] | None = None
        grad_gate = max(2.0, float(np.percentile(np.abs(row_grad[left_bound:right_bound]), 72)))
        jump_limit = max(18.0, last_width * (1.4 if misses == 0 else 2.0))

        for x_val in candidate_xs:
            valley_score = float(row_score[x_val])
            if valley_score < row_gate:
                continue

            left_lo = max(0, x_val - max_width)
            left_hi = max(left_lo + 1, x_val - max(2, min_width // 2))
            right_lo = min(width - 1, x_val + max(2, min_width // 2))
            right_hi = min(width, x_val + max_width + 1)
            if left_hi <= left_lo or right_hi <= right_lo:
                continue

            left_edge_slice = -row_grad[left_lo:left_hi]
            right_edge_slice = row_grad[right_lo:right_hi]
            if left_edge_slice.size == 0 or right_edge_slice.size == 0:
                continue

            left_rel = int(np.argmax(left_edge_slice))
            right_rel = int(np.argmax(right_edge_slice))
            left_mag = float(left_edge_slice[left_rel])
            right_mag = float(right_edge_slice[right_rel])
            if min(left_mag, right_mag) < grad_gate * 0.45:
                continue

            left_edge = left_lo + left_rel
            right_edge = right_lo + right_rel
            width_est = right_edge - left_edge
            if width_est < min_width or width_est > max_width:
                continue

            center_est = (left_edge + right_edge) / 2.0
            jump = abs(center_est - last_center)
            if points and jump > jump_limit:
                continue

            score = valley_score * 1.2 + min(left_mag, right_mag) * 1.6 + 0.25 * (left_mag + right_mag)
            score -= jump * 0.35
            score -= abs(width_est - last_width) * 0.18
            if best_choice is None or score > best_choice[2]:
                best_choice = (center_est, width_est, score)

        if best_choice is None:
            misses += 1
            if misses > 10 and row < int(height * 0.65):
                break
            continue

        center_est, width_est, _score = best_choice
        if points:
            smooth = 0.72 if misses == 0 else 0.55
            center_est = smooth * center_est + (1.0 - smooth) * last_center
            width_est = int(round(smooth * width_est + (1.0 - smooth) * last_width))

        points.append((row, float(center_est), int(width_est)))
        last_center = float(center_est)
        last_width = int(np.clip(width_est, min_width, max_width))
        misses = 0

    min_points = max(10, int(height * 0.10))
    if len(points) < min_points:
        return Image.fromarray(np.zeros_like(lightness), mode="L")

    centerline_mask = np.zeros_like(roi, dtype=np.uint8)
    points.sort(key=lambda item: item[0])
    draw_points = []
    median_width = int(np.median([width_est for _, _, width_est in points]))
    thickness = int(np.clip(max(3, round(median_width * 0.35), morph_kernel // 2 + 1), 3, 15))
    for row, center_est, _width_est in points:
        draw_points.append([int(round(center_est)), int(row)])

    polyline = np.array(draw_points, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(centerline_mask, [polyline], isClosed=False, color=255, thickness=thickness)
    for x_px, y_px in draw_points[:: max(1, len(draw_points) // 24)]:
        cv2.circle(centerline_mask, (int(x_px), int(y_px)), max(1, thickness // 2), 255, -1)

    if morph_kernel > 1:
        centerline_mask = cv2.morphologyEx(
            centerline_mask,
            cv2.MORPH_CLOSE,
            np.ones((max(3, morph_kernel), max(3, morph_kernel)), np.uint8),
        )

    full_mask = np.zeros_like(lightness)
    full_mask[crop_start:, :] = centerline_mask
    return Image.fromarray(full_mask, mode="L")


def extract_centerline_strip_from_mask(
    mask: np.ndarray,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
    base_thickness: int = 7,
) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8)
    if int(binary.sum()) == 0:
        return np.zeros_like(mask)

    height, width = binary.shape[:2]
    target_center = float(width / 2.0 if anchor_x is None else np.clip(anchor_x, 0.0, width - 1.0))
    if prefer_side != 0 and anchor_x is None:
        target_center = width * (0.64 if prefer_side > 0 else 0.36)

    points: list[tuple[int, float, int]] = []
    last_center: float | None = None
    last_width: float | None = None
    max_gap = max(10, int(height * 0.10))
    gap = 0

    for row in range(height - 1, -1, -1):
        xs = np.flatnonzero(binary[row] > 0)
        if xs.size == 0:
            gap += 1
            if points and gap > max_gap:
                break
            continue
        gap = 0

        splits = np.where(np.diff(xs) > 1)[0] + 1
        runs = np.split(xs, splits)
        if not runs:
            continue

        row_target = last_center if last_center is not None else target_center
        best_run = None
        best_score = None
        for run in runs:
            run_left = int(run[0])
            run_right = int(run[-1])
            run_width = run_right - run_left + 1
            run_center = (run_left + run_right) / 2.0
            score = run_width * 2.0 - abs(run_center - row_target) * 1.2
            if last_width is not None:
                score -= abs(run_width - last_width) * 0.6
            if best_score is None or score > best_score:
                best_score = score
                best_run = (run_center, run_width)
        if best_run is None:
            continue

        center, width_est = best_run
        if last_center is not None:
            smooth = 0.70 if gap == 0 else 0.55
            center = smooth * center + (1.0 - smooth) * last_center
            width_est = smooth * width_est + (1.0 - smooth) * last_width

        points.append((row, float(center), int(round(width_est))))
        last_center = float(center)
        last_width = float(width_est)

    min_points = max(12, int(height * 0.12))
    if len(points) < min_points:
        return np.zeros_like(mask)

    points.sort(key=lambda item: item[0])
    widths = np.array([max(1, w) for _, _, w in points], dtype=np.float32)
    median_width = float(np.median(widths))
    strip_width = int(np.clip(max(base_thickness, round(median_width * 0.42)), max(4, base_thickness), max(14, int(width * 0.10))))

    out = np.zeros_like(mask)
    last_draw_center: float | None = None
    for row, center, _w in points:
        if last_draw_center is not None:
            center = 0.78 * center + 0.22 * last_draw_center
        half = max(2, strip_width // 2)
        left = max(0, int(round(center)) - half)
        right = min(width - 1, int(round(center)) + half)
        out[row, left:right + 1] = 255
        last_draw_center = center

    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, np.ones((5, 3), np.uint8))
    return out


def refine_to_straight_strip(mask: np.ndarray, min_pixels: int = 80) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8)
    if int(binary.sum()) < min_pixels:
        return mask

    ys, xs = np.where(binary > 0)
    if ys.size < min_pixels:
        return mask

    vx, vy, x0, y0 = cv2.fitLine(np.column_stack([xs, ys]).astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01)
    vx = float(vx)
    vy = float(vy)
    if abs(vy) < 1e-5:
        return mask

    pred_x = x0 + (ys - y0) * (vx / vy)
    residual = np.abs(xs - pred_x)
    if float(np.median(residual)) > 3.5:
        return mask

    height, width = mask.shape[:2]
    strip_width = int(np.clip(max(4, round(np.median([len(np.flatnonzero(binary[r] > 0)) for r in np.unique(ys)[:min(24, len(np.unique(ys)))]] or [6]) * 0.5)), 4, 14))
    out = np.zeros_like(mask)
    for row in range(height):
        x = x0 + (row - y0) * (vx / vy)
        if not np.isfinite(x):
            continue
        x_i = int(round(float(x)))
        half = max(2, strip_width // 2)
        left = max(0, x_i - half)
        right = min(width - 1, x_i + half)
        out[row, left:right + 1] = 255
    return out


def build_black_tape_mask(
    image_rgb: Image.Image,
    crop_top_ratio: float = 0.35,
    blur_kernel: int = 5,
    morph_kernel: int = 7,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> Image.Image:
    region = build_black_tape_region_mask(
        image_rgb,
        crop_top_ratio=crop_top_ratio,
        blur_kernel=blur_kernel,
        morph_kernel=morph_kernel,
        anchor_x=anchor_x,
        prefer_side=prefer_side,
        loose=loose,
    )
    region_arr = keep_primary_path_component(
        np.array(region),
        anchor_x=anchor_x,
        prefer_side=prefer_side,
        loose=loose,
    )
    region_arr = prune_mask_row_speckles(region_arr, min_run=max(4, morph_kernel // 2 + 2))
    if int(np.count_nonzero(region_arr)) >= 120:
        center_strip = extract_centerline_strip_from_mask(
            region_arr,
            anchor_x=anchor_x,
            prefer_side=prefer_side,
            loose=loose,
            base_thickness=max(5, morph_kernel),
        )
        center_strip = keep_primary_path_component(
            center_strip,
            anchor_x=anchor_x,
            prefer_side=prefer_side,
            loose=loose,
        )
        center_strip = prune_mask_row_speckles(center_strip, min_run=max(4, morph_kernel // 2 + 2))
        if int(np.count_nonzero(center_strip)) >= 60:
            refined = refine_to_straight_strip(center_strip, min_pixels=60)
            refined = keep_primary_path_component(
                refined,
                anchor_x=anchor_x,
                prefer_side=prefer_side,
                loose=loose,
            )
            refined = prune_mask_row_speckles(refined, min_run=max(4, morph_kernel // 2 + 2))
            return Image.fromarray(refined, mode="L")
        return Image.fromarray(region_arr, mode="L")

    centerline = build_black_tape_centerline_mask(
        image_rgb,
        crop_top_ratio=crop_top_ratio,
        blur_kernel=blur_kernel,
        morph_kernel=morph_kernel,
        anchor_x=anchor_x,
        prefer_side=prefer_side,
        loose=loose,
    )
    center_arr = keep_primary_path_component(
        np.array(centerline),
        anchor_x=anchor_x,
        prefer_side=prefer_side,
        loose=loose,
    )
    center_arr = prune_mask_row_speckles(center_arr, min_run=max(4, morph_kernel // 2 + 2))
    if int(np.count_nonzero(center_arr)) >= 60:
        refined = refine_to_straight_strip(center_arr, min_pixels=60)
        refined = keep_primary_path_component(
            refined,
            anchor_x=anchor_x,
            prefer_side=prefer_side,
            loose=loose,
        )
        refined = prune_mask_row_speckles(refined, min_run=max(4, morph_kernel // 2 + 2))
        return Image.fromarray(refined, mode="L")

    return Image.fromarray(region_arr, mode="L")



def build_color_difference_mask(
    image_rgb: Image.Image,
    crop_top_ratio: float = 0.35,
    blur_kernel: int = 5,
    morph_kernel: int = 5,
    line_rgb: tuple[int, int, int] = (72, 159, 83),
    floor_rgb: tuple[int, int, int] = (143, 99, 82),
    color_distance_threshold: float = 42.0,
    color_distance_margin: float = 8.0,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> Image.Image:
    frame = np.array(image_rgb)
    crop_start = int(frame.shape[0] * crop_top_ratio)
    roi = frame[crop_start:, :]
    if roi.size == 0:
        return Image.fromarray(np.zeros(frame.shape[:2], dtype=np.uint8), mode="L")

    if blur_kernel > 1:
        k = blur_kernel if blur_kernel % 2 == 1 else blur_kernel + 1
        roi = cv2.GaussianBlur(roi, (k, k), 0)

    roi_lab = cv2.cvtColor(roi, cv2.COLOR_RGB2LAB).astype(np.float32)
    line_lab = rgb_triplet_to_lab(line_rgb)
    floor_lab = rgb_triplet_to_lab(floor_rgb)
    dist_line = np.linalg.norm(roi_lab - line_lab, axis=2)
    dist_floor = np.linalg.norm(roi_lab - floor_lab, axis=2)

    threshold = max(6.0, float(color_distance_threshold))
    margin = max(0.0, float(color_distance_margin))
    line_floor_delta = line_lab - floor_lab
    line_floor_span = float(np.linalg.norm(line_floor_delta))

    mask = (dist_line <= threshold).astype(np.uint8) * 255
    if line_floor_span > 1.0:
        closer_to_line = (dist_line + margin) < dist_floor

        # Project each pixel onto the sampled floor->line color axis so the line can
        # remain valid even when the same tape appears brighter or dimmer across the frame.
        axis = line_floor_delta / line_floor_span
        rel = roi_lab - floor_lab
        along = rel[..., 0] * axis[0] + rel[..., 1] * axis[1] + rel[..., 2] * axis[2]
        orth = np.linalg.norm(rel - along[..., None] * axis, axis=2)
        projection = along / line_floor_span

        projection_gate = 0.42 if loose else 0.50
        orth_gate = max(16.0, threshold * (0.78 if loose else 0.62))
        axis_mask = (projection >= projection_gate) & (orth <= orth_gate)

        mask = np.where(closer_to_line | axis_mask, 255, 0).astype(np.uint8)

    if morph_kernel > 1:
        kernel = np.ones((morph_kernel, morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    mask = select_target_component(mask, anchor_x=anchor_x, prefer_side=prefer_side, loose=loose)
    full_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    full_mask[crop_start:, :] = mask
    return Image.fromarray(full_mask, mode="L")


def build_tracking_mask(
    image_rgb: Image.Image,
    config: dict[str, int | float | str],
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> Image.Image:
    image_rgb = prepare_mask_working_image(image_rgb, config)
    mode = str(config.get("detector_mode", "contrast_line"))
    if mode == "black_tape":
        return build_black_tape_mask(
            image_rgb,
            crop_top_ratio=float(config["crop_top_ratio"]),
            blur_kernel=int(config["blur_kernel"]),
            morph_kernel=int(config["morph_kernel"]),
            anchor_x=anchor_x,
            prefer_side=prefer_side,
            loose=loose,
        )
    if mode == "dark_line":
        return build_dark_line_mask(
            image_rgb,
            crop_top_ratio=float(config["crop_top_ratio"]),
            blur_kernel=int(config["blur_kernel"]),
            morph_kernel=int(config["morph_kernel"]),
            anchor_x=anchor_x,
            prefer_side=prefer_side,
            loose=loose,
        )
    if mode == "color_difference":
        return build_color_difference_mask(
            image_rgb,
            crop_top_ratio=float(config["crop_top_ratio"]),
            blur_kernel=int(config["blur_kernel"]),
            morph_kernel=int(config["morph_kernel"]),
            line_rgb=(
                int(config.get("line_r", 72)),
                int(config.get("line_g", 159)),
                int(config.get("line_b", 83)),
            ),
            floor_rgb=(
                int(config.get("floor_r", 143)),
                int(config.get("floor_g", 99)),
                int(config.get("floor_b", 82)),
            ),
            color_distance_threshold=float(config.get("color_distance_threshold", 42.0)),
            color_distance_margin=float(config.get("color_distance_margin", 8.0)),
            anchor_x=anchor_x,
            prefer_side=prefer_side,
            loose=loose,
        )
    if mode == "hsv_color":
        mask = build_lane_mask(
            image_rgb,
            crop_top_ratio=float(config["crop_top_ratio"]),
            hsv_lower=(int(config["h_min"]), int(config["s_min"]), int(config["v_min"])),
            hsv_upper=(int(config["h_max"]), int(config["s_max"]), int(config["v_max"])),
            blur_kernel=int(config["blur_kernel"]),
            morph_kernel=int(config["morph_kernel"]),
        )
        return Image.fromarray(select_target_component(np.array(mask), anchor_x=anchor_x, prefer_side=prefer_side, loose=loose), mode="L")

    return build_contrast_line_mask(
        image_rgb,
        crop_top_ratio=float(config["crop_top_ratio"]),
        blur_kernel=int(config["blur_kernel"]),
        morph_kernel=int(config["morph_kernel"]),
        anchor_x=anchor_x,
        prefer_side=prefer_side,
        loose=loose,
    )


def build_detector_region_mask(
    image_rgb: Image.Image,
    config: dict[str, int | float | str],
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> Image.Image:
    image_rgb = prepare_mask_working_image(image_rgb, config)
    mode = str(config.get("detector_mode", "contrast_line"))
    if mode == "black_tape":
        return build_black_tape_region_mask(
            image_rgb,
            crop_top_ratio=float(config["crop_top_ratio"]),
            blur_kernel=int(config["blur_kernel"]),
            morph_kernel=int(config["morph_kernel"]),
            anchor_x=anchor_x,
            prefer_side=prefer_side,
            loose=loose,
        )
    if mode == "dark_line":
        return build_dark_line_mask(
            image_rgb,
            crop_top_ratio=float(config["crop_top_ratio"]),
            blur_kernel=int(config["blur_kernel"]),
            morph_kernel=int(config["morph_kernel"]),
            anchor_x=anchor_x,
            prefer_side=prefer_side,
            loose=loose,
        )
    if mode == "color_difference":
        return build_color_difference_mask(
            image_rgb,
            crop_top_ratio=float(config["crop_top_ratio"]),
            blur_kernel=int(config["blur_kernel"]),
            morph_kernel=int(config["morph_kernel"]),
            line_rgb=(
                int(config.get("line_r", 72)),
                int(config.get("line_g", 159)),
                int(config.get("line_b", 83)),
            ),
            floor_rgb=(
                int(config.get("floor_r", 143)),
                int(config.get("floor_g", 99)),
                int(config.get("floor_b", 82)),
            ),
            color_distance_threshold=float(config.get("color_distance_threshold", 42.0)),
            color_distance_margin=float(config.get("color_distance_margin", 8.0)),
            anchor_x=anchor_x,
            prefer_side=prefer_side,
            loose=loose,
        )
    if mode == "hsv_color":
        mask = build_lane_mask(
            image_rgb,
            crop_top_ratio=float(config["crop_top_ratio"]),
            hsv_lower=(int(config["h_min"]), int(config["s_min"]), int(config["v_min"])),
            hsv_upper=(int(config["h_max"]), int(config["s_max"]), int(config["v_max"])),
            blur_kernel=int(config["blur_kernel"]),
            morph_kernel=int(config["morph_kernel"]),
        )
        region = select_target_component(np.array(mask), anchor_x=anchor_x, prefer_side=prefer_side, loose=loose)
        return Image.fromarray(region, mode="L")

    return build_contrast_line_mask(
        image_rgb,
        crop_top_ratio=float(config["crop_top_ratio"]),
        blur_kernel=int(config["blur_kernel"]),
        morph_kernel=int(config["morph_kernel"]),
        anchor_x=anchor_x,
        prefer_side=prefer_side,
        loose=loose,
    )


def compute_row_center(mask: np.ndarray, row_ratio: float, search_radius: int | None = None) -> tuple[float | None, int]:
    row_index = int(mask.shape[0] * row_ratio)
    row_index = max(0, min(mask.shape[0] - 1, row_index))
    if search_radius is None:
        search_radius = max(4, int(mask.shape[0] * 0.035))

    def row_center(row: int) -> float | None:
        xs = np.flatnonzero(mask[row] > 0)
        if xs.size == 0:
            return None
        splits = np.where(np.diff(xs) > 1)[0]
        starts = np.r_[0, splits + 1]
        ends = np.r_[splits, xs.size - 1]
        best_run = None
        best_len = -1
        for start, end in zip(starts, ends):
            run = xs[start:end + 1]
            if run.size > best_len:
                best_len = int(run.size)
                best_run = run
        if best_run is None or best_run.size == 0:
            return float(xs.mean())
        return float(best_run.mean())

    center = row_center(row_index)
    if center is not None:
        return center, row_index

    for delta in range(1, search_radius + 1):
        upper = row_index - delta
        lower = row_index + delta
        if upper >= 0:
            center = row_center(upper)
            if center is not None:
                return center, upper
        if lower < mask.shape[0]:
            center = row_center(lower)
            if center is not None:
                return center, lower

    return None, row_index


def estimate_strip_pose(
    mask: np.ndarray,
    near_row_ratio: float = 0.82,
    far_row_ratio: float = 0.42,
) -> tuple[float | None, float | None, int, int, str]:
    x_near, row_near = compute_row_center(mask, near_row_ratio)
    x_far, row_far = compute_row_center(mask, far_row_ratio)
    if x_near is not None and x_far is not None:
        return x_near, x_far, row_near, row_far, "row"

    points = collect_centerline_points(mask)
    if len(points) >= 3:
        pts = np.array(points, dtype=np.float32)
        xs_pts = pts[:, 0]
        ys_pts = pts[:, 1]
        order = np.argsort(ys_pts)
        ys_pts = ys_pts[order]
        xs_pts = xs_pts[order]
        unique_ys, unique_idx = np.unique(ys_pts, return_index=True)
        xs_pts = xs_pts[unique_idx]
        ys_pts = unique_ys
        if ys_pts.size >= 3 and float(ys_pts[-1] - ys_pts[0]) >= max(12.0, mask.shape[0] * 0.08):
            target_near = float(np.clip(row_near, ys_pts[0], ys_pts[-1]))
            target_far = float(np.clip(row_far, ys_pts[0], ys_pts[-1]))
            if x_near is None:
                x_near = float(np.interp(target_near, ys_pts, xs_pts))
                row_near = int(round(target_near))
            if x_far is None:
                x_far = float(np.interp(target_far, ys_pts, xs_pts))
                row_far = int(round(target_far))
            if x_near is not None and x_far is not None:
                width = mask.shape[1]
                x_near = float(np.clip(x_near, 0, width - 1))
                x_far = float(np.clip(x_far, 0, width - 1))
                return x_near, x_far, row_near, row_far, "curve"

    ys, xs = np.where(mask > 0)
    if xs.size < 40:
        return x_near, x_far, row_near, row_far, "none"

    if ys.max() - ys.min() < max(18, int(mask.shape[0] * 0.12)):
        return x_near, x_far, row_near, row_far, "none"

    points = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
    vx, vy, x0, y0 = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
    vx = float(vx)
    vy = float(vy)
    x0 = float(x0)
    y0 = float(y0)
    if abs(vy) < 1e-4:
        return x_near, x_far, row_near, row_far, "none"

    if x_near is None:
        x_near = x0 + ((row_near - y0) * vx / vy)
    if x_far is None:
        x_far = x0 + ((row_far - y0) * vx / vy)

    width = mask.shape[1]
    x_near = float(np.clip(x_near, 0, width - 1))
    x_far = float(np.clip(x_far, 0, width - 1))
    return x_near, x_far, row_near, row_far, "fit"


def collect_centerline_points(
    mask: np.ndarray,
    row_ratios: list[float] | None = None,
) -> list[tuple[float, float]]:
    if row_ratios is None:
        row_ratios = [0.96, 0.92, 0.88, 0.84, 0.80, 0.74, 0.68, 0.62, 0.56, 0.50, 0.44, 0.38]

    def choose_run_center(row: int, target_x: float | None) -> float | None:
        xs = np.flatnonzero(mask[row] > 0)
        if xs.size == 0:
            return None
        splits = np.where(np.diff(xs) > 1)[0] + 1
        runs = np.split(xs, splits)
        if not runs:
            return None
        best_run = None
        best_score = None
        for run in runs:
            if run.size == 0:
                continue
            run_center = float((run[0] + run[-1]) / 2.0)
            run_width = float(run.size)
            score = run_width * 2.0
            if target_x is not None:
                score -= abs(run_center - target_x) * 0.9
            if best_score is None or score > best_score:
                best_score = score
                best_run = run_center
        return best_run

    points: list[tuple[float, float]] = []
    last_x: float | None = None
    for ratio in row_ratios:
        row = int(mask.shape[0] * ratio)
        row = max(0, min(mask.shape[0] - 1, row))
        x = choose_run_center(row, last_x)
        if x is None:
            x, row = compute_row_center(mask, ratio)
        if x is None:
            continue
        if last_x is not None:
            x = 0.72 * float(x) + 0.28 * float(last_x)
        points.append((float(x), float(row)))
        last_x = float(x)

    if len(points) >= 3:
        return points

    ys, xs = np.where(mask > 0)
    if xs.size < 40:
        return points

    sample_rows = np.linspace(max(float(ys.min()), mask.shape[0] * 0.38), float(ys.max()), 9)
    for row_f in sample_rows:
        row = int(np.clip(round(row_f), 0, mask.shape[0] - 1))
        row_xs = xs[ys == row]
        if row_xs.size == 0:
            nearby = np.abs(ys - row) <= 2
            row_xs = xs[nearby]
        if row_xs.size == 0:
            continue
        points.append((float(row_xs.mean()), float(row)))
    points.sort(key=lambda item: item[1], reverse=True)
    return points

    ys, xs = np.where(mask > 0)
    if xs.size < 40:
        return points

    sample_rows = np.linspace(max(float(ys.min()), mask.shape[0] * 0.38), float(ys.max()), 9)
    for row_f in sample_rows:
        row = int(np.clip(round(row_f), 0, mask.shape[0] - 1))
        row_xs = xs[ys == row]
        if row_xs.size == 0:
            nearby = np.abs(ys - row) <= 2
            row_xs = xs[nearby]
        if row_xs.size == 0:
            continue
        points.append((float(row_xs.mean()), float(row)))
    points.sort(key=lambda item: item[1], reverse=True)
    return points


def summarize_centerline(mask: np.ndarray) -> dict[str, float] | None:
    points = collect_centerline_points(mask)
    if len(points) < 3:
        return None

    mask_h, mask_w = mask.shape[:2]
    lateral_scale = max(mask_w / 2.0, 1.0)
    longitudinal_scale = max(mask_h - 1, 1.0)

    long_vals = np.array([(mask_h - 1 - y_px) / longitudinal_scale for _, y_px in points], dtype=np.float32)
    lateral_vals = np.array([(x_px - (mask_w / 2.0)) / lateral_scale for x_px, _ in points], dtype=np.float32)
    order = np.argsort(long_vals)
    long_vals = long_vals[order]
    lateral_vals = lateral_vals[order]

    unique_longs, unique_idx = np.unique(long_vals, return_index=True)
    unique_lats = lateral_vals[unique_idx]
    if unique_longs.size < 2:
        return None

    q_bottom = np.quantile(unique_longs, 0.06)
    q_near, q_mid, q_far = np.quantile(unique_longs, [0.15, 0.50, 0.85])
    bottom_offset = float(np.interp(q_bottom, unique_longs, unique_lats))
    near_offset = float(np.interp(q_near, unique_longs, unique_lats))
    mid_offset = float(np.interp(q_mid, unique_longs, unique_lats))
    far_offset = float(np.interp(q_far, unique_longs, unique_lats))

    seg_near = max(float(q_mid - q_near), 0.06)
    seg_far = max(float(q_far - q_mid), 0.06)
    heading_near = (mid_offset - near_offset) / seg_near
    heading_far = (far_offset - mid_offset) / seg_far
    heading_norm = float(np.clip(0.40 * (0.60 * heading_near + 0.40 * heading_far), -1.0, 1.0))

    curvature_raw = (heading_far - heading_near) / max(float(q_far - q_near), 0.10)
    curvature_norm = float(np.clip(0.22 * curvature_raw, -1.0, 1.0))
    future_offset = float(np.clip(0.65 * mid_offset + 0.35 * far_offset, -1.0, 1.0))
    path_span = float(np.clip(q_far - q_near, 0.0, 1.0))

    return {
        "bottom_offset_norm": float(np.clip(bottom_offset, -1.0, 1.0)),
        "near_offset_norm": float(np.clip(near_offset, -1.0, 1.0)),
        "mid_offset_norm": float(np.clip(mid_offset, -1.0, 1.0)),
        "far_offset_norm": float(np.clip(far_offset, -1.0, 1.0)),
        "future_offset_norm": future_offset,
        "heading_norm": heading_norm,
        "curvature_norm": curvature_norm,
        "path_span": path_span,
    }


def render_trajectory_prediction(mask: np.ndarray, camera_center_offset: float = 0.0) -> Image.Image:
    plot_h = 420
    plot_w = 420
    margin_left = 58
    margin_right = 20
    margin_top = 28
    margin_bottom = 42
    canvas = np.zeros((plot_h, plot_w, 3), dtype=np.uint8)
    canvas[:] = (15, 20, 38)

    plot_x0 = margin_left
    plot_y0 = margin_top
    plot_x1 = plot_w - margin_right
    plot_y1 = plot_h - margin_bottom
    cv2.rectangle(canvas, (plot_x0, plot_y0), (plot_x1, plot_y1), (82, 92, 128), 1)

    for frac in [0.25, 0.5, 0.75]:
        x = int(plot_x0 + frac * (plot_x1 - plot_x0))
        y = int(plot_y0 + frac * (plot_y1 - plot_y0))
        cv2.line(canvas, (x, plot_y0), (x, plot_y1), (40, 46, 72), 1)
        cv2.line(canvas, (plot_x0, y), (plot_x1, y), (40, 46, 72), 1)

    center_x = (plot_x0 + plot_x1) // 2
    cv2.line(canvas, (center_x, plot_y0), (center_x, plot_y1), (95, 104, 146), 1)

    points = collect_centerline_points(mask)
    if points:
        mask_h, mask_w = mask.shape[:2]
        history_xy: list[tuple[int, int]] = []
        center_offset = float(np.clip(camera_center_offset, -0.5, 0.5))
        for x_px, y_px in points:
            lateral = float((x_px - (mask_w / 2.0)) / max(mask_w / 2.0, 1.0)) - center_offset
            longitudinal = float((mask_h - 1 - y_px) / max(mask_h - 1, 1.0))
            px = int(np.clip(center_x + lateral * 0.46 * (plot_x1 - plot_x0), plot_x0, plot_x1))
            py = int(np.clip(plot_y1 - longitudinal * (plot_y1 - plot_y0), plot_y0, plot_y1))
            history_xy.append((px, py))

        if len(history_xy) >= 2:
            cv2.polylines(canvas, [np.array(history_xy, dtype=np.int32)], False, (92, 214, 122), 3, cv2.LINE_AA)
        for px, py in history_xy:
            cv2.circle(canvas, (px, py), 3, (92, 214, 122), -1)

    ego_w = 12
    ego_h = 22
    ego_y = plot_y1 - ego_h - 4
    cv2.rectangle(canvas, (center_x - ego_w // 2, ego_y), (center_x + ego_w // 2, ego_y + ego_h), (243, 209, 96), -1)

    cv2.putText(canvas, "Trajectory Centerline", (18, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (232, 236, 252), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Lateral", (plot_w // 2 - 24, plot_h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 188, 214), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Longitudinal", (8, plot_h // 2 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 188, 214), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Centerline", (plot_w - 118, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (92, 214, 122), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Ego", (plot_w - 118, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (243, 209, 96), 1, cv2.LINE_AA)
    return Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))


def render_live_mask_preview(region_mask: np.ndarray, tracking_mask: np.ndarray) -> Image.Image:
    mask_h, mask_w = tracking_mask.shape[:2]
    canvas = np.zeros((mask_h, mask_w, 3), dtype=np.uint8)
    canvas[:] = (8, 10, 14)

    region_binary = (region_mask > 0).astype(np.uint8) * 255
    tracking_binary = (tracking_mask > 0).astype(np.uint8) * 255
    if int(np.count_nonzero(region_binary)) == 0:
        region_binary = tracking_binary.copy()

    region_fill = cv2.GaussianBlur(region_binary, (0, 0), sigmaX=1.2, sigmaY=1.2)
    region_fill = np.clip(region_fill.astype(np.float32) / 255.0, 0.0, 1.0)
    canvas[..., 0] = np.maximum(canvas[..., 0], np.uint8(region_fill * 72))
    canvas[..., 1] = np.maximum(canvas[..., 1], np.uint8(region_fill * 124))
    canvas[..., 2] = np.maximum(canvas[..., 2], np.uint8(region_fill * 188))

    region_edge = cv2.morphologyEx(region_binary, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
    canvas[region_edge > 0] = np.array([82, 196, 255], dtype=np.uint8)

    tracking_core = cv2.dilate(tracking_binary, np.ones((3, 3), np.uint8), iterations=1)
    canvas[tracking_core > 0] = np.array([245, 248, 250], dtype=np.uint8)

    points = collect_centerline_points(tracking_binary)
    if len(points) >= 2:
        poly = np.array([(int(round(x)), int(round(y))) for x, y in points], dtype=np.int32)
        cv2.polylines(canvas, [poly], False, (106, 255, 140), 3, cv2.LINE_AA)
        for x, y in poly:
            cv2.circle(canvas, (int(x), int(y)), 3, (255, 208, 96), -1)

    return Image.fromarray(canvas, mode="RGB")




def prune_mask_row_speckles(
    mask: np.ndarray,
    min_run: int = SUNLIGHT_MIN_ROW_RUN,
    max_jump: int = SUNLIGHT_MAX_PATH_JUMP,
) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8)
    if int(binary.sum()) == 0:
        return np.zeros_like(mask)

    height, width = binary.shape[:2]
    out = np.zeros_like(binary)
    prev_center: float | None = None
    seed_row = None

    # Start from the lower region where the true path is expected to be strongest.
    for row in range(height - 1, max(-1, int(height * 0.40) - 1), -1):
        xs = np.flatnonzero(binary[row] > 0)
        if xs.size == 0:
            continue
        splits = np.where(np.diff(xs) > 1)[0] + 1
        runs = [run for run in np.split(xs, splits) if run.size > 0]
        if not runs:
            continue
        best_run = None
        best_score = None
        for run in runs:
            run_width = int(run.size)
            if run_width < max(2, min_run - 1):
                continue
            center = float((run[0] + run[-1]) / 2.0)
            score = run_width * 2.5
            if row >= int(height * 0.72):
                score += 12.0
            score -= abs(center - (width / 2.0)) * 0.12
            if best_score is None or score > best_score:
                best_score = score
                best_run = run
        if best_run is not None:
            out[row, best_run[0]:best_run[-1] + 1] = 1
            prev_center = float((best_run[0] + best_run[-1]) / 2.0)
            seed_row = row
            break

    if seed_row is None:
        return (binary * 255).astype(np.uint8)

    for row in range(seed_row - 1, -1, -1):
        xs = np.flatnonzero(binary[row] > 0)
        if xs.size == 0:
            continue
        splits = np.where(np.diff(xs) > 1)[0] + 1
        runs = [run for run in np.split(xs, splits) if run.size > 0]
        if not runs:
            continue
        best_run = None
        best_score = None
        for run in runs:
            run_width = int(run.size)
            center = float((run[0] + run[-1]) / 2.0)
            jump = abs(center - prev_center) if prev_center is not None else 0.0
            if run_width < max(2, min_run - 2) and jump > max_jump * 0.55:
                continue
            score = run_width * 2.2 - jump * 0.95
            if run_width >= min_run:
                score += 5.0
            if best_score is None or score > best_score:
                best_score = score
                best_run = run
        if best_run is None:
            continue
        center = float((best_run[0] + best_run[-1]) / 2.0)
        jump = abs(center - prev_center) if prev_center is not None else 0.0
        if prev_center is not None and jump > max_jump:
            continue
        out[row, best_run[0]:best_run[-1] + 1] = 1
        prev_center = center

    for row in range(seed_row + 1, height):
        xs = np.flatnonzero(binary[row] > 0)
        if xs.size == 0:
            continue
        splits = np.where(np.diff(xs) > 1)[0] + 1
        runs = [run for run in np.split(xs, splits) if run.size > 0]
        if not runs:
            continue
        best_run = None
        best_score = None
        for run in runs:
            run_width = int(run.size)
            center = float((run[0] + run[-1]) / 2.0)
            jump = abs(center - prev_center) if prev_center is not None else 0.0
            if run_width < max(2, min_run - 2) and jump > max_jump * 0.55:
                continue
            score = run_width * 2.2 - jump * 1.05
            if run_width >= min_run:
                score += 5.0
            if best_score is None or score > best_score:
                best_score = score
                best_run = run
        if best_run is None:
            continue
        center = float((best_run[0] + best_run[-1]) / 2.0)
        jump = abs(center - prev_center) if prev_center is not None else 0.0
        if prev_center is not None and jump > max_jump:
            continue
        out[row, best_run[0]:best_run[-1] + 1] = 1
        prev_center = center

    out = (out * 255).astype(np.uint8)
    kernel = np.ones((3, 5), np.uint8)
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel)
    return out


def keep_primary_path_component(
    mask: np.ndarray,
    anchor_x: float | None = None,
    prefer_side: int = 0,
    loose: bool = False,
) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8)
    if int(binary.sum()) == 0:
        return np.zeros_like(mask)

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 2:
        return mask.copy()

    height, width = mask.shape[:2]
    target_center = float(width / 2.0 if anchor_x is None else np.clip(anchor_x, 0.0, width - 1.0))
    if prefer_side != 0 and anchor_x is None:
        target_center = width * (0.64 if prefer_side > 0 else 0.36)
    bottom_band_start = int(height * 0.72)

    best_label = 0
    best_score = None
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < max(24, int(mask.size * 0.00025)):
            continue
        left = int(stats[label, cv2.CC_STAT_LEFT])
        top = int(stats[label, cv2.CC_STAT_TOP])
        comp_width = int(stats[label, cv2.CC_STAT_WIDTH])
        comp_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        if comp_width <= 0 or comp_height <= 0:
            continue
        ys, xs = np.where(labels == label)
        if ys.size == 0:
            continue
        bottom_y = int(ys.max())
        bottom_hits = int(np.count_nonzero(ys >= bottom_band_start))
        center_x = float(centroids[label][0])
        distance = abs(center_x - target_center)
        fill_ratio = area / float(max(1, comp_width * comp_height))
        aspect_ratio = comp_height / float(max(1, comp_width))
        side_penalty = 0.0
        if prefer_side != 0:
            side = 1 if center_x > width / 2.0 else -1
            if side != prefer_side:
                side_penalty = 80.0 if not loose else 40.0
        score = (
            area * 0.020
            + min(bottom_hits, 120) * 1.1
            + min(comp_height, height) * 0.8
            + min(aspect_ratio, 6.0) * 10.0
            + min(fill_ratio, 1.0) * 12.0
            - distance * 1.6
            - max(0, bottom_band_start - bottom_y) * 2.4
            - side_penalty
        )
        if best_score is None or score > best_score:
            best_score = score
            best_label = label

    if best_label == 0:
        return mask.copy()

    out = np.zeros_like(mask)
    out[labels == best_label] = 255
    return out


def analyze_tracking_mask_curve_fallback(
    mask: np.ndarray,
    camera_center_offset: float = 0.0,
    k_offset: float = 0.90,
    k_heading: float = 0.50,
    k_curve: float = 0.65,
) -> dict[str, float | str] | None:
    centerline = summarize_centerline(mask)
    if centerline is None:
        return None

    center_offset = float(np.clip(camera_center_offset, -0.5, 0.5))
    bottom_offset_norm = float(np.clip(centerline["bottom_offset_norm"] - center_offset, -1.0, 1.0))
    near_offset_norm = float(np.clip(centerline["near_offset_norm"] - center_offset, -1.0, 1.0))
    mid_offset_norm = float(np.clip(centerline["mid_offset_norm"] - center_offset, -1.0, 1.0))
    far_offset_norm = float(np.clip(centerline["far_offset_norm"] - center_offset, -1.0, 1.0))
    future_offset_norm = float(np.clip(centerline["future_offset_norm"] - center_offset, -1.0, 1.0))
    offset_norm = float(np.clip(0.80 * bottom_offset_norm + 0.20 * near_offset_norm, -1.0, 1.0))
    heading_norm = float(np.clip(centerline["heading_norm"], -1.0, 1.0))
    curvature_norm = float(np.clip(centerline["curvature_norm"], -1.0, 1.0))
    path_span = float(centerline["path_span"])
    recenter_priority = float(np.clip((abs(bottom_offset_norm) - 0.10) / 0.22, 0.0, 1.0))

    offset_term = float(np.clip(0.60 * bottom_offset_norm + 0.40 * offset_norm, -1.0, 1.0))
    if offset_term * heading_norm < 0.0 and abs(heading_norm) >= 0.04:
        offset_term = float(np.clip(0.50 * offset_term + 0.50 * heading_norm, -1.0, 1.0))
    heading_term = float(heading_norm * (1.0 - 0.88 * recenter_priority))
    curve_term = float(curvature_norm * (1.0 - 0.96 * recenter_priority))
    steer = float(
        np.clip(
            float(k_offset) * offset_term * (1.0 + 0.65 * recenter_priority)
            + float(k_heading) * heading_term
            + float(k_curve) * curve_term,
            -1.0,
            1.0,
        )
    )
    steer = float(np.clip(steer, -0.55, 0.55))

    points = collect_centerline_points(mask)
    width = mask.shape[1]
    if points:
        x_near = float(points[0][0])
    else:
        x_near = float((bottom_offset_norm + center_offset) * (width / 2.0) + (width / 2.0))
    x_near = float(np.clip(x_near, 0, width - 1))
    next_side = 0
    if x_near > width * 0.55:
        next_side = 1
    elif x_near < width * 0.45:
        next_side = -1

    metric = abs(bottom_offset_norm) + 0.55 * abs(heading_term) + 0.35 * abs(curve_term)
    return {
        "status": "analysis ready (mask-curve)",
        "bottom_offset_norm": float(bottom_offset_norm),
        "offset_norm": float(offset_norm),
        "mid_offset_norm": float(mid_offset_norm),
        "far_offset_norm": float(far_offset_norm),
        "future_offset_norm": float(future_offset_norm),
        "heading_norm": float(heading_norm),
        "curvature_norm": float(curvature_norm),
        "recenter_priority": float(recenter_priority),
        "path_span": float(path_span),
        "steer": float(steer),
        "metric": float(metric),
        "x_near": float(x_near),
        "x_far": float(x_near),
        "x_shift": None,
        "anchor_x": float(x_near),
        "last_side": int(next_side),
        "used_side": 0,
        "used_loose": True,
        "pose_mode": "mask-curve",
    }


def analyze_tracking_frame(
    frame: np.ndarray,
    config: dict[str, int | float | str],
    anchor_x: float | None = None,
    last_side: int = 0,
    prev_x_near: float | None = None,
    camera_center_offset: float = 0.0,
    k_offset: float = 0.90,
    k_heading: float = 0.50,
    k_curve: float = 0.65,
) -> dict[str, float | int | bool | str | None]:
    rgb_image = Image.fromarray(frame)
    center_offset = float(np.clip(camera_center_offset, -0.5, 0.5))
    frame_width = frame.shape[1]
    anchor_candidates: list[float | None] = []
    if anchor_x is not None:
        anchor_candidates.append(float(np.clip(anchor_x, 0.0, frame_width - 1.0)))
    anchor_candidates.append(None)
    candidate_prefs = []
    if last_side != 0:
        candidate_prefs.append((last_side, False))
        candidate_prefs.append((last_side, True))
        candidate_prefs.append((0, True))
        candidate_prefs.append((-last_side, True))
    else:
        candidate_prefs.append((0, False))
        candidate_prefs.append((0, True))

    x_near = x_far = None
    pose_mode = "none"
    used_side = 0
    used_loose = False
    used_anchor_x: float | None = None
    selected_mask: np.ndarray | None = None
    for candidate_anchor in anchor_candidates:
        for prefer_side, loose in candidate_prefs:
            candidate_mask = np.array(
                build_tracking_mask(
                    rgb_image,
                    config,
                    anchor_x=candidate_anchor,
                    prefer_side=prefer_side,
                    loose=loose,
                )
            )
            cand_near, cand_far, _row_near, _row_far, cand_mode = estimate_strip_pose(candidate_mask, 0.82, 0.42)
            if cand_near is not None and cand_far is not None:
                x_near = cand_near
                x_far = cand_far
                selected_mask = candidate_mask
                pose_mode = cand_mode
                used_side = prefer_side
                used_loose = loose
                used_anchor_x = candidate_anchor
                break
        if x_near is not None and x_far is not None:
            break

    if x_near is None or x_far is None:
        raise RuntimeError("Could not find target strip in mask.")

    width = frame_width
    next_side = int(last_side)
    if x_near > width * 0.55:
        next_side = 1
    elif x_near < width * 0.45:
        next_side = -1

    image_center_x = (width / 2.0) + center_offset * (width / 2.0)
    offset_norm = (x_near - image_center_x) / (width / 2.0)
    heading_norm = (x_far - x_near) / (width / 2.0)
    bottom_offset_norm = float(offset_norm)
    mid_offset_norm = float(np.clip(offset_norm + 0.5 * heading_norm, -1.0, 1.0))
    far_offset_norm = float(np.clip(offset_norm + heading_norm, -1.0, 1.0))
    future_offset_norm = float(np.clip(0.65 * mid_offset_norm + 0.35 * far_offset_norm, -1.0, 1.0))
    curvature_norm = 0.0
    path_span = 0.0
    if selected_mask is not None:
        centerline = summarize_centerline(selected_mask)
        if centerline is not None:
            bottom_offset_norm = float(np.clip(centerline["bottom_offset_norm"] - center_offset, -1.0, 1.0))
            near_offset_norm = float(np.clip(centerline["near_offset_norm"] - center_offset, -1.0, 1.0))
            mid_offset_norm = float(np.clip(centerline["mid_offset_norm"] - center_offset, -1.0, 1.0))
            far_offset_norm = float(np.clip(centerline["far_offset_norm"] - center_offset, -1.0, 1.0))
            future_offset_norm = float(np.clip(centerline["future_offset_norm"] - center_offset, -1.0, 1.0))
            offset_norm = float(np.clip(0.80 * bottom_offset_norm + 0.20 * near_offset_norm, -1.0, 1.0))
            heading_norm = float(np.clip(0.55 * centerline["heading_norm"] + 0.45 * heading_norm, -1.0, 1.0))
            curvature_norm = float(centerline["curvature_norm"])
            path_span = float(centerline["path_span"])
    recenter_priority = float(np.clip((abs(bottom_offset_norm) - 0.10) / 0.22, 0.0, 1.0))
    lookahead_priority = float(np.clip((path_span - 0.18) / 0.34, 0.0, 1.0))
    offset_term = float(np.clip(0.85 * bottom_offset_norm + 0.15 * offset_norm, -1.0, 1.0))
    future_term = float(np.clip(future_offset_norm * (0.45 + 0.55 * lookahead_priority), -1.0, 1.0))
    if abs(future_term) > abs(offset_term) + 0.04 and offset_term * future_term < 0.0:
        # When the near edge is slightly biased one way but the visible path clearly
        # bends the other way, favor the lookahead so we do not steer against the lane.
        offset_term = float(np.clip(0.55 * offset_term + 0.45 * future_term, -1.0, 1.0))
    heading_term = float((0.70 * heading_norm + 0.30 * future_term) * (1.0 - 0.88 * recenter_priority))
    curve_term = float(curvature_norm * (1.0 - 0.96 * recenter_priority))
    if abs(heading_term) > 0.08 and curve_term * heading_term < 0.0:
        curve_term *= 0.20
    if abs(future_term) > 0.10 and curve_term * future_term < 0.0:
        curve_term *= 0.35
    steer = float(
        np.clip(
            float(k_offset) * offset_term * (1.0 + 0.65 * recenter_priority)
            + float(k_heading) * heading_term
            + float(k_curve) * curve_term,
            -1.0,
            1.0,
        )
    )
    steer = float(np.clip(steer, -0.55, 0.55))
    x_shift = None if prev_x_near is None else abs(float(x_near) - float(prev_x_near))
    metric = abs(bottom_offset_norm) + 0.55 * abs(heading_term) + 0.35 * abs(curve_term)
    return {
        "status": "analysis ready" if pose_mode == "row" else f"analysis ready ({pose_mode})",
        "bottom_offset_norm": float(bottom_offset_norm),
        "offset_norm": float(offset_norm),
        "mid_offset_norm": float(mid_offset_norm),
        "far_offset_norm": float(far_offset_norm),
        "future_offset_norm": float(future_offset_norm),
        "heading_norm": float(heading_norm),
        "curvature_norm": float(curvature_norm),
        "recenter_priority": float(recenter_priority),
        "path_span": float(path_span),
        "steer": float(steer),
        "metric": float(metric),
        "x_near": float(x_near),
        "x_far": float(x_far),
        "x_shift": None if x_shift is None else float(x_shift),
        "anchor_x": float(x_near),
        "last_side": int(next_side),
        "used_side": int(used_side),
        "used_loose": bool(used_loose),
        "used_anchor_free": bool(used_anchor_x is None),
        "pose_mode": pose_mode,
    }


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

    def send(self, left: float, right: float) -> dict[str, float | str]:
        payload = {"T": 1, "L": float(left), "R": float(right)}
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        with self._lock:
            self._ser.write(line.encode("utf-8"))
            self._ser.flush()
        return payload

    def pulse(self, left: float, right: float, duration_s: float) -> dict[str, object]:
        duration_s = max(0.0, float(duration_s))
        refresh_s = 0.10
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


def compute_left_right(steer: float, throttle: float, mix: float) -> tuple[float, float]:
    left = max(-1.0, min(1.0, throttle + mix * steer))
    right = max(-1.0, min(1.0, throttle - mix * steer))
    return float(left), float(right)

class CameraFeed:
    def __init__(
        self,
        source: str,
        sensor_id: int,
        device_index: int,
        width: int,
        height: int,
        warmup_frames: int,
        mask_config: MaskConfig,
        auto_settings: AutoSettings,
    ) -> None:
        self._cap = open_camera(
            source=source,
            sensor_id=sensor_id,
            device_index=device_index,
            width=width,
            height=height,
            warmup_frames=warmup_frames,
        )
        self._mask_config = mask_config
        self._auto_settings = auto_settings
        self._lock = threading.Lock()
        self._latest_rgb: Image.Image | None = None
        self._latest_mask_input: Image.Image | None = None
        self._latest_mask: Image.Image | None = None
        self._latest_heatmap: Image.Image | None = None
        self._latest_trajectory: Image.Image | None = None
        self._jpeg_cache: dict[str, bytes] = {}
        self._anchor_x: float | None = None
        self._last_side = 0
        self._auto_mode = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @staticmethod
    def _encode_jpeg(image: Image.Image) -> bytes:
        with BytesIO() as buf:
            image.save(buf, format="JPEG", quality=82)
            return buf.getvalue()

    def set_auto_mode(self, enabled: bool) -> None:
        with self._lock:
            self._auto_mode = bool(enabled)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = read_rgb_frame(self._cap)
                rgb = Image.fromarray(frame)
                config = self._mask_config.get()
                settings = self._auto_settings.get()
                with self._lock:
                    auto_mode = self._auto_mode
                working_rgb = prepare_mask_working_image(rgb, config)
                mask_input = render_mask_input_preview(working_rgb, config)
                heatmap = render_color_index_heatmap(working_rgb, config)
                region_mask = build_detector_region_mask(rgb, config, anchor_x=self._anchor_x, prefer_side=self._last_side)
                region_mask_arr = np.array(region_mask)
                mask = build_tracking_mask(rgb, config, anchor_x=self._anchor_x, prefer_side=self._last_side)
                mask_arr = np.array(mask)
                mask_preview = render_live_mask_preview(region_mask_arr, mask_arr)
                trajectory_mask = mask_arr
                if len(collect_centerline_points(trajectory_mask)) < 4 and int(np.count_nonzero(region_mask_arr)) > int(np.count_nonzero(mask_arr)):
                    trajectory_mask = region_mask_arr
                trajectory = render_trajectory_prediction(
                    trajectory_mask,
                    camera_center_offset=float(settings.get("camera_center_offset", AUTO_SETTINGS_DEFAULTS["camera_center_offset"])),
                )
                x_near, _, _, _, _ = estimate_strip_pose(mask_arr, 0.82, 0.42)
                if x_near is not None:
                    self._anchor_x = x_near
                    width = mask_arr.shape[1]
                    if x_near > width * 0.55:
                        self._last_side = 1
                    elif x_near < width * 0.45:
                        self._last_side = -1

                jpeg_cache_update = {
                    "rgb": self._encode_jpeg(rgb),
                    "mask": self._encode_jpeg(mask_preview),
                    "trajectory": self._encode_jpeg(trajectory),
                }
                if mask_input is not None:
                    jpeg_cache_update["mask_input"] = self._encode_jpeg(mask_input)
                if heatmap is not None:
                    jpeg_cache_update["heatmap"] = self._encode_jpeg(heatmap)

                with self._lock:
                    self._latest_rgb = rgb
                    if mask_input is not None:
                        self._latest_mask_input = mask_input
                    self._latest_mask = mask
                    if heatmap is not None:
                        self._latest_heatmap = heatmap
                    self._latest_trajectory = trajectory
                    self._jpeg_cache.update(jpeg_cache_update)
            except Exception:
                time.sleep(0.03)
                continue
            time.sleep(0.02 if auto_mode else 0.04)

    def jpeg_bytes(self, kind: str) -> bytes:
        with self._lock:
            cached = self._jpeg_cache.get(kind)
            if cached is not None:
                return cached
            if kind == "rgb":
                image = self._latest_rgb
            elif kind == "mask_input":
                image = self._latest_mask_input
            elif kind == "mask":
                image = self._latest_mask
            elif kind == "heatmap":
                image = self._latest_heatmap
            else:
                image = self._latest_trajectory
        if image is None:
            image = Image.new("RGB", (640, 360), color="black")
        return self._encode_jpeg(image)

    def latest_rgb_array(self) -> np.ndarray | None:
        with self._lock:
            if self._latest_rgb is None:
                return None
            return np.array(self._latest_rgb)

    def latest_mask_array(self) -> np.ndarray | None:
        with self._lock:
            if self._latest_mask is None:
                return None
            return np.array(self._latest_mask)

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._cap.release()


class AppState:
    def __init__(
        self,
        rover: RoverSerial | None,
        rover_error: str | None,
        camera: CameraFeed | None,
        camera_error: str | None,
        port: str,
        baudrate: int,
        camera_source: str,
        width: int,
        height: int,
        auto_settings: AutoSettings,
    ) -> None:
        self.rover = rover
        self.rover_error = rover_error
        self.camera = camera
        self.camera_error = camera_error
        self.port = port
        self.baudrate = baudrate
        self.camera_source = camera_source
        self.width = width
        self.height = height
        self.mask_config = MaskConfig()
        self.calibration = MotionCalibration()
        self.auto_settings = auto_settings
        self.auto_controller: AutoController | None = None

    @staticmethod
    def _apply_turn_traction(
        left: float,
        right: float,
        steer: float,
        turn_severity: float,
        forward_min: float,
        base_forward_drive: float,
        effective_max_drive: float,
    ) -> tuple[float, float]:
        if abs(steer) <= 1e-6:
            return float(left), float(right)

        severity = float(np.clip(turn_severity, 0.0, 1.0))
        turning_right = steer > 0.0

        if left >= 0.0 and right >= 0.0:
            outer = float(left if turning_right else right)
            inner = float(right if turning_right else left)

            outer_floor = min(
                effective_max_drive,
                max(
                    float(base_forward_drive) * (1.05 + 0.22 * severity),
                    float(forward_min) + 0.020 + TURN_OUTER_EXTRA * severity,
                ),
            )
            outer = max(outer, outer_floor)

            target_inner_ratio = float(np.clip(TURN_INNER_RATIO_BASE - 0.36 * severity, TURN_INNER_RATIO_MIN, 0.82))
            inner_cap = max(0.0, outer * target_inner_ratio)
            min_diff = float(np.clip(TURN_DIFF_BASE + 0.16 * severity, TURN_DIFF_BASE, TURN_DIFF_MAX))
            inner = min(inner, inner_cap, max(0.0, outer - min_diff))

            if severity > TURN_BRAKE_SEVERITY:
                inner = min(inner, max(0.0, float(forward_min) * 0.15))
            if severity > TURN_PIVOT_SEVERITY:
                inner = 0.0

            if turning_right:
                left, right = float(outer), float(inner)
            else:
                left, right = float(inner), float(outer)

        return AppState._blend_turn_anchor_pair(left, right, steer, severity)

    @staticmethod
    def _blend_turn_anchor_pair(
        left: float,
        right: float,
        steer: float,
        turn_severity: float,
    ) -> tuple[float, float]:
        if abs(steer) <= 1e-6:
            return float(left), float(right)

        severity = float(np.clip(turn_severity, 0.0, 1.0))
        turning_right = steer > 0.0
        arc_target = ARC_RIGHT_ANCHOR if turning_right else ARC_LEFT_ANCHOR
        pivot_target = PIVOT_RIGHT_ANCHOR if turning_right else PIVOT_LEFT_ANCHOR

        arc_blend = float(np.clip((severity - TURN_ANCHOR_BLEND_START) / max(1e-6, TURN_ANCHOR_BLEND_FULL - TURN_ANCHOR_BLEND_START), 0.0, 1.0))
        pivot_blend = float(np.clip((severity - TURN_PIVOT_BLEND_START) / max(1e-6, TURN_PIVOT_BLEND_FULL - TURN_PIVOT_BLEND_START), 0.0, 1.0))
        if arc_blend <= 1e-6 and pivot_blend <= 1e-6:
            return float(left), float(right)

        target_left = float(arc_target[0])
        target_right = float(arc_target[1])
        if pivot_blend > 1e-6:
            target_left = float((1.0 - pivot_blend) * target_left + pivot_blend * float(pivot_target[0]))
            target_right = float((1.0 - pivot_blend) * target_right + pivot_blend * float(pivot_target[1]))

        hold_forward = 0.0
        if severity < 0.45:
            hold_forward = float(np.clip((0.45 - severity) / 0.45, 0.0, 1.0))
        if turning_right:
            target_left = max(target_left, 0.12 * hold_forward)
        else:
            target_right = max(target_right, 0.12 * hold_forward)

        blended_left = float((1.0 - arc_blend) * float(left) + arc_blend * target_left)
        blended_right = float((1.0 - arc_blend) * float(right) + arc_blend * target_right)
        return float(np.clip(blended_left, -0.50, 0.50)), float(np.clip(blended_right, -0.50, 0.50))

    @staticmethod
    def _enforce_forward_turn_progress(
        left: float,
        right: float,
        steer: float,
        turn_severity: float,
        forward_min: float,
        base_forward_drive: float,
        effective_max_drive: float,
    ) -> tuple[float, float]:
        if abs(steer) <= 1e-6:
            return float(left), float(right)

        severity = float(np.clip(turn_severity, 0.0, 1.0))
        turning_right = steer > 0.0
        arc_anchor = ARC_RIGHT_ANCHOR if turning_right else ARC_LEFT_ANCHOR
        pivot_anchor = PIVOT_RIGHT_ANCHOR if turning_right else PIVOT_LEFT_ANCHOR

        if turning_right:
            outer = float(left)
            inner = float(right)
        else:
            outer = float(right)
            inner = float(left)

        strong_outer_floor = min(
            0.50,
            max(
                float(base_forward_drive) * (1.02 + 0.20 * severity),
                float(forward_min) + 0.015,
                float(arc_anchor[0 if turning_right else 1]) * (0.96 + 0.06 * severity),
            ),
        )
        outer = max(outer, strong_outer_floor)

        if severity < 0.34:
            inner = max(inner, max(0.025, float(forward_min) * 0.35))
            inner = min(inner, max(0.04, outer * 0.62))
        elif severity < 0.58:
            inner = float(np.clip(inner, 0.0, max(0.02, outer * 0.40)))
        elif severity < 0.82:
            inner_cap = max(0.02, outer * 0.16)
            inner_floor = float(min(0.0, arc_anchor[1 if turning_right else 0]))
            inner = float(np.clip(inner, inner_floor, inner_cap))
        else:
            pivot_floor = float(min(0.0, pivot_anchor[1 if turning_right else 0]))
            inner = float(np.clip(inner, pivot_floor, 0.04))
            outer = max(outer, float(arc_anchor[0 if turning_right else 1]) * 0.98)

        if severity < 0.70:
            avg_forward = (outer + max(inner, 0.0)) * 0.5
            target_avg = max(0.035, float(base_forward_drive) * (0.92 - 0.28 * severity))
            if avg_forward < target_avg:
                outer = min(0.50, max(outer, 2.0 * target_avg - max(inner, 0.0)))

        if turning_right:
            left, right = outer, inner
        else:
            left, right = inner, outer

        return float(np.clip(left, -0.50, 0.50)), float(np.clip(right, -0.50, 0.50))

    def status_payload(self) -> dict[str, object]:
        warnings = []
        if self.rover_error:
            warnings.append(f"Serial is offline: {self.rover_error}")
        if self.camera_error:
            warnings.append(f"Camera is offline: {self.camera_error}")
        if self.auto_controller is not None and self.auto_controller.last_error:
            warnings.append(f"Auto follow warning: {self.auto_controller.last_error}")
        return {
            "serial": {
                "ok": self.rover is not None,
                "port": self.port,
                "baudrate": self.baudrate,
                "error": self.rover_error,
            },
            "camera": {
                "ok": self.camera is not None,
                "source": self.camera_source,
                "width": self.width,
                "height": self.height,
                "error": self.camera_error,
            },
            "warnings": warnings,
            "mask": self.mask_config.get(),
            "calibration": self.calibration.get(),
            "auto": self.auto_controller.status_payload() if self.auto_controller is not None else {"running": False},
        }

    def close(self) -> None:
        if self.rover is not None:
            self.rover.close()
        if self.camera is not None:
            self.camera.close()
        if self.auto_controller is not None:
            self.auto_controller.stop()


class AutoController:
    """Simplified auto follower that keeps the existing mask / panel stack.

    Workflow:
    - GO_STRAIGHT while the line stays within a wide tolerance.
    - If the offset / heading exceeds the tolerance, do a fixed in-place rotate
      episode using the empirically tested raw pair (+0.40 / -0.10).
    - After every rotate episode, do a short forward nudge to refresh the view.
    - If the mask flickers or the line is briefly not detected, do NOT search.
      Instead, finish the current rotate / nudge episode if one is already
      underway, otherwise hold still and wait for the mask to recover.

    The web panel, mask generation, previews, and APIs stay unchanged. Only the
    drive / decision logic is simplified here.
    """

    ROTATE_OUTER = 0.40
    ROTATE_INNER = -0.10
    ROTATE_SECONDS = 0.50
    NUDGE_SECONDS = 0.18
    SEARCH_ROTATE_SECONDS = 0.35
    SEARCH_NUDGE_SECONDS = 0.14

    def __init__(
        self,
        rover: RoverSerial | None,
        camera: CameraFeed | None,
        mask_config: MaskConfig,
        calibration: MotionCalibration,
        auto_settings: AutoSettings,
    ) -> None:
        self._rover = rover
        self._camera = camera
        self._mask_config = mask_config
        self._calibration = calibration
        self._auto_settings = auto_settings
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._actuator_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._settings = {
            "drive": 0.180,
            "min_drive": 0.120,
            "max_drive": 0.240,
            "k_offset": 0.90,
            "k_heading": 0.70,
            "k_curve": 0.75,
            "turn_mix": 0.45,
            "loop_hz": 10.0,
            "center_tolerance": 0.10,
            "min_steer": 0.10,
            "min_wheel": 0.05,
            "forward_floor": 0.160,
            "curve_slowdown": 0.24,
            "motor_bias": 0.00,
            "step_up": 0.02,
            "stuck_shift_px": 10.0,
            "search_turn": 0.18,
            "search_step_up": 0.015,
            "search_max_turn": 0.30,
            "search_drive": 0.00,
        }
        self._settings.update(self._auto_settings.get())
        self.last_analysis: dict[str, float | str] | None = None
        self.last_status = "idle"
        self.last_error: str | None = None
        self._anchor_x: float | None = None
        self._last_side = 0
        self._last_steer_sign = 0
        self._stuck_steps = 0
        self._search_steps = 0
        self._target_left = 0.0
        self._target_right = 0.0
        self._current_left = 0.0
        self._current_right = 0.0
        self._mode = "idle"
        self._mode_deadline = 0.0
        self._mode_turn_dir = 1
        self._last_left_cmd = 0.0
        self._last_right_cmd = 0.0
        self._last_send = (None, None)

    def apply_saved_settings(self, values: dict[str, Any]) -> dict[str, float]:
        with self._lock:
            for key in AUTO_SETTINGS_DEFAULTS:
                if key not in values:
                    continue
                try:
                    self._settings[key] = float(values[key])
                except Exception:
                    continue
            self._settings["drive"] = max(self._settings["drive"], 0.16)
            self._settings["max_drive"] = max(self._settings["max_drive"], self._settings["drive"], 0.22)
            self._settings["forward_floor"] = max(self._settings["forward_floor"], 0.14)
            return {key: float(self._settings[key]) for key in AUTO_SETTINGS_DEFAULTS}

    @staticmethod
    def _decision_label(left: float, right: float, searching: bool = False) -> str:
        avg = (left + right) / 2.0
        diff = left - right
        turn_eps = 0.02
        move_eps = 0.02
        if searching:
            if diff > turn_eps:
                return "search right"
            if diff < -turn_eps:
                return "search left"
            return "search hold"
        if avg > move_eps:
            if diff > turn_eps:
                return "forward-right"
            if diff < -turn_eps:
                return "forward-left"
            return "forward"
        if avg < -move_eps:
            if diff > turn_eps:
                return "reverse-right"
            if diff < -turn_eps:
                return "reverse-left"
            return "reverse"
        if diff > turn_eps:
            return "turn right"
        if diff < -turn_eps:
            return "turn left"
        return "hold"

    def status_payload(self) -> dict[str, object]:
        with self._lock:
            running = self._thread is not None and self._thread.is_alive()
            return {
                "running": running,
                "settings": dict(self._settings),
                "last_status": self.last_status,
                "last_error": self.last_error,
                "last_analysis": self.last_analysis,
                "stuck_steps": self._stuck_steps,
                "search_steps": self._search_steps,
                "target_left": self._target_left,
                "target_right": self._target_right,
                "current_left": self._current_left,
                "current_right": self._current_right,
                "control_mode": self._mode,
                "mode_deadline": self._mode_deadline,
            }

    def start(self, settings: dict[str, float]) -> dict[str, object]:
        if self._rover is None:
            raise RuntimeError("Serial is unavailable.")
        if self._camera is None:
            raise RuntimeError("Camera is unavailable.")
        with self._lock:
            self._settings.update(settings)
            self._settings["drive"] = max(float(self._settings.get("drive", 0.0)), 0.16)
            self._settings["max_drive"] = max(float(self._settings.get("max_drive", 0.0)), float(self._settings["drive"]), 0.22)
            self._settings["forward_floor"] = max(float(self._settings.get("forward_floor", 0.0)), 0.14)
            self._stuck_steps = 0
            self._search_steps = 0
            self._last_side = 0
            self._last_steer_sign = 0
            self._anchor_x = None
            self._target_left = 0.0
            self._target_right = 0.0
            self._current_left = 0.0
            self._current_right = 0.0
            self._mode = "go_straight"
            self._mode_deadline = 0.0
            self._mode_turn_dir = 1
            self._last_left_cmd = 0.0
            self._last_right_cmd = 0.0
            self._last_send = (None, None)
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("Auto follow is already running.")
            self._stop.clear()
            if self._camera is not None:
                self._camera.set_auto_mode(True)
            self.last_error = None
            self.last_status = "starting"
            self._actuator_thread = threading.Thread(target=self._actuate_loop, daemon=True)
            self._actuator_thread.start()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        return self.status_payload()

    def stop(self) -> dict[str, object]:
        self._stop.set()
        for thread in (self._thread, self._actuator_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)
        self._thread = None
        self._actuator_thread = None
        if self._camera is not None:
            self._camera.set_auto_mode(False)
        if self._rover is not None:
            try:
                self._rover.send(0.0, 0.0)
            except Exception:
                pass
        with self._lock:
            self.last_status = "stopped"
            self._target_left = 0.0
            self._target_right = 0.0
            self._current_left = 0.0
            self._current_right = 0.0
            self._mode = "idle"
            self._mode_deadline = 0.0
            self._last_send = (None, None)
        return self.status_payload()

    def _set_target(self, left: float, right: float, mode: str) -> None:
        with self._lock:
            self._target_left = float(np.clip(left, -0.50, 0.50))
            self._target_right = float(np.clip(right, -0.50, 0.50))
            self._mode = mode

    @staticmethod
    def _move_towards(current: float, target: float) -> float:
        if abs(target) > abs(current):
            step = AUTO_RAMP_UP_PER_TICK
        else:
            step = AUTO_RAMP_DOWN_PER_TICK
        if target > current:
            return min(target, current + step)
        return max(target, current - step)

    def _actuate_loop(self) -> None:
        period_s = 1.0 / max(8.0, AUTO_CONTROL_HZ)
        while not self._stop.is_set():
            if self._rover is None:
                time.sleep(period_s)
                continue
            with self._lock:
                target_left = float(self._target_left)
                target_right = float(self._target_right)
                current_left = float(self._current_left)
                current_right = float(self._current_right)
            next_left = self._move_towards(current_left, target_left)
            next_right = self._move_towards(current_right, target_right)
            should_send = (
                self._last_send[0] is None
                or abs(next_left - float(self._last_send[0])) > AUTO_SEND_EPS
                or abs(next_right - float(self._last_send[1])) > AUTO_SEND_EPS
            )
            if should_send:
                try:
                    self._rover.send(next_left, next_right)
                    self._last_send = (next_left, next_right)
                    self._last_left_cmd = next_left
                    self._last_right_cmd = next_right
                except Exception:
                    pass
            with self._lock:
                self._current_left = next_left
                self._current_right = next_right
            time.sleep(period_s)
        if self._rover is not None:
            try:
                self._rover.send(0.0, 0.0)
            except Exception:
                pass
        with self._lock:
            self._current_left = 0.0
            self._current_right = 0.0
            self._last_send = (None, None)

    def _apply_bias(self, left: float, right: float) -> tuple[float, float]:
        motor_bias = float(self._settings.get("motor_bias", 0.0))
        left = float(np.clip(left + motor_bias, -0.50, 0.50))
        right = float(np.clip(right - motor_bias, -0.50, 0.50))
        return left, right

    def _forward_pair(self, drive: float) -> tuple[float, float]:
        calibration = self._calibration.get()
        raw_left = float(drive)
        raw_right = float(drive)
        left, right = apply_drive_calibration(raw_left, raw_right, calibration)
        left, right = self._apply_bias(left, right)
        return float(np.clip(left, -0.50, 0.50)), float(np.clip(right, -0.50, 0.50))

    def _fixed_rotate_pair(self, direction: int) -> tuple[float, float]:
        if direction >= 0:
            left, right = self.ROTATE_OUTER, self.ROTATE_INNER
        else:
            left, right = self.ROTATE_INNER, self.ROTATE_OUTER
        left, right = self._apply_bias(left, right)
        return float(np.clip(left, -0.50, 0.50)), float(np.clip(right, -0.50, 0.50))

    @staticmethod
    def _turn_dir_from_analysis(analysis: dict[str, float | str]) -> int:
        future = float(analysis.get("future_offset_norm", 0.0))
        heading = float(analysis.get("heading_norm", 0.0))
        bottom = float(analysis.get("bottom_offset_norm", analysis.get("offset_norm", 0.0)))
        source = future if abs(future) >= 0.05 else heading if abs(heading) >= 0.04 else bottom
        return 1 if source >= 0.0 else -1

    def _start_mode(self, mode: str, duration_s: float, turn_dir: int | None = None) -> None:
        now = time.monotonic()
        with self._lock:
            self._mode = mode
            self._mode_deadline = now + max(0.01, float(duration_s))
            if turn_dir is not None:
                self._mode_turn_dir = 1 if turn_dir >= 0 else -1
                self._last_steer_sign = self._mode_turn_dir

    def _current_mode(self) -> tuple[str, float, int]:
        with self._lock:
            return self._mode, self._mode_deadline, self._mode_turn_dir

    def _build_go_straight(self, analysis: dict[str, float | str]) -> dict[str, float | str]:
        drive_setting = max(float(self._settings.get("drive", 0.18)), 0.16)
        max_drive = max(float(self._settings.get("max_drive", 0.24)), drive_setting)
        forward_floor = max(float(self._settings.get("forward_floor", 0.16)), 0.14)
        drive = min(max_drive, max(drive_setting, forward_floor, 0.18))
        left, right = self._forward_pair(drive)
        self._set_target(left, right, mode="go_straight")
        center_error = float(np.clip(analysis.get("bottom_offset_norm", analysis.get("offset_norm", 0.0)), -1.0, 1.0))
        out = dict(analysis)
        out.update({
            "status": "following line (go_straight)",
            "decision": self._decision_label(left, right),
            "decision_reason": f"go straight: within tolerance, center={center_error:+.3f}",
            "steer": 0.0,
            "drive": float(drive),
            "left": float(left),
            "right": float(right),
            "mix_used": 0.0,
            "stuck_steps": int(self._stuck_steps),
            "search_steps": int(self._search_steps),
            "search_turn": 0.0,
            "control_mode": "go_straight",
            "control_phase": "steady forward",
        })
        return out

    def _build_rotate(self, analysis: dict[str, float | str], turn_dir: int, mode_name: str) -> dict[str, float | str]:
        left, right = self._fixed_rotate_pair(turn_dir)
        self._set_target(left, right, mode=mode_name)
        center_error = float(np.clip(analysis.get("bottom_offset_norm", analysis.get("offset_norm", 0.0)), -1.0, 1.0))
        heading = float(analysis.get("heading_norm", 0.0))
        out = dict(analysis)
        out.update({
            "status": f"following line ({mode_name})",
            "decision": self._decision_label(left, right),
            "decision_reason": f"rotate only: center={center_error:+.3f} heading={heading:+.3f}",
            "steer": float(turn_dir),
            "drive": 0.0,
            "left": float(left),
            "right": float(right),
            "mix_used": 0.0,
            "stuck_steps": int(self._stuck_steps),
            "search_steps": int(self._search_steps),
            "search_turn": 0.0,
            "control_mode": mode_name,
            "control_phase": "fixed rotate",
        })
        return out

    def _build_nudge(self, analysis: dict[str, float | str], mode_name: str) -> dict[str, float | str]:
        drive_setting = max(float(self._settings.get("drive", 0.18)), 0.16)
        forward_floor = max(float(self._settings.get("forward_floor", 0.16)), 0.14)
        max_drive = max(float(self._settings.get("max_drive", 0.24)), drive_setting)
        # Short refresh move after rotate, intentionally weaker than steady cruise.
        drive = min(max_drive, max(0.12, forward_floor * 0.85, drive_setting * 0.80))
        left, right = self._forward_pair(drive)
        self._set_target(left, right, mode=mode_name)
        out = dict(analysis)
        out.update({
            "status": f"following line ({mode_name})",
            "decision": self._decision_label(left, right),
            "decision_reason": f"short forward refresh after rotate, drive={drive:.2f}",
            "steer": 0.0,
            "drive": float(drive),
            "left": float(left),
            "right": float(right),
            "mix_used": 0.0,
            "stuck_steps": int(self._stuck_steps),
            "search_steps": int(self._search_steps),
            "search_turn": 0.0,
            "control_mode": mode_name,
            "control_phase": "post-rotate nudge",
        })
        return out

    def _build_search(self, mode_name: str, reason: str) -> dict[str, float | str]:
        turn_dir = self._last_steer_sign or self._last_side or 1
        if mode_name == "search_rotate":
            left, right = self._fixed_rotate_pair(turn_dir)
            drive = 0.0
            searching = True
            phase = "search rotate"
        else:
            drive = 0.10
            left, right = self._forward_pair(drive)
            searching = False
            phase = "search nudge"
        self._set_target(left, right, mode=mode_name)
        return {
            "status": "searching for line",
            "decision": self._decision_label(left, right, searching=searching),
            "decision_reason": reason,
            "bottom_offset_norm": 0.0,
            "offset_norm": 0.0,
            "heading_norm": 0.0,
            "curvature_norm": 0.0,
            "recenter_priority": 1.0,
            "steer": float(turn_dir if mode_name == "search_rotate" else 0.0),
            "drive": float(drive),
            "left": float(left),
            "right": float(right),
            "x_near": self._anchor_x if self._anchor_x is not None else 0.0,
            "x_far": self._anchor_x if self._anchor_x is not None else 0.0,
            "x_shift": 0.0,
            "future_offset_norm": 0.0,
            "path_span": 0.0,
            "mix_used": 0.0,
            "stuck_steps": int(self._stuck_steps),
            "search_steps": int(self._search_steps),
            "search_turn": float(self.ROTATE_OUTER),
            "control_mode": mode_name,
            "control_phase": phase,
            "anchor_x": self._anchor_x if self._anchor_x is not None else 0.0,
            "last_side": int(self._last_side),
        }

    def _select_target(self, analysis: dict[str, float | str]) -> dict[str, float | str]:
        now = time.monotonic()
        mode, deadline, turn_dir = self._current_mode()
        center_error = float(np.clip(analysis.get("bottom_offset_norm", analysis.get("offset_norm", 0.0)), -1.0, 1.0))
        heading = float(analysis.get("heading_norm", 0.0))
        future = float(analysis.get("future_offset_norm", 0.0))
        x_shift = analysis.get("x_shift")
        if x_shift is not None:
            if float(x_shift) < float(self._settings.get("stuck_shift_px", 10.0)):
                self._stuck_steps += 1
            else:
                self._stuck_steps = 0
        turn_dir = self._turn_dir_from_analysis(analysis)
        self._last_side = 1 if center_error >= 0.0 else -1
        self._last_steer_sign = turn_dir

        enter_thresh = max(float(self._settings.get("center_tolerance", 0.10)), 0.10)
        heading_enter = max(0.16, float(self._settings.get("min_steer", 0.10)) * 1.5)
        # Wide straight acceptance: approximately +/- 30 degrees equivalent.
        needs_rotate = abs(center_error) > enter_thresh or abs(heading) > heading_enter or abs(future) > 0.22

        if mode == "rotate_to_align":
            if now < deadline:
                return self._build_rotate(analysis, turn_dir, "rotate_to_align")
            self._start_mode("forward_nudge", self.NUDGE_SECONDS)
            return self._build_nudge(analysis, "forward_nudge")

        if mode == "forward_nudge":
            if now < deadline:
                return self._build_nudge(analysis, "forward_nudge")
            self._start_mode("go_straight", 0.0)
            return self._build_go_straight(analysis)

        # Fresh decision from go_straight / idle / recovered hold.
        if needs_rotate:
            self._start_mode("rotate_to_align", self.ROTATE_SECONDS, turn_dir=turn_dir)
            return self._build_rotate(analysis, turn_dir, "rotate_to_align")
        self._start_mode("go_straight", 0.0)
        return self._build_go_straight(analysis)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                analysis = self._analyze_step()
                with self._lock:
                    self._search_steps = 0
                    self.last_analysis = analysis
                    self.last_status = analysis["status"]
                    self.last_error = None
                hz = max(4.0, float(self._settings.get("loop_hz", 10.0)))
                time.sleep(max(AUTO_ANALYSIS_SLEEP_FLOOR_S, 1.0 / hz))
            except Exception as exc:
                self._search_steps += 1
                mode, deadline, turn_dir = self._current_mode()
                now = time.monotonic()
                if mode == "search_rotate":
                    if now >= deadline:
                        self._start_mode("search_nudge", self.SEARCH_NUDGE_SECONDS, turn_dir=turn_dir)
                        analysis = self._build_search("search_nudge", f"line lost: {exc}")
                    else:
                        analysis = self._build_search("search_rotate", f"line lost: {exc}")
                elif mode == "search_nudge":
                    if now >= deadline:
                        self._start_mode("search_rotate", self.SEARCH_ROTATE_SECONDS, turn_dir=turn_dir)
                        analysis = self._build_search("search_rotate", f"line lost: {exc}")
                    else:
                        analysis = self._build_search("search_nudge", f"line lost: {exc}")
                else:
                    self._start_mode("search_rotate", self.SEARCH_ROTATE_SECONDS, turn_dir=(self._last_steer_sign or self._last_side or 1))
                    analysis = self._build_search("search_rotate", f"line lost: {exc}")
                with self._lock:
                    self.last_error = str(exc)
                    self.last_status = analysis["status"]
                    self.last_analysis = analysis
                time.sleep(0.08)
        self._set_target(0.0, 0.0, mode="idle")

    def _analyze_step(self) -> dict[str, float | str]:
        if self._camera is None or self._rover is None:
            raise RuntimeError("Camera or serial unavailable.")
        frame = self._camera.latest_rgb_array()
        if frame is None:
            raise RuntimeError("No camera frame yet.")
        config = self._mask_config.get()
        prev_x_near = None
        if self.last_analysis is not None and "x_near" in self.last_analysis:
            prev_x_near = float(self.last_analysis["x_near"])
        analysis = analyze_tracking_frame(
            frame,
            config,
            anchor_x=self._anchor_x,
            last_side=self._last_side,
            prev_x_near=prev_x_near,
            camera_center_offset=float(self._settings.get("camera_center_offset", 0.0)),
            k_offset=float(self._settings.get("k_offset", 0.90)),
            k_heading=float(self._settings.get("k_heading", 0.70)),
            k_curve=float(self._settings.get("k_curve", 0.75)),
        )
        self._anchor_x = float(analysis["anchor_x"])
        self._last_side = int(analysis["last_side"])
        return self._select_target(analysis)


def create_app(
    port: str,
    baudrate: int,
    camera_source: str,
    sensor_id: int,
    device_index: int,
    width: int,
    height: int,
    warmup_frames: int,
) -> Flask:
    mask_config = MaskConfig()
    auto_settings = AutoSettings()
    resolved_port = resolve_serial_port(port, baudrate=baudrate)
    camera_selection = resolve_camera_selection(camera_source, sensor_id, device_index, width, height, warmup_frames)
    rover = None
    rover_error = None
    try:
        rover = RoverSerial(resolved_port, baudrate)
    except Exception as exc:
        rover_error = str(exc)

    camera = None
    camera_error = None
    try:
        camera = CameraFeed(
            camera_selection.source,
            camera_selection.sensor_id,
            camera_selection.device_index,
            width,
            height,
            warmup_frames,
            mask_config,
            auto_settings,
        )
    except Exception as exc:
        camera_error = str(exc)

    state = AppState(
        rover=rover,
        rover_error=rover_error,
        camera=camera,
        camera_error=camera_error,
        port=resolved_port,
        baudrate=baudrate,
        camera_source=camera_selection.source,
        width=width,
        height=height,
        auto_settings=auto_settings,
    )
    state.mask_config = mask_config
    state.auto_controller = AutoController(rover, camera, mask_config, state.calibration, state.auto_settings)
    app = Flask(__name__)
    atexit.register(state.close)

    @app.get("/")
    def index():
        return HTML

    @app.get("/api/status")
    def api_status():
        return jsonify(state.status_payload())

    @app.post("/api/analyze")
    def api_analyze():
        if state.camera is None:
            return jsonify({"status": "camera unavailable", "error": state.camera_error}), 503
        frame = state.camera.latest_rgb_array()
        if frame is None:
            return jsonify({"status": "no frame yet"}), 503
        data = request.get_json(silent=True) or {}
        saved_settings = state.auto_settings.get()
        try:
                analysis = analyze_tracking_frame(
                    frame,
                    state.mask_config.get(),
                    anchor_x=None if data.get("anchor_x") is None else float(data.get("anchor_x")),
                    last_side=int(data.get("last_side", 0)),
                    prev_x_near=None if data.get("prev_x_near") is None else float(data.get("prev_x_near")),
                    camera_center_offset=float(data.get("camera_center_offset", saved_settings.get("camera_center_offset", AUTO_SETTINGS_DEFAULTS["camera_center_offset"]))),
                    k_offset=float(data.get("k_offset", saved_settings.get("k_offset", AUTO_SETTINGS_DEFAULTS["k_offset"]))),
                    k_heading=float(data.get("k_heading", saved_settings.get("k_heading", AUTO_SETTINGS_DEFAULTS["k_heading"]))),
                    k_curve=float(data.get("k_curve", saved_settings.get("k_curve", AUTO_SETTINGS_DEFAULTS["k_curve"]))),
                )
        except Exception as exc:
            return jsonify({"status": f"error: {exc}"}), 400
        return jsonify(analysis)

    @app.post("/api/drive")
    def api_drive():
        if state.rover is None:
            return jsonify({"status": "serial unavailable", "error": state.rover_error}), 503
        data = request.get_json(force=True)
        steer = float(data.get("steer", 0.0))
        throttle = float(data.get("throttle", 0.0))
        mix = float(data.get("mix", 0.45))
        requested_left, requested_right = compute_left_right(steer, throttle, mix)
        calibration = state.calibration.get()
        left, right = apply_drive_calibration(requested_left, requested_right, calibration)
        payload = state.rover.send(left, right)
        return jsonify({
            "status": "driving",
            "steer": steer,
            "throttle": throttle,
            "requested_left": requested_left,
            "requested_right": requested_right,
            "left": left,
            "right": right,
            "calibration": calibration,
            "payload": payload,
        })

    @app.post("/api/pulse")
    def api_pulse():
        if state.rover is None:
            return jsonify({"status": "serial unavailable", "error": state.rover_error}), 503
        data = request.get_json(force=True)
        steer = float(data.get("steer", 0.0))
        throttle = float(data.get("throttle", 0.0))
        mix = float(data.get("mix", 0.45))
        duration_s = float(data.get("duration_s", 0.35))
        requested_left, requested_right = compute_left_right(steer, throttle, mix)
        calibration = state.calibration.get()
        left, right = apply_drive_calibration(requested_left, requested_right, calibration)
        payload = state.rover.pulse(left, right, duration_s)
        return jsonify({
            "status": "pulse sent",
            "steer": steer,
            "throttle": throttle,
            "requested_left": requested_left,
            "requested_right": requested_right,
            "left": left,
            "right": right,
            "calibration": calibration,
            "duration_s": duration_s,
            "payload": payload,
        })

    @app.post("/api/pulse_raw")
    def api_pulse_raw():
        if state.rover is None:
            return jsonify({"status": "serial unavailable", "error": state.rover_error}), 503
        data = request.get_json(force=True)
        requested_left = float(data.get("left", 0.0))
        requested_right = float(data.get("right", 0.0))
        duration_s = float(data.get("duration_s", 0.35))
        bypass_calibration = bool(data.get("bypass_calibration", False))
        calibration = state.calibration.get()
        if bypass_calibration:
            left, right = requested_left, requested_right
            status = "raw pulse sent (bypass calibration)"
        else:
            left, right = apply_drive_calibration(requested_left, requested_right, calibration)
            status = "raw pulse sent"
        payload = state.rover.pulse(left, right, duration_s)
        return jsonify({
            "status": status,
            "requested_left": requested_left,
            "requested_right": requested_right,
            "left": left,
            "right": right,
            "bypass_calibration": bypass_calibration,
            "calibration": calibration,
            "duration_s": duration_s,
            "payload": payload,
        })

    @app.post("/api/stop")
    def api_stop():
        if state.rover is None:
            return jsonify({"status": "serial unavailable", "error": state.rover_error}), 503
        if state.auto_controller is not None:
            state.auto_controller.stop()
        payload = state.rover.send(0.0, 0.0)
        return jsonify({"status": "stopped", "payload": payload})

    @app.post("/api/auto/start")
    def api_auto_start():
        if state.auto_controller is None:
            return jsonify({"status": "auto unavailable"}), 503
        data = request.get_json(force=True)
        saved = state.auto_settings.update(data)
        state.auto_controller.apply_saved_settings(saved)
        base_drive = float(saved.get("drive", 0.0))
        try:
            payload = state.auto_controller.start(
                {
                    "drive": base_drive,
                    "min_drive": max(0.0, base_drive - 0.01),
                    "max_drive": float(saved.get("max_drive", AUTO_SETTINGS_DEFAULTS["max_drive"])),
                    "camera_center_offset": float(saved.get("camera_center_offset", AUTO_SETTINGS_DEFAULTS["camera_center_offset"])),
                    "k_offset": float(saved.get("k_offset", AUTO_SETTINGS_DEFAULTS["k_offset"])),
                    "k_heading": float(saved.get("k_heading", AUTO_SETTINGS_DEFAULTS["k_heading"])),
                    "k_curve": float(saved.get("k_curve", AUTO_SETTINGS_DEFAULTS["k_curve"])),
                    "turn_mix": float(saved.get("turn_mix", AUTO_SETTINGS_DEFAULTS["turn_mix"])),
                    "loop_hz": float(saved.get("loop_hz", AUTO_SETTINGS_DEFAULTS["loop_hz"])),
                    "center_tolerance": float(saved.get("center_tolerance", AUTO_SETTINGS_DEFAULTS["center_tolerance"])),
                    "min_steer": float(saved.get("min_steer", AUTO_SETTINGS_DEFAULTS["min_steer"])),
                    "forward_floor": float(saved.get("forward_floor", AUTO_SETTINGS_DEFAULTS["forward_floor"])),
                    "curve_slowdown": float(saved.get("curve_slowdown", AUTO_SETTINGS_DEFAULTS["curve_slowdown"])),
                    "motor_bias": float(saved.get("motor_bias", AUTO_SETTINGS_DEFAULTS["motor_bias"])),
                    "step_up": float(saved.get("step_up", AUTO_SETTINGS_DEFAULTS["step_up"])),
                    "stuck_shift_px": float(saved.get("stuck_shift_px", AUTO_SETTINGS_DEFAULTS["stuck_shift_px"])),
                    "min_wheel": max(0.05, base_drive - 0.005),
                }
            )
        except Exception as exc:
            return jsonify({"status": f"error: {exc}"}), 400
        return jsonify({"status": "auto started", "auto": payload})

    @app.post("/api/auto/stop")
    def api_auto_stop():
        if state.auto_controller is None:
            return jsonify({"status": "auto unavailable"}), 503
        payload = state.auto_controller.stop()
        return jsonify({"status": "auto stopped", "auto": payload})

    @app.get("/api/auto/settings")
    def api_auto_settings_get():
        return jsonify(state.auto_settings.get())

    @app.post("/api/auto/settings")
    def api_auto_settings_set():
        updated = state.auto_settings.update(request.get_json(force=True))
        if state.auto_controller is not None:
            state.auto_controller.apply_saved_settings(updated)
        return jsonify(updated)

    @app.post("/api/auto/settings/reset")
    def api_auto_settings_reset():
        updated = state.auto_settings.reset()
        if state.auto_controller is not None:
            state.auto_controller.apply_saved_settings(updated)
        return jsonify(updated)

    @app.get("/api/mask/config")
    def api_mask_config():
        return jsonify(state.mask_config.get())

    @app.get("/api/calibration")
    def api_calibration_get():
        return jsonify(state.calibration.get())

    @app.post("/api/calibration")
    def api_calibration_set():
        data = request.get_json(force=True)
        try:
            values = state.calibration.set(str(data.get("direction", "")), float(data.get("value", 0.0)))
        except KeyError as exc:
            return jsonify({"status": f"unknown direction: {exc}"}), 400
        return jsonify(values)

    @app.post("/api/calibration/reset")
    def api_calibration_reset():
        values = state.calibration.clear()
        return jsonify(state.calibration.get())

    @app.post("/api/mask/config")
    def api_mask_config_update():
        data = request.get_json(force=True)
        updated = state.mask_config.update(
            {
                "detector_mode": str(data.get("detector_mode", state.mask_config.get()["detector_mode"])),
                "crop_top_ratio": float(data.get("crop_top_ratio", state.mask_config.get()["crop_top_ratio"])),
                "processing_scale": float(data.get("processing_scale", state.mask_config.get().get("processing_scale", 1.0))),
                "color_distance_threshold": float(data.get("color_distance_threshold", state.mask_config.get()["color_distance_threshold"])),
                "color_distance_margin": float(data.get("color_distance_margin", state.mask_config.get()["color_distance_margin"])),
                "h_min": int(data.get("h_min", state.mask_config.get()["h_min"])),
                "h_max": int(data.get("h_max", state.mask_config.get()["h_max"])),
                "s_min": int(data.get("s_min", state.mask_config.get()["s_min"])),
                "s_max": int(data.get("s_max", state.mask_config.get()["s_max"])),
                "v_min": int(data.get("v_min", state.mask_config.get()["v_min"])),
                "v_max": int(data.get("v_max", state.mask_config.get()["v_max"])),
                "blur_kernel": int(data.get("blur_kernel", state.mask_config.get()["blur_kernel"])),
                "morph_kernel": int(data.get("morph_kernel", state.mask_config.get()["morph_kernel"])),
                "sample_radius": int(data.get("sample_radius", state.mask_config.get()["sample_radius"])),
                "h_margin": int(data.get("h_margin", state.mask_config.get()["h_margin"])),
                "s_margin": int(data.get("s_margin", state.mask_config.get()["s_margin"])),
                "v_margin": int(data.get("v_margin", state.mask_config.get()["v_margin"])),
                "line_r": int(data.get("line_r", state.mask_config.get()["line_r"])),
                "line_g": int(data.get("line_g", state.mask_config.get()["line_g"])),
                "line_b": int(data.get("line_b", state.mask_config.get()["line_b"])),
                "floor_r": int(data.get("floor_r", state.mask_config.get()["floor_r"])),
                "floor_g": int(data.get("floor_g", state.mask_config.get()["floor_g"])),
                "floor_b": int(data.get("floor_b", state.mask_config.get()["floor_b"])),
            }
        )
        return jsonify(updated)

    @app.post("/api/mask/preset/<name>")
    def api_mask_preset(name: str):
        try:
            return jsonify(state.mask_config.apply_preset(name))
        except KeyError:
            return jsonify({"error": f"unknown preset: {name}"}), 404

    @app.post("/api/mask/sample")
    def api_mask_sample():
        if state.camera is None:
            return jsonify({"status": "camera unavailable", "error": state.camera_error}), 503
        frame = state.camera.latest_rgb_array()
        if frame is None:
            return jsonify({"status": "no frame yet"}), 503

        data = request.get_json(force=True)
        x_ratio = float(data.get("x", 0.5))
        y_ratio = float(data.get("y", 0.5))
        role = str(data.get("role", "line")).strip().lower()
        if role not in {"line", "floor"}:
            return jsonify({"status": f"unknown role: {role}"}), 400
        config = state.mask_config.get()
        radius = int(config["sample_radius"])

        height_px, width_px = frame.shape[:2]
        x = int(np.clip(round(x_ratio * width_px), 0, width_px - 1))
        y = int(np.clip(round(y_ratio * height_px), 0, height_px - 1))
        x0 = max(0, x - radius)
        x1 = min(width_px, x + radius + 1)
        y0 = max(0, y - radius)
        y1 = min(height_px, y + radius + 1)
        patch = frame[y0:y1, x0:x1]
        if patch.size == 0:
            return jsonify({"status": "empty sample patch"}), 400
        sample_rgb = np.median(patch.reshape(-1, 3), axis=0)
        sample_rgb = [int(np.clip(round(value), 0, 255)) for value in sample_rgb]

        updated = state.mask_config.update(
            {
                f"{role}_r": sample_rgb[0],
                f"{role}_g": sample_rgb[1],
                f"{role}_b": sample_rgb[2],
            }
        )
        return jsonify(
            {
                "status": f"sampled {role} color at ({x},{y})",
                "role": role,
                "sample_rgb": {"r": sample_rgb[0], "g": sample_rgb[1], "b": sample_rgb[2]},
                "patch": {"x0": x0, "x1": x1, "y0": y0, "y1": y1},
                "config": updated,
            }
        )

    @app.get("/camera/rgb.jpg")
    def camera_rgb():
        if state.camera is None:
            return _jpeg_response(_placeholder_jpeg("Camera offline"))
        return _jpeg_response(state.camera.jpeg_bytes("rgb"))

    @app.get("/camera/mask.jpg")
    def camera_mask():
        if state.camera is None:
            return _jpeg_response(_placeholder_jpeg("Mask unavailable"))
        return _jpeg_response(state.camera.jpeg_bytes("mask"))

    @app.get("/camera/heatmap.jpg")
    def camera_heatmap():
        if state.camera is None:
            return _jpeg_response(_placeholder_jpeg("Heatmap unavailable"))
        return _jpeg_response(state.camera.jpeg_bytes("heatmap"))

    @app.get("/camera/mask_input.jpg")
    def camera_mask_input():
        if state.camera is None:
            return _jpeg_response(_placeholder_jpeg("Mask input unavailable"))
        return _jpeg_response(state.camera.jpeg_bytes("mask_input"))

    @app.get("/camera/trajectory.jpg")
    def camera_trajectory():
        if state.camera is None:
            return _jpeg_response(_placeholder_jpeg("Trajectory unavailable"))
        return _jpeg_response(state.camera.jpeg_bytes("trajectory"))

    @app.teardown_appcontext
    def _close(_exc):
        return None

    return app


def _placeholder_jpeg(message: str) -> bytes:
    image = Image.new("RGB", (960, 540), color=(22, 32, 37))
    with BytesIO() as buf:
            image.save(buf, format="JPEG")
            return buf.getvalue()


def _jpeg_response(payload: bytes) -> Response:
    response = Response(payload, mimetype="image/jpeg")
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response


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
        try:
            follow_up = subprocess.run(
                ["ss", "-ltnp", f"sport = :{port}"],
                check=False,
                capture_output=True,
                text=True,
            )
            remaining = sorted({int(pid) for pid in re.findall(r"pid=(\d+)", follow_up.stdout)})
            remaining = [pid for pid in remaining if pid != current_pid]
        except FileNotFoundError:
            remaining = []
        for pid in remaining:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                continue
    return pids


def free_stale_teleop_servers() -> list[int]:
    try:
        result = subprocess.run(
            ["pgrep", "-f", r"python .*scripts/teleop_server\.py"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return []

    current_pid = os.getpid()
    pids = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            pid = int(line)
        except ValueError:
            continue
        if pid != current_pid:
            pids.append(pid)

    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue

    if pids:
        time.sleep(0.8)
        for pid in pids:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                continue
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                continue
    return pids


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a local joystick teleop server.")
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
    parser.add_argument("--kill-port", action="store_true", default=True)
    parser.add_argument("--no-kill-port", action="store_false", dest="kill_port")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stale = free_stale_teleop_servers()
    if stale:
        print(f"Freed stale teleop servers: {', '.join(str(pid) for pid in stale)}")
    if args.kill_port:
        freed = free_listening_port(args.http_port)
        if freed:
            print(f"Freed port {args.http_port} from PIDs: {', '.join(str(pid) for pid in freed)}")
    app = create_app(
        args.port,
        args.baudrate,
        args.camera_source,
        args.sensor_id,
        args.device_index,
        args.width,
        args.height,
        args.warmup_frames,
    )
    app.run(host=args.host, port=args.http_port, debug=False, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
