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
from jetcar.motion import MotionCalibration, apply_drive_calibration
from jetcar.vision import build_lane_mask


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
            <div><label>Cal Step<br><input id="calStep" type="range" min="0.01" max="0.10" step="0.01" value="0.01"></label></div>
            <div><label>Cal Pulse s<br><input id="calPulse" type="range" min="0.20" max="3.00" step="0.10" value="2.00"></label></div>
            <div><label>Auto Drive<br><input id="autoDrive" type="range" min="0.00" max="0.50" step="0.01" value="0.00"></label></div>
            <div><label>Auto Max Drive<br><input id="autoMaxDrive" type="range" min="0.02" max="0.60" step="0.01" value="0.12"></label></div>
            <div><label>Auto K Offset<br><input id="autoKOffset" type="range" min="0.2" max="2.0" step="0.05" value="0.90"></label></div>
            <div><label>Auto K Heading<br><input id="autoKHeading" type="range" min="0.0" max="2.0" step="0.05" value="0.50"></label></div>
            <div><label>Auto Turn Mix<br><input id="autoTurnMix" type="range" min="0.10" max="1.20" step="0.05" value="0.24"></label></div>
            <div><label>Loop Hz<br><input id="autoHz" type="range" min="1" max="15" step="1" value="2"></label></div>
            <div><label>Center Tol<br><input id="autoCenterTol" type="range" min="0.01" max="0.25" step="0.01" value="0.06"></label></div>
            <div><label>Min Steer<br><input id="autoMinSteer" type="range" min="0.00" max="0.40" step="0.01" value="0.10"></label></div>
            <div><label>Fwd Floor<br><input id="autoForwardFloor" type="range" min="0.00" max="0.40" step="0.01" value="0.00"></label></div>
            <div><label>Motor Bias<br><input id="autoMotorBias" type="range" min="-0.12" max="0.12" step="0.01" value="0.00"></label></div>
            <div><label>Step Up<br><input id="autoStepUp" type="range" min="0.00" max="0.08" step="0.01" value="0.02"></label></div>
            <div><label>Stuck Px<br><input id="autoStuckShift" type="range" min="2" max="60" step="1" value="10"></label></div>
          </div>
        </details>
        <p class="mono" id="log"></p>
      </div>
      <div class="card panel wide">
        <details class="drawer">
          <summary>Mask Tuning</summary>
          <div class="drawer-body">
            <p class="small">Beginner mode: pick the line color, pick the floor color, and let the mask compare color difference. Contrast and HSV stay available as advanced fallbacks.</p>
            <div class="row-buttons" style="margin: 12px 0;">
              <button id="pickLine">Pick Line</button>
              <button id="pickFloor">Pick Floor</button>
              <button id="presetGreen">Green Track</button>
              <button id="presetYellow">Yellow On Red</button>
              <button id="refreshMask">Refresh Mask</button>
            </div>
            <div class="sample-bar">
              <div class="swatch"><span class="swatch-chip" id="lineSwatch"></span><span id="lineSampleLabel">Line sample</span></div>
              <div class="swatch"><span class="swatch-chip" id="floorSwatch"></span><span id="floorSampleLabel">Floor sample</span></div>
            </div>
            <div class="mini-grid">
              <div><label>Detector<br><select id="detectorMode"><option value="color_difference">Color Difference</option><option value="contrast_line">Contrast / Shape</option><option value="hsv_color">HSV Color</option></select></label></div>
              <div><label>Crop Top<br><input id="cropTop" type="range" min="0.0" max="0.8" step="0.05" value="0.35"></label></div>
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
            <p class="advanced-note">If Color Difference struggles, try Contrast / Shape or HSV Color as advanced fallback modes.</p>
          </div>
        </details>
        <p class="mono" id="maskConfig"></p>
      </div>
      <div class="card panel stream">
        <div class="big">Live RGB</div>
        <img id="rgb" class="clickable" alt="RGB stream">
      </div>
      <div class="card panel stream">
        <div class="big">Live Mask</div>
        <img id="mask" alt="Mask stream">
        <p class="mono" id="maskDecision">decision=-</p>
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

    function updateSampleSwatches(config) {
      const lineCss = `rgb(${config.line_r}, ${config.line_g}, ${config.line_b})`;
      const floorCss = `rgb(${config.floor_r}, ${config.floor_g}, ${config.floor_b})`;
      document.getElementById('lineSwatch').style.background = lineCss;
      document.getElementById('floorSwatch').style.background = floorCss;
      document.getElementById('lineSampleLabel').innerText = rgbLabel('Line', config.line_r, config.line_g, config.line_b);
      document.getElementById('floorSampleLabel').innerText = rgbLabel('Floor', config.floor_r, config.floor_g, config.floor_b);
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
      document.getElementById('maskConfig').innerText =
        `mode=${config.detector_mode || 'color_difference'} | crop=${config.crop_top_ratio.toFixed(2)} blur=${config.blur_kernel} morph=${config.morph_kernel} | tol=${Number(config.color_distance_threshold).toFixed(0)} margin=${Number(config.color_distance_margin).toFixed(0)} | line=rgb(${config.line_r},${config.line_g},${config.line_b}) floor=rgb(${config.floor_r},${config.floor_g},${config.floor_b})`;
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

    function refreshFramesOnce() {
      const t = Date.now();
      document.getElementById('rgb').src = `/camera/rgb.jpg?t=${t}`;
      document.getElementById('mask').src = `/camera/mask.jpg?t=${t}`;
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
        if (data.auto) {
          document.getElementById('status').innerText = data.auto.running ? `auto: ${data.auto.last_status}` : document.getElementById('status').innerText;
          if (data.auto.last_analysis) {
            document.getElementById('maskDecision').innerText =
              `decision=${data.auto.last_analysis.decision || '-'}\n${data.auto.last_analysis.decision_reason || ''}`;
            document.getElementById('metrics').innerText =
              `decision=${data.auto.last_analysis.decision || '-'} offset=${data.auto.last_analysis.offset_norm.toFixed(3)} heading=${data.auto.last_analysis.heading_norm.toFixed(3)} steer=${data.auto.last_analysis.steer.toFixed(3)} drive=${data.auto.last_analysis.drive.toFixed(2)} boost=${(data.auto.last_analysis.boost_drive ?? 0).toFixed(2)} left=${data.auto.last_analysis.left.toFixed(2)} right=${data.auto.last_analysis.right.toFixed(2)} shift=${(data.auto.last_analysis.x_shift ?? 0).toFixed(1)} stuck=${data.auto.last_analysis.stuck_steps ?? 0} search=${(data.auto.last_analysis.search_turn ?? 0).toFixed(2)} sstep=${data.auto.last_analysis.search_steps ?? 0}`;
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
        refreshFramesOnce();
      }, 180);
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
      const r = await fetch('/api/auto/start', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          drive: parseFloat(document.getElementById('autoDrive').value),
          max_drive: parseFloat(document.getElementById('autoMaxDrive').value),
          k_offset: parseFloat(document.getElementById('autoKOffset').value),
          k_heading: parseFloat(document.getElementById('autoKHeading').value),
          turn_mix: parseFloat(document.getElementById('autoTurnMix').value),
          loop_hz: parseFloat(document.getElementById('autoHz').value),
          center_tolerance: parseFloat(document.getElementById('autoCenterTol').value),
          min_steer: parseFloat(document.getElementById('autoMinSteer').value),
          forward_floor: parseFloat(document.getElementById('autoForwardFloor').value),
          motor_bias: parseFloat(document.getElementById('autoMotorBias').value),
          step_up: parseFloat(document.getElementById('autoStepUp').value),
          stuck_shift_px: parseFloat(document.getElementById('autoStuckShift').value),
        })
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
    document.getElementById('hz').addEventListener('input', restartLoop);
    ['detectorMode', 'cropTop', 'colorTolerance', 'colorMargin', 'hMin', 'hMax', 'sMin', 'sMax', 'vMin', 'vMax', 'blurKernel', 'morphKernel', 'sampleRadius', 'hPad', 'sPad', 'vPad']
      .forEach((id) => document.getElementById(id).addEventListener('input', scheduleMaskPush));
    document.getElementById('pickLine').addEventListener('click', () => {
      sampleTarget = 'line';
      document.getElementById('status').innerText = 'click the RGB image to sample the line color';
    });
    document.getElementById('pickFloor').addEventListener('click', () => {
      sampleTarget = 'floor';
      document.getElementById('status').innerText = 'click the RGB image to sample the floor color';
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
    renderCalibration();
  </script>
</body>
</html>
"""


PRESETS = {
    "green_track": {
        "detector_mode": "color_difference",
        "crop_top_ratio": 0.35,
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


def rgb_triplet_to_lab(rgb: tuple[int, int, int]) -> np.ndarray:
    swatch = np.array([[list(rgb)]], dtype=np.uint8)
    lab = cv2.cvtColor(swatch, cv2.COLOR_RGB2LAB).astype(np.float32)
    return lab[0, 0]


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
        min_aspect = 0.75 if loose else 1.0
        min_fill = 0.08 if loose else 0.14
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
            + max(0, min_area - area) * 0.35
            + max(0, min_height - comp_height) * 2.2
            + max(0, min_bottom_hits - bottom_hits) * 12.0
            + max(0.0, min_aspect - aspect_ratio) * 40.0
            + max(0.0, min_fill - fill_ratio) * 120.0
            + side_penalty
            - min(area, 6000) * 0.01
            - min(comp_height, height) * 1.2
            - min(aspect_ratio, 8.0) * 18.0
            - min(fill_ratio, 1.0) * 30.0
            - min(bottom_hits, 80) * 1.2
        )
        if best_score is None or score < best_score:
            best_score = score
            best_label = label

    if best_label == 0:
        return np.zeros_like(mask)

    filtered = np.zeros_like(mask)
    filtered[labels == best_label] = 255
    return filtered


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

    mask = select_target_component(mask, anchor_x=anchor_x, prefer_side=prefer_side, loose=loose)
    full_mask = np.zeros_like(gray)
    full_mask[crop_start:, :] = mask
    return Image.fromarray(full_mask, mode="L")


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
    mask = (dist_line <= threshold).astype(np.uint8) * 255
    if np.linalg.norm(line_lab - floor_lab) > 1.0:
        closer_to_line = (dist_line + margin) < dist_floor
        mask = np.where(closer_to_line, mask, 0).astype(np.uint8)

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
    mode = str(config.get("detector_mode", "contrast_line"))
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


def compute_row_center(mask: np.ndarray, row_ratio: float) -> tuple[float | None, int]:
    row_index = int(mask.shape[0] * row_ratio)
    row_index = max(0, min(mask.shape[0] - 1, row_index))
    xs = np.flatnonzero(mask[row_index] > 0)
    if xs.size == 0:
        return None, row_index
    return float(xs.mean()), row_index


def estimate_strip_pose(
    mask: np.ndarray,
    near_row_ratio: float = 0.82,
    far_row_ratio: float = 0.42,
) -> tuple[float | None, float | None, int, int, str]:
    x_near, row_near = compute_row_center(mask, near_row_ratio)
    x_far, row_far = compute_row_center(mask, far_row_ratio)
    if x_near is not None and x_far is not None:
        return x_near, x_far, row_near, row_far, "row"

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
        self._lock = threading.Lock()
        self._latest_rgb: Image.Image | None = None
        self._latest_mask: Image.Image | None = None
        self._anchor_x: float | None = None
        self._last_side = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = read_rgb_frame(self._cap)
                rgb = Image.fromarray(frame)
                config = self._mask_config.get()
                mask = build_tracking_mask(rgb, config, anchor_x=self._anchor_x, prefer_side=self._last_side)
                mask_arr = np.array(mask)
                x_near, _, _, _, _ = estimate_strip_pose(mask_arr, 0.82, 0.42)
                if x_near is not None:
                    self._anchor_x = x_near
                    width = mask_arr.shape[1]
                    if x_near > width * 0.55:
                        self._last_side = 1
                    elif x_near < width * 0.45:
                        self._last_side = -1
                with self._lock:
                    self._latest_rgb = rgb
                    self._latest_mask = mask
            except Exception:
                time.sleep(0.1)
                continue
            time.sleep(0.05)

    def jpeg_bytes(self, kind: str) -> bytes:
        with self._lock:
            image = self._latest_rgb if kind == "rgb" else self._latest_mask
        if image is None:
            image = Image.new("RGB", (640, 360), color="black")
        with BytesIO() as buf:
            image.save(buf, format="JPEG")
            return buf.getvalue()

    def latest_rgb_array(self) -> np.ndarray | None:
        with self._lock:
            if self._latest_rgb is None:
                return None
            return np.array(self._latest_rgb)

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
        self.auto_controller: AutoController | None = None

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
    def __init__(
        self,
        rover: RoverSerial | None,
        camera: CameraFeed | None,
        mask_config: MaskConfig,
        calibration: MotionCalibration,
    ) -> None:
        self._rover = rover
        self._camera = camera
        self._mask_config = mask_config
        self._calibration = calibration
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._settings = {
            "drive": 0.0,
            "min_drive": 0.0,
            "max_drive": 0.12,
            "k_offset": 0.90,
            "k_heading": 0.50,
            "turn_mix": 0.24,
            "loop_hz": 2.0,
            "center_tolerance": 0.06,
            "min_steer": 0.10,
            "min_wheel": 0.04,
            "forward_floor": 0.0,
            "motor_bias": 0.00,
            "step_up": 0.02,
            "stuck_shift_px": 10.0,
            "search_turn": 0.05,
            "search_step_up": 0.015,
            "search_max_turn": 0.18,
            "search_drive": 0.04,
        }
        self.last_analysis: dict[str, float | str] | None = None
        self.last_status = "idle"
        self.last_error: str | None = None
        self._anchor_x: float | None = None
        self._last_side = 0
        self._base_drive = float(self._settings["drive"])
        self._base_forward_floor = float(self._settings["forward_floor"])
        self._stuck_steps = 0
        self._search_steps = 0
        self._last_steer_sign = 0

    @staticmethod
    def _decision_label(left: float, right: float, searching: bool = False) -> str:
        avg = (left + right) / 2.0
        diff = left - right
        turn_eps = 0.03
        move_eps = 0.03
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

    @staticmethod
    def _decision_reason(offset_norm: float, heading_norm: float, searching: bool = False, search_direction: int = 0) -> str:
        if searching:
            if search_direction > 0:
                return "searching toward right side"
            if search_direction < 0:
                return "searching toward left side"
            return "searching for line"
        offset_side = "right" if offset_norm > 0.03 else "left" if offset_norm < -0.03 else "center"
        heading_side = "right" if heading_norm > 0.03 else "left" if heading_norm < -0.03 else "straight"
        dominant = "offset" if abs(offset_norm) >= abs(heading_norm) else "heading"
        return f"near line={offset_side}, curve={heading_side}, dominant={dominant}"

    @staticmethod
    def _gentle_turn_values(
        direction: int,
        forward_min: float,
        left_turn_min: float,
        right_turn_min: float,
        extra_force: float,
        turn_mix: float,
        max_drive: float,
        searching: bool = False,
    ) -> tuple[float, float, float]:
        effective_max_drive = max(0.05, float(max_drive))
        forward_ref = min(effective_max_drive, max(0.05, float(forward_min) + 0.015))
        crawl = forward_ref + min(0.012 if searching else 0.008, extra_force * 0.10)
        crawl = min(crawl, effective_max_drive)
        min_delta = 0.025 if searching else 0.015
        mix = float(np.clip(turn_mix, 0.0, 1.0))
        if direction >= 0:
            turn_floor = max(float(right_turn_min), crawl + min_delta)
            outer = crawl + max(min_delta, (turn_floor - crawl) * mix)
            outer += min(0.015 if searching else 0.010, extra_force * 0.12)
            outer = min(outer, max(crawl + min_delta, effective_max_drive))
            return float(outer), float(crawl), float(crawl)
        turn_floor = max(float(left_turn_min), crawl + min_delta)
        outer = crawl + max(min_delta, (turn_floor - crawl) * mix)
        outer += min(0.015 if searching else 0.010, extra_force * 0.12)
        outer = min(outer, max(crawl + min_delta, effective_max_drive))
        return float(crawl), float(outer), float(crawl)

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
            }

    def start(self, settings: dict[str, float]) -> dict[str, object]:
        if self._rover is None:
            raise RuntimeError("Serial is unavailable.")
        if self._camera is None:
            raise RuntimeError("Camera is unavailable.")
        with self._lock:
            self._settings.update(settings)
            self._base_drive = float(self._settings["drive"])
            self._base_forward_floor = float(self._settings["forward_floor"])
            self._stuck_steps = 0
            self._search_steps = 0
            self._last_side = 0
            self._last_steer_sign = 0
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("Auto follow is already running.")
            self._stop.clear()
            self.last_error = None
            self.last_status = "starting"
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            payload = {
                "running": True,
                "settings": dict(self._settings),
                "last_status": self.last_status,
                "last_error": self.last_error,
                "last_analysis": self.last_analysis,
                "stuck_steps": self._stuck_steps,
                "search_steps": self._search_steps,
            }
        return payload

    def stop(self) -> dict[str, object]:
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
            self.last_status = "stopped"
        return self.status_payload()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                analysis = self._analyze_and_drive()
                with self._lock:
                    self._search_steps = 0
                    self.last_analysis = analysis
                    self.last_status = analysis["status"]
                    self.last_error = None
            except Exception as exc:
                if self._rover is not None:
                    try:
                        calibration = self._calibration.get()
                        left_turn_min = max(0.0, float(calibration.get("left", 0.0)))
                        right_turn_min = max(0.0, float(calibration.get("right", 0.0)))
                        forward_min = max(0.0, float(calibration.get("forward", 0.0)))
                        search_turn = float(self._settings["search_turn"])
                        search_turn += self._search_steps * float(self._settings["search_step_up"])
                        search_turn = min(search_turn, float(self._settings["search_max_turn"]))
                        direction = -self._last_steer_sign if self._last_steer_sign != 0 else self._last_side
                        if direction == 0:
                            direction = 1
                        bias = float(self._settings["motor_bias"])
                        left, right, _search_drive = self._gentle_turn_values(
                            direction,
                            forward_min,
                            left_turn_min,
                            right_turn_min,
                            search_turn,
                            float(self._settings["turn_mix"]),
                            max(float(self._settings["max_drive"]), forward_min),
                            searching=True,
                        )
                        left = float(np.clip(left + bias, -1.0, 1.0))
                        right = float(np.clip(right - bias, -1.0, 1.0))
                        self._rover.send(left, right)
                    except Exception:
                        try:
                            self._rover.send(0.0, 0.0)
                        except Exception:
                            pass
                with self._lock:
                    self._search_steps += 1
                    self.last_error = str(exc)
                    self.last_status = "waiting for line, searching"
                    self.last_analysis = {
                        "status": "searching",
                        "decision": self._decision_label(
                            left if 'left' in locals() else 0.0,
                            right if 'right' in locals() else 0.0,
                            searching=True,
                        ),
                        "decision_reason": self._decision_reason(
                            0.0,
                            0.0,
                            searching=True,
                            search_direction=direction if 'direction' in locals() else 0,
                        ),
                        "offset_norm": 0.0,
                        "heading_norm": 0.0,
                        "steer": 0.0,
                        "drive": 0.0,
                        "boost_drive": 0.0,
                        "left": left if 'left' in locals() else 0.0,
                        "right": right if 'right' in locals() else 0.0,
                        "x_near": self._anchor_x if self._anchor_x is not None else 0.0,
                        "x_shift": 0.0,
                        "stuck_steps": int(self._stuck_steps),
                        "search_steps": int(self._search_steps),
                        "search_turn": float(search_turn if 'search_turn' in locals() else 0.0),
                    }
                time.sleep(0.25)
                continue
            hz = max(1.0, float(self._settings["loop_hz"]))
            time.sleep(1.0 / hz)

        if self._rover is not None:
            try:
                self._rover.send(0.0, 0.0)
            except Exception:
                pass

    def _analyze_and_drive(self) -> dict[str, float | str]:
        if self._camera is None or self._rover is None:
            raise RuntimeError("Camera or serial unavailable.")
        frame = self._camera.latest_rgb_array()
        if frame is None:
            raise RuntimeError("No camera frame yet.")

        config = self._mask_config.get()
        rgb_image = Image.fromarray(frame)
        candidate_prefs = []
        if self._last_side != 0:
            candidate_prefs.append((self._last_side, False))
            candidate_prefs.append((self._last_side, True))
            candidate_prefs.append((0, True))
            candidate_prefs.append((-self._last_side, True))
        else:
            candidate_prefs.append((0, False))
            candidate_prefs.append((0, True))

        mask = None
        x_near = x_far = None
        pose_mode = "none"
        used_side = 0
        used_loose = False
        for prefer_side, loose in candidate_prefs:
            candidate_mask = np.array(
                build_tracking_mask(
                    rgb_image,
                    config,
                    anchor_x=self._anchor_x,
                    prefer_side=prefer_side,
                    loose=loose,
                )
            )
            cand_near, cand_far, _row_near, _row_far, cand_mode = estimate_strip_pose(candidate_mask, 0.82, 0.42)
            if cand_near is not None and cand_far is not None:
                mask = candidate_mask
                x_near = cand_near
                x_far = cand_far
                pose_mode = cand_mode
                used_side = prefer_side
                used_loose = loose
                break

        if x_near is None or x_far is None:
            raise RuntimeError("Could not find target strip in mask.")
        self._anchor_x = x_near
        width = frame.shape[1]
        if x_near > width * 0.55:
            self._last_side = 1
        elif x_near < width * 0.45:
            self._last_side = -1

        image_center_x = width / 2.0
        offset_norm = (x_near - image_center_x) / (width / 2.0)
        heading_norm = (x_far - x_near) / (width / 2.0)
        x_shift = None
        if self.last_analysis is not None and "x_near" in self.last_analysis:
            x_shift = abs(float(x_near) - float(self.last_analysis["x_near"]))
            if x_shift < float(self._settings["stuck_shift_px"]):
                self._stuck_steps += 1
            else:
                self._stuck_steps = 0

        steer = float(
            np.clip(
                float(self._settings["k_offset"]) * offset_norm +
                float(self._settings["k_heading"]) * heading_norm,
                -1.0,
                1.0,
            )
        )
        steer = float(np.clip(steer, -0.55, 0.55))
        if abs(steer) > 0.04:
            self._last_steer_sign = 1 if steer > 0 else -1

        if abs(steer) < float(self._settings["min_steer"]) and abs(offset_norm) > float(self._settings["center_tolerance"]):
            steer = float(np.sign(steer) or np.sign(offset_norm)) * float(self._settings["min_steer"])

        calibration = self._calibration.get()
        forward_min = max(0.0, float(calibration.get("forward", 0.0)))
        left_turn_min = max(0.0, float(calibration.get("left", 0.0)))
        right_turn_min = max(0.0, float(calibration.get("right", 0.0)))
        effective_max_drive = max(float(self._settings["max_drive"]), forward_min)
        base_forward_drive = min(effective_max_drive, max(0.05, forward_min + 0.015))
        drive_boost = self._stuck_steps * float(self._settings["step_up"])
        stuck_extra = min(0.18, drive_boost)
        center_tol = float(self._settings["center_tolerance"])
        heading_turn_tol = 0.08
        turn_only = abs(offset_norm) > center_tol or abs(heading_norm) > heading_turn_tol

        if turn_only:
            drive = 0.0
            left, right, crawl_drive = self._gentle_turn_values(
                1 if steer >= 0 else -1,
                forward_min,
                left_turn_min,
                right_turn_min,
                stuck_extra,
                float(self._settings["turn_mix"]),
                effective_max_drive,
                searching=False,
            )
            drive = crawl_drive
            decision_reason = (
                f"re-centering first: near line={'right' if offset_norm > 0 else 'left'}, "
                f"curve={'right' if heading_norm > 0 else 'left' if heading_norm < 0 else 'straight'}"
            )
        else:
            drive_target = max(base_forward_drive, float(self._settings["min_drive"]), self._base_drive)
            drive = min(drive_target, effective_max_drive)
            if self._stuck_steps > 0:
                drive = min(effective_max_drive, drive + min(0.020, stuck_extra * 0.12))
            if abs(offset_norm) <= center_tol:
                drive = min(drive + 0.004, effective_max_drive)

            left = float(drive)
            right = float(drive)
            forward_floor = max(base_forward_drive, min(effective_max_drive, self._base_forward_floor))
            if self._stuck_steps > 0:
                forward_floor = min(effective_max_drive, max(forward_floor, base_forward_drive + min(0.010, stuck_extra * 0.10)))
            left = max(left, forward_floor)
            right = max(right, forward_floor)
            decision_reason = "line centered enough, moving forward"

        motor_bias = float(self._settings["motor_bias"])
        left = float(np.clip(left + motor_bias, -1.0, 1.0))
        right = float(np.clip(right - motor_bias, -1.0, 1.0))
        min_wheel = float(self._settings["min_wheel"])
        if abs(left) < min_wheel and abs(left) > 1e-6:
            left = float(np.sign(left) * min_wheel)
        if abs(right) < min_wheel and abs(right) > 1e-6:
            right = float(np.sign(right) * min_wheel)
        self._rover.send(left, right)
        return {
            "status": "following line" if pose_mode == "row" else f"following line ({pose_mode})",
            "decision": self._decision_label(left, right),
            "decision_reason": decision_reason,
            "offset_norm": float(offset_norm),
            "heading_norm": float(heading_norm),
            "steer": float(steer),
            "drive": float(drive),
            "boost_drive": float(stuck_extra),
            "left": float(left),
            "right": float(right),
            "x_near": float(x_near),
            "x_shift": None if x_shift is None else float(x_shift),
            "stuck_steps": int(self._stuck_steps),
            "used_side": int(used_side),
            "used_loose": bool(used_loose),
        }


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
    rover = None
    rover_error = None
    try:
        rover = RoverSerial(port, baudrate)
    except Exception as exc:
        rover_error = str(exc)

    camera = None
    camera_error = None
    try:
        camera = CameraFeed(camera_source, sensor_id, device_index, width, height, warmup_frames, mask_config)
    except Exception as exc:
        camera_error = str(exc)

    state = AppState(
        rover=rover,
        rover_error=rover_error,
        camera=camera,
        camera_error=camera_error,
        port=port,
        baudrate=baudrate,
        camera_source=camera_source,
        width=width,
        height=height,
    )
    state.mask_config = mask_config
    state.auto_controller = AutoController(rover, camera, mask_config, state.calibration)
    app = Flask(__name__)
    atexit.register(state.close)

    @app.get("/")
    def index():
        return HTML

    @app.get("/api/status")
    def api_status():
        return jsonify(state.status_payload())

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
        base_drive = float(data.get("drive", 0.0))
        try:
            payload = state.auto_controller.start(
                {
                    "drive": base_drive,
                    "min_drive": max(0.0, base_drive - 0.01),
                    "max_drive": float(data.get("max_drive", 0.12)),
                    "k_offset": float(data.get("k_offset", 0.90)),
                    "k_heading": float(data.get("k_heading", 0.50)),
                    "turn_mix": float(data.get("turn_mix", 0.24)),
                    "loop_hz": float(data.get("loop_hz", 2.0)),
                    "center_tolerance": float(data.get("center_tolerance", 0.06)),
                    "min_steer": float(data.get("min_steer", 0.10)),
                    "forward_floor": float(data.get("forward_floor", 0.0)),
                    "motor_bias": float(data.get("motor_bias", 0.00)),
                    "step_up": float(data.get("step_up", 0.02)),
                    "stuck_shift_px": float(data.get("stuck_shift_px", 10.0)),
                    "min_wheel": max(0.03, base_drive - 0.005),
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
                "detector_mode": "color_difference",
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
            return Response(_placeholder_jpeg("Camera offline"), mimetype="image/jpeg")
        return Response(state.camera.jpeg_bytes("rgb"), mimetype="image/jpeg")

    @app.get("/camera/mask.jpg")
    def camera_mask():
        if state.camera is None:
            return Response(_placeholder_jpeg("Mask unavailable"), mimetype="image/jpeg")
        return Response(state.camera.jpeg_bytes("mask"), mimetype="image/jpeg")

    @app.teardown_appcontext
    def _close(_exc):
        return None

    return app


def _placeholder_jpeg(message: str) -> bytes:
    image = Image.new("RGB", (960, 540), color=(22, 32, 37))
    with BytesIO() as buf:
            image.save(buf, format="JPEG")
            return buf.getvalue()


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
    parser.add_argument("--port", default="/dev/ttyTHS1")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--camera-source", default="usb", choices=["usb", "csi"])
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
