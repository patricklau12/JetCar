#!/usr/bin/env python3
"""Tune line-follow settings by making short pulse corrections and scoring improvement."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from typing import Any
from urllib import error, request


PROJECT_ROOT = Path(__file__).resolve().parents[1]

HEADING_WEIGHT = 0.60
STEER_DEADBAND = 0.05
HEADING_TOLERANCE = 0.08
IMPROVEMENT_MARGIN = 0.010
MIN_OFFSET_IMPROVEMENT = 0.010
MIN_HEADING_IMPROVEMENT = 0.010
MIN_STEER_IMPROVEMENT = 0.020
MIN_SHIFT_PX = 4.0
LOCK_METRIC = 0.10


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def http_json(url: str, method: str = "GET", payload: dict[str, Any] | None = None, timeout: float = 3.0) -> Any:
    body = None
    headers = {}
    if payload is not None:
        body = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = request.Request(url, data=body, headers=headers, method=method)
    with request.urlopen(req, timeout=timeout) as resp:
        charset = resp.headers.get_content_charset("utf-8")
        return json.loads(resp.read().decode(charset))


def fetch_bytes(url: str, timeout: float = 3.0) -> bytes:
    with request.urlopen(url, timeout=timeout) as resp:
        return resp.read()


def save_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2))


def safe_slug(label: str) -> str:
    cleaned = "".join(ch.lower() if ch.isalnum() else "-" for ch in label.strip())
    cleaned = "-".join(part for part in cleaned.split("-") if part)
    return cleaned or "autotune"


@dataclass
class SessionState:
    anchor_x: float | None = None
    last_side: int = 0


@dataclass
class ControllerState:
    drive: float
    stuck_steps: int = 0
    move_streak: int = 0
    steps: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manual-like autotune loop for JetCar line following.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8765")
    parser.add_argument("--steps", type=int, default=16)
    parser.add_argument("--pulse-s", type=float, default=0.25)
    parser.add_argument("--settle-s", type=float, default=0.35)
    parser.add_argument("--loop-pause-s", type=float, default=0.20)
    parser.add_argument("--label", default="")
    parser.add_argument("--output-root", default=str(PROJECT_ROOT / "runs" / "manual_autotune"))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-save", action="store_true")
    return parser.parse_args()


def capture_pair(base_url: str, out_dir: Path, stem: str) -> None:
    (out_dir / f"{stem}-rgb.jpg").write_bytes(fetch_bytes(f"{base_url}/camera/rgb.jpg"))
    (out_dir / f"{stem}-mask.jpg").write_bytes(fetch_bytes(f"{base_url}/camera/mask.jpg"))


def analyze(base_url: str, session: SessionState, settings: dict[str, float], prev_x_near: float | None = None) -> dict[str, Any]:
    payload = {
        "anchor_x": session.anchor_x,
        "last_side": session.last_side,
        "prev_x_near": prev_x_near,
        "k_offset": settings["k_offset"],
        "k_heading": settings["k_heading"],
    }
    result = http_json(f"{base_url}/api/analyze", method="POST", payload=payload)
    session.anchor_x = None if result.get("anchor_x") is None else float(result["anchor_x"])
    session.last_side = int(result.get("last_side", 0))
    return result


def compute_action(before: dict[str, Any], controller: ControllerState, settings: dict[str, float], calibration: dict[str, float], pulse_s: float) -> dict[str, float | str]:
    steer = float(before["steer"])
    if abs(steer) < STEER_DEADBAND:
        steer = 0.0
    elif abs(steer) < settings["min_steer"]:
        steer = float((1.0 if steer >= 0 else -1.0) * settings["min_steer"])

    base_drive = clamp(max(controller.drive, calibration["forward"]), calibration["forward"], settings["max_drive"])
    turn_mix = settings["turn_mix"]
    duration = pulse_s + min(controller.move_streak, 4) * 0.04
    mode = "normal"

    centered = (
        abs(float(before["offset_norm"])) <= settings["center_tolerance"]
        and abs(float(before["heading_norm"])) <= HEADING_TOLERANCE
    )

    if centered:
        base_drive = min(base_drive, max(calibration["forward"], settings["drive"]))

    if controller.stuck_steps >= 2:
        mode = "boost"
        base_drive = min(settings["max_drive"], base_drive + 0.01)
        turn_mix = min(1.00, turn_mix + 0.05)
        duration = min(0.70, duration + 0.06)

    if controller.stuck_steps >= 4:
        mode = "pivot"
        steer_sign = 1.0 if steer > 0 else -1.0 if steer < 0 else float(1 if int(before.get("last_side", 0)) >= 0 else -1)
        return {
            "mode": mode,
            "steer": steer_sign,
            "throttle": 0.0,
            "mix": max(0.28, min(turn_mix + 0.06, 0.55)),
            "duration_s": min(0.85, max(duration, pulse_s + 0.10)),
        }

    return {
        "mode": mode,
        "steer": clamp(steer, -1.0, 1.0),
        "throttle": base_drive,
        "mix": clamp(turn_mix, 0.10, 1.20),
        "duration_s": min(0.70, duration),
    }


def judge_improvement(before: dict[str, Any], after: dict[str, Any], settings: dict[str, float]) -> dict[str, Any]:
    metric_drop = float(before["metric"] - after["metric"])
    offset_drop = float(abs(before["offset_norm"]) - abs(after["offset_norm"]))
    heading_drop = float(abs(before["heading_norm"]) - abs(after["heading_norm"]))
    steer_drop = float(abs(before["steer"]) - abs(after["steer"]))
    x_shift = float(after.get("x_shift") or 0.0)

    centered_after = (
        float(after["metric"]) <= LOCK_METRIC
        and abs(float(after["offset_norm"])) <= settings["center_tolerance"]
        and abs(float(after["heading_norm"])) <= HEADING_TOLERANCE
    )

    metric_ok = metric_drop >= IMPROVEMENT_MARGIN or centered_after
    shift_ok = x_shift >= MIN_SHIFT_PX or centered_after
    axis_ok = (
        offset_drop >= MIN_OFFSET_IMPROVEMENT
        or heading_drop >= MIN_HEADING_IMPROVEMENT
        or steer_drop >= MIN_STEER_IMPROVEMENT
        or centered_after
    )
    improved = bool(metric_ok and shift_ok and axis_ok)
    return {
        "improved": improved,
        "centered_after": centered_after,
        "metric_drop": metric_drop,
        "offset_drop": offset_drop,
        "heading_drop": heading_drop,
        "steer_drop": steer_drop,
        "x_shift": x_shift,
    }


def tune_settings(
    settings: dict[str, float],
    controller: ControllerState,
    calibration: dict[str, float],
    before: dict[str, Any],
    after: dict[str, Any],
    action: dict[str, float | str],
    result: dict[str, Any],
) -> None:
    if result["improved"]:
        controller.stuck_steps = 0
        controller.move_streak += 1
        controller.drive = max(calibration["forward"], controller.drive - 0.003)
    else:
        controller.stuck_steps += 1
        controller.move_streak = 0
        controller.drive = min(settings["max_drive"], controller.drive + 0.010)

    settings["drive"] = round(clamp(controller.drive, calibration["forward"], settings["max_drive"]), 3)
    settings["forward_floor"] = round(clamp(max(calibration["forward"], settings["drive"] - 0.004), 0.0, settings["max_drive"]), 3)

    action_steer = float(action["steer"])
    offset_before = float(before["offset_norm"])
    offset_after = float(after["offset_norm"])
    heading_before = float(before["heading_norm"])
    heading_after = float(after["heading_norm"])

    if not result["improved"] and abs(action_steer) > 0.25:
        settings["turn_mix"] = round(clamp(settings["turn_mix"] + 0.02, 0.10, 1.20), 3)
    if offset_before * offset_after < 0 and abs(offset_after) > 0.04:
        settings["turn_mix"] = round(clamp(settings["turn_mix"] - 0.03, 0.10, 1.20), 3)

    if abs(offset_before) > abs(heading_before) * 1.25:
        if result["offset_drop"] < 0.005 and not result["improved"]:
            settings["k_offset"] = round(clamp(settings["k_offset"] + 0.05, 0.20, 2.00), 3)
        elif offset_before * offset_after < 0 and abs(offset_after) > 0.04:
            settings["k_offset"] = round(clamp(settings["k_offset"] - 0.05, 0.20, 2.00), 3)
    elif abs(heading_before) > abs(offset_before) * 1.25:
        if result["heading_drop"] < 0.005 and not result["improved"]:
            settings["k_heading"] = round(clamp(settings["k_heading"] + 0.05, 0.00, 2.00), 3)
        elif heading_before * heading_after < 0 and abs(heading_after) > 0.04:
            settings["k_heading"] = round(clamp(settings["k_heading"] - 0.05, 0.00, 2.00), 3)

    if not result["improved"] and 0.0 < abs(float(before["steer"])) < settings["min_steer"] + 0.02:
        settings["min_steer"] = round(clamp(settings["min_steer"] + 0.01, 0.00, 0.40), 3)
    elif result["improved"] and offset_before * offset_after < 0 and abs(offset_after) > 0.04:
        settings["min_steer"] = round(clamp(settings["min_steer"] - 0.01, 0.00, 0.40), 3)

    if abs(action_steer) < 0.05 and abs(offset_before) <= settings["center_tolerance"] * 1.5:
        drift = offset_after - offset_before
        settings["motor_bias"] = round(clamp(settings["motor_bias"] - drift * 0.08, -0.12, 0.12), 3)


def main() -> int:
    args = parse_args()
    base_url = args.base_url.rstrip("/")
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    round_dir = Path(args.output_root) / f"{stamp}-{safe_slug(args.label)}"
    round_dir.mkdir(parents=True, exist_ok=True)

    try:
        status = http_json(f"{base_url}/api/status")
    except error.URLError as exc:
        print(f"Could not reach teleop server at {base_url}: {exc}", file=sys.stderr)
        return 1

    if not status["camera"]["ok"] or not status["serial"]["ok"]:
        print(json.dumps(status, indent=2), file=sys.stderr)
        print("Camera and serial both need to be ready before autotune can run.", file=sys.stderr)
        return 1

    http_json(f"{base_url}/api/stop", method="POST")

    settings = http_json(f"{base_url}/api/auto/settings")
    calibration = http_json(f"{base_url}/api/calibration")
    controller = ControllerState(drive=max(float(settings["drive"]), float(calibration["forward"])))
    session = SessionState()

    save_json(round_dir / "initial_status.json", status)
    save_json(round_dir / "initial_auto_settings.json", settings)
    save_json(round_dir / "calibration.json", calibration)
    capture_pair(base_url, round_dir, "initial")

    best_settings = dict(settings)
    best_metric = float("inf")
    step_records: list[dict[str, Any]] = []

    for step in range(1, max(0, args.steps) + 1):
        before = analyze(base_url, session, settings)
        capture_pair(base_url, round_dir, f"step-{step:03d}-before")

        action = compute_action(before, controller, calibration=calibration, settings=settings, pulse_s=args.pulse_s)
        if not args.dry_run:
            http_json(
                f"{base_url}/api/pulse",
                method="POST",
                payload={
                    "steer": float(action["steer"]),
                    "throttle": float(action["throttle"]),
                    "mix": float(action["mix"]),
                    "duration_s": float(action["duration_s"]),
                },
                timeout=max(3.0, float(action["duration_s"]) + 2.0),
            )
        time.sleep(args.settle_s)
        after = analyze(base_url, session, settings, prev_x_near=float(before["x_near"]))
        capture_pair(base_url, round_dir, f"step-{step:03d}-after")

        result = judge_improvement(before, after, settings)
        tune_settings(settings, controller, calibration, before, after, action, result)
        controller.steps = step

        score = float(after["metric"]) + 0.03 * controller.stuck_steps
        if score < best_metric:
            best_metric = score
            best_settings = dict(settings)

        record = {
            "step": step,
            "before": before,
            "action": action,
            "after": after,
            "result": result,
            "controller": {
                "drive": controller.drive,
                "stuck_steps": controller.stuck_steps,
                "move_streak": controller.move_streak,
            },
            "settings": dict(settings),
            "best_metric": best_metric,
        }
        step_records.append(record)
        save_json(round_dir / "steps.json", step_records)
        time.sleep(args.loop_pause_s)

    http_json(f"{base_url}/api/stop", method="POST")

    final_status = http_json(f"{base_url}/api/status")
    capture_pair(base_url, round_dir, "final")
    save_json(round_dir / "final_status.json", final_status)

    if not args.no_save:
        best_settings["drive"] = round(clamp(best_settings["drive"], calibration["forward"], best_settings["max_drive"]), 3)
        best_settings["forward_floor"] = round(clamp(max(calibration["forward"], best_settings["drive"] - 0.004), 0.0, best_settings["max_drive"]), 3)
        saved_response = http_json(f"{base_url}/api/auto/settings", method="POST", payload=best_settings)
        save_json(round_dir / "saved_auto_settings.json", saved_response)
    else:
        save_json(round_dir / "saved_auto_settings.json", {"saved": False, "settings": best_settings})

    summary = {
        "round_dir": str(round_dir),
        "dry_run": bool(args.dry_run),
        "steps": int(args.steps),
        "best_metric": best_metric,
        "best_settings": best_settings,
        "final_status": final_status.get("auto", {}).get("last_status"),
        "warnings": final_status.get("warnings", []),
    }
    save_json(round_dir / "summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
