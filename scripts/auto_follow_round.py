#!/usr/bin/env python3
"""Run one logged auto-follow round against the local teleop server."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from typing import Any
from urllib import error, request


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run and log one auto-follow tuning round.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8765")
    parser.add_argument("--duration-s", type=float, default=4.0)
    parser.add_argument("--poll-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=0.8)
    parser.add_argument("--label", default="")
    parser.add_argument("--output-root", default=str(PROJECT_ROOT / "runs" / "auto_follow_rounds"))
    parser.add_argument("--drive", type=float)
    parser.add_argument("--max-drive", type=float)
    parser.add_argument("--k-offset", type=float)
    parser.add_argument("--k-heading", type=float)
    parser.add_argument("--turn-mix", type=float)
    parser.add_argument("--loop-hz", type=float)
    parser.add_argument("--center-tolerance", type=float)
    parser.add_argument("--min-steer", type=float)
    parser.add_argument("--forward-floor", type=float)
    parser.add_argument("--motor-bias", type=float)
    parser.add_argument("--step-up", type=float)
    parser.add_argument("--stuck-shift-px", type=float)
    return parser.parse_args()


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


def safe_slug(label: str) -> str:
    cleaned = "".join(ch.lower() if ch.isalnum() else "-" for ch in label.strip())
    cleaned = "-".join(part for part in cleaned.split("-") if part)
    return cleaned or "round"


def resolved_settings(saved: dict[str, Any], args: argparse.Namespace) -> dict[str, float]:
    settings = dict(saved)
    mapping = {
        "drive": args.drive,
        "max_drive": args.max_drive,
        "k_offset": args.k_offset,
        "k_heading": args.k_heading,
        "turn_mix": args.turn_mix,
        "loop_hz": args.loop_hz,
        "center_tolerance": args.center_tolerance,
        "min_steer": args.min_steer,
        "forward_floor": args.forward_floor,
        "motor_bias": args.motor_bias,
        "step_up": args.step_up,
        "stuck_shift_px": args.stuck_shift_px,
    }
    for key, value in mapping.items():
        if value is not None:
            settings[key] = float(value)
    return settings


def save_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2))


def main() -> int:
    args = parse_args()
    base_url = args.base_url.rstrip("/")
    output_root = Path(args.output_root)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    round_dir = output_root / f"{stamp}-{safe_slug(args.label)}"
    round_dir.mkdir(parents=True, exist_ok=True)

    try:
        status_before = http_json(f"{base_url}/api/status")
        saved_settings = http_json(f"{base_url}/api/auto/settings")
        run_settings = resolved_settings(saved_settings, args)
        save_json(round_dir / "status_before.json", status_before)
        save_json(round_dir / "auto_settings.json", run_settings)
        (round_dir / "rgb_before.jpg").write_bytes(fetch_bytes(f"{base_url}/camera/rgb.jpg"))
        (round_dir / "mask_before.jpg").write_bytes(fetch_bytes(f"{base_url}/camera/mask.jpg"))

        if args.settle_s > 0:
            time.sleep(args.settle_s)

        start_payload = http_json(f"{base_url}/api/auto/start", method="POST", payload=run_settings)
        save_json(round_dir / "start_response.json", start_payload)

        deadline = time.monotonic() + max(0.1, args.duration_s)
        timeline: list[dict[str, Any]] = []
        while time.monotonic() < deadline:
            time.sleep(max(0.05, args.poll_s))
            sample = http_json(f"{base_url}/api/status")
            sample["elapsed_s"] = round(max(0.0, args.duration_s - max(0.0, deadline - time.monotonic())), 3)
            timeline.append(sample)
        save_json(round_dir / "timeline.json", timeline)
    except error.URLError as exc:
        print(f"Failed to reach teleop server at {base_url}: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            stop_payload = http_json(f"{base_url}/api/auto/stop", method="POST")
            save_json(round_dir / "stop_response.json", stop_payload)
        except Exception:
            pass

    status_after = http_json(f"{base_url}/api/status")
    save_json(round_dir / "status_after.json", status_after)
    (round_dir / "rgb_after.jpg").write_bytes(fetch_bytes(f"{base_url}/camera/rgb.jpg"))
    (round_dir / "mask_after.jpg").write_bytes(fetch_bytes(f"{base_url}/camera/mask.jpg"))

    last_analysis = status_after.get("auto", {}).get("last_analysis") or {}
    summary = {
        "round_dir": str(round_dir),
        "label": args.label,
        "duration_s": float(args.duration_s),
        "poll_s": float(args.poll_s),
        "last_status": status_after.get("auto", {}).get("last_status"),
        "last_error": status_after.get("auto", {}).get("last_error"),
        "last_decision": last_analysis.get("decision"),
        "last_reason": last_analysis.get("decision_reason"),
        "last_offset_norm": last_analysis.get("offset_norm"),
        "last_heading_norm": last_analysis.get("heading_norm"),
        "last_left": last_analysis.get("left"),
        "last_right": last_analysis.get("right"),
        "warnings": status_after.get("warnings", []),
    }
    save_json(round_dir / "summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
