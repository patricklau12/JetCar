#!/usr/bin/env python3
"""Send Waveshare WAVE ROVER JSON commands over a Jetson serial port."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import serial

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetcar.motion import MotionCalibration, resolve_directional_speed


def send_command(port: str, payload: dict[str, float | int], baudrate: int) -> None:
    line = json.dumps(payload, separators=(",", ":")) + "\n"
    with serial.Serial(port, baudrate=baudrate, timeout=1) as ser:
        ser.write(line.encode("utf-8"))
        ser.flush()
        time.sleep(0.1)
        waiting = ser.in_waiting
        if waiting:
            response = ser.read(waiting).decode("utf-8", errors="replace")
            sys.stdout.write(response)


def build_motion_payload(
    command: str,
    speed: float | None,
    calibration: dict[str, float] | None = None,
    *,
    use_calibration: bool = True,
) -> dict[str, float | int]:
    requested_speed = None if speed is None else abs(float(speed))

    if command == "stop":
        return {"T": 1, "L": 0.0, "R": 0.0}
    if use_calibration:
        direction = "back" if command == "backward" else command
        requested_speed = resolve_directional_speed(direction, requested_speed, calibration)
    elif requested_speed is None:
        raise ValueError("speed is required when calibration is disabled")
    if not 0.0 <= requested_speed <= 0.5:
        raise ValueError("speed must be between 0.0 and 0.5")
    if command == "forward":
        return {"T": 1, "L": requested_speed, "R": requested_speed}
    if command == "backward":
        return {"T": 1, "L": -requested_speed, "R": -requested_speed}
    if command == "left":
        return {"T": 1, "L": -requested_speed, "R": requested_speed}
    if command == "right":
        return {"T": 1, "L": requested_speed, "R": -requested_speed}
    raise ValueError(f"unsupported command: {command}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control a Waveshare WAVE ROVER over UART from Jetson."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyTHS1",
        help="Serial device for the Jetson header UART, for example /dev/ttyTHS1.",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Serial baud rate. Waveshare's example uses 115200.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=None,
        help="Requested wheel speed in the range 0.0 to 0.5. If omitted, the saved calibration minimum for that direction is used.",
    )
    parser.add_argument(
        "--ignore-calibration",
        action="store_true",
        help="Use the requested speed as-is instead of applying the saved calibration minimum.",
    )
    parser.add_argument(
        "command",
        choices=["stop", "forward", "backward", "left", "right", "raw"],
        help="Motion command to send. Use raw to send your own JSON payload.",
    )
    parser.add_argument(
        "--json",
        help='Raw JSON payload to send when command is "raw". Example: \'{"T":1,"L":0.2,"R":0.2}\'',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    calibration = MotionCalibration().get()

    try:
        if args.command == "raw":
            if not args.json:
                raise ValueError('--json is required when command is "raw"')
            payload = json.loads(args.json)
        else:
            payload = build_motion_payload(
                args.command,
                args.speed,
                calibration,
                use_calibration=not args.ignore_calibration,
            )
        send_command(args.port, payload, args.baudrate)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
