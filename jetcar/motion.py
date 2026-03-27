"""Shared motion calibration helpers for rover control."""

from __future__ import annotations

import json
from pathlib import Path
import threading

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_PATH = PROJECT_ROOT / ".teleop_calibration.json"
CALIBRATION_DEFAULTS = {
    "forward": 0.09,
    "back": 0.08,
    "left": 0.22,
    "right": 0.22,
}


def _clip_unit(value: float) -> float:
    return float(max(-1.0, min(1.0, value)))


def _direction_floor(value: float, positive_floor: float, negative_floor: float) -> float:
    if abs(value) <= 1e-6:
        return 0.0
    floor = positive_floor if value > 0 else negative_floor
    magnitude = max(abs(value), max(0.0, float(floor)))
    return _clip_unit(magnitude if value > 0 else -magnitude)


def normalize_calibration(values: dict[str, float] | None = None) -> dict[str, float]:
    merged = dict(CALIBRATION_DEFAULTS)
    if values:
        for key in merged:
            if key in values:
                try:
                    merged[key] = max(0.0, float(values[key]))
                except Exception:
                    continue
    return merged


def minimum_directional_speed(direction: str, values: dict[str, float] | None = None) -> float:
    calibration = normalize_calibration(values)
    if direction not in calibration:
        raise KeyError(direction)
    return float(calibration[direction])


def resolve_directional_speed(
    direction: str,
    requested_speed: float | None,
    values: dict[str, float] | None = None,
) -> float:
    floor = minimum_directional_speed(direction, values)
    if requested_speed is None:
        return floor
    return float(max(abs(float(requested_speed)), floor))


def apply_drive_calibration(
    left: float,
    right: float,
    values: dict[str, float] | None = None,
) -> tuple[float, float]:
    calibration = normalize_calibration(values)
    left = float(left)
    right = float(right)
    eps = 1e-6

    if left > eps and right > eps:
        floor = calibration["forward"]
        return _direction_floor(left, floor, floor), _direction_floor(right, floor, floor)
    if left < -eps and right < -eps:
        floor = calibration["back"]
        return _direction_floor(left, floor, floor), _direction_floor(right, floor, floor)
    if left < -eps and right > eps:
        floor = calibration["left"]
        return _direction_floor(left, floor, floor), _direction_floor(right, floor, floor)
    if left > eps and right < -eps:
        floor = calibration["right"]
        return _direction_floor(left, floor, floor), _direction_floor(right, floor, floor)

    return (
        _direction_floor(left, calibration["forward"], calibration["back"]),
        _direction_floor(right, calibration["forward"], calibration["back"]),
    )


class MotionCalibration:
    def __init__(self, path: Path = CALIBRATION_PATH) -> None:
        self._lock = threading.Lock()
        self._path = path
        self._values = dict(CALIBRATION_DEFAULTS)
        self._load()

    def _load(self) -> None:
        try:
            if not self._path.exists():
                return
            data = json.loads(self._path.read_text())
        except Exception:
            return
        self._values = normalize_calibration(data)

    def _save(self) -> None:
        try:
            self._path.write_text(json.dumps(self._values, indent=2))
        except Exception:
            pass

    def get(self) -> dict[str, float]:
        with self._lock:
            return dict(self._values)

    def set(self, direction: str, value: float) -> dict[str, float]:
        if direction not in CALIBRATION_DEFAULTS:
            raise KeyError(direction)
        with self._lock:
            self._values[direction] = max(0.0, float(value))
            self._save()
            return dict(self._values)

    def clear(self) -> dict[str, float]:
        with self._lock:
            self._values = dict(CALIBRATION_DEFAULTS)
            self._save()
            return dict(self._values)
