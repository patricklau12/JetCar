from __future__ import annotations

import glob
import os
from dataclasses import dataclass

import serial

try:
    import serial.tools.list_ports
except Exception:  # pragma: no cover - optional at import time
    serial.tools = None  # type: ignore[attr-defined]

from .camera import open_camera


SERIAL_FALLBACKS = (
    "/dev/ttyUSB0",
    "/dev/ttyACM0",
    "/dev/ttyTHS1",
    "/dev/ttyTHS2",
)


@dataclass(frozen=True)
class CameraSelection:
    source: str
    sensor_id: int
    device_index: int


def serial_candidates() -> list[str]:
    candidates: list[str] = []
    if getattr(serial, "tools", None) is not None and getattr(serial.tools, "list_ports", None) is not None:
        try:
            for port in serial.tools.list_ports.comports():
                if port.device not in candidates:
                    candidates.append(port.device)
        except Exception:
            pass

    preferred = [
        path
        for path in candidates
        if path.startswith("/dev/ttyUSB") or path.startswith("/dev/ttyACM")
    ]
    preferred.extend(
        path
        for path in candidates
        if path.startswith("/dev/ttyTHS") and path not in preferred
    )

    for fallback in SERIAL_FALLBACKS:
        if os.path.exists(fallback) and fallback not in preferred:
            preferred.append(fallback)
    return preferred


def resolve_serial_port(port: str = "auto", baudrate: int = 115200, timeout: float = 0.2) -> str:
    if port != "auto":
        return port

    candidates = serial_candidates()
    errors: list[str] = []
    for candidate in candidates:
        try:
            ser = serial.Serial(candidate, baudrate=baudrate, timeout=timeout)
        except Exception as exc:
            errors.append(f"{candidate}: {exc}")
            continue
        ser.close()
        return candidate

    details = "; ".join(errors) if errors else "no serial candidates detected"
    raise RuntimeError(f"Could not auto-detect a rover serial port: {details}.")


def usb_video_device_indices() -> list[int]:
    indices: list[int] = []
    for path in sorted(glob.glob("/dev/video*")):
        suffix = path.removeprefix("/dev/video")
        if not suffix.isdigit():
            continue
        sysfs = os.path.realpath(f"/sys/class/video4linux/video{suffix}/device")
        if "/usb" not in sysfs:
            continue
        indices.append(int(suffix))
    return indices


def resolve_camera_selection(
    source: str = "auto",
    sensor_id: int = 0,
    device_index: int = 0,
    width: int = 1280,
    height: int = 720,
    warmup_frames: int = 12,
) -> CameraSelection:
    if source != "auto":
        return CameraSelection(source=source, sensor_id=sensor_id, device_index=device_index)

    probe_width = min(width, 640)
    probe_height = min(height, 480)
    errors: list[str] = []

    try:
        cap = open_camera(
            source="csi",
            sensor_id=sensor_id,
            device_index=device_index,
            width=width,
            height=height,
            warmup_frames=min(warmup_frames, 2),
        )
    except Exception as exc:
        errors.append(f"csi(sensor_id={sensor_id}): {exc}")
    else:
        cap.release()
        return CameraSelection(source="csi", sensor_id=sensor_id, device_index=device_index)

    for usb_index in usb_video_device_indices():
        try:
            cap = open_camera(
                source="usb",
                device_index=usb_index,
                width=probe_width,
                height=probe_height,
                warmup_frames=1,
            )
        except Exception as exc:
            errors.append(f"usb(device_index={usb_index}): {exc}")
            continue
        cap.release()
        return CameraSelection(source="usb", sensor_id=sensor_id, device_index=usb_index)

    raise RuntimeError("Could not auto-detect a working camera. " + " | ".join(errors))
