#!/usr/bin/env python3
from __future__ import annotations

import glob
import importlib
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetcar.hardware import serial_candidates, usb_video_device_indices


CORE_PACKAGES = [
    "numpy",
    "cv2",
    "torch",
    "torchvision",
    "flask",
    "serial",
    "PIL",
    "pandas",
    "matplotlib",
    "tqdm",
    "ultralytics",
]

NOTEBOOK_PACKAGES = [
    "jupyterlab",
    "ipykernel",
    "ipywidgets",
]


def version_of(module_name: str) -> str:
    module = importlib.import_module(module_name)
    return getattr(module, "__version__", "ok")


def main() -> int:
    print(f"Project root: {PROJECT_ROOT}")
    print(f"Python: {sys.executable}")
    print(f".venv exists: {(PROJECT_ROOT / '.venv' / 'bin' / 'python').exists()}")

    failures: dict[str, str] = {}
    print("\nCore package check:")
    for name in CORE_PACKAGES:
        try:
            print(f"  {name}: {version_of(name)}")
        except Exception as exc:
            failures[name] = str(exc)
            print(f"  {name}: FAILED -> {exc}")

    print("\nNotebook package check:")
    for name in NOTEBOOK_PACKAGES:
        try:
            print(f"  {name}: {version_of(name)}")
        except Exception as exc:
            failures[name] = str(exc)
            print(f"  {name}: FAILED -> {exc}")

    print("\nTorch / CUDA:")
    try:
        import torch

        print(f"  CUDA available: {torch.cuda.is_available()}")
        print(f"  CUDA device count: {torch.cuda.device_count()}")
        print(f"  CUDA version: {torch.version.cuda}")
        if torch.cuda.is_available():
            print(f"  CUDA device 0: {torch.cuda.get_device_name(0)}")
    except Exception as exc:
        failures["torch_cuda"] = str(exc)
        print(f"  torch CUDA check FAILED -> {exc}")

    print("\nDevices:")
    print(f"  Video devices: {sorted(glob.glob('/dev/video*')) or ['none']}")
    print(f"  USB capture indices: {usb_video_device_indices() or ['none']}")
    print(f"  Serial candidates: {serial_candidates() or ['none']}")
    print(f"  Calibration file exists: {(PROJECT_ROOT / '.jetcar_motor_calibration.json').exists()}")
    print(f"  Contour settings file exists: {(PROJECT_ROOT / '.contour_follow_settings.json').exists()}")
    print(f"  Default YOLO model cached locally: {os.path.exists(PROJECT_ROOT / 'yolo11n.pt')}")
    print("  Default YOLO note: yolo11n.pt can be auto-downloaded by Ultralytics if missing.")

    if failures:
        print("\nEnvironment check FAILED.")
        return 1

    print("\nEnvironment check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
