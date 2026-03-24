"""JetCar workshop helpers."""

from .camera import gstreamer_pipeline, open_camera, read_rgb_frame
from .motion import (
    CALIBRATION_DEFAULTS,
    CALIBRATION_PATH,
    MotionCalibration,
    apply_drive_calibration,
    minimum_directional_speed,
    resolve_directional_speed,
)
from .vision import build_lane_mask
from .yolo import (
    DEFAULT_YOLO_MODEL,
    annotated_frame,
    detection_lines,
    load_yolo_model,
    preferred_yolo_device,
    predict_frame,
    resolve_yolo_model_path,
)


def __getattr__(name: str):
    if name in {"PilotDataset", "SmallPilotNet"}:
        from .training import PilotDataset, SmallPilotNet

        return {"PilotDataset": PilotDataset, "SmallPilotNet": SmallPilotNet}[name]
    raise AttributeError(f"module 'jetcar' has no attribute {name}")

__all__ = [
    "gstreamer_pipeline",
    "open_camera",
    "read_rgb_frame",
    "build_lane_mask",
    "DEFAULT_YOLO_MODEL",
    "preferred_yolo_device",
    "resolve_yolo_model_path",
    "load_yolo_model",
    "predict_frame",
    "annotated_frame",
    "detection_lines",
    "CALIBRATION_DEFAULTS",
    "CALIBRATION_PATH",
    "MotionCalibration",
    "apply_drive_calibration",
    "minimum_directional_speed",
    "resolve_directional_speed",
    "PilotDataset",
    "SmallPilotNet",
]
