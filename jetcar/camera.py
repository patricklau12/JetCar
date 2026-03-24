from __future__ import annotations

import time

import cv2


def gstreamer_pipeline(
    sensor_id: int = 0,
    capture_width: int = 1280,
    capture_height: int = 720,
    display_width: int = 320,
    display_height: int = 180,
    framerate: int = 30,
    flip_method: int = 0,
) -> str:
    """Return a Jetson CSI camera pipeline string for OpenCV."""
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true sync=false"
    ) % (
        sensor_id,
        capture_width,
        capture_height,
        framerate,
        flip_method,
        display_width,
        display_height,
    )


def open_camera(
    source: str = "csi",
    sensor_id: int = 0,
    device_index: int = 0,
    width: int = 1280,
    height: int = 720,
    warmup_frames: int = 12,
) -> cv2.VideoCapture:
    """Open an RGB camera from either the Jetson CSI bus or a USB webcam."""
    if source == "csi":
        pipeline = gstreamer_pipeline(
            sensor_id=sensor_id,
            capture_width=width,
            capture_height=height,
            display_width=width,
            display_height=height,
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError("Could not open CSI RGB camera. Check cable, sensor, and nvargus service.")
        for _ in range(max(warmup_frames, 0)):
            cap.read()
            time.sleep(0.03)
        return cap

    if source == "usb":
        cap = cv2.VideoCapture(f"/dev/video{device_index}", cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if not cap.isOpened():
            raise RuntimeError("Could not open USB RGB camera. Check the device path and USB connection.")
        for _ in range(max(warmup_frames, 0)):
            cap.read()
            time.sleep(0.10)
        return cap

    raise ValueError("source must be either 'csi' or 'usb'")


def read_rgb_frame(cap: cv2.VideoCapture):
    """Read one frame and convert BGR to RGB for notebook display."""
    ok, frame_bgr = cap.read()
    if not ok:
        raise RuntimeError("Camera read failed.")
    return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
