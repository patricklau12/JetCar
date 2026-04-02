from __future__ import annotations

import glob
import time
from typing import Any

import cv2
import numpy as np


class _GstCameraCapture:
    """Minimal capture wrapper for CSI cameras when OpenCV lacks GStreamer."""

    def __init__(self, pipeline: str, warmup_frames: int = 0) -> None:
        try:
            import gi

            gi.require_version("Gst", "1.0")
            gi.require_version("GstApp", "1.0")
            from gi.repository import Gst
        except Exception as exc:  # pragma: no cover - hardware-specific dependency
            raise RuntimeError(
                "Could not import Python GStreamer bindings for CSI camera fallback."
            ) from exc

        Gst.init(None)
        self._gst = Gst
        self._pipeline = Gst.parse_launch(pipeline)
        self._appsink = self._pipeline.get_by_name("appsink")
        if self._appsink is None:
            self._pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("CSI fallback pipeline did not create an appsink.")

        self._pipeline.set_state(Gst.State.PLAYING)
        try:
            self._prime()
            for _ in range(max(warmup_frames, 0)):
                self.read()
                time.sleep(0.03)
        except Exception:
            self.release()
            raise

    def _prime(self) -> None:
        sample = self._appsink.emit("try-pull-sample", 5 * self._gst.SECOND)
        if sample is None:
            raise RuntimeError("CSI camera fallback did not receive a frame from GStreamer.")

    def isOpened(self) -> bool:
        return self._pipeline is not None

    def read(self) -> tuple[bool, np.ndarray | None]:
        sample = self._appsink.emit("try-pull-sample", 5 * self._gst.SECOND)
        if sample is None:
            return False, None

        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")
        buffer = sample.get_buffer()
        ok, map_info = buffer.map(self._gst.MapFlags.READ)
        if not ok:
            return False, None

        try:
            frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3)).copy()
        finally:
            buffer.unmap(map_info)
        return True, frame

    def release(self) -> None:
        if self._pipeline is None:
            return
        self._pipeline.set_state(self._gst.State.NULL)
        self._pipeline = None
        self._appsink = None


def _opencv_has_gstreamer() -> bool:
    return "GStreamer:                   YES" in cv2.getBuildInformation()


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


def gst_appsink_pipeline(
    sensor_id: int = 0,
    capture_width: int = 1280,
    capture_height: int = 720,
    display_width: int = 320,
    display_height: int = 180,
    framerate: int = 30,
    flip_method: int = 0,
) -> str:
    """Return a CSI pipeline string for Python GStreamer appsink capture."""
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=%d, height=%d, "
        "format=NV12, framerate=%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=%d, height=%d, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink name=appsink drop=true max-buffers=1 sync=false emit-signals=false"
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
) -> Any:
    """Open an RGB camera from either the Jetson CSI bus or a USB webcam."""
    source = source.lower()

    if source == "csi":
        if _opencv_has_gstreamer():
            pipeline = gstreamer_pipeline(
                sensor_id=sensor_id,
                capture_width=width,
                capture_height=height,
                display_width=width,
                display_height=height,
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                for _ in range(max(warmup_frames, 0)):
                    cap.read()
                    time.sleep(0.03)
                return cap

        try:
            return _GstCameraCapture(
                gst_appsink_pipeline(
                    sensor_id=sensor_id,
                    capture_width=width,
                    capture_height=height,
                    display_width=width,
                    display_height=height,
                ),
                warmup_frames=warmup_frames,
            )
        except Exception as exc:
            raise RuntimeError(
                "Could not open CSI RGB camera. "
                f"sensor_id={sensor_id}, size={width}x{height}. "
                "OpenCV GStreamer support is unavailable, and the Python GStreamer fallback also failed. "
                "Check the ribbon cable orientation, camera support package, and nvargus-daemon status."
            ) from exc

    if source == "usb":
        candidates = [
            cv2.VideoCapture(device_index, cv2.CAP_V4L2),
            cv2.VideoCapture(f"/dev/video{device_index}", cv2.CAP_V4L2),
        ]
        cap = None
        for candidate in candidates:
            candidate.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            candidate.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            candidate.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            if candidate.isOpened():
                cap = candidate
                break
            candidate.release()
        if cap is None:
            video_devices = ", ".join(sorted(glob.glob("/dev/video*"))) or "none"
            raise RuntimeError(
                "Could not open USB RGB camera. "
                f"Tried /dev/video{device_index} and index {device_index}; detected video devices: {video_devices}."
            )
        for _ in range(max(warmup_frames, 0)):
            cap.read()
            time.sleep(0.10)
        return cap

    raise ValueError("source must be either 'csi' or 'usb'")


def read_rgb_frame(cap: Any):
    """Read one frame and convert BGR to RGB for notebook display."""
    ok, frame_bgr = cap.read()
    if not ok:
        raise RuntimeError("Camera read failed.")
    return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
