from __future__ import annotations

from collections import Counter
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO

DEFAULT_YOLO_MODEL = "yolo11n.pt"


def preferred_yolo_device() -> int | str:
    """Prefer CUDA on Jetson when it is available."""
    return 0 if torch.cuda.is_available() else "cpu"


def resolve_yolo_model_path(model: str | Path = DEFAULT_YOLO_MODEL, project_root: Path | None = None) -> str:
    """Resolve a YOLO weight file from common workshop locations."""
    requested = Path(model).expanduser()
    if requested.is_file():
        return str(requested)
    if requested.is_absolute():
        return str(requested)

    search_roots: list[Path] = []
    if project_root is not None:
        search_roots.extend([project_root, project_root / "models", project_root.parent])
    search_roots.extend([Path.home(), Path.home() / "models"])

    for root in search_roots:
        candidate = root / requested
        if candidate.is_file():
            return str(candidate)

    return str(requested)


def load_yolo_model(model: str | Path = DEFAULT_YOLO_MODEL, project_root: Path | None = None) -> YOLO:
    """Load a YOLO model, falling back to Ultralytics auto-download when needed."""
    return YOLO(resolve_yolo_model_path(model, project_root=project_root))


def predict_frame(
    model: YOLO,
    frame_rgb,
    *,
    conf: float = 0.25,
    imgsz: int = 640,
    max_det: int = 20,
    device: int | str | None = None,
):
    """Run YOLO on an RGB frame and return the first result."""
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    results = model.predict(
        source=frame_bgr,
        conf=conf,
        imgsz=imgsz,
        max_det=max_det,
        device=preferred_yolo_device() if device is None else device,
        verbose=False,
    )
    return results[0]


def annotated_frame(result) -> object:
    """Convert YOLO's BGR annotation image back to RGB for notebooks."""
    return cv2.cvtColor(result.plot(), cv2.COLOR_BGR2RGB)


def detection_lines(result) -> list[str]:
    """Build short human-readable detection summaries."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return ["No objects detected."]

    names = result.names
    counts: Counter[str] = Counter()
    best_confidence: dict[str, float] = {}

    for cls_id, conf in zip(boxes.cls.tolist(), boxes.conf.tolist()):
        label = names[int(cls_id)]
        counts[label] += 1
        best_confidence[label] = max(best_confidence.get(label, 0.0), float(conf))

    return [
        f"{label}: {counts[label]} (best {best_confidence[label]:.2f})"
        for label in sorted(counts)
    ]
