from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

project_root = Path(__file__).resolve().parents[1]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from jetcar.camera import open_camera, read_rgb_frame
from jetcar.yolo import (
    annotated_frame,
    detection_lines,
    load_yolo_model,
    predict_frame,
    resolve_yolo_model_path,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a small YOLO model on the JetCar camera feed.")
    parser.add_argument("--camera-source", default="usb", choices=["usb", "csi"])
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--warmup-frames", type=int, default=12)
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--max-det", type=int, default=20)
    parser.add_argument("--frames", type=int, default=1, help="Number of frames to process before exiting.")
    parser.add_argument("--show", action="store_true", help="Show an OpenCV preview window.")
    parser.add_argument("--delay-ms", type=int, default=1)
    parser.add_argument("--save-dir", default="", help="Optional directory for annotated JPG frames.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model_path = resolve_yolo_model_path(args.model, project_root=project_root)
    save_dir = Path(args.save_dir).expanduser() if args.save_dir else None
    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)

    print(f"Project root: {project_root}")
    print(f"Camera source: {args.camera_source}")
    print(f"YOLO model: {model_path}")

    model = load_yolo_model(model_path, project_root=project_root)
    cap = open_camera(
        source=args.camera_source,
        sensor_id=args.sensor_id,
        device_index=args.device_index,
        width=args.width,
        height=args.height,
        warmup_frames=args.warmup_frames,
    )

    try:
        for frame_index in range(max(args.frames, 1)):
            frame = read_rgb_frame(cap)
            result = predict_frame(
                model,
                frame,
                conf=args.conf,
                imgsz=args.imgsz,
                max_det=args.max_det,
            )
            overlay = annotated_frame(result)

            print(f"\nFrame {frame_index + 1}:")
            for line in detection_lines(result):
                print(f"  {line}")

            if save_dir is not None:
                save_path = save_dir / f"frame_{frame_index + 1:03d}.jpg"
                cv2.imwrite(str(save_path), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
                print(f"  saved: {save_path}")

            if args.show:
                cv2.imshow("JetCar YOLO Demo", cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
                key = cv2.waitKey(max(args.delay_ms, 1)) & 0xFF
                if key in {27, ord("q")}:
                    break
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
