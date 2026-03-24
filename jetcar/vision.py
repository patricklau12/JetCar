from __future__ import annotations

import cv2
import numpy as np
from PIL import Image


def build_lane_mask(
    image_rgb: Image.Image,
    crop_top_ratio: float = 0.35,
    hsv_lower: tuple[int, int, int] = (35, 40, 40),
    hsv_upper: tuple[int, int, int] = (95, 255, 255),
    blur_kernel: int = 5,
    morph_kernel: int = 5,
) -> Image.Image:
    frame = np.array(image_rgb)
    crop_start = int(frame.shape[0] * crop_top_ratio)
    roi = frame[crop_start:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(hsv_lower, dtype=np.uint8),
        np.array(hsv_upper, dtype=np.uint8),
    )

    if blur_kernel > 1:
        k = blur_kernel if blur_kernel % 2 == 1 else blur_kernel + 1
        mask = cv2.GaussianBlur(mask, (k, k), 0)
        _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

    if morph_kernel > 1:
        kernel = np.ones((morph_kernel, morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    full_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    full_mask[crop_start:, :] = mask
    return Image.fromarray(full_mask, mode="L")
