from __future__ import annotations

from pathlib import Path

import pandas as pd
import torch
from PIL import Image
from torch import nn
from torch.utils.data import Dataset
from torchvision import transforms

from .vision import build_lane_mask


class PilotDataset(Dataset):
    """Dataset for image-to-steering regression."""

    def __init__(
        self,
        csv_path: str | Path,
        image_root: str | Path = ".",
        image_size: tuple[int, int] | int = 224,
        use_mask: bool = False,
        crop_top_ratio: float = 0.35,
        hsv_lower: tuple[int, int, int] = (35, 40, 40),
        hsv_upper: tuple[int, int, int] = (95, 255, 255),
        blur_kernel: int = 5,
        morph_kernel: int = 5,
    ):
        self.csv_path = Path(csv_path)
        self.image_root = Path(image_root)
        self.frame = pd.read_csv(self.csv_path)
        self.use_mask = use_mask
        self.crop_top_ratio = crop_top_ratio
        self.hsv_lower = hsv_lower
        self.hsv_upper = hsv_upper
        self.blur_kernel = blur_kernel
        self.morph_kernel = morph_kernel

        if isinstance(image_size, int):
            resize_shape = (image_size, image_size)
        else:
            resize_shape = image_size

        self.transform = transforms.Compose(
            [
                transforms.Resize(resize_shape),
                transforms.ToTensor(),
            ]
        )

    def __len__(self) -> int:
        return len(self.frame)

    def __getitem__(self, index: int):
        row = self.frame.iloc[index]
        image = Image.open(self.image_root / row["image_path"]).convert("RGB")
        if self.use_mask:
            image = build_lane_mask(
                image,
                crop_top_ratio=self.crop_top_ratio,
                hsv_lower=self.hsv_lower,
                hsv_upper=self.hsv_upper,
                blur_kernel=self.blur_kernel,
                morph_kernel=self.morph_kernel,
            )
        x = self.transform(image)
        y = torch.tensor([float(row["steering"])], dtype=torch.float32)
        return x, y


class SmallPilotNet(nn.Module):
    """A compact CNN that is easy to explain in a workshop."""

    def __init__(self, in_channels: int = 3) -> None:
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(in_channels, 16, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv2d(16, 24, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv2d(24, 32, kernel_size=3, stride=2),
            nn.ReLU(),
            nn.Conv2d(32, 48, kernel_size=3, stride=2),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((4, 4)),
        )
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(48 * 4 * 4, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Tanh(),
        )

    def forward(self, x):
        x = self.features(x)
        return self.head(x)
