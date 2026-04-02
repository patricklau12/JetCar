# JetPack Status And Upgrade Notes

This Jetson was checked on **2026-04-02**.

## Current Board Status

- Jetson Linux / L4T: `36.4.7`
- Kernel: `5.15.148-tegra`
- NVIDIA APT channel: `r36.4`
- `nvidia-jetpack` meta package: not installed
- `nvidia-jetpack` candidate on this channel: `6.2.1+b38`
- Python: `3.10.12`
- Project venv: `/home/orin/JetCar/.venv`
- Venv mode: `--system-site-packages`
- Torch in the project venv: `2.8.0`
- Torchvision in the project venv: `0.23.0`
- CUDA seen by Torch: `12.6`
- Current power mode from `nvpmodel -q`: `15W`
- CSI camera service: `nvargus-daemon` is `enabled` and `active`
- Current serial device: `/dev/ttyTHS1`
- Current USB camera device: `/dev/video0`

The machine already has the core NVIDIA Jetson packages installed directly, including CUDA, TensorRT, VPI, multimedia, camera, and kernel packages. In practice, this means the board is usable right now for JetCar development.

## Important Reproducibility Notes

A new Jetson that only clones the repo will not match this board exactly.

The current user relies on these access groups:

- `dialout` for `/dev/ttyTHS1`
- `video` for `/dev/video0`
- `render`, `i2c`, and `gpio` are also present on this board for Jetson development tasks

These local runtime files are ignored by git and must be copied or regenerated on a new board:

- `.jetcar_motor_calibration.json`
- `.teleop_auto_settings.json`
- `yolo11n.pt`

If you want "same behavior" instead of only "same code", include those files in your handoff plan or repeat the calibration and tuning flow on the target board.

The repo `requirements.txt` and `requirements-notebook.txt` files are currently compatibility-oriented because they use `>=` ranges for most packages. That is fine for a workshop setup, but it is not an exact lockfile.

## Current Python Package Snapshot

These are the package versions currently active in `/home/orin/JetCar/.venv`:

- `Flask==3.1.2`
- `ipykernel==7.2.0`
- `ipywidgets==8.1.8`
- `jupyter_server==2.17.0`
- `jupyterlab==4.5.6`
- `matplotlib==3.10.8`
- `numpy==1.26.4`
- `pandas==2.3.3`
- `Pillow==12.1.1`
- `pyserial==3.5`
- `torch==2.8.0`
- `torchvision==0.23.0`
- `tqdm==4.67.3`
- `ultralytics==8.3.193`

OpenCV is also importable on this board because the venv uses `--system-site-packages`, so a new Jetson should match the JetPack base image closely as well as the pip installs above.

## Latest Official Production Release

NVIDIA's JetPack archive shows **JetPack 6.2.2** for Jetson Orin Nano on **L4T 36.5.0**, while **JetPack 6.2.1** maps to **L4T 36.4.4**.

## What This Means

- The board is on the `36.4.x` software train.
- Inference: the closest JetPack family label is still `JetPack 6.2.1`, but this board has newer `36.4.7` patched L4T packages than the original `36.4.4` release snapshot.
- The board is still one release train behind `36.5 / JetPack 6.2.2`.
- A full move to `36.5` requires changing the NVIDIA APT source from `r36.4` to `r36.5`, upgrading packages, and rebooting.

## Current NVIDIA APT Source

```text
deb https://repo.download.nvidia.com/jetson/common r36.4 main
deb https://repo.download.nvidia.com/jetson/t234 r36.4 main
deb https://repo.download.nvidia.com/jetson/ffmpeg r36.4 main
```

## Upgrade Commands

These commands require sudo access:

```bash
sudo sed -i 's/r36.4/r36.5/g' /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
sudo apt update
sudo apt dist-upgrade
sudo apt install nvidia-jetpack
sudo reboot
```

## Verify After Reboot

```bash
cat /etc/nv_tegra_release
apt-cache policy nvidia-l4t-core nvidia-jetpack
```

## Recommendation For The Workshop

If the workshop is soon, it is reasonable to keep the current `36.4.7` setup for now because the camera and accelerated stack are already present. If you want the board aligned to NVIDIA's latest official release before students use it, run the `36.5` upgrade on a maintenance window and verify camera, Torch, and notebooks again after reboot.
