# JetPack Status And Upgrade Notes

This Jetson was checked on **2026-03-19**.

## Current Board Status

- Jetson Linux: `36.4.7`
- Kernel: `5.15.148-tegra`
- NVIDIA APT channel: `r36.4`
- `nvidia-jetpack` meta package: not installed

The machine already has the core NVIDIA Jetson packages installed directly, including CUDA, TensorRT, VPI, multimedia, camera, and kernel packages. In practice, this means the board is usable right now for JetCar development.

## Latest Official Production Release

NVIDIA's current production release for Jetson Orin Nano is **JetPack 6.2.2**, which corresponds to the **Jetson Linux 36.5** software train.

## What This Means

- The board is newer than the `36.4.4 / JetPack 6.2.1` snapshot.
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
