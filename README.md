# JetCar

JetCar is a beginner-friendly robot car project for NVIDIA Jetson Orin Nano.

The current workshop flow is not "open every notebook one by one." The normal path is:

1. Pull the repo
2. Install the Python environment
3. Launch the browser control panel
4. Check camera and motor
5. Calibrate the minimum motor speeds for the current floor
6. Run a line-following test

The repo now keeps only the beginner notebooks at the top level. Older notebook experiments are stored in `notebooks/archive/`.

## Start Here

If you want the shortest path, follow [docs/beginner-quickstart.md](/home/orin/JetCar/docs/beginner-quickstart.md).

## Beginner Workflow

1. Read [hardware-checklist.md](/home/orin/JetCar/docs/hardware-checklist.md).
2. Clone the repo, or `git pull` if you already have it.
3. Create the local Python environment and install packages.
4. Launch the browser teleop panel.
5. Confirm the camera feed works.
6. Confirm serial motor control works.
7. Calibrate `forward`, `back`, `left`, and `right` until the rover moves reliably on the current floor.
8. Tune the lane mask if needed.
9. Run the built-in line-following test from the browser page.

## Clone Or Update

First time:

```bash
cd /home/orin
git clone <YOUR_GITHUB_REPO_URL> JetCar
cd /home/orin/JetCar
```

Already installed:

```bash
cd /home/orin/JetCar
git pull
```

## Install Packages

```bash
cd /home/orin/JetCar
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -r requirements-notebook.txt
python -m pip install --force-reinstall --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 torch==2.8.0 torchvision==0.23.0
python -m pip install --force-reinstall "numpy<2"
python -m pip install -e .
python -m ipykernel install --user --name jetcar --display-name "Python (jetcar)"
```

## Run The Web Panel

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/teleop_server.py --host 0.0.0.0 --http-port 8765 --camera-source usb --port /dev/ttyTHS1
```

Then open:

- `http://127.0.0.1:8765` on the Jetson itself
- `http://JETSON_IP:8765` from another device on the same network

If you are using a CSI camera, change `--camera-source usb` to `--camera-source csi`.

## What To Do In The Web Panel

1. Put the rover on a stand first.
2. Check the status cards at the top.
3. Make sure `Camera` says ready.
4. Make sure `Serial` says ready.
5. Confirm the live RGB image updates.
6. Confirm the mask image updates.
7. Press the manual `Forward`, `Back`, `Left`, and `Right` buttons carefully.
8. Use the calibration buttons to save the smallest values that reliably move the rover on this floor.
9. Put the rover on the track.
10. In Mask Tuning, press `Pick Line`, click the line in the RGB image, then press `Pick Floor` and click the floor.
11. Adjust `Tolerance` only if needed.
12. Press `Auto Start` for a line-following test.
13. Press `Stop Auto` or `Stop` immediately if the rover behaves badly.

## Motor Calibration Rule

Treat motor calibration as required for every workshop venue.

Different floors have different friction. A speed that works on one floor may be too weak or too strong on another floor. The saved calibration values are stored in `.teleop_calibration.json` and are used as shared motion minimums by:

- manual driving in the browser page
- automatic line-following in the browser page
- `scripts/wave_rover_serial.py`

That means the recommended setup order is:

1. launch the web panel
2. check camera and serial
3. calibrate motion
4. start driving tests

## Optional Notebooks

The main top-level notebooks are now:

- [00_environment_check.ipynb](/home/orin/JetCar/notebooks/00_environment_check.ipynb) for safe environment and device checks
- [01_web_panel_intro.ipynb](/home/orin/JetCar/notebooks/01_web_panel_intro.ipynb) for the web-panel command, URLs, and usage notes
- [02_yolo_object_detection.ipynb](/home/orin/JetCar/notebooks/02_yolo_object_detection.ipynb) for a camera-based YOLO nano object-detection demo with default COCO labels

Older notebook experiments are kept in [archive/README.md](/home/orin/JetCar/notebooks/archive/README.md).

If you want Jupyter:

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
jupyter lab
```

## Project Layout

```text
JetCar/
├── docs/        beginner setup notes and workshop docs
├── jetcar/      shared Python helpers
├── notebooks/   beginner notebooks
│   └── archive/ older notebook experiments
├── data/        captured images and processed data
├── models/      trained models
├── scripts/     runnable scripts such as the web panel
├── requirements.txt
├── requirements-notebook.txt
└── pyproject.toml
```

## Waveshare WAVE ROVER Serial Control

The WAVE ROVER is controlled by sending JSON commands to its onboard ESP32 over serial. The Jetson should not drive the wheel motors directly from GPIO pins.

Example commands:

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/wave_rover_serial.py --port /dev/ttyTHS1 stop
python scripts/wave_rover_serial.py --port /dev/ttyTHS1 forward
python scripts/wave_rover_serial.py --port /dev/ttyTHS1 left
```

If you pass `--speed`, the saved calibration still acts as the minimum unless you also pass `--ignore-calibration`.

## Notes

- The virtual environment uses `--system-site-packages` so Jetson-provided packages like `cv2` stay visible.
- Jetson GPU-enabled PyTorch is JetPack-specific. For this project, use the JetPack 6 / CUDA 12.6 wheel index shown above.
- The browser panel is now the main preflight tool for workshops.

## Wiring Reminder

- Use the Jetson UART pins, not only the first 8 header pins.
- Common serial wiring is `pin 6` GND, `pin 8` TX, and `pin 10` RX.
- TX and RX must be crossed between the Jetson and the rover.
- Linux serial access often requires membership in the `dialout` group.
