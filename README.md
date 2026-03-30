# JetCar

JetCar is a beginner-friendly robot car project for NVIDIA Jetson Orin Nano.

The current workshop flow is not "open every notebook one by one." The normal path is:

1. Pull the repo
2. Install the Python environment
3. Run the environment check notebook
4. Launch the motor calibration panel
5. Save the floor-specific forward and turn presets
6. Launch the contour-follow panel
7. Run the line-following test

The repo now keeps only the beginner notebooks at the top level. Older notebook experiments are stored in `notebooks/archive/`.

## Start Here

If you want the shortest path, follow [docs/beginner-quickstart.md](/home/orin/JetCar/docs/beginner-quickstart.md).

## Beginner Workflow

1. Read [hardware-checklist.md](/home/orin/JetCar/docs/hardware-checklist.md).
2. Clone the repo, or `git pull` if you already have it.
3. Create the local Python environment and install packages.
4. Run [00_environment_check.ipynb](/home/orin/JetCar/notebooks/00_environment_check.ipynb).
5. Launch [01_motor_calibration_panel.ipynb](/home/orin/JetCar/notebooks/01_motor_calibration_panel.ipynb).
6. Confirm serial motor control works.
7. Calibrate `forward`, `rotate_left`, `rotate_right`, and `forward_nudge` until the rover moves reliably on the current floor.
8. Launch [02_article_contour_follow.ipynb](/home/orin/JetCar/notebooks/02_article_contour_follow.ipynb).
9. Confirm the camera feed and contour overlay work.
10. Run the contour-based line-following test.

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

## Run The Motor Calibration Panel

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/jetcar_motor_calibration_panel.py --host 0.0.0.0 --http-port 8766 --port /dev/ttyTHS1
```

Then open:

- `http://127.0.0.1:8766` on the Jetson itself
- `http://JETSON_IP:8766` from another device on the same network

This panel is only for motor preset tuning. It writes `.jetcar_motor_calibration.json`.

## Run The Contour Follow Panel

USB camera:

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/teleop_server_article_contour_panel.py --host 0.0.0.0 --http-port 8765 --camera-source usb --port /dev/ttyTHS1
```

CSI camera:

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/teleop_server_article_contour_panel.py --host 0.0.0.0 --http-port 8765 --camera-source csi --port /dev/ttyTHS1
```

Then open:

- `http://127.0.0.1:8765` on the Jetson itself
- `http://JETSON_IP:8765` from another device on the same network

## What To Do In The Calibration Panel

1. Put the rover on a stand first.
2. Check the status cards at the top.
3. Make sure `Serial` says ready.
4. Use `Run Raw Test` for short safe pulses first.
5. Save a reliable `forward` preset.
6. Save `rotate_left` and `rotate_right` so the rover turns cleanly.
7. Save `forward_nudge` for the short move after a turn.
8. Confirm the JSON preview looks correct.

## What To Do In The Contour Follow Panel

1. Start with the rover on a stand.
2. Make sure `Camera` says ready.
3. Make sure `Serial` says ready.
4. Confirm the live RGB, grayscale ROI, mask, and contour overlay all update.
5. Press `Test Forward`, `Test Rotate Left`, `Test Rotate Right`, and `Test Nudge` carefully.
6. Put the rover on the track.
7. Adjust the detection sliders only if the contour or mask looks unstable.
8. Press `Auto Start` for the contour-based line-following test.
9. Press `Stop Auto` or `Stop Motors` immediately if the rover behaves badly.

## Motor Calibration Rule

Treat motor calibration as required for every workshop venue.

Different floors have different friction. A speed that works on one floor may be too weak or too strong on another floor. The saved calibration values are stored in `.jetcar_motor_calibration.json` and are reused by:

- `scripts/jetcar_motor_calibration_panel.py`
- `scripts/teleop_server_article_contour_panel.py`

That means the recommended setup order is:

1. run `00_environment_check.ipynb`
2. launch the motor calibration panel
3. save motion presets
4. launch the contour follow panel
5. start driving tests

## Logged Auto-Follow Rounds

If you want a repeatable tuning loop, keep the teleop server running and log one round at a time:

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/auto_follow_round.py --label first-pass --duration-s 4
```

Each run saves before/after RGB and mask frames, status snapshots, and a timeline under `runs/auto_follow_rounds/`.

## Manual-Like Autotune

If you want the Jetson to do the short correction pulses itself, while you watch the live page:

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/manual_like_autotune.py --steps 16 --label floor-test
```

Recommended flow:

1. launch `scripts/teleop_server.py`
2. open `http://127.0.0.1:8765` so you can watch the live RGB and mask view
3. place the rover on the line
4. run `manual_like_autotune.py`
5. let it make short pulse corrections and save the best result to `.teleop_auto_settings.json`

The tuner writes logs and captured frames under `runs/manual_autotune/`.

## Optional Notebooks

The main top-level notebooks are now:

- [00_environment_check.ipynb](/home/orin/JetCar/notebooks/00_environment_check.ipynb) for safe environment and device checks
- [01_motor_calibration_panel.ipynb](/home/orin/JetCar/notebooks/01_motor_calibration_panel.ipynb) for launching the motor calibration panel and saving motion presets
- [02_article_contour_follow.ipynb](/home/orin/JetCar/notebooks/02_article_contour_follow.ipynb) for launching the new contour-based article driving panel

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
