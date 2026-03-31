# Beginner Quickstart

This guide is for the simplest workshop setup.

Goal:

1. get the repo
2. install the environment
3. run the environment check notebook
4. calibrate motor motion
5. launch the contour-follow panel
6. run a line-following test

## 1. Safety First

- Put the rover on a stand before any motor test.
- Keep one hand ready near the stop button.
- Do not start on the floor until camera, serial, and calibration are already working.

## 2. Get The Repo

If this is your first setup:

```bash
cd /home/orin
git clone <YOUR_GITHUB_REPO_URL> JetCar
cd /home/orin/JetCar
```

If the folder already exists:

```bash
cd /home/orin/JetCar
git pull
```

## 3. Install The Environment

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

## 4. Run The Environment Check Notebook

Open [00_package_install_and_check.ipynb](/home/orin/JetCar/notebooks/00_package_install_and_check.ipynb) in Jupyter and run the cells.

That notebook gives you:

- the exact install commands if `.venv` is missing or broken
- import checks for the main Python packages
- CUDA and Torch visibility checks
- camera and serial device checks

## 5. Launch The Motor Calibration Panel

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/jetcar_motor_calibration_panel.py --host 0.0.0.0 --http-port 8766 --port /dev/ttyTHS1
```

Then open `http://127.0.0.1:8766`.

You can also launch the same panel from [01_motor_speed_tuning.ipynb](/home/orin/JetCar/notebooks/01_motor_speed_tuning.ipynb).

## 6. Check Serial And Save Calibration Presets

At the top of the page:

- `Server` should become ready
- `Serial` should become ready

Then check:

1. `Run Raw Test` works with small safe pulses
2. `Stop` works immediately
3. `forward` is saved at a reliable value
4. `rotate_left` and `rotate_right` are saved at reliable values
5. `forward_nudge` is saved for the short post-turn move

If `Serial` is unavailable:

- check UART wiring
- check the serial port path
- check Linux `dialout` access
- restart the panel

The saved values are written to `.jetcar_motor_calibration.json`.

## 7. Launch The Contour Follow Panel

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

Then open `http://127.0.0.1:8765`.

You can also launch the same panel from [02_line_following.ipynb](/home/orin/JetCar/notebooks/02_line_following.ipynb).

## 8. Check Camera And Start Driving

At the top of the page:

- `Server` should become ready
- `Camera` should become ready
- `Serial` should become ready

Then check:

1. the RGB image is live
2. the grayscale ROI, mask, and contour overlay are live
3. `Test Forward`, `Test Rotate Left`, `Test Rotate Right`, and `Test Nudge` all respond
4. `Stop Motors` works immediately

## 9. Run A Line-Following Test

1. put the rover on the actual track
2. make sure the line shows clearly in the contour overlay and mask preview
3. if needed, adjust the detection sliders
4. confirm the saved turn presets still feel correct
5. press `Auto Start`
6. watch closely
7. press `Stop Auto` or `Stop Motors` if the rover drifts, spins, or loses the line

## 10. What To Do If It Fails

If the camera image is black:

- wait for warmup
- confirm the correct camera source
- confirm the camera is seated properly

If the contour or mask is bad:

- change lighting
- adjust the contour detection controls
- make sure the track color stands out from the floor

If the rover does not move:

- recalibrate
- check battery level
- check serial wiring

If the rover moves too hard:

- reset calibration and save smaller values
- keep testing on the stand first

## 11. Optional Next Steps

After the beginner flow is working, you can explore:

- [README.md](/home/orin/JetCar/README.md)
- [00_package_install_and_check.ipynb](/home/orin/JetCar/notebooks/00_package_install_and_check.ipynb)
- [01_motor_speed_tuning.ipynb](/home/orin/JetCar/notebooks/01_motor_speed_tuning.ipynb)
- [02_line_following.ipynb](/home/orin/JetCar/notebooks/02_line_following.ipynb)
- [03_line_following_with_default_yolo_detection.ipynb](/home/orin/JetCar/notebooks/03_line_following_with_default_yolo_detection.ipynb)
- [04_line_following_with_stop_sign_action.ipynb](/home/orin/JetCar/notebooks/04_line_following_with_stop_sign_action.ipynb)
- [archive/README.md](/home/orin/JetCar/notebooks/archive/README.md)
