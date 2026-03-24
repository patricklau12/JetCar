# Beginner Quickstart

This guide is for the simplest workshop setup.

Goal:

1. get the repo
2. install the environment
3. open the web panel
4. check camera and motor
5. calibrate motor speed for the floor
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
```

## 4. Launch The Web Panel

USB camera:

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/teleop_server.py --host 0.0.0.0 --http-port 8765 --camera-source usb --port /dev/ttyTHS1
```

CSI camera:

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/teleop_server.py --host 0.0.0.0 --http-port 8765 --camera-source csi --port /dev/ttyTHS1
```

Then open `http://127.0.0.1:8765`.

## 5. Check Camera And Motor

At the top of the page:

- `Server` should become ready
- `Camera` should become ready
- `Serial` should become ready

Then check:

1. the RGB image is live
2. the mask image is live
3. `Forward`, `Back`, `Left`, and `Right` all respond
4. `Stop` works immediately

If `Serial` is unavailable:

- check UART wiring
- check the serial port path
- check Linux `dialout` access
- restart the server

## 6. Calibrate The Motor Speeds

Do this every time you change venue or floor type.

Why:

- smooth tile, rough tile, carpet, foam mat, and wood all need different minimum motor power

How:

1. keep the rover on a stand first
2. press `Cal Fwd`
3. use `Next Step` until the rover just starts moving reliably
4. press `Save Moved`
5. repeat for `Cal Back`, `Cal Left`, and `Cal Right`

The saved values are written to `.teleop_calibration.json`.

Those saved values are then reused by:

- manual driving in the web panel
- auto line following in the web panel
- `scripts/wave_rover_serial.py`

## 7. Run A Line-Following Test

1. put the rover on the actual track
2. press `Pick Line` and click the line in the RGB image
3. press `Pick Floor` and click the floor in the RGB image
4. make sure the line shows clearly in the mask preview
5. if needed, adjust `Tolerance`
6. press `Auto Start`
7. watch closely
8. press `Stop Auto` or `Stop` if the rover drifts, spins, or loses the line

## 8. What To Do If It Fails

If the camera image is black:

- wait for warmup
- confirm the correct camera source
- confirm the camera is seated properly

If the mask is bad:

- change lighting
- adjust mask controls
- make sure the track color stands out from the floor

If the rover does not move:

- recalibrate
- check battery level
- check serial wiring

If the rover moves too hard:

- reset calibration and save smaller values
- keep testing on the stand first

## 9. Optional Next Steps

After the beginner flow is working, you can explore:

- [README.md](/home/orin/JetCar/README.md)
- [00_environment_check.ipynb](/home/orin/JetCar/notebooks/00_environment_check.ipynb)
- [01_web_panel_intro.ipynb](/home/orin/JetCar/notebooks/01_web_panel_intro.ipynb)
- [archive/README.md](/home/orin/JetCar/notebooks/archive/README.md)
