# Command Workflow

This is the main project guide after the Jetson has completed first boot.

This is the simplest way to use JetCar on the Jetson without relying on Jupyter notebooks.

The main runtime scripts now support:
- `--port auto` to detect the rover serial connection over USB or Jetson GPIO UART
- `--camera-source auto` to try CSI first and fall back to a working USB camera

That means the normal user command does not need to mention:
- a specific serial path like `/dev/ttyUSB0` or `/dev/ttyTHS1`
- a specific camera type like `usb` or `csi`
- a specific USB camera index unless you want to force one manually

Use one project environment only:
- `/home/orin/JetCar/.venv`

Use the steps below in order.

## Chapter 00. Environment And Device Check

Install once:

```bash
cd /home/orin/JetCar
bash scripts/install_jetcar.sh
```

Then check:

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/check_jetcar_env.py
```

What success looks like:
- you see at least one serial candidate
- you see a camera device if you expect one
- the environment check passes

## Chapter 01. Motor Speed Tuning

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/jetcar_motor_calibration_panel.py \
  --port auto \
  --http-port 8766
```

Open:

```text
http://127.0.0.1:8766
```

This writes:
- `.jetcar_motor_calibration.json`

## Chapter 02. Line Following

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/teleop_server_article_contour_panel.py \
  --port auto \
  --camera-source auto \
  --http-port 8765
```

Open:

```text
http://127.0.0.1:8765
```

This reads:
- `.jetcar_motor_calibration.json`

This writes:
- `.contour_follow_settings.json`

## Chapter 03. Line Following With Default YOLO

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/teleop_server_article_contour_panel_with_yolo.py \
  --port auto \
  --camera-source auto \
  --http-port 8765
```

Open:

```text
http://127.0.0.1:8765
```

Default model:
- `yolo11n.pt`

If `yolo11n.pt` is not already in the repo folder, Ultralytics can download it automatically on first use.

## Chapter 04. Line Following With Stop Sign Action

```bash
cd /home/orin/JetCar
source /home/orin/JetCar/.venv/bin/activate
python scripts/jetcar_contour_sign_panel.py \
  --port auto \
  --camera-source auto \
  --http-port 8765
```

Open:

```text
http://127.0.0.1:8765
```

Default model:
- `models/traffic_sign_detector.pt`

Unlike `yolo11n.pt`, this stop-sign model is a repo asset and is expected to exist locally.

## Tips

- If a page says the camera is busy, stop the earlier panel before starting another one.
- On your current Jetson, `auto` will pause for a few seconds while CSI fails, then it will fall back to USB camera if one is available.
- For the main workflow, the user should normally leave `--port auto` and `--camera-source auto` unchanged.
- If you want to force USB camera instead of auto:

```bash
python scripts/teleop_server_article_contour_panel_with_yolo.py \
  --port auto \
  --camera-source usb \
  --device-index 1 \
  --http-port 8765
```

- If you want to force CSI camera instead of auto:

```bash
python scripts/teleop_server_article_contour_panel_with_yolo.py \
  --port auto \
  --camera-source csi \
  --sensor-id 0 \
  --http-port 8765
```

- If you need to stop old panel processes:

```bash
pkill -f jetcar_motor_calibration_panel
pkill -f teleop_server_article_contour_panel
pkill -f teleop_server_article_contour_panel_with_yolo
pkill -f jetcar_contour_sign_panel
```
