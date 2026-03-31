# Notebook Chapters Guide

This page explains the purpose of each beginner notebook in `JetCar/notebooks/`.

It is written like a small wiki chapter guide:
- what problem each notebook solves
- what script it launches
- what files it reads and writes
- what the learner should understand before moving on

---

## Big picture

The beginner workflow is not random notebook clicking.

The intended order is:

1. `00_package_install_and_check.ipynb`
2. `01_motor_speed_tuning.ipynb`
3. `02_line_following.ipynb`
4. `03_line_following_with_default_yolo_detection.ipynb`
5. `04_line_following_with_stop_sign_action.ipynb`

Why this order matters:

- first make sure the Jetson environment works
- then make sure the car can move safely
- then make sure the camera-based line following works
- only after that add YOLO perception features

A lot of beginner frustration comes from trying YOLO or sign logic before the basic motor and camera pipeline is stable.

---

## Chapter 00 — Package install and system check

Notebook:
- `notebooks/00_package_install_and_check.ipynb`

### What this notebook is for

This is the "do not skip" notebook.

It checks whether the project environment is healthy before you touch rover motion.

### What it teaches

This chapter teaches that robotics projects fail for simple reasons first:
- wrong Python environment
- missing packages
- CUDA not visible
- camera device missing
- serial device missing

Before debugging algorithms, first debug the machine.

### What it checks

- repo location and folder structure
- `.venv` existence
- Python imports such as `numpy`, `cv2`, `torch`, `flask`, `serial`, `ultralytics`
- CUDA visibility through PyTorch
- camera devices like `/dev/video*`
- serial devices like `/dev/ttyTHS1`
- whether saved JSON configuration files already exist

### What success looks like

The learner should understand:
- what environment is being used
- whether GPU support is visible
- whether the system can even see the camera and serial port

Only after that should they continue.

---

## Chapter 01 — Motor speed tuning

Notebook:
- `notebooks/01_motor_speed_tuning.ipynb`

Launches:
- `scripts/jetcar_motor_calibration_panel.py`

Writes:
- `.jetcar_motor_calibration.json`

### What this notebook is for

This chapter calibrates motion presets for the rover.

It answers:
- how much left and right motor power is needed to move forward
- how much is needed to rotate left
- how much is needed to rotate right
- what short forward "nudge" should be used after turning

### What it teaches

This notebook teaches an important robotics idea:

**The algorithm cannot drive well if the motor commands themselves are not calibrated.**

The same numerical motor values can behave differently on different floors because friction changes.

### What the learner should understand

The rover is controlled by sending JSON serial commands with left and right motor values.

Examples of preset types:
- `forward`
- `rotate_left`
- `rotate_right`
- `forward_nudge`

The learner should understand that these are not abstract settings.
They are the low-level motion building blocks used later by the auto-drive code.

### What success looks like

The learner can:
- run a short raw test safely
- stop the rover immediately
- save reliable presets
- explain why calibration must be repeated when the floor changes

---

## Chapter 02 — Line following

Notebook:
- `notebooks/02_line_following.ipynb`

Launches:
- `scripts/teleop_server_article_contour_panel.py`

Reads:
- `.jetcar_motor_calibration.json`

Writes:
- `.contour_follow_settings.json`

### What this notebook is for

This is the main classical computer vision chapter.

The rover uses the lower part of the camera image to find a line and decide whether to go straight, rotate left, or rotate right.

### What it teaches

This chapter teaches the core perception-and-control loop:

1. get camera image
2. crop the lower region of interest
3. convert to grayscale
4. threshold into a mask
5. select the best contour
6. estimate line direction and horizontal offset
7. convert that analysis into motion commands

This chapter is the best place for a beginner to understand the actual control logic.

### What the learner should understand

The learner should be able to explain:
- why the top part of the frame is ignored
- why a binary mask is easier to analyze than the raw RGB image
- what a contour is
- why the biggest contour is not always the best contour
- why the car needs both angle and left/right shift information

### What success looks like

The learner can:
- see the live RGB view
- see the grayscale ROI and mask
- see the chosen contour overlay
- explain why the rover turned left or right on a given frame

---

## Chapter 03 — Line following with default YOLO detection

Notebook:
- `notebooks/03_line_following_with_default_yolo_detection.ipynb`

Launches:
- `scripts/teleop_server_article_contour_panel_with_yolo.py`

Typical model:
- `yolo11n.pt`

### What this notebook is for

This chapter adds general object detection on top of the existing line-following pipeline.

### What it teaches

This notebook teaches that perception modules can be layered.

The contour detector still controls steering.
YOLO does not replace the line-following logic here.
Instead, YOLO adds a second perception stream using the full RGB frame.

### What the learner should understand

The learner should understand the difference between:
- the **control vision path**: lower ROI contour logic for steering
- the **awareness vision path**: full-frame YOLO detection for object labels and boxes

This is an important design concept in robotics systems.
Different modules can observe the same camera for different purposes.

### What success looks like

The learner can explain:
- why the car can still follow the line even if YOLO is disabled
- why YOLO boxes are drawn on the full frame while the contour logic only uses the lower ROI

---

## Chapter 04 — Line following with stop-sign action

Notebook:
- `notebooks/04_line_following_with_stop_sign_action.ipynb`

Launches:
- `scripts/jetcar_contour_sign_panel.py`

Typical model:
- `models/traffic_sign_detector.pt`

### What this notebook is for

This chapter adds a sign-triggered behavior on top of line following.

The rover still follows the line, but when a stable stop sign is detected, the rover pauses.

### What it teaches

This chapter teaches event-driven behavior.

Instead of only reacting to the line, the rover reacts to a recognized traffic sign.

This shows a beginner how a robotics controller can combine:
- continuous control from line following
- discrete events from object/sign detection

### What the learner should understand

The learner should understand:
- why one-frame detections are noisy
- why stable voting or persistence checks are useful
- why a stop action should trigger once and then re-arm later, instead of stopping forever

### What success looks like

The learner can explain:
- how the rover follows the line normally
- how a detected sign temporarily overrides or modifies normal behavior
- why detection stability matters before triggering actions

---

## What each chapter should leave the learner with

After `00`:
- "My system is ready."

After `01`:
- "I know how the rover is driven at the motor-command level."

After `02`:
- "I understand how the line-following calculation works."

After `03`:
- "I understand how a second perception model can be added without replacing the control loop."

After `04`:
- "I understand how perception can trigger a behavior change like a stop action."

---

## Recommended teaching rule

If you are using this repo in a class or workshop, do not let beginners jump directly to notebook `04`.

The learning path should be:

- machine readiness first
- motor calibration second
- contour logic third
- YOLO extras fourth

That order matches how the project is built and how the rover actually works.
