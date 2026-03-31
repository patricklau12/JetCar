# Notebook Chapters for Students

This page is the easy-reading version of the notebook guide.

Use it if you are new to Jetson, robot cars, or computer vision.

The goal is simple:
- know which notebook to open
- know why you are opening it
- know what you should learn from it
- know when you are ready to move to the next one

---

# Start here: what this repo is really doing

This repo teaches a robot car to do three things:

1. move correctly
2. see a line on the floor
3. react to extra things like objects or a stop sign

That means the learning order must also be simple:

1. check the machine
2. check the motors
3. check the camera and line following
4. add extra perception

Do **not** start with the stop-sign notebook.
If the car cannot already move and follow a line, the stop-sign part is not the real problem.

---

# The notebook order

Use the notebooks in this order:

1. `00_package_install_and_check.ipynb`
2. `01_motor_speed_tuning.ipynb`
3. `02_line_following.ipynb`
4. `03_line_following_with_default_yolo_detection.ipynb`
5. `04_line_following_with_stop_sign_action.ipynb`

Think of them as five chapters in one story.

---

# Chapter 00 — Is my Jetson setup ready?

Notebook:
- `notebooks/00_package_install_and_check.ipynb`

## What this chapter does

This chapter checks the machine before the rover moves.

It checks things like:
- Python environment
- important packages
- CUDA / Torch visibility
- camera device
- serial device

## What this chapter is trying to teach

Before you debug robot behavior, first make sure the computer is healthy.

A lot of problems are not algorithm problems.
They are environment problems.

## In simple words

This notebook answers:

> Can this Jetson even run the project correctly?

## You are ready to move on when

You can say:
- my Python environment is correct
- my packages import correctly
- my camera exists
- my serial device exists
- CUDA is visible if needed

---

# Chapter 01 — How do we control the car motors?

Notebook:
- `notebooks/01_motor_speed_tuning.ipynb`

Launches:
- `scripts/jetcar_motor_calibration_panel.py`

Saves:
- `.jetcar_motor_calibration.json`

## What this chapter does

This chapter helps you find motor values that actually work on your rover.

You test and save four useful motion presets:
- `forward`
- `rotate_left`
- `rotate_right`
- `forward_nudge`

## What this chapter is trying to teach

The rover cannot drive well if the motor commands are bad.

Even a good vision algorithm will look broken if the forward and turning motion are not calibrated.

## In simple words

This notebook answers:

> If I tell the car to move, does it move in a predictable way?

## Very important idea

Motor values depend on the floor.

A value that works on one floor may not work well on another.
So this chapter is not a one-time theory exercise.
It is a real robotics calibration step.

## You are ready to move on when

You can say:
- I can safely stop the rover
- I know which values make it move forward
- I know which values rotate it left and right
- I understand why these values are saved into a JSON file

---

# Chapter 02 — How does the rover follow a line?

Notebook:
- `notebooks/02_line_following.ipynb`

Launches:
- `scripts/teleop_server_article_contour_panel.py`

Reads:
- `.jetcar_motor_calibration.json`

Saves:
- `.contour_follow_settings.json`

## What this chapter does

This is the main chapter for understanding the car.

The rover uses the camera image to find the floor line and decide how to move.

## What this chapter is trying to teach

This chapter teaches the full basic control loop:

- see the line
- simplify the image
- choose the line shape
- estimate direction
- send motion commands

## In simple words

This notebook answers:

> Can the car look at the line and steer itself?

## What the learner should notice

In the browser panel you can see:
- the live camera image
- the grayscale region of interest
- the mask
- the contour overlay

These are not just pretty views.
They show each stage of the algorithm.

## You are ready to move on when

You can explain:
- why only the lower part of the image matters most
- what the mask is
- what a contour is
- why the rover turns left or right in a given frame

---

# Chapter 03 — What happens when we add YOLO?

Notebook:
- `notebooks/03_line_following_with_default_yolo_detection.ipynb`

Launches:
- `scripts/teleop_server_article_contour_panel_with_yolo.py`

Typical model:
- `yolo11n.pt`

## What this chapter does

This chapter adds general object detection to the full RGB frame.

The rover still follows the line using the contour logic.
YOLO is an extra perception layer.

## What this chapter is trying to teach

One camera can be used for more than one job.

- contour logic uses the lower part for steering
- YOLO uses the full image for object detection

## In simple words

This notebook answers:

> Can we keep the same line following, but also detect things in the scene?

## You are ready to move on when

You can explain:
- YOLO is not replacing the line-following algorithm here
- the car can still follow the line even if YOLO is turned off
- the two vision paths are doing different jobs

---

# Chapter 04 — How do we make the car react to a stop sign?

Notebook:
- `notebooks/04_line_following_with_stop_sign_action.ipynb`

Launches:
- `scripts/jetcar_contour_sign_panel.py`

Typical model:
- `models/traffic_sign_detector.pt`

## What this chapter does

This chapter keeps the normal line following, but adds one extra behavior:

if a stable stop sign is seen, the rover pauses.

## What this chapter is trying to teach

This chapter teaches the idea of event-driven behavior.

The rover does not only do continuous steering.
It can also respond to a special event from vision.

## In simple words

This notebook answers:

> Can the car keep following the line, but stop when it sees a stop sign?

## Why stability matters

A single detection frame may be wrong.
So the rover should not react instantly to every one-frame detection.

This is why stable voting or repeated confirmation is important.

## You are ready to finish when

You can explain:
- how normal line following works
- how stop-sign detection sits on top of it
- why the stop behavior should trigger once, not forever

---

# The most important lesson from all five chapters

These notebooks are not random demos.
They build one robot system step by step.

## The real learning path is:

### First
Make sure the Jetson system works.

### Second
Make sure the rover can move safely.

### Third
Make sure the line-following algorithm works.

### Fourth
Add extra perception.

### Fifth
Add behavior based on that perception.

That is how the repo should be learned.

---

# Quick self-check for students

Before you say "I understand this repo," you should be able to answer these questions:

1. Why do we calibrate motors before testing line following?
2. Why do we use only part of the image for steering?
3. What is a mask?
4. What is a contour?
5. Why is YOLO not the main steering method here?
6. Why does a stop sign need stable detection before action?

If you can answer those six questions clearly, you are not just following instructions.
You actually understand the project.