# Workshop Plan

## Beginner Session Goal

Students should leave the first session able to:

- pull the repo
- install the environment
- launch the browser control panel
- verify camera and motor control
- calibrate motion for the current floor
- run a basic line-following test

## Recommended Session Flow

1. Safety briefing
2. Hardware wiring check
3. Repo pull and package install
4. Launch the browser teleop page
5. Camera check
6. Motor check
7. Motor calibration
8. Line-following test
9. Only after that, move into notebooks for deeper learning

## Suggested Teaching Order After Preflight

- use `00_environment_check.ipynb` for basic environment diagnostics
- use `01_web_panel_intro.ipynb` for the panel command and the panel workflow
- use `02_yolo_object_detection.ipynb` for a simple computer-vision demo before any custom data collection
- use `notebooks/archive/` only for older experiments that you intentionally want to revisit

## Instructor Notes

- treat floor calibration as required, not optional
- do not assume one saved speed works in every venue
- keep early success simple: camera, control, calibration, then one line-following demo
- delay model training until the basic rover workflow is stable
