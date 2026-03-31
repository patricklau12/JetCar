# How JetCar Works: Line Following, Masking, and Motor Communication

This page explains the actual concepts behind the beginner workflow.

It is not only a setup guide.
It is intended to help the learner understand:
- what the rover is calculating
- how the mask is created
- how the line-following decision is made
- how motion commands are sent to the car
- how YOLO fits into the system without replacing the classical control path

---

## 1. System overview

At a high level, the JetCar project has three linked parts:

1. **Perception**
   - read camera image
   - process image to find the line
   - optionally detect objects or signs with YOLO

2. **Decision making**
   - decide whether the rover should go straight, turn left, turn right, or stop

3. **Actuation**
   - send left/right motor commands over serial to the rover controller

A beginner should think of the rover as a loop:

**camera -> calculation -> motor command -> rover moves -> camera sees new frame**

---

## 2. Why the line-following algorithm uses classical CV first

The core line-following script is:
- `scripts/teleop_server_article_contour_panel.py`

This script does not need a trained deep model to follow a simple track.

Instead, it uses classical computer vision because:
- it is fast
- it is easier to teach
- it is easier to debug visually
- it works well when the line and floor have enough contrast

The project adds YOLO later as an additional perception layer, not as the first driving method.

---

## 3. The image-processing pipeline

The line-following pipeline can be understood as a series of simplifications.

### Step 1 — capture a camera frame

The rover gets a live RGB image from the camera.

The helper code in `jetcar/camera.py` opens either:
- a CSI camera using Jetson-friendly GStreamer logic
- a USB webcam using `/dev/videoX`

The goal is simply to produce a usable frame for analysis.

---

### Step 2 — crop to the lower region of interest

The line-following code does not analyze the entire frame equally.

It crops the image so that only the lower portion of the image is used for contour detection.

Why?
Because the most useful line information for steering is usually near the rover, not far away in the sky or background.

This reduces distraction and computation.

Conceptually:
- top of image = less useful for immediate steering
- bottom of image = most useful for immediate steering

---

### Step 3 — convert to grayscale

The script converts the ROI into grayscale.

Why?
Because the first goal is not object recognition.
The goal is just to separate the line from the floor.

Grayscale makes this easier because every pixel becomes a brightness value instead of 3 separate color channels.

This simplifies thresholding.

---

### Step 4 — threshold into a binary mask

This is one of the most important steps.

A threshold converts the grayscale ROI into a binary image:
- pixels brighter than the threshold become white
- pixels darker than the threshold become black

This produces the **mask**.

A mask is useful because it answers a very simple question:

> which pixels probably belong to the line-like region?

Instead of analyzing a full photo, the algorithm now works with a shape.

---

## 4. Why the script balances the mask

The script does not just use one fixed threshold blindly.
It tries to keep the amount of white pixels within a useful range.

This matters because:
- if the threshold is too low, too much of the image becomes white
- if the threshold is too high, almost nothing becomes white

So the code tries to keep the white percentage between a minimum and maximum range.

This is a practical idea for beginners:

**A good mask is not only about a number. It is about whether the line shape becomes usable.**

The code measures white-pixel percentage and adjusts threshold step-by-step until the mask is in a more reasonable range.

---

## 5. Morphological cleanup

After thresholding, the mask may contain noise:
- tiny specks
- gaps
- rough edges

The script uses operations like:
- open
- close
- optional erosion

These operations help clean the mask.

Conceptually:
- **open** removes small noisy spots
- **close** fills small holes and connects nearby regions
- **erode** can shrink noisy blobs and sharpen the selected region

A beginner does not need to memorize the OpenCV names first.
The key concept is:

**clean the mask so the line shape becomes more stable and less noisy**

---

## 6. Why a trapezoid ROI is used

The code does not allow the full cropped area equally.
It applies a trapezoid-shaped region of interest.

Why?
Because the rover expects the useful line to be somewhere near the center driving corridor.

This helps reject irrelevant bright regions at the edges.

The trapezoid is wider near the bottom and narrower near the top, which roughly matches perspective in a forward-facing camera view.

This is a very teachable robotics idea:

**use geometry and prior knowledge to simplify perception**

You often know where useful information should appear.
Use that knowledge.

---

## 7. Contours: turning the mask into shapes

Once the mask is ready, the script extracts contours.

A contour is basically a detected boundary of a connected white region.

At this point, the rover is no longer reasoning about raw pixels.
It is reasoning about candidate shapes that might represent the line.

Possible contours can include:
- the real track line
- glare or reflections
- floor markings
- noisy blobs

So the next problem is not "find a contour."
It is:

**which contour is the best one to steer from?**

---

## 8. Why the biggest contour is not always enough

A naive line follower might just pick the largest contour.

This repo does something smarter.
It scores contours using multiple clues such as:
- area
- shape fill ratio
- aspect ratio
- closeness to the center
- closeness to the previous target position
- whether the contour touches image borders
- whether it looks like a corner artifact

This is important pedagogically because it shows a real robotics engineering idea:

**good perception often uses a scoring heuristic, not one single rule**

The best contour is chosen because it is the most plausible line region, not simply the largest white blob.

---

## 9. Why scanlines are used

After a good contour is selected, the script looks at several horizontal scanlines across it.

This gives sample points such as:
- near point
- middle point
- far point

Why use these?
Because they help estimate where the line is and where it is heading.

A line is not just a position.
It is also a direction.

These scan points help build a target x-position that represents the line geometry more intelligently than just the contour center.

---

## 10. Angle and shift: the two main steering signals

The rover uses two core geometric signals.

### A. Shift

Shift tells us how far the line target is from the horizontal center of the image.

Interpretation:
- line target left of center -> rover likely needs left correction
- line target right of center -> rover likely needs right correction

### B. Angle

The script fits a line through contour points and estimates an angle.

Interpretation:
- line leaning one way suggests one turning tendency
- line leaning the other way suggests the opposite tendency

These two values are combined because they describe different things:
- **shift** = where the line is now
- **angle** = where the line is heading

This is one of the best concepts for a beginner to understand.

A robot should not steer from position only.
It should also use direction information.

---

## 11. From analysis to action

The script turns angle and shift into simple drive states:
- go straight
- turn left
- turn right
- wait if no line is found

This is intentionally simpler than a continuous PID controller.

Instead of computing exact steering geometry, the code uses calibrated motion presets:
- `forward`
- `rotate_left`
- `rotate_right`
- `forward_nudge`

This means the controller is more like a sequence of safe motion primitives.

This is good for beginners because it is understandable and easier to tune.

---

## 12. How motor control actually reaches the rover

The rover is not driven by directly toggling Jetson GPIO motor pins.

Instead, the Jetson sends JSON commands over a serial link to the rover controller.

The calibration panel and the contour panel both use a serial class that sends payloads like:

```json
{"T":1,"L":0.16,"R":0.16}
```

Interpretation:
- `L` = left motor command
- `R` = right motor command
- `T` = command type used by the rover firmware interface

A pulse is sent repeatedly for a short duration, then a stop command is sent.

This repeated refreshing helps ensure the motion command remains active during the pulse window.

### Why this matters conceptually

The learner should understand that the Jetson is acting like a higher-level controller.

It does not directly do low-level motor commutation.
It sends a command to another controller board, which is responsible for actually driving the motors.

That is a common robotics system pattern.

---

## 13. Why calibration presets are reused everywhere

The calibration panel saves motor reference values to:
- `.jetcar_motor_calibration.json`

Later scripts reuse the same file.

This means the auto-drive code does not guess motor power every time.
It uses the already-tested presets.

This is a strong software engineering pattern:
- measure once
- save configuration
- reuse it in multiple tools

A beginner should learn that configuration files are part of the robot system, not a side detail.

---

## 14. What the browser panel is really showing

The browser panel is not just a convenience UI.
It is a teaching and debugging surface.

The different views mean:

- **Live RGB**: what the camera sees
- **Grayscale ROI**: what brightness-based analysis sees
- **Balanced Binary Mask**: what pixels are currently being treated as candidate line regions
- **Contour / Vector Debug**: which shape was selected and what direction was inferred

This lets the learner compare:
- reality
- simplified vision signal
- chosen shape
- resulting decision

That is exactly how a good robotics debugging interface should work.

---

## 15. Where YOLO fits in

In later notebooks, YOLO is added on the full RGB frame.

Important concept:

YOLO is not the original steering algorithm.

The project architecture is:
- contour logic handles low-latency line following
- YOLO adds higher-level perception like object or sign detection

This is a good systems design lesson.

Not every task needs deep learning.
Use the simplest reliable method for each subproblem.

---

## 16. How stop-sign behavior fits into the control loop

The stop-sign notebook adds a new behavior layer.

Normal mode:
- follow line continuously

If a stable stop sign is detected:
- temporarily stop the rover
- then return to the normal line-follow loop

This introduces an event-based override.

The learner should understand the difference between:
- **continuous control**: line following happens frame after frame
- **discrete event logic**: stop sign triggers a special action

That is a valuable control concept for larger robotics systems.

---

## 17. What a beginner should be able to explain after studying this repo

A beginner should be able to explain the following in simple words:

1. the camera image is cropped because the lower part matters most for steering
2. grayscale and thresholding make the line easier to isolate
3. a mask is a simplified image used to find candidate regions
4. contours are shapes extracted from that mask
5. the rover scores contours to find the most likely line
6. the rover uses both line direction and horizontal offset to decide movement
7. the Jetson sends serial JSON commands to the rover controller
8. motor calibration values are stored and reused
9. YOLO is an added perception module, not the original steering method
10. stop-sign behavior is an event-driven override on top of normal line following

If the learner can explain those ten points, they do not just know how to run the project.
They understand the project.

---

## 18. Recommended teaching progression

For classes or self-study, use this progression:

1. inspect the hardware path
2. inspect the serial path
3. inspect the motor calibration panel
4. inspect the grayscale and mask views
5. inspect contour selection and angle/shift logic
6. inspect how those signals map to motion presets
7. only then add YOLO and sign actions

That order teaches concept first, then extension.
