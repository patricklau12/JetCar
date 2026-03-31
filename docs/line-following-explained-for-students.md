# Line Following Explained for Students

This page explains the main idea behind the JetCar project in a more beginner-friendly way.

It focuses on three questions:

1. how does the rover see the line?
2. how does it decide left or right?
3. how does the Jetson tell the rover motors what to do?

---

# The big picture

The rover is doing the same loop again and again:

1. look through the camera
2. simplify the image
3. guess where the line is
4. decide how to move
5. send a motor command
6. repeat

So the whole project can be remembered like this:

**camera -> calculation -> motion**

That is the heart of the system.

---

# Part 1 — How the rover sees the line

## Step 1: get a camera frame

The Jetson reads an image from the camera.

The repo supports:
- CSI camera
- USB camera

The helper code opens the camera and gives the project a frame to analyze.

At this stage, the rover only has a normal image.
It still does not know where the line is.

---

## Step 2: ignore the less useful part of the image

The rover does not need to study the whole image equally.

For steering, the most useful area is usually the lower part of the frame, because that is the part closest to the car.

So the project crops the image and focuses mainly on the lower region.

### Why?
Because:
- the floor line near the rover matters most for immediate turning
- the upper image may contain background clutter
- less image area means simpler processing

So one key idea is:

**not all pixels are equally important**

---

## Step 3: convert the image to grayscale

A color image has 3 channels.
A grayscale image mainly keeps brightness information.

For basic line following, we do not need rich object understanding yet.
We mostly need to separate the line from the floor.

That is why grayscale is useful.
It makes the next step easier.

---

## Step 4: turn the grayscale image into a mask

Now the rover applies a threshold.

This means:
- bright pixels become white
- dark pixels become black

The result is called a **mask**.

A mask is a simplified image.
Instead of asking:

> what is every pixel exactly?

we ask:

> which pixels might belong to the line region?

This makes the problem much easier.

---

# Part 2 — Why the mask matters so much

A beginner should understand this clearly:

The rover does **not** drive from the raw camera image directly.
It drives from a simplified shape extracted from the image.

That simplified shape begins with the mask.

If the mask is bad:
- the contour will be bad
- the direction estimate will be bad
- the motion decision will be bad

So the mask is one of the most important parts of the whole pipeline.

---

## Why the threshold is adjusted

A single threshold value is not always perfect.

If the threshold is too low:
- too much of the image becomes white
- the rover sees too many false regions

If the threshold is too high:
- almost nothing becomes white
- the rover may lose the line completely

So the project tries to keep the white amount in a reasonable range.

This is sometimes easier to understand as:

- too much white = too noisy
- too little white = too empty
- good mask = line shape is visible and usable

---

## Why the mask is cleaned

After thresholding, the mask can still be messy.
There may be:
- tiny white noise
- broken gaps
- rough edges

So the project cleans the mask with image-processing steps.

You do not need to memorize the exact OpenCV function names first.
The idea is enough:

**remove noise and make the line region more stable**

---

## Why the project uses a trapezoid region

Even after cropping the lower image, not every pixel is trusted equally.

The project uses a trapezoid-like region of interest.

This is because the useful driving corridor is expected near the center.
The edges are more likely to contain irrelevant bright areas or floor distractions.

This is an example of using prior knowledge.

The project is saying:

> I already know roughly where the useful line should appear, so I will focus there.

That is smart engineering.

---

# Part 3 — From mask to contour

Once the mask is ready, the rover finds **contours**.

A contour is a detected outline of a connected white region.

At this point, the rover is no longer thinking in terms of millions of pixels.
It is thinking in terms of a few candidate shapes.

That is much easier to reason about.

---

## Why not just pick the biggest contour?

A beginner might think:

> just choose the largest white shape

But that is not always correct.

A large bright blob could be:
- glare
- floor reflection
- wrong marking
- edge artifact

So the project scores contours using several clues, such as:
- size
- shape quality
- how centered it is
- whether it matches the previous target area
- whether it touches suspicious borders

This is important:

**the rover is trying to choose the most believable line shape, not just the biggest shape**

---

# Part 4 — How the rover guesses direction

Once the best contour is selected, the rover tries to answer two questions:

1. where is the line relative to the center?
2. where is the line heading?

These become two important signals.

---

## Signal A: shift

Shift means horizontal offset.

If the chosen line target is left of the image center, the rover likely needs a left correction.
If it is right of the center, the rover likely needs a right correction.

So shift tells the rover:

> how far away am I from the center of the path?

---

## Signal B: angle

The rover also fits a line through the contour shape and estimates an angle.

That angle gives direction information.

So angle tells the rover:

> where is the path pointing?

---

## Why both are needed

This is a very important concept.

If you use only shift, you know where the line is now, but not where it is heading.
If you use only angle, you know direction, but not exact sideways error.

Using both is better.

A simple way to remember it:
- **shift** = current sideways error
- **angle** = path direction

Together, they give a better steering decision.

---

# Part 5 — How the rover turns analysis into motion

The project does not use a very advanced continuous steering controller here.

Instead, it uses a simpler and more teachable method.
It maps the result into motion presets such as:
- go forward
- rotate left
- rotate right
- do a short forward nudge

These are like motion building blocks.

This makes the system easier for beginners to understand and tune.

---

## Why calibration matters again here

Those motion blocks are not magic.
They come from the earlier motor calibration notebook.

So the line-following notebook depends on the calibration notebook.

That is why the notebook order matters so much.

If calibration is bad, line following will look bad even if the image processing is good.

---

# Part 6 — How the Jetson talks to the rover motors

The Jetson does not directly spin the motors itself.

Instead, it sends a serial JSON command to the rover controller.

An example looks like this:

```json
{"T":1,"L":0.16,"R":0.16}
```

A simple way to read that is:
- `L` = left motor value
- `R` = right motor value
- `T` = command type used by the rover interface

So the Jetson is the high-level brain.
The rover controller board handles the lower-level motor actuation.

This is a very common robotics architecture.

---

## Why commands are often sent as pulses

The code often sends a command for a short time window, then sends stop.
Sometimes it refreshes the command during that short pulse.

This gives controlled movement bursts instead of one uncontrolled long action.

That is useful for:
- calibration
- safe testing
- simple line-follow correction behavior

---

# Part 7 — What the browser page is teaching you

The browser page is not just a remote control panel.
It is a learning tool.

The views mean:

- **RGB view** = what the camera really sees
- **grayscale ROI** = simplified brightness view
- **mask** = candidate line region
- **contour overlay** = chosen line shape and direction estimate

A student should look at all four together.

That way, you can connect:
- the real world
- the simplified image
- the chosen shape
- the final motion decision

That is how to truly understand the algorithm.

---

# Part 8 — Where YOLO fits in

Later in the repo, YOLO is added.

Important idea:

YOLO is not the original line-following brain.

The core line following is still classical CV on the lower ROI.
YOLO is added on top for extra perception on the full image.

So the project teaches an important system idea:

- one perception path handles steering
- another perception path can handle higher-level awareness

That is often better than forcing one model to do everything.

---

# Part 9 — How stop-sign behavior fits in

The stop-sign version adds one more layer.

Normal behavior:
- keep following the line

Special event:
- if a stable stop sign is seen, pause the rover

This teaches the difference between:
- continuous control
- event-triggered behavior

That is a valuable robotics concept.

The rover is not only continuously steering.
It can also react to discrete external events.

---

# Part 10 — What a student should be able to explain at the end

If you really understand this repo, you should be able to explain these points in your own words:

1. why the lower part of the image is most useful for steering
2. why we convert the image to grayscale
3. what a mask is and why it helps
4. why the mask must be cleaned
5. what a contour is
6. why the rover scores contours instead of always taking the biggest one
7. what shift means
8. what angle means
9. how the Jetson sends motor commands to the rover
10. why YOLO is an added layer, not the original steering method

If you can explain those ten points clearly, then you understand both the concept and the workflow.