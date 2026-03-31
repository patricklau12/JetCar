# GitHub Wiki Homepage Draft for JetCar

This page is a draft structure for a future **GitHub Wiki home page**.

The idea is:
- keep the repo `docs/` folder as the source of truth
- use the GitHub Wiki as a simple front door for students and readers

You can copy this structure into the GitHub Wiki later.

---

# JetCar Wiki

Welcome to the JetCar wiki.

JetCar is a beginner-friendly robot car project for **NVIDIA Jetson Orin Nano**.

This wiki is designed to help students and first-time users understand:
- how to boot and prepare the Jetson
- how to install the JetCar project
- how the notebooks are meant to be used
- how the line-following algorithm works
- how YOLO and stop-sign logic fit into the system

If you are new, do **not** start by opening random scripts.
Use the guides below in order.

---

## Start Here

### 1. Brand-new Jetson setup
Read this first if your Jetson has never been booted before.

- **Brand-New Jetson Orin Nano: First Boot Guide**

This covers:
- boot media preparation
- first power-on
- Ubuntu first-boot wizard
- basic checks before installing JetCar

### 2. Beginner project setup
Read this next if your Jetson already boots and you want to run JetCar.

- **Beginner Quickstart**
- **Hardware Checklist**

This covers:
- cloning the repo
- creating the Python environment
- installing dependencies
- running the first notebooks

### 3. Student learning path
Read this if you want to understand the notebook order.

- **Notebook Chapters for Students**

This explains:
- what each notebook does
- what each notebook launches
- what you should learn from each chapter
- when you are ready to move on

### 4. Understand the algorithm
Read this if you want to understand the actual line-following logic.

- **Line Following Explained for Students**

This explains:
- how the mask is created
- what contours are
- how the rover decides left or right
- how the Jetson sends motor commands to the rover

---

## Recommended Learning Order

If you are a complete beginner, use this order:

1. Brand-New Jetson Orin Nano: First Boot Guide
2. Beginner Quickstart
3. Notebook Chapters for Students
4. Line Following Explained for Students
5. Open notebook `00_package_install_and_check.ipynb`

---

## Notebook Guide

Main beginner notebooks:

- `00_package_install_and_check.ipynb`
- `01_motor_speed_tuning.ipynb`
- `02_line_following.ipynb`
- `03_line_following_with_default_yolo_detection.ipynb`
- `04_line_following_with_stop_sign_action.ipynb`

Suggested wiki subpages:
- **Notebook 00: Environment Check**
- **Notebook 01: Motor Speed Tuning**
- **Notebook 02: Line Following**
- **Notebook 03: Default YOLO Detection**
- **Notebook 04: Stop Sign Action**

Each page should explain:
- purpose
- launched script
- inputs and outputs
- what the learner should understand

---

## Core Concepts

Suggested wiki concept pages:

- **How JetCar Works: Line Following, Masking, and Motor Communication**
- **Line Following Explained for Students**
- **Jetson PyTorch Installation Notes**

These pages help students understand the project, not just run commands.

---

## Troubleshooting

Suggested wiki troubleshooting sections:

- Jetson will not boot
- camera not detected
- serial port not available
- rover does not move
- contour mask looks wrong
- YOLO model does not load
- stop sign is not detected

---

## Maintainer / Teacher Docs

Suggested wiki pages for maintainers:

- **Notebook Chapters Guide**
- **How JetCar Works: Line Following, Masking, and Motor Communication**
- **README Install Snippets To Apply**

These are useful when improving the teaching materials or updating the setup flow.

---

## Clone Command

The real repo clone command is:

```bash
cd /home/orin
git clone https://github.com/patricklau12/JetCar.git JetCar
cd /home/orin/JetCar
```

---

## Important Note About PyTorch on Jetson

PyTorch installation on Jetson is **JetPack-specific**.

Do not assume one `torch` install command works for every Orin Nano.

Check your Jetson stack first, then install the matching Jetson PyTorch wheels.

A useful check is:

```bash
cat /etc/nv_tegra_release
dpkg-query -W nvidia-jetpack nvidia-l4t-core
```

---

## Suggested Wiki Sidebar Structure

A simple wiki sidebar could look like this:

- Home
- Brand-New Jetson Orin Nano: First Boot Guide
- Beginner Quickstart
- Hardware Checklist
- Notebook Chapters for Students
- Line Following Explained for Students
- Notebook 00: Environment Check
- Notebook 01: Motor Speed Tuning
- Notebook 02: Line Following
- Notebook 03: Default YOLO Detection
- Notebook 04: Stop Sign Action
- Jetson PyTorch Installation Notes
- Troubleshooting
- Maintainer Notes

---

## Best Practice Recommendation

For this project, the best documentation split is:

- **Repo `docs/` folder** = source of truth
- **GitHub Wiki** = easier browsing layer

That way:
- docs stay versioned with the code
- PR review still covers documentation changes
- students still get a cleaner reading experience in the wiki

---

## One-line summary for students

If you are lost, start here:

1. make sure the Jetson boots
2. install the repo
3. follow the notebooks in order
4. understand the mask and contour logic before worrying about YOLO
