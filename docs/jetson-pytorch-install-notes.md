# Jetson PyTorch Installation Notes

This page explains an important detail in the JetCar setup:

**The PyTorch install command is Jetson-stack specific.**

It is not safe to assume that one PyTorch command works for every Jetson Orin Nano.

---

## Why this matters

In the current project setup, the install block includes a command like:

```bash
python -m pip install --force-reinstall --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 torch==2.8.0 torchvision==0.23.0
```

That command assumes a specific software stack, not just a specific board model.

It is effectively assuming something like:
- JetPack 6
- CUDA 12.6
- a matching Jetson wheel source for those versions

So this command should be treated as:

> a tested project default for a known Jetson stack

not:

> a universal command for every Jetson

---

## What to do first

Before installing Jetson PyTorch, check the board's actual software stack.

Run these commands on the Jetson:

```bash
cat /etc/nv_tegra_release

dpkg-query -W nvidia-jetpack

dpkg-query -W nvidia-l4t-core

nvcc --version 2>/dev/null || true
```

A compact check block is:

```bash
echo "=== nv_tegra_release ==="
cat /etc/nv_tegra_release || true
echo

echo "=== nvidia-jetpack ==="
dpkg-query -W nvidia-jetpack 2>/dev/null || true
echo

echo "=== nvidia-l4t-core ==="
dpkg-query -W nvidia-l4t-core 2>/dev/null || true
echo

echo "=== CUDA ==="
nvcc --version 2>/dev/null || echo "nvcc not found"
```

---

## How to interpret the result

### If the Jetson matches the repo's tested stack

If the Jetson is on the same JetPack/CUDA combination that this repo was tested with, then the project default command is a reasonable choice.

### If the Jetson does not match

Do **not** blindly run the repo's PyTorch command.

Instead:
1. identify the actual JetPack / L4T stack
2. find the matching Jetson wheel source for that stack
3. then install the matching `torch` and `torchvision`

---

## Safe beginner rule

For beginners, use this rule:

- same board model does **not** guarantee same PyTorch command
- JetPack / L4T version matters
- CUDA compatibility matters

The board type is only part of the story.
The software stack is the important part.

---

## Suggested project wording

A clearer setup section for the repo would say:

1. install the base requirements
2. check the Jetson stack
3. only then install the matching Jetson PyTorch wheels

That is safer than making the PyTorch command look universal.

---

## Example wording for the README

You can use text like this in the setup instructions:

> The PyTorch install command below is for the JetPack stack currently tested with this repo. Before running it, check your Jetson version using `cat /etc/nv_tegra_release` and `dpkg-query -W nvidia-jetpack`. If your stack is different, do not use the command blindly. Install the matching Jetson PyTorch wheel for your exact JetPack/L4T version instead.

---

## Practical recommendation for JetCar users

Recommended order:

```bash
cd /home/orin/JetCar
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -r requirements-notebook.txt
```

Then check the Jetson stack:

```bash
cat /etc/nv_tegra_release
dpkg-query -W nvidia-jetpack nvidia-l4t-core
```

Then install the matching Jetson PyTorch wheels.

If the board matches the repo's tested JetPack 6 / CUDA 12.6 setup, use the repo default command.

---

## Bottom line

Yes, the repo should explain this more clearly.

The current PyTorch line is useful as a tested default, but it should be presented as **JetPack-specific**, not as a universal Jetson install command.
