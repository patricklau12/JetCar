# README Install Snippets To Apply

This page contains exact text snippets that can be pasted into `README.md` or `docs/beginner-quickstart.md`.

These snippets fix two beginner issues:
- the clone command should use the real repo URL
- the PyTorch install step should be described as JetPack-specific, not universal

---

## 1. Replace the clone block with this

```bash
cd /home/orin
git clone https://github.com/patricklau12/JetCar.git JetCar
cd /home/orin/JetCar
```

---

## 2. Add this note before the PyTorch install command

> **Important:** the PyTorch install command below is JetPack-specific. Before running it, check your Jetson stack using `cat /etc/nv_tegra_release` and `dpkg-query -W nvidia-jetpack`. If your JetPack/L4T stack is different from the one tested with this repo, do not use the command blindly. Install the matching Jetson PyTorch wheel for your exact software stack instead.

---

## 3. Example full install section

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

If the board matches the repo's tested JetPack 6 / CUDA 12.6 setup, then use the repo default PyTorch command:

```bash
python -m pip install --force-reinstall --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 torch==2.8.0 torchvision==0.23.0
```

Then continue:

```bash
python -m pip install --force-reinstall "numpy<2"
python -m pip install -e .
python -m ipykernel install --user --name jetcar --display-name "Python (jetcar)"
```

---

## 4. Best short wording for the repo

If you want only one short sentence in the README, use this:

> The PyTorch install command below is for the JetPack stack currently tested with this repo. Check your Jetson version first, and do not use the command blindly on a different JetPack/L4T stack.
