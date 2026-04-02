#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ROOT_DIR}/.venv"

cd "${ROOT_DIR}"

python3 -m venv --system-site-packages "${VENV_DIR}"
source "${VENV_DIR}/bin/activate"

python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -r requirements-notebook.txt
python -m pip install --force-reinstall --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 torch==2.8.0 torchvision==0.23.0
python -m pip install --force-reinstall "numpy<2"
python -m pip install -e .
python -m ipykernel install --user --name jetcar --display-name "Python (jetcar)"

echo
echo "JetCar install complete."
echo "Activate with: source ${VENV_DIR}/bin/activate"
