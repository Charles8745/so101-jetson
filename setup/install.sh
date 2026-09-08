#!/usr/bin/env bash
# Set up the SO-101 station on a fresh machine. Safe to re-run.
# NOTE: intentionally no `set -e` and NO `apt upgrade` -- never full-upgrade a
# JetPack system, it breaks the NVIDIA-pinned CUDA/L4T stack.
set -u
VENV="${1:-$HOME/so101venv}"

echo "[1/4] apt deps (needs sudo)"
sudo apt-get update
sudo apt-get install -y python3-venv python3-dev v4l-utils

echo "[2/4] python venv at $VENV"
python3 -m venv "$VENV"
"$VENV/bin/python" -m pip install -U pip setuptools wheel

echo "[3/4] python packages"
"$VENV/bin/python" -m pip install "lerobot[feetech]==0.6.1"
# live display needs the FULL opencv, not the headless build lerobot pulls in
"$VENV/bin/python" -m pip uninstall -y opencv-python-headless || true
"$VENV/bin/python" -m pip install opencv-python

echo "[4/4] serial port permissions"
sudo usermod -aG dialout "$USER"

echo
echo "DONE. Now:"
echo "  1) log out/in (or run: newgrp dialout) so serial access takes effect"
echo "  2) copy the arm calibration into:"
echo "       ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json"
echo "       ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json"
echo "  3) cp setup/devices.example.env devices.env  &&  edit  &&  source devices.env"
echo "  4) python tools/list_devices.py   # fill devices.env from this"
echo "Run everything with:  $VENV/bin/python ..."
