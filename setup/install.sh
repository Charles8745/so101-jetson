#!/usr/bin/env bash
# Set up the SO-101 station on a fresh machine. Safe to re-run.
# NOTE: intentionally no `set -e` and NO `apt upgrade` -- never full-upgrade a
# JetPack system, it breaks the NVIDIA-pinned CUDA/L4T stack.
set -u
VENV="${1:-$HOME/so101venv}"
REPO="$(cd -- "$(dirname -- "$0")/.." && pwd)"

echo "[1/5] apt deps (needs sudo)"
sudo apt-get update
# fonts-dejavu-core: OpenCV's bundled Qt has no fontconfig, so it looks for
# fonts in one hard-coded directory inside the cv2 package and warns on every
# window when they are not there. Step 3 puts them where it looks.
sudo apt-get install -y python3-venv python3-dev v4l-utils fonts-dejavu-core

echo "[2/5] python venv at $VENV"
python3 -m venv "$VENV"
"$VENV/bin/python" -m pip install -U pip setuptools wheel

echo "[3/5] python packages"
"$VENV/bin/python" -m pip install "lerobot[feetech]==0.6.1"
# live display needs the FULL opencv, not the headless build lerobot pulls in
"$VENV/bin/python" -m pip uninstall -y opencv-python-headless || true
"$VENV/bin/python" -m pip install opencv-python

# Silence "QFontDatabase: Cannot find font directory .../cv2/qt/fonts" -- a
# warning, not an error: it is OpenCV's own window chrome that goes unstyled,
# and every character p2 draws on the picture uses OpenCV's built-in Hershey
# font, not Qt. But it prints on every window and buries p2's real output.
# The path is asked of cv2 rather than written down, so this keeps working
# wherever the venv is.
"$VENV/bin/python" - <<'FONTS'
import glob, os, shutil
try:
    import cv2
except ImportError:
    raise SystemExit
d = os.path.join(os.path.dirname(cv2.__file__), "qt", "fonts")
src = sorted(glob.glob("/usr/share/fonts/truetype/dejavu/DejaVuSans*.ttf"))
if not src:
    print("      no DejaVu fonts found -- skipping (harmless, Qt will warn)")
    raise SystemExit
os.makedirs(d, exist_ok=True)
for f in src:
    shutil.copy(f, d)
print(f"      {len(src)} fonts -> {d}")
FONTS

echo "[4/5] serial port permissions"
sudo usermod -aG dialout "$USER"

echo "[5/5] put the launchers on your PATH"
# So that p1 / p2 / p3 / so101 work as bare commands from any directory.
# Written to .bashrc rather than a symlink into /usr/local/bin: no sudo, and
# `so101 env` can still tell you exactly which checkout it came from.
LINE="export PATH=\"$REPO/bin:\$PATH\""
touch "$HOME/.bashrc"
if grep -qF "$REPO/bin" "$HOME/.bashrc"; then
  echo "      already in ~/.bashrc"
else
  printf '\n# SO-101 station launchers\n%s\n' "$LINE" >> "$HOME/.bashrc"
  echo "      added to ~/.bashrc"
fi

echo
echo "DONE. Now:"
echo "  1) open a NEW terminal (or run: source ~/.bashrc && newgrp dialout)"
echo "     -- the new terminal is what picks up BOTH the PATH and dialout"
echo "  2) copy the arm calibration into:"
echo "       ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json"
echo "       ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json"
echo "  3) cd $REPO && cp setup/devices.example.env devices.env"
echo "  4) so101 devices        # fill devices.env in from what this prints"
echo "  5) so101 env            # check every choice it made before you run p1"
echo
echo "From then on, from any directory:"
echo "  p1 --duration 20"
echo "  p2 --duration 60"
echo "  so101 latency"
