# SOP -- bring up the SO-101 station on a new machine

Goal: on a fresh machine, reach "leader controls follower + signal out" and
"two cameras recording + live view", exactly as on the reference Jetson.

## 0. Prerequisites
- Linux with Python 3.10+ (reference: JetPack 7.2 / Ubuntu 24.04 / Py 3.12).
- The two SO-101 arms (leader + follower) and the USB cameras.
- The arm calibration JSONs for THESE physical arms.

## 1. Install
```
git clone https://github.com/Charles8745/so101-jetson.git
cd so101-jetson
bash setup/install.sh            # makes ~/so101venv, installs deps, adds dialout
newgrp dialout                   # or log out/in
```

## 2. Place calibration
```
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so_follower
mkdir -p ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader
cp my_follower.json ~/.cache/huggingface/lerobot/calibration/robots/so_follower/
cp my_leader.json   ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/
```
Verify (should print the file path, True, and 6):
```
~/so101venv/bin/python -c "from lerobot.robots.so_follower import SOFollower,SOFollowerRobotConfig as C; r=SOFollower(C(port='/dev/null',id='my_follower')); print(r.calibration_fpath, r.calibration_fpath.is_file(), len(r.calibration))"
```

## 3. Plug in hardware, then record device IDs
```
python tools/list_devices.py
cp setup/devices.example.env devices.env
#   edit devices.env with the by-id / by-path values printed above
source devices.env
```

## 4. Program 1 -- follower follows leader  (owns the servo bus)
```
~/so101venv/bin/python programs/p1_follow_leader.py --fps 30
```
It runs an 8-point pre-flight first and refuses to start if anything fails --
including if the USB serials do not match the registry (arms swapped) or if the
leader and follower poses disagree by more than 15 deg (would cause a fast snap).
`--force` overrides only the pose check.

**Torque policy**: a fault does NOT release torque -- the follower freezes where
it is. Torque is released on exactly two exits: you press ENTER, or you close the
program (Ctrl+C / SIGTERM).

Logs land in `logs/p1/<timestamp>/{rows,events}.jsonl`. Then:
```
python tools/analyze_latency.py logs/p1/<timestamp>/rows.jsonl
```
which reports loop rate, the per-step breakdown, and the **follow latency**
(how far the physical follower lags the leader, by cross-correlation). That
number is the baseline Isaac (p3) and the VLA (p5) get compared against.

## 5. Program 2 -- cameras: record + live view  (owns the cameras)
Runs in a SEPARATE terminal / process; independent of the arm.
```
source devices.env
~/so101venv/bin/python programs/p2_record_cameras.py \
    --cam wrist=$CAM_WRIST --cam front=$CAM_FRONT \
    --width 1024 --height 768 --fps 30 --fourcc MJPG \
    --out ./recordings --display on
# press 'q' in a window, or Ctrl+C, to stop
```
Output per session in `recordings/<timestamp>/`:
`<cam>.avi`, `<cam>_timestamps.csv`, `<cam>_faults.jsonl`, `clock_epoch.json`.
On a headless machine use `--display off` (recording is unaffected).

## 6. Verify
```
ls -R recordings/<timestamp>/
cat recordings/<timestamp>/*_faults.jsonl   # empty (or only reconnect events) is good
```
An episode whose time window overlaps a `down` fault should be discarded.

## Isolation contract (why two programs)
- The arm program opens ONLY the servo bus; the camera program opens ONLY the
  cameras. They are separate OS processes -- if one crashes, the other is
  untouched.
- Both stamp `t_mono` from the same system clock, so arm signal and camera
  frames can be merged into one synchronized episode offline.

## Gotchas (see docs/HARDWARE.md for detail)
- Two cameras: MUST use MJPG. Uncompressed YUYV overruns the single USB 2.0 bus.
- Cameras by-path, arms by-id -- never `/dev/videoN` / `/dev/ttyACMn`.
- Never run `sudo apt upgrade` on a JetPack machine.
- If teleop prints "not-calibrated / refusing to auto-recalibrate", the motor
  values disagree with the file -- do NOT blindly recalibrate; investigate.
