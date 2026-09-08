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
Separate terminal, separate process; independent of the arm.
```
source devices.env
~/so101venv/bin/python programs/p2_record_cameras.py \
    --width 1024 --height 768 --fps 30 --fourcc MJPG
```
A 7-point pre-flight runs first: nodes exist, cameras are distinct, **the mode
actually negotiated matches the one requested**, the USB bandwidth budget, a
control dump, and a simultaneous capture probe. It also saves one frame from
each camera (`<cam>_first.jpg`) -- see the identity note below.

Stop with `q` in a window, or Ctrl+C. Output in `logs/p2/<timestamp>/`:
`<cam>.avi`, `rows.jsonl`, `events.jsonl`, `<cam>_faults.jsonl`, `<cam>_first.jpg`.

### Exposure -- matters more than it looks
Auto-exposure is bad for a training set. When the arm enters frame the exposure
compensates, so **background brightness becomes correlated with arm position** --
a policy can read arm pose off the background, a cue that dies on deployment. It
also varies exposure TIME, so frame intervals jitter and timestamps drift from
the true sampling instants. Mains flicker (Taiwan: 60 Hz) adds banding.

This program leaves the controls alone by default and just records them (it is a
test tool -- changing settings would confuse what you are testing). For real data:
```
    --lock-exposure --power-line-hz 60
```

### Identity: cameras can only be bound to a PORT
The arms carry unique USB serials, so pre-flight catches a swap. These cameras do
not: same model, no serial in either by-id string. The only thing telling them
apart is the physical port, which is what by-path encodes. **Move a cable to
another port and the labels are silently wrong.** The startup snapshots are the
remedy: here they are decisive by content -- the wrist camera sees the gripper.

### Fault policy
A camera fault does NOT stop the run: the other camera keeps recording, the
failing one reconnects by by-path with exponential backoff, and every drop,
reconnect and second of downtime is written to `<cam>_faults.jsonl`. The end-of-run
summary prints the totals. **If it reports a mode change, data before and after
are not the same exam paper** -- treat them separately.

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
