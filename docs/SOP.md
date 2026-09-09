# SOP — bring up the SO-101 station, from a bare machine to p1 and p2 running

Written to be followed by someone who has not touched this rig before. Every
command is meant to be pasted whole. Where a step can go wrong quietly, it says
so and tells you what "right" looks like.

Reference machine: Jetson Orin Nano Super, JetPack 7.2.1 / Ubuntu 24.04 /
Python 3.12. Any Linux box with the same hardware attached will do.

---

## What the two programs are for

| | question it answers |
|---|---|
| **p1** | does the arm work? leader → follower, nothing else running |
| **p2** | does running the arm **and** the cameras together degrade either? |
| **p2 `--no-arm`** | do the cameras work? — no arm, no threads of ours, nothing else to blame |

Reach for `--no-arm` first whenever a camera misbehaves. Being able to accuse
one component on its own is the whole reason there are separate programs.

---

## 0. What you need before you start

- The two SO-101 arms — **leader** (the one you move by hand) and **follower**
  (the one that copies it) — and their USB-serial adapters.
- The two USB cameras. One is mounted on the follower's wrist; one looks at the
  scene from outside.
- **The calibration files for THESE PHYSICAL ARMS.** Calibration lives in the
  servos and in a JSON file, and it is per-arm — it is not transferable. If you
  are using the same arms as before, copy the files (§2). If the arms are
  different ones, you must calibrate them yourself (§2b).
- A power supply for the follower. It holds itself up with torque; unpowered it
  is limp and will flop over.

---

## 1. Install

```
git clone https://github.com/Charles8745/so101-jetson.git
cd so101-jetson
bash setup/install.sh
newgrp dialout
```

`install.sh` makes a virtualenv at `~/so101venv`, installs
`lerobot[feetech]==0.6.1`, swaps the headless OpenCV for the full one (the live
display needs it), and adds you to the `dialout` group so you can open the
serial ports. It deliberately does **not** run `apt upgrade`: a full upgrade
breaks JetPack's pinned CUDA/L4T stack.

`newgrp dialout` is not optional — without it every serial open fails with a
permission error that looks like a broken cable.

### Which python

`install.sh` makes `~/so101venv`. The **original Jetson** was set up by hand
before `install.sh` existed and has `~/step0venv` instead. So this SOP never
writes the path out: `devices.env` finds it once and exports `$PY`, and every
command below runs as `$PY programs/...`.

If your `devices.env` predates this, add these lines to it:

```
export PY="$(ls -d "$HOME"/so101venv/bin/python "$HOME"/step0venv/bin/python 2>/dev/null | head -1)"
echo "PY=$PY"
```

`source devices.env` should then print a path. If it prints nothing, the
virtualenv is missing — run `bash setup/install.sh`.

### Check it took

```
python3 tools/selftest.py | tail -1
```

**Expect `ALL PASS` and 28 checks.** These tests deliberately need no arm, no
camera, no lerobot and no OpenCV — they run anywhere. That is the point: if
something later goes wrong, this tells you whether the *code* is fine, so you
can go and look at the hardware instead.

---

## 2. Put the calibration where lerobot looks for it

```
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so_follower
mkdir -p ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader
cp my_follower.json ~/.cache/huggingface/lerobot/calibration/robots/so_follower/
cp my_leader.json   ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/
```

Verify — this should print a path, `True`, and `6`:

```
$PY -c "from lerobot.robots.so_follower import SOFollower,SOFollowerRobotConfig as C; r=SOFollower(C(port='/dev/null',id='my_follower')); print(r.calibration_fpath, r.calibration_fpath.is_file(), len(r.calibration))"
```

### 2b. Only if these are DIFFERENT arms

```
source devices.env
$(dirname $PY)/lerobot-calibrate --robot.type=so_follower --robot.port="$FOLLOWER" --robot.id=my_follower
$(dirname $PY)/lerobot-calibrate --teleop.type=so_leader  --teleop.port="$LEADER"   --teleop.id=my_leader
```

(Do §3 first so `devices.env` exists — the ports have to be right before you
calibrate, or you will calibrate the wrong arm.)

Calibration writes an offset into each servo's EEPROM **and** the JSON file, and
pre-flight later compares the two. So the file alone is not enough: a file
copied from another arm will be rejected, correctly.

⚠ **Never recalibrate to make an error go away.** If pre-flight says the motor
values disagree with the file, something moved that should not have — find out
what before overwriting the reference.

---

## 3. Plug the hardware in and write down what is where

```
python3 tools/list_devices.py
```

You get the arms' **by-id** paths (which carry a USB serial) and the cameras'
**by-path** paths (which encode the physical socket).

### Which arm is which — do NOT guess

`/dev/ttyACM0` and `/dev/ttyACM1` are assigned in plug order and change. The
serial in the by-id path is the real identity. To find out which serial is the
leader, unplug it and see which one disappears:

```
ls /dev/serial/by-id/ > /tmp/before.txt ; cat /tmp/before.txt
```

Now **unplug the leader's USB cable** (the arm you can turn freely by hand), then:

```
ls /dev/serial/by-id/ > /tmp/after.txt
echo "--- this one is the LEADER ---" ; comm -23 /tmp/before.txt /tmp/after.txt
echo "--- this one is the FOLLOWER ---" ; cat /tmp/after.txt
```

Plug it back in.

This costs two minutes and it is not optional. Connecting the wrong class to an
arm is physically bad — `SOLeader` **disables** torque (the follower would go
limp and drop) and `SOFollower` **enables** it (the leader would stiffen in your
hand). And if you get away with it, nothing errors: both arms move and every
recording has its `leader` and `follower` fields the wrong way round.

### Which camera is which

The cameras are the same model with no serial number, so **the only thing
telling them apart is which socket they are in**. Grab one frame from each and
look:

```
cp setup/devices.example.env devices.env
```

Edit `devices.env` with the values `list_devices.py` printed, then:

```
source devices.env
$PY programs/p2_record_cameras.py --no-arm --duration 3
```

Open the two `logs/p2/<timestamp>/*_first.jpg` files. **The wrist camera is the
one that can see the gripper.** If they are the wrong way round, swap
`CAM_WRIST` and `CAM_FRONT` in `devices.env`.

Move a camera to a different socket later and the labels are silently wrong
again. Re-check the snapshots whenever anything is unplugged.

### A finished `devices.env`

```
export LEADER=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79050417-if00
export FOLLOWER=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79050450-if00
export LEADER_SERIAL=5B79050417
export FOLLOWER_SERIAL=5B79050450
export CAM_WRIST=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.3:1.0-video-index0
export CAM_FRONT=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0
export ROTATE_FRONT=180
```

`LEADER_SERIAL` / `FOLLOWER_SERIAL` are what arm the anti-swap check. Leave them
out and the check passes vacuously — it will say so, but it will not stop you.

`ROTATE_FRONT=180` is there because that camera is mounted upside down. Rotation
is applied **in the capture thread**, so the video file and the live view get
the same picture. Correcting it only in the viewer is the trap: the recording
stays upside down and nobody notices until a policy is trained on it.

**Those values are the reference Jetson's. Yours will differ.** Fill in what
`list_devices.py` prints on your machine.

---

## 4. Program 1 — the arm alone

Before you touch the keyboard:

1. Put the follower somewhere stable, with nothing within its reach — including
   the cameras and your coffee.
2. Turn the leader **by hand** so it roughly matches the follower's pose.
3. Keep a hand near the power.

```
source devices.env
$PY programs/p1_follow_leader.py --duration 20
```

An 8-point pre-flight runs first and prints every check. It refuses to start on
a serial mismatch (arms swapped), a missing calibration, or a motor that does
not answer.

Check `[7] leader/follower poses aligned` is a **warning only**. If the two arms
disagree, the follower **walks** to the leader at the rate limit — it does not
snap. (That is only true because the rate limiter is seeded with the follower's
measured position; unseeded, the first command would be unlimited.)

### Stopping

- `--duration` runs out, or
- press **Enter**, or
- **Ctrl+C**.

All three release torque. **A fault does not**: the program stops sending
commands but keeps the follower powered, so it freezes where it is instead of
collapsing. Press Enter to release. Servos warm up while holding — do not walk
away from a faulted arm.

### Then measure the latency

```
$PY tools/analyze_latency.py "logs/p1/$(ls -t logs/p1 | head -1)/rows.jsonl"
```

(That picks the most recent run. Name the directory explicitly if you want an
older one.)

Move **every one of the six joints** back and forth during the run, or the ones
that did not move get skipped and there is nothing to correlate.

---

## 5. Program 2 — arm and cameras together

```
source devices.env
$PY programs/p2_record_cameras.py \
    --fourcc MJPG --width 1024 --height 768 --fps 30 --arm-fps 120 \
    --max-step-deg 2 --max-step-gripper-pct 3.75 \
    --power-line-hz 60 --display on --duration 60
```

Cameras only:

```
$PY programs/p2_record_cameras.py --no-arm \
    --power-line-hz 60 --duration 60
```

Output in `logs/p2/<timestamp>/`: `<cam>.avi`, `cam_rows.jsonl`,
`arm_rows.jsonl`, `events.jsonl`, `<cam>_faults.jsonl`, `<cam>_first.jpg`.

The arm loop and each camera run in **separate threads that share nothing**.
A camera dropping out does not touch the arm; an arm fault does not touch the
cameras. p2 records two *independent* streams — it does not merge them onto one
timeline and it does not discard anything. That is p4's job, and p4 does not
exist yet.

### `--max-step-deg` must scale with `--arm-fps`

It is a limit **per step**, so at 120 Hz a limit of 8° is 960°/s. Keep the
product near 240°/s: `--fps 30 → 8`, `--fps 60 → 4`, `--fps 120 → 2`.

### Exposure

Default is `--exposure auto`, which **actively sets** auto exposure and auto
white balance. That matters: V4L2 controls belong to the *camera*, not to the
program, so a run that locked the exposure leaves it locked for every run after
it, with nothing in the later logs to say so.

`--power-line-hz 60` (Taiwan mains) removes rolling bands under fluorescent
light. It is a separate setting and does **not** touch the exposure.

For a real dataset use `--exposure lock`. Auto exposure varies the exposure
*time*, so the frame interval moves and the brightness drifts across a
recording — and when the arm enters frame the background brightness starts
tracking the arm's pose, which is a cue a policy will happily learn and which
will not survive deployment.

---

## 6. What a healthy run looks like

Measured on the reference Jetson, 2026-09-09. Compare yours against these.

| | reference |
|---|---|
| `[8] read-rate probe` | **425–502 Hz**, 0 failures |
| p1 loop total | **2.67 ms** median at 120 Hz |
| p1 loop rate | 119.0 Hz median when asked for 120 |
| follow latency (120 Hz) | **~105 ms** measured, ~97 ms after the correction below |
| p2 arm loop, cameras streaming, display off | **118.1 Hz, 2.697 ms** — i.e. the cameras cost 0.9 Hz |
| p2 arm loop, display **on** | 115.9 Hz, 3.11 ms — the display costs another 2.2 Hz |
| camera frames | exactly `fps × duration`, 0 dropped, 0 reconnects, **0 mode changes** |
| USB bandwidth | 2 × 28 = 57 Mbps against a 384 Mbps budget |

**If a camera reports a mode change, stop.** It came back at a different
resolution or format after a reconnect; recording continues and looks fine, and
the data before and after are not the same exam paper.

### Two known quirks — not faults

- **The wrist camera's frame arrival jitters ~20× more than the front one**
  (`gap spread` ~280 ms vs ~10 ms in the capture probe), consistently. It drops
  no frames, so it does not affect p1/p2. It will matter for p4, which has to
  align the streams.
- **Follow latency does not improve when you raise the loop rate.** 60 Hz and
  120 Hz measure the same. The ~100 ms is the servo's own control loop and
  inertia, not our sampling — our software contributes 2.7 ms of it.

### One measurement caveat, if you are quoting the latency

p1 reads the follower **before** it sends that step's command, so row N's
follower position reflects commands up to row N−1 — the cross-correlation
therefore includes exactly one extra sample period. Subtract it:

| fps | measured | corrected | bias |
|---:|---:|---:|---:|
| 30 | 133.6 | 100.3 | 25% — **do not quote** |
| 60 | 100.5 | 83.8 | 17% |
| 120 | 105.0 | **96.7** | 8% — use this one |

Always quote the 120 Hz figure, and always say whether it is corrected.

---

## 7. When something goes wrong

| symptom | first thing to do |
|---|---|
| any serial open fails | did you run `newgrp dialout`, or log out and back in? |
| `no status packet` on one motor id | it is a servo not answering. Ping the bus (below). It has happened once, transiently, on the follower's `wrist_roll` — unexplained |
| a camera is missing from `list_devices.py` | `lsusb -t` — the kernel has not enumerated it. Not a code problem |
| pre-flight `[3]` fails | the arms are swapped, or `devices.env` is stale |
| the picture is upside down | `ROTATE_<NAME>` in `devices.env` |
| the arm faults with a tracking error | the follower could not keep up or is obstructed. It is holding torque — press Enter |
| everything looks fine but the data is wrong | check `<cam>_first.jpg`: are the camera labels the right way round? |

Ping every servo without energising anything:

```
source devices.env
$PY - <<'PY'
import os
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
for label, obj in (
    ("FOLLOWER", SOFollower(SOFollowerRobotConfig(port=os.environ["FOLLOWER"],
                            id="my_follower", use_degrees=True))),
    ("LEADER", SOLeader(SOLeaderTeleopConfig(port=os.environ["LEADER"],
                        id="my_leader", use_degrees=True)))):
    bus = obj.bus
    print(label, {n: m.id for n, m in bus.motors.items()})
    try:
        bus.connect(handshake=False)
    except TypeError:
        bus.connect()
    print("  answering:", bus.broadcast_ping())
    bus.disconnect()
PY
```

All six ids on both arms should answer. A missing id is that servo or the
daisy-chain cable feeding it — and the chain runs 1→2→3→4→5→6, so a bad cable
between 4 and 5 takes 5 *and* 6 with it.

---

## Gotchas

- **Cameras by-path, arms by-id. Never `/dev/videoN` or `/dev/ttyACMn`** — both
  renumber, and after a camera reconnect you can land on the *other* camera:
  recording continues, with the wrong label.
- **Use MJPG for two cameras.** 2 × MJPG 1024×768@30 is 57 Mbps and is measured
  good on the single 480 Mbps USB 2.0 bus everything shares. Two YUYV streams
  would be ~294 Mbps — that is *not proven to work here*, which is not the same
  as known-bad, but there is no reason to find out.
- Both cameras, both serial adapters and Bluetooth share **one** USB 2.0 bus.
  Moving to a different socket or adding a powered hub does not change that.
- **Never `sudo apt upgrade`** on a JetPack machine.
- If teleop says "not calibrated / refusing to auto-recalibrate", the motors
  disagree with the file. Investigate; do not recalibrate reflexively.
