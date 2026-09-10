# SOP — bring up the SO-101 station, from a bare machine to p1 and p2 running

Written to be followed by someone who has not touched this rig before. Every
command is meant to be pasted whole. Where a step can go wrong quietly, it says
so and tells you what "right" looks like.

Reference machine: Jetson Orin Nano Super, JetPack 7.2.1 / Ubuntu 24.04 /
Python 3.12. Any Linux box with the same hardware attached will do.

**Every command here is a bare word — `p1`, `p2`, `so101 devices` — and works
from any directory.** They are the launchers in `bin/`, which `install.sh` puts
on your PATH (§1). Each one finds the virtualenv, loads `devices.env`, and runs
the program from the repo so the logs always land in the same place. You never
type a python path, a script path, or `source devices.env` again.

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

```sh
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

The last step adds `bin/` to your PATH in `~/.bashrc`. **Open a new terminal**
before going on — that is what picks up both the PATH and `dialout`.

### Which python — you do not choose

`install.sh` makes `~/so101venv`. The **original Jetson** was set up by hand
before `install.sh` existed and has `~/step0venv` instead. The launchers look
for both, in that order, so the same command works on either machine. Nothing
in this SOP writes an interpreter path.

To see which one you actually got, along with everything else the launcher
decided:

```sh
so101 env
```

That prints the repo, the `devices.env` in use, the interpreter and where it
came from, the lerobot and OpenCV versions, and whether each arm and camera path
in `devices.env` exists **right now**. Run it first whenever something is wrong;
it turns most "it does not work" into one line.

`so101` on its own lists every command.

### Check it took

```sh
so101 selftest | tail -1
```

**Expect `ALL PASS` and 32 checks.** These tests deliberately need no arm, no
camera, no lerobot and no OpenCV — they run anywhere. That is the point: if
something later goes wrong, this tells you whether the *code* is fine, so you
can go and look at the hardware instead.

---

## 2. Put the calibration where lerobot looks for it

```sh
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so_follower
mkdir -p ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader
cp my_follower.json ~/.cache/huggingface/lerobot/calibration/robots/so_follower/
cp my_leader.json   ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/
```

Verify — this should print a path, `True`, and `6`:

```sh
so101 python -c "from lerobot.robots.so_follower import SOFollower,SOFollowerRobotConfig as C; r=SOFollower(C(port='/dev/null',id='my_follower')); print(r.calibration_fpath, r.calibration_fpath.is_file(), len(r.calibration))"
```

### 2b. Only if these are DIFFERENT arms

```sh
so101 calibrate follower
so101 calibrate leader
```

(Do §3 first so `devices.env` exists — the ports have to be right before you
calibrate, or you will calibrate the wrong arm.)

Use these rather than calling `lerobot-calibrate` yourself. Its flags are a
trap: the follower is `--robot.*` and the leader is `--teleop.*`, and lerobot
**enables torque** on whatever it is told is a robot. Point `--robot.port` at
the leader by hand and you have just powered the arm you are about to grab.

Calibration writes an offset into each servo's EEPROM **and** the JSON file, and
pre-flight later compares the two. So the file alone is not enough: a file
copied from another arm will be rejected, correctly.

⚠ **Never recalibrate to make an error go away.** If pre-flight says the motor
values disagree with the file, something moved that should not have — find out
what before overwriting the reference.

---

## 3. Plug the hardware in and write down what is where

```sh
so101 devices
```

You get the arms' **by-id** paths (which carry a USB serial) and the cameras'
**by-path** paths (which encode the physical socket).

### Which arm is which — do NOT guess

`/dev/ttyACM0` and `/dev/ttyACM1` are assigned in plug order and change. The
serial in the by-id path is the real identity. To find out which serial is the
leader, unplug it and see which one disappears:

```sh
ls /dev/serial/by-id/ > /tmp/before.txt ; cat /tmp/before.txt
```

Now **unplug the leader's USB cable** (the arm you can turn freely by hand), then:

```sh
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

```sh
cp setup/devices.example.env devices.env
```

Edit `devices.env` with the values `so101 devices` printed, then:

```sh
p2 --no-arm --duration 3
```

Open the two `logs/p2/<timestamp>/*_first.jpg` files. **The wrist camera is the
one that can see the gripper.** If they are the wrong way round, swap
`CAM_WRIST` and `CAM_FRONT` in `devices.env`.

Move a camera to a different socket later and the labels are silently wrong
again. Re-check the snapshots whenever anything is unplugged.

### A finished `devices.env`

```sh
export LEADER=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79050417-if00
export FOLLOWER=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79050450-if00
export LEADER_SERIAL=5B79050417
export FOLLOWER_SERIAL=5B79050450
export CAM_WRIST=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.3:1.0-video-index0
export CAM_FRONT=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0
export POWER_LINE_HZ=60
export ROTATE_FRONT=180
```

`LEADER_SERIAL` / `FOLLOWER_SERIAL` are what arm the anti-swap check. Leave them
out and the check passes vacuously — it will say so, but it will not stop you.

`POWER_LINE_HZ=60` is Taiwan's mains. It removes the rolling bands you get
under fluorescent light. It is a property of the room, not of the run, which is
why it lives here — and it does **not** touch the exposure.

`ROTATE_FRONT=180` is there because that camera is mounted upside down. Rotation
is applied **in the capture thread**, so the video file and the live view get
the same picture. Correcting it only in the viewer is the trap: the recording
stays upside down and nobody notices until a policy is trained on it.

**Those values are the reference Jetson's. Yours will differ.** Fill in what
`so101 devices` prints on your machine.

Check it with `so101 env`: every path should say `ok`. `NOT THERE` means that
device is unplugged, or in a different socket than the one written down.

---

## 4. Program 1 — the arm alone

Before you touch the keyboard:

1. Put the follower somewhere stable, with nothing within its reach — including
   the cameras and your coffee.
2. Turn the leader **by hand** so it roughly matches the follower's pose.
3. Keep a hand near the power.

```sh
p1 --duration 20
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

```sh
so101 latency
```

(With no argument that is the most recent p1 run, and it prints which one it
picked. Pass a `rows.jsonl` path for an older one.)

Move **every one of the six joints** back and forth during the run, or the ones
that did not move get skipped and there is nothing to correlate.

---

## 5. Program 2 — arm and cameras together

```sh
p2 --duration 60
```

Cameras only:

```sh
p2 --no-arm --duration 60
```

That is the whole standard run. MJPG 1024×768 at 30 fps, the arm at 120 Hz,
the display on, the mains frequency from `devices.env` — all defaults, and all
the same numbers §6's reference table was measured at. Drop `--duration` to run
until you press `q` in the video window, or Enter in the terminal.

Output in `logs/p2/<timestamp>/`: `<cam>.avi`, `cam_rows.jsonl`,
`arm_rows.jsonl`, `events.jsonl`, `<cam>_faults.jsonl`, `<cam>_first.jpg`.

The arm loop and each camera run in **separate threads that share nothing**.
A camera dropping out does not touch the arm; an arm fault does not touch the
cameras. p2 records two *independent* streams — it does not merge them onto one
timeline and it does not discard anything. That is p4's job, and p4 does not
exist yet.

### The step limit scales itself — do not set it by hand

`--max-step-deg` is a limit **per step**, so the same number means a different
speed at every loop rate: 8° per step is 240°/s at 30 Hz and 960°/s at 120 Hz.
It is now derived from the rate you asked for, so the **speed** stays fixed at
240°/s (and the gripper at 450 %/s) wherever you set `--fps`:

| rate | derived limit | speed |
|---:|---:|---:|
| 30 Hz | 8°/step | 240°/s |
| 60 Hz | 4°/step | 240°/s |
| 120 Hz | 2°/step | 240°/s |

p1 and p2 do the identical calculation, which is what makes their arm numbers
comparable — that comparison is the only reason p2 exists.

Both programs print the limit they arrived at and whether it was derived or
given, and write the same into `events.jsonl`, so a run's own log says where
its limits came from. Passing `--max-step-deg` still pins it; you then own the
arithmetic.

⚠ This used to be a fixed 8.0 in both programs. p2's old default of 60 Hz was
therefore running at **480°/s**, twice what this section documented as safe,
and `--arm-fps 120` without the matching `--max-step-deg 2` was four times.

### Exposure

Default is `--exposure auto`, which **actively sets** auto exposure and auto
white balance. That matters: V4L2 controls belong to the *camera*, not to the
program, so a run that locked the exposure leaves it locked for every run after
it, with nothing in the later logs to say so.

Mains frequency comes from `POWER_LINE_HZ` in `devices.env` (60 for Taiwan) and
removes the rolling bands you get under fluorescent light. It is a separate
setting and does **not** touch the exposure. `--power-line-hz` overrides it for
one run; `0` disables it.

For a real dataset use `--exposure lock`. Auto exposure varies the exposure
*time*, so the frame interval moves and the brightness drifts across a
recording — and when the arm enters frame the background brightness starts
tracking the arm's pose, which is a cue a policy will happily learn and which
will not survive deployment.

---

## 6. What a healthy run looks like

Measured on the reference Jetson, 2026-09-09. Compare yours against these.
These are what a bare `p1 --duration 20` and `p2 --duration 60` now produce —
120 Hz is the default in both, so you are comparing like with like.

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
| `p1: command not found` | the PATH line has not taken. Open a new terminal, or `source ~/.bashrc`. `which so101` should print a path inside the repo |
| a command runs but uses the wrong python or stale ports | `so101 env` — it prints every choice it made and where each came from |
| any serial open fails | did you run `newgrp dialout`, or log out and back in? |
| `no status packet` on one motor id | it is a servo not answering. Run `so101 ping --repeat 20`. It has happened once, transiently, on the follower's `wrist_roll` — unexplained |
| a camera is missing from `so101 devices` | `lsusb -t` — the kernel has not enumerated it. Not a code problem |
| pre-flight `[3]` fails | the arms are swapped, or `devices.env` is stale |
| the picture is upside down | `ROTATE_<NAME>` in `devices.env` |
| the arm faults with a tracking error | the follower could not keep up or is obstructed. It is holding torque — press Enter |
| everything looks fine but the data is wrong | check `<cam>_first.jpg`: are the camera labels the right way round? |

Ping every servo without energising anything:

```sh
so101 ping
```

If the fault is intermittent — a connect that fails once and then works — pin it
down instead of guessing:

```sh
so101 ping --repeat 20
```

All six ids on both arms should answer. A missing id is that servo or the
daisy-chain cable feeding it — and the chain runs 1→2→3→4→5→6, so a bad cable
between 4 and 5 takes 5 *and* 6 with it. `so101 ping` says so when the missing
ids are consecutive.

The servo logic is USB-powered, so **an arm with no power supply answers exactly
like one with power.** A clean ping does not mean the follower can hold itself
up.

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
