# so101-jetson

Hardware station for the SO-101 arm pair on a Jetson (or any Linux box):
**teleoperation + signal output** and **resilient camera recording + live view**,
as two independent processes.

This repo is the hardware/driver layer for the thesis project. The pluggable
`env.type = real` adapter that feeds the detector/retriever/repair pipeline lives
in the MAIN repo (`vla-self-repair`) and pins a commit of this repo -- keep the
boundary: drivers here, the pipeline seam there.

## Two independent programs
| program | file | owns | does |
|---|---|---|---|
| 1. Arm | `arm/teleop_arm.py` | servo bus | leader drives follower; emits joint signal (UDP + JSONL) for Isaac Sim |
| 2. Camera | `camera/record_cameras.py` | cameras | records each camera (raw AVI + timestamp CSV), live window, auto-reconnect on disconnect |

They are separate OS processes: one failing cannot affect the other. Both stamp
the same system clock (`t_mono`) so their streams align offline.

## Quick start
See **[docs/SOP.md](docs/SOP.md)**. Short version:
```
bash setup/install.sh && newgrp dialout
# place calibration, then:
cp setup/devices.example.env devices.env   # edit from: python tools/list_devices.py
source devices.env
~/so101venv/bin/python arm/teleop_arm.py    --fps 30 --jsonl ./arm_signal.jsonl
~/so101venv/bin/python camera/record_cameras.py --cam wrist=$CAM_WRIST --cam front=$CAM_FRONT --fourcc MJPG
```

## Docs
- [docs/SOP.md](docs/SOP.md) -- new-machine bring-up
- [docs/HARDWARE.md](docs/HARDWARE.md) -- measured platform facts, USB bandwidth limit, cable fault
- [docs/SIGNAL_SCHEMA.md](docs/SIGNAL_SCHEMA.md) -- the joint signal format for Isaac Sim

## Self-test (no hardware needed)
```
python tools/selftest.py
```

## Recording format
Raw MJPG-in-AVI per camera + a `_timestamps.csv` sidecar (ground truth). Convert
to a lerobot dataset offline in the main repo (keeps this station dependency-light
and independent of the video-codec toolchain).
