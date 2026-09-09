# so101-jetson

Hardware station for the SO-101 arm pair on a Jetson (or any Linux box). The
Jetson owns the servo bus and the cameras; other machines reach the arm through
it, so nobody else has to set hardware up from scratch.

The pluggable `env.type = real` adapter that feeds the detector / retriever /
repair pipeline lives in the MAIN repo (`vla-self-repair`) and pins a commit of
this one. Keep the boundary: **drivers here, the pipeline seam there.**

## Two layers -- and why

| layer | program | owns hardware | network |
|---|---|---|---|
| **0 · local, direct** | `programs/p1_follow_leader.py` — follower follows leader | itself | none |
| | `programs/p2_record_cameras.py` — record + live view | itself | none |
| | `programs/p4_collect_real.py` — real teleop + multi-cam dataset ⬜ | itself | none |
| **1 · gateway** | `programs/p3_teleop_sim.py` — real leader drives Isaac's virtual SO-101 | leader only | Jetson→Spark, UDP |
| | `net/so101_host.py` — resident service ⬜ | **exclusive** | ZMQ |
| | `programs/p5_vla_control.py` — VLA on Spark drives the real arm ⬜ | via host | both ways |

p3 sits in layer 1 but deliberately does **not** go through the gateway: it
drives a *simulated* follower, so it needs the leader and nothing else. That
keeps it usable before the gateway exists, and keeps the real follower out of
the picture entirely.

★ **Layer 0 must never go through the network.** Programs 1 and 2 exist to
validate hardware; put a transport under them and a failure can no longer be
attributed to the arm rather than the link.

★ **Only one thing may own the servo bus at a time.** When the host is running,
the layer-0 programs cannot open the port — and must say so in those words.

## Quick start
See **[docs/SOP.md](docs/SOP.md)**.

```
bash setup/install.sh && newgrp dialout
cp setup/devices.example.env devices.env    # fill from: python tools/list_devices.py
source devices.env
~/so101venv/bin/python programs/p1_follow_leader.py
```

## Docs
- [docs/SOP.md](docs/SOP.md) — new-machine bring-up
- [docs/HARDWARE.md](docs/HARDWARE.md) — measured platform facts, the single USB 2.0 bus, the wrist-cable fault
- [docs/SIGNAL_SCHEMA.md](docs/SIGNAL_SCHEMA.md) — joint-signal format
- [docs/SIM_BRIDGE.md](docs/SIM_BRIDGE.md) — program 3: the map, the protocol, the fault policy

## Tests (no hardware)
```
python tools/selftest.py        # 23 unit checks, no OpenCV / lerobot / arm
python tools/loopback_test.py   # the whole p3 bridge against itself, real UDP
python tools/loopback_test.py --sim-fps 10     # ... with a simulator that lags
```

## Status
| | |
|---|---|
| p1, p2 | written — pre-flight, fault policy, logging. **Not yet run against hardware.** |
| p3 | written, adversarially reviewed, tested end to end over real UDP with an echo backend. **Arm and Isaac still untested.** |
| `sim/isaac_adapter.py` | written, **never run** — needs Spark's Isaac version |
| p4, p5, gateway | not started |
