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
| **1 · gateway** | `net/so101_host.py` — resident service ⬜ | **exclusive** | ZMQ |
| | `programs/p3_collect_sim.py` — drive Isaac Sim's virtual SO-101 ⬜ | via host | Jetson→Spark |
| | `programs/p5_vla_control.py` — VLA on Spark drives the real arm ⬜ | via host | both ways |

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

## Self-test (no hardware)
```
python tools/selftest.py        # 9 checks
```

## Status
✅ p1, p2 written · ⬜ not yet run against hardware · ⬜ p3/p4/p5 and the gateway not written
