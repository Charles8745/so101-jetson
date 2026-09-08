# Hardware notes (measured 2026-09-05/06, this Jetson)

## Platform
- Jetson Orin Nano Super, 8 GB
- JetPack 7.2.1-b49 / L4T (Jetson Linux) R39.2.0, built 2026-06-01
- Ubuntu 24.04 userspace, Python 3.12.3
- torch 2.11.0+cu130 (from PyPI; `torch.cuda.is_available()` is True).
  The station never runs inference on the Jetson (thin gateway), so a CPU torch
  would also be fine.

## Arms (Feetech / CH340 USB-serial, idVendor 1a86)
- leader  serial 5B79050417
- follower serial 5B79050450
- Always address them by `/dev/serial/by-id/...` (see devices.example.env).
  `/dev/ttyACMn` numbering flips on replug.
- User must be in the `dialout` group (install.sh does this).
- Calibration files (`my_follower.json`, `my_leader.json`) belong to the PHYSICAL
  arms and travel with them; they were validated on this Jetson (is_calibrated
  True on both). Do not re-run calibration unless the hardware actually changed.

## Cameras (Sonix UVC, 0c45:6368) -- there are two, identical
Per-resolution the ONLY offered frame rates are:

| format | resolution | fps |
|--------|-----------|-----|
| MJPG   | 640x480   | 120 |
| MJPG   | 800x600   | 60  |
| MJPG   | 1024x768  | 30  |
| YUYV   | 640x480   | 30  |
| YUYV   | 320x240   | 30  |
| YUYV   | 800x600   | 20  |

- There is NO 256x256 mode. 256x256 is the model/pipeline size (D57), produced by
  cropping+resizing downstream -- NOT a camera setting.
- One camera lacks a by-id string, so BOTH cameras are addressed by `by-path`
  (stable per physical USB port). See devices.example.env.

## *** USB bandwidth: the Jetson has ONE USB 2.0 bus (480 Mbps) ***
All four ports share it, along with both servo adapters. A USB 2.0 device lands
on this bus whichever port it is plugged into, so **swapping ports does not help
and neither does a powered hub** -- the upstream bus is the limit.

Budget (bytes/pixel: YUYV 2.0 exactly; MJPG ~0.15 measured on these cameras):

| config | per camera | two cameras |
|---|---|---|
| YUYV 640x480@30 | 147 Mbps | 294 Mbps |
| MJPG 1024x768@30 | 28 Mbps | **56 Mbps** |

USB 2.0 caps isochronous traffic at ~384 Mbps, so 294 is inside the spec but has
almost no headroom. **=> Use MJPG.**

⚠ **Honest limit of what we know.** The only configuration MEASURED good here is
two cameras at MJPG 1024x768@30, 60 s, zero dropped frames. The 2026-09-05
attempt at two YUYV 640x480@30 did collapse, but the cause was traced to a faulty
wrist-camera cable, **not** to bandwidth (MJPG at 1/5 the data rate collapsed
too, and sooner). So 294 Mbps is *unproven here*, not *known-bad*. Do not cite
that run as a bandwidth result.

## *** Wrist-camera cable fault (open item) ***
The camera mounted on the moving follower drops off the bus at certain bend
angles. Reproduced with the arm STATIONARY, by hand-flexing the cable (failed at
3.5 s). It is mechanical (a cable/connector), not bandwidth and not EMI.
Mitigations, in order: replace the cable; strain-relief a service loop that
tracks the joint; if it persists, suspect the camera-side socket (replace module).
The recorder tolerates this (auto-reconnect + fault log), but demonstrations
recorded while a fault overlaps should be dropped -- check `<cam>_faults.jsonl`.
