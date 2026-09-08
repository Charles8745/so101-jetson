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
All four ports share it. Two cameras streaming **uncompressed YUYV 640x480@30**
= ~294 Mbps and they collapse within ~10 s (errno 71 EPROTO, device re-enumerates).
Neither swapping ports nor a powered hub helps -- the upstream bus is the limit.
=> Use **MJPG** (compressed at the camera). MJPG 1024x768@30 for two cameras is
~32-80 Mbps and fits easily. If you ever add a third camera, redo this budget.

## *** Wrist-camera cable fault (open item) ***
The camera mounted on the moving follower drops off the bus at certain bend
angles. Reproduced with the arm STATIONARY, by hand-flexing the cable (failed at
3.5 s). It is mechanical (a cable/connector), not bandwidth and not EMI.
Mitigations, in order: replace the cable; strain-relief a service loop that
tracks the joint; if it persists, suspect the camera-side socket (replace module).
The recorder tolerates this (auto-reconnect + fault log), but demonstrations
recorded while a fault overlaps should be dropped -- check `<cam>_faults.jsonl`.
