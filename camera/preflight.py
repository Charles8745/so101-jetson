"""Pre-flight self-check for the USB cameras.

The camera equivalent of arm/preflight.py, with one honest difference:

  ** cameras cannot be identity-checked by serial. **

The arms each carry a unique USB serial, so a swap is caught before anything
moves. These two cameras are the same model; one has no by-id entry at all and
the other's carries only vendor and product -- no serial. The ONLY thing that
distinguishes them is which physical USB port they are in, which is what by-path
encodes. Move a cable to another port and the labels are silently wrong.

Mitigation: save one frame from each camera at startup. Here that is decisive by
content -- the wrist camera sees the gripper, the external one sees the bench.

Checks:
  [1] each by-path node exists
  [2] the cameras are not the same device
  [3] identity: by-path registered, snapshot saved for after-the-fact proof
  [4] mode negotiation: what we ASKED for vs what we actually GOT
  [5] USB bandwidth budget against the single 480 Mbps bus
  [6] control dump (exposure / white balance / mains frequency)
  [7] simultaneous capture probe: real fps, failures, frame-interval jitter
"""
import os
import statistics as st
import time

# Bytes per pixel per frame.
#   YUYV is uncompressed 4:2:2 -> exactly 2.
#   MJPG measured on these Sonix cameras: 1024x768 frames land near 100 kB,
#   i.e. ~0.13 B/px. We use 0.15 to stay on the pessimistic side.
BYTES_PER_PIXEL = {"YUYV": 2.0, "MJPG": 0.15}

# One USB 2.0 bus. This Jetson has exactly one, shared by both cameras and both
# servo adapters (see docs/HARDWARE.md).
USB2_TOTAL_MBPS = 480.0
USB2_ISO_BUDGET_MBPS = 384.0   # spec caps isochronous at 80% of the frame

# What we have actually MEASURED good on this Jetson, as opposed to what the
# spec allows: two cameras at MJPG 1024x768@30 (~56 Mbps total) ran 60 s with
# zero dropped frames. Nothing above that has been cleanly tested here -- the
# 2026-09-05 attempt at YUYV 640x480@30 did collapse, but the cause was traced
# to a faulty wrist-camera cable, NOT to bandwidth. So treat anything much above
# this as unproven rather than as known-bad.
MEASURED_GOOD_MBPS = 60.0


def bandwidth_mbps(width, height, fps, fourcc):
    bpp = BYTES_PER_PIXEL.get(fourcc.upper())
    if bpp is None:
        return None
    return width * height * bpp * fps * 8.0 / 1e6


def budget_report(cams, width, height, fps, fourcc):
    """cams: list of (name, device). Returns (ok, unproven, total, per_cam).

    ok       -- within the USB 2.0 isochronous budget (hard limit)
    unproven -- above the highest configuration measured good on this machine
    """
    per = bandwidth_mbps(width, height, fps, fourcc)
    if per is None:
        return True, False, None, None
    total = per * len(cams)
    return (total <= USB2_ISO_BUDGET_MBPS, total > MEASURED_GOOD_MBPS,
            total, per)


def negotiated_mode(cap, cv2):
    """What the driver actually gave us, as opposed to what we asked for."""
    fourcc_i = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((fourcc_i >> (8 * i)) & 0xFF) for i in range(4))
    return {"width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": round(float(cap.get(cv2.CAP_PROP_FPS)), 3),
            "fourcc": fourcc.strip("\x00")}


def mode_matches(asked, got, fps_tol=1.0):
    bad = []
    if got["width"] != asked["width"] or got["height"] != asked["height"]:
        bad.append(f"size {got['width']}x{got['height']} != "
                   f"{asked['width']}x{asked['height']}")
    if abs(got["fps"] - asked["fps"]) > fps_tol:
        bad.append(f"fps {got['fps']} != {asked['fps']}")
    if got["fourcc"].upper() != asked["fourcc"].upper():
        bad.append(f"fourcc {got['fourcc']} != {asked['fourcc']}")
    return (not bad), "; ".join(bad)


def jitter_ms(times):
    if len(times) < 3:
        return None, None
    gaps = [(b - a) * 1e3 for a, b in zip(times, times[1:])]
    return st.median(gaps), (max(gaps) - min(gaps))


def run_preflight(cams, width, height, fps, fourcc, out_dir,
                  probe_frames=60, cv2=None, controls_mod=None):
    """cams: list of (name, device). Opens each camera, probes, then RELEASES.

    Returns (CheckResult-like object, info dict). Import cv2 lazily via arg so
    the pure helpers above stay testable without OpenCV.
    """
    from arm.preflight import CheckResult
    if cv2 is None:
        import cv2 as cv2
    r = CheckResult()
    info = {}

    for name, dev in cams:
        r.add(f"[1] {name} node exists", os.path.exists(dev), dev)
    real = [os.path.realpath(d) for _, d in cams]
    r.add("[2] cameras are distinct devices", len(set(real)) == len(real),
          f"resolved: {real}")
    if not r.ok:
        return r, info

    r.add("[3] identity is by-path only", True,
          "cameras have no usable serial; a snapshot per camera is saved so a "
          "wrong label can be seen after the fact", fatal=False)

    ok_b, warn_b, total, per = budget_report(cams, width, height, fps, fourcc)
    info["bandwidth_mbps"] = {"per_camera": per, "total": total,
                              "iso_budget": USB2_ISO_BUDGET_MBPS}
    detail = (f"{len(cams)} x {per:.0f} = {total:.0f} Mbps against a "
              f"{USB2_ISO_BUDGET_MBPS:.0f} Mbps isochronous budget on one "
              f"{USB2_TOTAL_MBPS:.0f} Mbps bus" if per else "unknown fourcc")
    if per and not ok_b:
        detail += "  <-- over the USB 2.0 budget: use MJPG, or lower res/fps"
    elif per and warn_b:
        detail += (f"  <-- above the {MEASURED_GOOD_MBPS:.0f} Mbps we have "
                   f"actually measured good here; unproven, not known-bad")
    r.add("[5] USB bandwidth budget", ok_b, detail)
    if not r.ok:
        return r, info

    caps, modes, firsts = {}, {}, {}
    asked = {"width": width, "height": height, "fps": float(fps),
             "fourcc": fourcc}
    try:
        for name, dev in cams:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                r.add(f"[4] {name} opens", False, dev)
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            caps[name] = cap
            got = negotiated_mode(cap, cv2)
            modes[name] = got
            same, why = mode_matches(asked, got)
            r.add(f"[4] {name} mode as requested", same,
                  f"asked {width}x{height}@{fps} {fourcc}, got "
                  f"{got['width']}x{got['height']}@{got['fps']} {got['fourcc']}"
                  + (f"  ({why})" if why else ""))
        info["negotiated"] = modes
        if not r.ok:
            return r, info

        if controls_mod is not None:
            info["controls"] = {n: controls_mod.snapshot(d) for n, d in cams}
            r.add("[6] control dump", True,
                  "exposure / white balance / mains frequency recorded",
                  fatal=False)

        os.makedirs(out_dir, exist_ok=True)
        stamps = {n: [] for n, _ in cams}
        fails = {n: 0 for n, _ in cams}
        for _ in range(probe_frames):
            for name, _dev in cams:
                ok, frame = caps[name].read()
                if ok:
                    stamps[name].append(time.monotonic())
                    if name not in firsts:
                        firsts[name] = frame
                else:
                    fails[name] += 1
        probe = {}
        for name, _dev in cams:
            med, spread = jitter_ms(stamps[name])
            hz = (1000.0 / med) if med else 0.0
            probe[name] = {"frames": len(stamps[name]), "failures": fails[name],
                           "median_gap_ms": med, "gap_spread_ms": spread,
                           "hz": round(hz, 1)}
            r.add(f"[7] {name} capture probe", fails[name] == 0,
                  f"{len(stamps[name])} frames, {fails[name]} failures, "
                  f"{hz:.1f} Hz, gap spread {spread:.1f} ms"
                  if med else "no frames")
        info["probe"] = probe

        for name, frame in firsts.items():
            p = os.path.join(out_dir, f"{name}_first.jpg")
            cv2.imwrite(p, frame)
            info.setdefault("snapshots", {})[name] = p
        r.add("[3] startup snapshots saved", len(firsts) == len(cams),
              ", ".join(info.get("snapshots", {}).values()), fatal=False)
    finally:
        for cap in caps.values():
            try:
                cap.release()
            except Exception:
                pass
    return r, info
