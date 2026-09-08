#!/usr/bin/env python3
"""Program 2 -- resilient multi-camera recording + live display.

Runs as an INDEPENDENT process. It owns the cameras only and never touches the
servo bus, so an arm failure cannot affect it and vice-versa.

  * Records each camera to a raw MJPG-in-AVI file plus a timestamp sidecar CSV.
  * Live display via cv2.imshow, BEST-EFFORT: if no GUI is available the display
    is disabled and recording KEEPS GOING. Display never blocks recording.
  * Each camera reconnects on its own (see resilient_camera.py). One camera
    dying does not stop the others.
  * SIGINT/SIGTERM -> flush and finalize every video file.

Alignment: both this program and teleop_arm.py stamp t_mono from the same
system clock, so the arm signal (JSONL) and each camera (timestamp CSV) can be
merged into one synchronized episode offline.
"""
import argparse
import csv
import json
import os
import signal
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera.resilient_camera import ResilientCamera  # noqa: E402
from common.clock import epoch                        # noqa: E402


def parse_cam(spec):
    name, dev = spec.split("=", 1)
    return name.strip(), dev.strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cam", action="append", default=[],
                    help='NAME=BYPATH ; repeatable. '
                         'e.g. wrist=/dev/v4l/by-path/...-index0')
    ap.add_argument("--width", type=int, default=1024)
    ap.add_argument("--height", type=int, default=768)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--fourcc", default="MJPG",
                    help="capture fourcc; MJPG keeps USB 2.0 bandwidth low")
    ap.add_argument("--out", default="./recordings")
    ap.add_argument("--display", choices=["on", "off"], default="on")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds; 0 = run until Ctrl+C / 'q'")
    args = ap.parse_args()

    cams = [parse_cam(c) for c in args.cam]
    if not cams:
        for env, nm in [("CAM_WRIST", "wrist"), ("CAM_FRONT", "front")]:
            if os.environ.get(env):
                cams.append((nm, os.environ[env]))
    if not cams:
        ap.error("no cameras: pass --cam NAME=BYPATH or set CAM_WRIST/CAM_FRONT")

    outdir = os.path.join(args.out, time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "clock_epoch.json"), "w") as fh:
        json.dump(epoch(), fh)

    writer_fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    writers, sidecars, rc = {}, {}, {}
    for name, dev in cams:
        rc[name] = ResilientCamera(
            name, dev, args.width, args.height, args.fps, fourcc=args.fourcc,
            fault_log=os.path.join(outdir, f"{name}_faults.jsonl")).start()
        writers[name] = cv2.VideoWriter(
            os.path.join(outdir, f"{name}.avi"),
            writer_fourcc, args.fps, (args.width, args.height))
        fh = open(os.path.join(outdir, f"{name}_timestamps.csv"), "w", newline="")
        w = csv.writer(fh)
        w.writerow(["frame_idx", "t_mono", "t_unix", "stale_s", "state"])
        sidecars[name] = (fh, w)

    stop = {"flag": False}

    def _sig(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    display = args.display == "on"
    counts = {n: 0 for n, _ in cams}
    period = 1.0 / args.fps
    print(f"[record] {len(cams)} cam(s) -> {outdir}  display={display}")
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while not stop["flag"]:
            t0 = time.monotonic()
            for name, _ in cams:
                frame, stale, state = rc[name].read_latest()
                if frame is None:
                    continue
                if (frame.shape[1], frame.shape[0]) != (args.width, args.height):
                    frame = cv2.resize(frame, (args.width, args.height))
                writers[name].write(frame)
                fh, w = sidecars[name]
                w.writerow([counts[name], round(t0, 6), round(time.time(), 6),
                            round(stale, 4), state])
                counts[name] += 1
                if display:
                    try:
                        show = frame.copy()
                        cv2.putText(show, f"{name} {state} stale={stale:.2f}s",
                                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 0), 2)
                        cv2.imshow(name, show)
                    except cv2.error:
                        display = False
                        print("[record] display unavailable -> "
                              "recording continues")
            if display:
                try:
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break
                except cv2.error:
                    display = False
            if t_end and time.monotonic() >= t_end:
                break
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)
    finally:
        for name, _ in cams:
            rc[name].stop()
            writers[name].release()
            sidecars[name][0].flush()
            sidecars[name][0].close()
        if display:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        print("[record] finalized frame counts:",
              {n: counts[n] for n, _ in cams})


if __name__ == "__main__":
    main()
