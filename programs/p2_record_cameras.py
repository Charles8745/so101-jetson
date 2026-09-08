#!/usr/bin/env python3
"""Program 2 -- camera recording + live view (hardware validation).

Runs entirely on the Jetson with no network layer, for the same reason as p1: a
diagnostic tool must be the shortest path or a failure cannot be attributed.

Exception policy for cameras is deliberately NOT the same as for the arm:

    p1 (arm)     fault -> STOP and wait for a human   (physical hazard)
    p2 (this)    fault -> KEEP GOING and mark it      (data-quality only)
    p4 (dataset) fault -> ABORT THE EPISODE           (that episode is spoilt)

Same components, three policies, chosen by the caller. A camera hiccup must not
throw away the other camera's data -- but it must never pass unrecorded either,
so every drop and every reconnect is written to <cam>_faults.jsonl with its
magnitude (frames, seconds down, reconnect count), not as a boolean.

Live display is best-effort: if no GUI is available it is switched off and
RECORDING CONTINUES. Display must never be able to stop a recording.
"""
import argparse
import json
import os
import signal
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera import controls as camctl                    # noqa: E402
from camera.preflight import run_preflight               # noqa: E402
from camera.resilient_camera import ResilientCamera      # noqa: E402
from common.clock import epoch, stamp                    # noqa: E402
from common.jsonl import JsonlWriter                     # noqa: E402


def parse_cam(spec):
    name, dev = spec.split("=", 1)
    return name.strip(), dev.strip()


def build_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", action="append", default=[],
                    help="NAME=BYPATH ; repeatable. Defaults to CAM_WRIST/CAM_FRONT")
    ap.add_argument("--width", type=int, default=1024)
    ap.add_argument("--height", type=int, default=768)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--fourcc", default="MJPG",
                    help="MJPG keeps the single USB 2.0 bus comfortable")
    ap.add_argument("--out", default="./logs/p2")
    ap.add_argument("--display", choices=["on", "off"], default="on")
    ap.add_argument("--duration", type=float, default=0.0, help="0 = until q/Ctrl+C")
    ap.add_argument("--lock-exposure", action="store_true",
                    help="turn OFF auto exposure and auto white balance "
                         "(default for datasets; off here, this is a test tool)")
    ap.add_argument("--exposure-time", type=int, default=None)
    ap.add_argument("--white-balance", type=int, default=None)
    ap.add_argument("--power-line-hz", type=int, default=None,
                    choices=[0, 50, 60], help="Taiwan mains is 60")
    ap.add_argument("--skip-preflight", action="store_true")
    return ap


def main():
    ap = build_args()
    args = ap.parse_args()

    cams = [parse_cam(c) for c in args.cam]
    if not cams:
        for env, nm in (("CAM_WRIST", "wrist"), ("CAM_FRONT", "front")):
            if os.environ.get(env):
                cams.append((nm, os.environ[env]))
    if not cams:
        ap.error("no cameras: pass --cam NAME=BYPATH or set CAM_WRIST/CAM_FRONT")

    run_dir = os.path.join(args.out, time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    rows = JsonlWriter(os.path.join(run_dir, "rows.jsonl"))
    events = JsonlWriter(os.path.join(run_dir, "events.jsonl"))
    events.event("start", argv=sys.argv[1:], epoch=epoch(), run_dir=run_dir,
                 cams=dict(cams), mode={"width": args.width,
                                        "height": args.height,
                                        "fps": args.fps, "fourcc": args.fourcc})
    print(f"[p2] logging to {run_dir}")

    if not args.skip_preflight:
        print("[p2] pre-flight ...")
        report, info = run_preflight(cams, args.width, args.height, args.fps,
                                     args.fourcc, run_dir, cv2=cv2,
                                     controls_mod=camctl)
        print(report.render())
        events.event("preflight", ok=report.ok, checks=report.rows, info=info)
        if not report.ok:
            print("\n[p2] PRE-FLIGHT FAILED -- not recording.")
            rows.close()
            events.close()
            return 1

    if args.lock_exposure or args.power_line_hz is not None:
        for name, dev in cams:
            steps = camctl.lock_exposure(dev, args.exposure_time,
                                         args.white_balance, args.power_line_hz)
            events.event("lock_exposure", cam=name, steps=steps)
            for s in steps:
                print(f"[p2] {name}: {'ok ' if s['ok'] else 'FAIL'} {s['step']}"
                      + (f"  {s['detail']}" if s["detail"] else ""))

    writer_fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    rc, writers, counts = {}, {}, {}
    for name, dev in cams:
        rc[name] = ResilientCamera(
            name, dev, args.width, args.height, args.fps, fourcc=args.fourcc,
            fault_log=os.path.join(run_dir, f"{name}_faults.jsonl")).start()
        writers[name] = cv2.VideoWriter(os.path.join(run_dir, f"{name}.avi"),
                                        writer_fourcc, args.fps,
                                        (args.width, args.height))
        counts[name] = 0

    # verify the controls actually took, once the cameras are streaming
    time.sleep(1.0)
    events.event("controls_after_open",
                 **{n: camctl.snapshot(d) for n, d in cams})

    stop = {"flag": False}

    def _sig(signum, _f):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    display = args.display == "on"
    period = 1.0 / args.fps
    seq = 0
    print(f"\n[p2] RECORDING {len(cams)} camera(s). "
          f"{'Press q in a window or ' if display else ''}Ctrl+C to stop.")
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while not stop["flag"]:
            t0 = time.monotonic()
            per_cam = {}
            for name, _dev in cams:
                frame, stale, state = rc[name].read_latest()
                entry = {"state": state, "stale_s": round(stale, 4)
                         if stale != float("inf") else None}
                if frame is not None:
                    if (frame.shape[1], frame.shape[0]) != (args.width, args.height):
                        frame = cv2.resize(frame, (args.width, args.height))
                    writers[name].write(frame)
                    entry["idx"] = counts[name]
                    counts[name] += 1
                    if display:
                        try:
                            show = frame.copy()
                            cv2.putText(show, f"{name} {state} "
                                        f"stale={stale:.2f}s "
                                        f"rc={rc[name].stats['reconnects']}",
                                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                        (0, 255, 0), 2)
                            cv2.imshow(name, show)
                        except cv2.error:
                            display = False
                            print("[p2] display unavailable -> recording continues")
                else:
                    entry["idx"] = None
                per_cam[name] = entry
            rows.write({"seq": seq, **stamp(), "cams": per_cam})
            seq += 1
            if display:
                try:
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break
                except cv2.error:
                    display = False
            if t_end and time.monotonic() >= t_end:
                break
            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)
    finally:
        summary = {}
        for name, _dev in cams:
            rc[name].stop()
            writers[name].release()
            summary[name] = {"written_frames": counts[name], **rc[name].stats,
                             "final_mode": rc[name].mode}
        if display:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        events.event("stop", steps=seq, summary=summary)
        rows.close()
        events.close()

    print("\n[p2] summary")
    for name, s in summary.items():
        print(f"  {name}: {s['written_frames']} frames written, "
              f"{s['dropped']} dropped, {s['reconnects']} reconnects, "
              f"{s['total_down_s']:.1f}s down, "
              f"{s['mode_changes']} mode changes")
        if s["mode_changes"]:
            print(f"    ** mode changed during the run -- see "
                  f"{name}_faults.jsonl. Data before and after are NOT the "
                  f"same exam paper.")
    print(f"[p2] output in {run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
