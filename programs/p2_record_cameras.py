#!/usr/bin/env python3
"""Program 2 -- the whole rig at once: teleoperation AND cameras, uncoupled.

p1 answers "does the arm work". This answers a different question, and the only
one that needs everything running at the same time:

    ** Does running them together degrade either of them? **

So the arm loop and each camera live in their own thread, share nothing, and
neither can stop the other. What p2 measures is whether that isolation actually
holds: the arm's loop rate and per-step timing are logged here exactly as p1
logs them, so the two runs can be compared directly. If the arm still does
2.7 ms at 120 Hz with two cameras streaming, the threading is honest. If it
does not, this is where you find out -- not halfway through recording a dataset.

    --no-arm   cameras only. That is the old p2, and it is still the tool to
               reach for when a camera misbehaves: no arm, no threads of ours,
               nothing else to blame.

Deliberately NOT here (they belong to p4, the dataset collector):
  * aligning the arm and camera streams onto one timeline
  * episodes
  * "a gap in either stream spoils the recording"
p2 records two independent streams and reports on each. Nothing is joined and
nothing is thrown away, because there is no episode here to spoil.

Fault policy, per stream, unchanged from the family rule:

    p1 (arm)     fault -> STOP and wait for a human   (physical hazard)
    p2 (this)    fault -> that STREAM stops or marks itself; the others carry
                          on untouched                (validation, not data)
    p4 (dataset) fault -> ABORT THE EPISODE           (that episode is spoilt)

An arm fault stops commands but HOLDS torque and does not disconnect, exactly
as in p1: a powered servo keeps its last goal, so the arm freezes instead of
collapsing. The cameras never notice.

Live display is best-effort. If no GUI is available it switches off and
RECORDING CONTINUES. Display must never be able to stop a recording.

Exposure is set to AUTO by default -- set, not merely left alone. V4L2 controls
belong to the DEVICE, so a run that locked the exposure leaves the camera locked
for every run after it, with nothing in the later logs to say so. Mains
frequency (--power-line-hz) is a separate setting and no longer drags the
exposure lock along with it. --exposure lock is what a dataset wants: auto
exposure varies the exposure TIME, so the real frame interval moves and the
brightness drifts across a recording -- and when the arm enters frame the
background brightness starts tracking the arm's pose, which is a cue a policy
will happily learn and which will not survive deployment.
"""
import argparse
import os
import queue
import signal
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm.control import TrackingWatchdog, pose_diff, rate_limit      # noqa: E402
from arm.preflight import run_preflight as arm_preflight             # noqa: E402
from arm.preflight import strip_pos                                  # noqa: E402
from arm.units import UNITS, per_joint                               # noqa: E402
from camera import controls as camctl                                # noqa: E402
from camera.preflight import run_preflight as cam_preflight          # noqa: E402
from camera.resilient_camera import ResilientCamera                  # noqa: E402
from common.clock import epoch, stamp                                # noqa: E402
from common.jsonl import JsonlWriter                                 # noqa: E402

ARM_SCHEMA = "so101.joints.v1"


class ArmWorker(threading.Thread):
    """p1's control loop, in a thread of its own.

    It owns nothing the cameras touch and touches nothing they own. On a fault
    it stops commanding and sets `self.fault`; it does NOT disconnect, so the
    follower stays powered and holds its last goal. The main thread releases
    torque at shutdown -- always, including when this thread died of something
    unexpected.
    """

    def __init__(self, leader, follower, fps, step_limits, track_tol,
                 strikes, rows, events, stop_evt):
        super().__init__(daemon=True, name="arm")
        self.leader, self.follower = leader, follower
        self.fps = fps
        self.step_limits, self.track_tol, self.strikes = step_limits, track_tol, strikes
        self.rows, self.events, self.stop_evt = rows, events, stop_evt
        self.fault = None
        self.steps = 0
        self.clamped = 0
        self.loop_ms = []
        self.hz = None

    def run(self):
        period = 1.0 / self.fps
        watchdog = TrackingWatchdog(self.track_tol, self.strikes)
        armed = False
        # Seed from where the follower ACTUALLY is: rate_limit() passes a joint
        # straight through when it has no previous command, so an unseeded
        # first step is the one step that could snap the arm across a gap.
        try:
            prev_cmd = strip_pos(self.follower.get_observation())
        except Exception as e:
            self.events.event("arm_seed_failed", err=repr(e))
            prev_cmd = None
        self.events.event("arm_started", fps=self.fps, seeded=prev_cmd is not None)
        t_first = time.monotonic()
        try:
            while not self.stop_evt.is_set():
                t0 = time.monotonic()
                target = strip_pos(self.leader.get_action())
                t1 = time.monotonic()
                meas = strip_pos(self.follower.get_observation())
                t2 = time.monotonic()
                cmd, n = rate_limit(target, prev_cmd, self.step_limits)
                prev_cmd = cmd
                self.clamped += n
                self.follower.send_action({f"{k}.pos": v for k, v in cmd.items()})
                t3 = time.monotonic()

                _, wj, wv = pose_diff(cmd, meas)
                self.rows.write({"schema": ARM_SCHEMA, "seq": self.steps, **stamp(),
                                 "leader": target, "follower": meas, "command": cmd,
                                 "units": UNITS,
                                 "dt_read_leader_ms": round((t1 - t0) * 1e3, 3),
                                 "dt_read_follower_ms": round((t2 - t1) * 1e3, 3),
                                 "dt_write_ms": round((t3 - t2) * 1e3, 3),
                                 "dt_loop_ms": round((t3 - t0) * 1e3, 3),
                                 "clamped": n,
                                 "track_worst_joint": wj,
                                 "track_worst_deg": round(wv, 2)})
                self.loop_ms.append((t3 - t0) * 1e3)
                self.steps += 1

                if not armed:
                    if watchdog.in_tolerance(cmd, meas):
                        armed = True
                        self.events.event("arm_watchdog_armed", seq=self.steps)
                elif watchdog.update(cmd, meas):
                    self.fault = "tracking_watchdog: " + watchdog.reason()
                    break
                sleep = period - (time.monotonic() - t0)
                if sleep > 0:
                    time.sleep(sleep)
        except Exception as e:
            self.fault = f"{type(e).__name__}: {e}"
        dt = time.monotonic() - t_first
        self.hz = self.steps / dt if dt > 0 else None
        if self.fault:
            self.events.event("arm_fault", reason=self.fault, seq=self.steps)
            print(f"\n[p2] ARM FAULT -- no further commands. The follower is "
                  f"HOLDING (torque still on). Cameras are unaffected.\n"
                  f"      {self.fault}\n")
        self.events.event("arm_stopped", steps=self.steps, hz=self.hz,
                          clamped=self.clamped, fault=self.fault)


class Console(threading.Thread):
    """Line commands when there is no GUI. EOF at a terminal means quit; EOF
    with no terminal means there is no keyboard, which is not a command."""

    def __init__(self):
        super().__init__(daemon=True)
        self.q = queue.Queue()

    def run(self):
        try:
            tty = bool(sys.stdin) and sys.stdin.isatty()
        except (ValueError, AttributeError):
            tty = False
        while True:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                if tty:
                    self.q.put("q")
                return
            except Exception:
                return
            self.q.put(line.strip().lower())

    def get(self):
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return None


def parse_cam(spec):
    name, dev = spec.split("=", 1)
    return name.strip(), dev.strip()


def tile(frames, names, scale, status):
    """One window, cameras side by side. Cheaper over a remote desktop than N
    windows, and it keeps the status line in view."""
    shown = []
    for n in names:
        f = frames.get(n)
        if f is None:
            f = np.zeros((120, 160, 3), dtype=np.uint8)
            cv2.putText(f, "no frame", (8, 64), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 1)
        else:
            f = cv2.resize(f, (int(f.shape[1] * scale), int(f.shape[0] * scale)))
        cv2.putText(f, n, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        shown.append(f)
    h = max(f.shape[0] for f in shown)
    shown = [cv2.copyMakeBorder(f, 0, h - f.shape[0], 0, 0,
                                cv2.BORDER_CONSTANT, value=(0, 0, 0))
             for f in shown]
    grid = np.hstack(shown)
    bar = np.zeros((28, grid.shape[1], 3), dtype=np.uint8)
    cv2.putText(bar, status[:200], (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1)
    return np.vstack([grid, bar])


def build_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", action="append", default=[],
                    help="NAME=BYPATH ; repeatable. Defaults to CAM_WRIST/CAM_FRONT")
    ap.add_argument("--rotate", action="append", default=[],
                    help="NAME=DEG (0/90/180/270) ; repeatable. Defaults to "
                         "ROTATE_<NAME> in the environment. Applied at the "
                         "SOURCE, so the file and the live view agree")
    ap.add_argument("--width", type=int, default=1024)
    ap.add_argument("--height", type=int, default=768)
    ap.add_argument("--fps", type=int, default=30, help="camera rate")
    ap.add_argument("--fourcc", default="MJPG",
                    help="MJPG keeps the single USB 2.0 bus comfortable")
    ap.add_argument("--no-arm", action="store_true",
                    help="cameras only -- the old p2, for blaming a camera")
    ap.add_argument("--arm-fps", type=float, default=60.0,
                    help="arm loop rate, independent of the camera rate")
    ap.add_argument("--leader-port", default=os.environ.get("LEADER"))
    ap.add_argument("--follower-port", default=os.environ.get("FOLLOWER"))
    ap.add_argument("--leader-id", default="my_leader")
    ap.add_argument("--follower-id", default="my_follower")
    ap.add_argument("--leader-serial", default=os.environ.get("LEADER_SERIAL"))
    ap.add_argument("--follower-serial", default=os.environ.get("FOLLOWER_SERIAL"))
    ap.add_argument("--max-step-deg", type=float, default=8.0)
    ap.add_argument("--max-step-gripper-pct", type=float, default=15.0)
    ap.add_argument("--track-tol-deg", type=float, default=25.0)
    ap.add_argument("--track-tol-gripper-pct", type=float, default=30.0)
    ap.add_argument("--track-strikes", type=int, default=15)
    ap.add_argument("--out", default="./logs/p2")
    ap.add_argument("--display", choices=["on", "off"], default="on")
    ap.add_argument("--display-scale", type=float, default=0.5)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = until q/Enter")
    ap.add_argument("--exposure", choices=["auto", "lock", "leave"],
                    default="auto",
                    help="auto (default): explicitly restore auto exposure and "
                         "auto white balance -- V4L2 controls are DEVICE state, "
                         "so a previous locked run would otherwise carry over "
                         "silently. lock: pin them (what a dataset wants). "
                         "leave: touch nothing, whatever the camera already has")
    ap.add_argument("--lock-exposure", action="store_true",
                    help="deprecated spelling of --exposure lock")
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

    rot = {}
    for spec in args.rotate:
        n, d = spec.split("=", 1)
        rot[n.strip()] = int(d)
    for name, _dev in cams:
        if name not in rot:
            rot[name] = int(os.environ.get(f"ROTATE_{name.upper()}", 0))
    bad = {n: d for n, d in rot.items() if d % 360 not in (0, 90, 180, 270)}
    if bad:
        ap.error(f"--rotate must be 0/90/180/270: {bad}")
    use_arm = not args.no_arm
    if use_arm and not (args.leader_port and args.follower_port):
        ap.error("need LEADER/FOLLOWER (source devices.env), or pass --no-arm")

    run_dir = os.path.join(args.out, time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    cam_rows = JsonlWriter(os.path.join(run_dir, "cam_rows.jsonl"))
    arm_rows = JsonlWriter(os.path.join(run_dir, "arm_rows.jsonl"))
    events = JsonlWriter(os.path.join(run_dir, "events.jsonl"))
    events.event("start", argv=sys.argv[1:], epoch=epoch(), run_dir=run_dir,
                 cams=dict(cams), arm=use_arm, arm_fps=args.arm_fps,
                 rotate=rot,
                 mode={"width": args.width, "height": args.height,
                       "fps": args.fps, "fourcc": args.fourcc})
    print(f"[p2] logging to {run_dir}")
    print(f"[p2] {len(cams)} camera(s)" + ("" if use_arm else " -- ARM DISABLED"))
    for name, _dev in cams:
        if rot[name]:
            print(f"[p2] {name}: rotated {rot[name]} deg at the source "
                  f"-- the recording and the live view both get it")

    leader = follower = None
    if use_arm:
        from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
        from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
        follower = SOFollower(SOFollowerRobotConfig(
            port=args.follower_port, id=args.follower_id, use_degrees=True,
            max_relative_target=None))
        leader = SOLeader(SOLeaderTeleopConfig(
            port=args.leader_port, id=args.leader_id, use_degrees=True))

    def release():
        for obj in (follower, leader):
            if obj is None:
                continue
            try:
                obj.disconnect()
            except Exception as e:
                events.event("disconnect_error", err=repr(e))

    if not args.skip_preflight:
        print("[p2] camera pre-flight ...")
        report, info = cam_preflight(cams, args.width, args.height, args.fps,
                                     args.fourcc, run_dir, cv2=cv2,
                                     controls_mod=camctl)
        print(report.render())
        events.event("cam_preflight", ok=report.ok, checks=report.rows, info=info)
        if not report.ok:
            print("\n[p2] CAMERA PRE-FLIGHT FAILED -- not recording.")
            for w in (cam_rows, arm_rows, events):
                w.close()
            return 1
        if use_arm:
            print("[p2] arm pre-flight ...")
            areport, ainfo = arm_preflight(
                leader, follower, args.leader_port, args.follower_port,
                expect_leader_serial=args.leader_serial,
                expect_follower_serial=args.follower_serial)
            print(areport.render())
            events.event("arm_preflight", ok=areport.ok, checks=areport.rows,
                         info=ainfo)
            if not areport.ok:
                print("\n[p2] ARM PRE-FLIGHT FAILED -- releasing torque, not "
                      "starting. (--no-arm records the cameras alone.)")
                release()
                for w in (cam_rows, arm_rows, events):
                    w.close()
                return 1
    elif use_arm:
        leader.connect(calibrate=False)
        follower.connect(calibrate=False)

    # Exposure and mains frequency are INDEPENDENT. Asking for anti-flicker
    # used to lock the exposure as a side effect, so there was no way to have
    # one without the other -- and because V4L2 controls live on the device,
    # that lock then survived into every later run.
    mode = "lock" if args.lock_exposure else args.exposure
    for name, dev in cams:
        steps = []
        if mode == "lock":
            steps += camctl.lock_exposure(dev, args.exposure_time,
                                          args.white_balance)
        elif mode == "auto":
            steps += camctl.unlock_exposure(dev)
        if args.power_line_hz is not None:
            steps += camctl.set_power_line(dev, args.power_line_hz)
        if not steps:
            continue
        events.event("camera_controls", cam=name, mode=mode, steps=steps)
        for st in steps:
            print(f"[p2] {name}: {'ok ' if st['ok'] else 'FAIL'} {st['step']}"
                  + (f"  {st['detail']}" if st["detail"] else ""))

    writer_fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    rc, writers, counts, out_size = {}, {}, {}, {}
    for name, dev in cams:
        cam = ResilientCamera(
            name, dev, args.width, args.height, args.fps, fourcc=args.fourcc,
            rotate=rot[name],
            fault_log=os.path.join(run_dir, f"{name}_faults.jsonl"))
        rc[name] = cam.start()
        # 90 and 270 swap the frame's width and height; a writer sized for the
        # sensor's mode would silently write nothing.
        out_size[name] = cam.out_size()
        writers[name] = cv2.VideoWriter(os.path.join(run_dir, f"{name}.avi"),
                                        writer_fourcc, args.fps, out_size[name])
        counts[name] = 0
    time.sleep(1.0)
    events.event("controls_after_open",
                 **{n: camctl.snapshot(d) for n, d in cams})

    stop_evt = threading.Event()
    stop_why = {"reason": None}

    def _sig(signum, _f):
        stop_why["reason"] = f"signal:{signal.Signals(signum).name}"
        stop_evt.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    arm = None
    if use_arm:
        arm = ArmWorker(leader, follower, args.arm_fps,
                        per_joint(args.max_step_deg, args.max_step_gripper_pct),
                        per_joint(args.track_tol_deg, args.track_tol_gripper_pct),
                        args.track_strikes, arm_rows, events, stop_evt)
        arm.start()

    console = Console()
    console.start()
    display = args.display == "on"
    period = 1.0 / args.fps
    seq = 0
    print(f"\n[p2] RUNNING. " + ("Move the leader; the follower follows. "
                                 if use_arm else "")
          + "Press q in the window, or Enter here, to stop.")
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while not stop_evt.is_set():
            t0 = time.monotonic()
            per_cam, frames = {}, {}
            for name, _dev in cams:
                frame, stale, state = rc[name].read_latest()
                entry = {"state": state,
                         "stale_s": (round(stale, 4) if stale != float("inf")
                                     else None)}
                if frame is not None:
                    if (frame.shape[1], frame.shape[0]) != out_size[name]:
                        frame = cv2.resize(frame, out_size[name])
                    writers[name].write(frame)
                    entry["idx"] = counts[name]
                    counts[name] += 1
                    frames[name] = frame
                else:
                    entry["idx"] = None
                per_cam[name] = entry
            cam_rows.write({"seq": seq, **stamp(), "cams": per_cam})
            seq += 1

            if display:
                a = ""
                if arm is not None:
                    a = (f" | arm {arm.steps} steps"
                         + (f" FAULT" if arm.fault else
                            f" {(arm.hz or args.arm_fps):.0f}Hz"))
                status = (" ".join(f"{n}:{rc[n].state}/rc{rc[n].stats['reconnects']}"
                                   for n, _ in cams) + a)
                try:
                    cv2.imshow("p2", tile(frames, [n for n, _ in cams],
                                          args.display_scale, status))
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        stop_why["reason"] = "q"
                        break
                except cv2.error:
                    display = False
                    print("[p2] display unavailable -> recording continues")

            k = console.get()
            if k is not None:
                stop_why["reason"] = "enter"
                break
            if t_end and time.monotonic() >= t_end:
                stop_why["reason"] = "duration"
                break
            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)
    finally:
        stop_evt.set()
        if arm is not None:
            arm.join(timeout=5.0)
        release()
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
        arm_stats = None
        if arm is not None:
            import statistics as st
            arm_stats = {"steps": arm.steps, "hz": arm.hz,
                         "clamped": arm.clamped, "fault": arm.fault,
                         "loop_ms_median": (round(st.median(arm.loop_ms), 3)
                                            if arm.loop_ms else None)}
        events.event("stop", reason=stop_why["reason"], cam_steps=seq,
                     summary=summary, arm=arm_stats)
        for w in (cam_rows, arm_rows, events):
            w.close()

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
    if arm_stats:
        print(f"  arm: {arm_stats['steps']} steps at "
              f"{(arm_stats['hz'] or 0):.1f} Hz "
              f"(asked for {args.arm_fps:.0f}), "
              f"loop {arm_stats['loop_ms_median']} ms median, "
              f"{arm_stats['clamped']} clamped")
        if arm_stats["fault"]:
            print(f"    ** arm faulted: {arm_stats['fault']}")
        if arm_stats["hz"] and arm_stats["hz"] < 0.9 * args.arm_fps:
            print(f"    ** the arm loop ran more than 10% below its target "
                  f"while the cameras were streaming. Compare against a p1 run "
                  f"at the same --fps: if p1 holds the rate and this does not, "
                  f"the two are contending and p4 cannot rely on this rig.")
    print(f"[p2] output in {run_dir}")
    return 2 if (arm_stats and arm_stats["fault"]) else 0


if __name__ == "__main__":
    sys.exit(main())
