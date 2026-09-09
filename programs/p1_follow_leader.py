#!/usr/bin/env python3
"""Program 1 -- follower follows leader (hardware validation).

Purpose: prove the arm pair is wired, calibrated and addressable BEFORE anything
else is built on top. It runs entirely on the Jetson with no network layer, on
purpose: a diagnostic tool must be the shortest possible path, or a failure
cannot be attributed.

How teleoperation actually works, in one line: a Feetech STS servo is a position
servo -- you write a target angle and its internal PID drives there. So
"following" is just: read the leader's six angles, write them as the follower's
six goal positions, thirty times a second.

Torque policy (as specified):
  * A fault does NOT release torque. We simply stop sending new goals and do NOT
    disconnect -- a powered servo keeps holding its last goal by itself. The arm
    freezes where it is instead of collapsing.
  * Torque is released on exactly two exits: you press Enter, or the program is
    closed (Ctrl+C / SIGTERM). Both go through disconnect(), which disables
    torque because disable_torque_on_disconnect is True.

Safety, without extra bus traffic:
  * rate limit against the previous command (see arm/control.py)
  * tracking watchdog on |command - measured|
  * pre-flight refuses to start if leader and follower poses disagree
"""
import argparse
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm.control import TrackingWatchdog, pose_diff, rate_limit   # noqa: E402
from arm.preflight import run_preflight, strip_pos                # noqa: E402
from arm.units import UNITS, per_joint                             # noqa: E402
from arm.signal_pub import SCHEMA, SignalPublisher                # noqa: E402
from common.clock import epoch, stamp                             # noqa: E402
from common.jsonl import JsonlWriter                              # noqa: E402


def build_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--leader-port", default=os.environ.get("LEADER"))
    ap.add_argument("--follower-port", default=os.environ.get("FOLLOWER"))
    ap.add_argument("--leader-id", default="my_leader")
    ap.add_argument("--follower-id", default="my_follower")
    ap.add_argument("--leader-serial", default=os.environ.get("LEADER_SERIAL"))
    ap.add_argument("--follower-serial", default=os.environ.get("FOLLOWER_SERIAL"))
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--max-step-deg", type=float, default=8.0,
                    help="max BODY joint move per step, vs previous command (0=off)")
    ap.add_argument("--max-step-gripper-pct", type=float, default=15.0,
                    help="same for the gripper, which is PERCENT not degrees")
    ap.add_argument("--track-tol-deg", type=float, default=25.0,
                    help="tracking-error tolerance, body joints (degrees)")
    ap.add_argument("--track-tol-gripper-pct", type=float, default=30.0,
                    help="tracking-error tolerance, gripper (percent)")
    ap.add_argument("--track-strikes", type=int, default=15,
                    help="consecutive out-of-tolerance steps before fault")
    ap.add_argument("--max-pose-diff-deg", type=float, default=15.0,
                    help="pre-flight: max leader/follower disagreement at start")
    ap.add_argument("--force", action="store_true",
                    help="start even if the poses disagree (escape hatch)")
    ap.add_argument("--out", default="./logs/p1",
                    help="directory for rows.jsonl / events.jsonl")
    ap.add_argument("--udp", default=None, help='also stream to "host:port"')
    ap.add_argument("--duration", type=float, default=0.0, help="0 = until Enter")
    return ap


def main():
    ap = build_args()
    args = ap.parse_args()
    if not args.leader_port or not args.follower_port:
        ap.error("need --leader-port/--follower-port (or LEADER/FOLLOWER env)")

    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
    from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

    # max_relative_target is deliberately left None: it would force an extra
    # sync_read on the follower bus every step. We rate-limit in software instead.
    follower = SOFollower(SOFollowerRobotConfig(
        port=args.follower_port, id=args.follower_id, use_degrees=True,
        max_relative_target=None))
    leader = SOLeader(SOLeaderTeleopConfig(
        port=args.leader_port, id=args.leader_id, use_degrees=True))

    run_dir = os.path.join(args.out, time.strftime("%Y%m%d-%H%M%S"))
    rows = JsonlWriter(os.path.join(run_dir, "rows.jsonl"))
    events = JsonlWriter(os.path.join(run_dir, "events.jsonl"))
    pub = SignalPublisher(udp_addr=args.udp)
    events.event("start", argv=sys.argv[1:], epoch=epoch(), run_dir=run_dir,
                 units=UNITS)
    print(f"[p1] logging to {run_dir}")

    print("[p1] pre-flight ...")
    report, info = run_preflight(
        leader, follower, args.leader_port, args.follower_port,
        expect_leader_serial=args.leader_serial,
        expect_follower_serial=args.follower_serial,
        max_pose_diff_deg=args.max_pose_diff_deg, force=args.force)
    print(report.render())
    events.event("preflight", ok=report.ok, checks=report.rows, info=info)

    if not report.ok:
        print("\n[p1] PRE-FLIGHT FAILED -- not starting. Releasing torque.")
        for obj in (follower, leader):
            try:
                obj.disconnect()
            except Exception:
                pass
        rows.close()
        events.close()
        pub.close()
        return 1

    # ---- stop signalling: Enter, Ctrl+C and SIGTERM all mean "release" ----
    stop = threading.Event()
    stop_why = {"reason": None}

    def _sig(signum, _frame):
        stop_why["reason"] = f"signal:{signal.Signals(signum).name}"
        stop.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    def _wait_enter():
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            pass
        if stop_why["reason"] is None:
            stop_why["reason"] = "enter"
        stop.set()
    threading.Thread(target=_wait_enter, daemon=True).start()

    print("\n[p1] RUNNING. Move the leader; the follower follows.")
    print("[p1] Press ENTER to stop and release torque. Ctrl+C does the same.")

    step_limits = per_joint(args.max_step_deg, args.max_step_gripper_pct)
    track_tol = per_joint(args.track_tol_deg, args.track_tol_gripper_pct)
    watchdog = TrackingWatchdog(track_tol, args.track_strikes)
    period = 1.0 / args.fps

    # ** Seed the rate limiter with where the follower ACTUALLY IS. **
    # rate_limit() passes a joint straight through when it has no previous
    # command for it, so leaving this None makes the FIRST step unclamped: the
    # follower receives the leader's full pose as a goal and drives there at
    # whatever speed the servo can manage. Every later step is limited; only
    # step one was not, which is exactly the step where the two arms are
    # furthest apart. Seeded, the follower walks to the leader at the same
    # limit as everything else.
    try:
        prev_cmd = strip_pos(follower.get_observation())
    except Exception as e:
        events.event("seed_failed", err=repr(e))
        prev_cmd = None
    events.event("rate_limiter_seeded", prev_cmd=prev_cmd)

    # The watchdog must not fire DURING that walk. While the follower is
    # catching up the command legitimately leads the measurement, and a long
    # catch-up would otherwise trip a fault that means nothing. Arm it once
    # tracking has been good a single time.
    watchdog_armed = False
    seq = 0
    clamped_total = 0
    fault = None
    t_end = time.monotonic() + args.duration if args.duration > 0 else None

    try:
        while not stop.is_set():
            t0 = time.monotonic()
            raw = leader.get_action()
            t1 = time.monotonic()
            meas = strip_pos(follower.get_observation())
            t2 = time.monotonic()

            target = strip_pos(raw)
            cmd, n_clamped = rate_limit(target, prev_cmd, step_limits)
            clamped_total += n_clamped
            follower.send_action({f"{k}.pos": v for k, v in cmd.items()})
            t3 = time.monotonic()
            prev_cmd = cmd

            _, wj, wv = pose_diff(cmd, meas)
            rec = {"schema": SCHEMA, "seq": seq, **stamp(),
                   "leader": target, "follower": meas, "command": cmd,
                   "units": UNITS,
                   "dt_read_leader_ms": round((t1 - t0) * 1e3, 3),
                   "dt_read_follower_ms": round((t2 - t1) * 1e3, 3),
                   "dt_write_ms": round((t3 - t2) * 1e3, 3),
                   "dt_loop_ms": round((t3 - t0) * 1e3, 3),
                   "clamped": n_clamped,
                   "track_worst_joint": wj, "track_worst_deg": round(wv, 2)}
            rows.write(rec)
            pub.publish(rec)
            seq += 1

            if not watchdog_armed:
                if watchdog.in_tolerance(cmd, meas):
                    watchdog_armed = True
                    events.event("watchdog_armed", seq=seq)
            elif watchdog.update(cmd, meas):
                fault = "tracking_watchdog: " + watchdog.reason()
                break
            if t_end and time.monotonic() >= t_end:
                stop_why["reason"] = "duration"
                break
            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)
    except Exception as e:
        fault = f"{type(e).__name__}: {e}"

    # ---- fault: HOLD. Do not disconnect; a powered servo keeps its goal. ----
    if fault:
        events.event("fault", reason=fault, seq=seq)
        print("\n" + "=" * 68)
        print("[p1] FAULT -- no further commands are being sent.")
        print(f"      {fault}")
        print("      The follower is HOLDING its position (torque still on).")
        print("      Servos will warm up while holding; do not leave it long.")
        print("      Press ENTER to release torque and exit.")
        print("=" * 68)
        stop.wait()

    reason = fault or stop_why["reason"] or "unknown"
    print(f"\n[p1] releasing torque and disconnecting ({reason})")
    for obj in (follower, leader):
        try:
            obj.disconnect()
        except Exception as e:
            events.event("disconnect_error", who=type(obj).__name__, err=repr(e))

    events.event("stop", reason=reason, steps=seq, clamped_total=clamped_total)
    print(f"[p1] {seq} steps, {clamped_total} clamped. Logs in {run_dir}")
    print(f"[p1] latency: python tools/analyze_latency.py {run_dir}/rows.jsonl")
    rows.close()
    events.close()
    pub.close()
    return 2 if fault else 0


if __name__ == "__main__":
    sys.exit(main())
