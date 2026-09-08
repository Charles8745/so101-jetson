#!/usr/bin/env python3
"""Program 1 -- teleoperation + signal output.

The leader arm drives the real follower; every step the joint state is emitted
as a signal (UDP + optional JSONL) for a later Isaac Sim bridge to mirror onto a
virtual SO-101.

Runs as an INDEPENDENT process. It owns the servo bus only and never touches the
cameras, so a camera failure cannot affect it and vice-versa.

Safety rules baked in:
  * connect(calibrate=False): we NEVER auto-recalibrate. If a motor's stored
    calibration does not match the on-disk file, we ABORT with a message rather
    than silently overwriting a validated calibration.
  * SIGINT/SIGTERM -> clean disconnect (follower torque released).
"""
import argparse
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.clock import stamp, epoch          # noqa: E402
from arm.signal_pub import SignalPublisher, SCHEMA  # noqa: E402


def _clean_pos(d):
    """lerobot uses keys like 'shoulder_pan.pos'; strip the '.pos' suffix."""
    out = {}
    for k, v in d.items():
        out[k[:-4] if k.endswith(".pos") else k] = float(v)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--leader-port", default=os.environ.get("LEADER"))
    ap.add_argument("--follower-port", default=os.environ.get("FOLLOWER"))
    ap.add_argument("--leader-id", default="my_leader")
    ap.add_argument("--follower-id", default="my_follower")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--udp", default=None,
                    help='signal sink "host:port", e.g. 192.168.0.9:9870')
    ap.add_argument("--jsonl", default=None,
                    help="also append the joint signal to this file")
    ap.add_argument("--max-relative-target", type=float, default=None,
                    help="clamp per-step follower motion (safety); omit for none")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds; 0 = run until Ctrl+C")
    args = ap.parse_args()
    if not args.leader_port or not args.follower_port:
        ap.error("need --leader-port/--follower-port (or LEADER/FOLLOWER env)")

    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
    from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

    fcfg = SOFollowerRobotConfig(port=args.follower_port, id=args.follower_id,
                                 use_degrees=True,
                                 max_relative_target=args.max_relative_target)
    lcfg = SOLeaderTeleopConfig(port=args.leader_port, id=args.leader_id,
                                use_degrees=True)
    follower = SOFollower(fcfg)
    leader = SOLeader(lcfg)

    # verify calibration files exist BEFORE we power the motors
    for label, obj in [("follower", follower), ("leader", leader)]:
        if not obj.calibration_fpath.is_file() or len(obj.calibration) != 6:
            sys.exit(f"ABORT: {label} calibration missing/incomplete at "
                     f"{obj.calibration_fpath}")

    pub = SignalPublisher(udp_addr=args.udp, jsonl_path=args.jsonl)
    stop = {"flag": False}

    def _sig(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    leader.connect(calibrate=False)
    follower.connect(calibrate=False)
    if not leader.is_calibrated or not follower.is_calibrated:
        leader.disconnect()
        follower.disconnect()
        sys.exit("ABORT: motors report not-calibrated (mismatch with file). "
                 "Refusing to auto-recalibrate -- run a deliberate calibration "
                 "if the hardware really changed.")

    print(f"[teleop] {args.fps} Hz  udp={args.udp}  jsonl={args.jsonl}")
    print("[teleop] align the leader to the follower's pose before moving. "
          "Ctrl+C to stop.")
    if args.jsonl:
        pub.publish({"schema": SCHEMA + ".epoch", **epoch()})

    period = 1.0 / args.fps
    seq = 0
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while not stop["flag"]:
            t0 = time.monotonic()
            action = leader.get_action()          # leader joint targets
            follower.send_action(action)          # drive the real follower
            obs = follower.get_observation()      # follower state (no cameras)
            rec = {
                "schema": SCHEMA, "seq": seq, **stamp(),
                "leader": _clean_pos(action),
                "follower": _clean_pos({k: v for k, v in obs.items()
                                        if k.endswith(".pos")}),
                "units": "deg",
            }
            pub.publish(rec)
            seq += 1
            if t_end and time.monotonic() >= t_end:
                break
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)
    finally:
        try:
            follower.disconnect()
        finally:
            try:
                leader.disconnect()
            finally:
                pub.close()
        print(f"[teleop] stopped after {seq} steps")


if __name__ == "__main__":
    main()
