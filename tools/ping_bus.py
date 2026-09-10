#!/usr/bin/env python3
"""Ask every servo on both arms to identify itself. Energises nothing.

This is the first thing to run when a motor "disappears" -- a preflight failure
with `no status packet` on one id, or a connect that fails once and then works.
It answers one question and no others: is the servo on the bus at all?

It deliberately does NOT enable torque and does NOT read positions. It opens
the bus with handshake=False and sends a broadcast ping, which is the lowest
level of contact there is. Nothing moves, and an arm with no power supply
answers exactly the same as one with power (the servo logic board is USB fed) --
so a full set of answers here does not mean the arm can hold itself up.

Reading the result: the servos are daisy-chained 1 -> 2 -> 3 -> 4 -> 5 -> 6, so
a missing id is either that servo or the cable feeding it, and a bad cable takes
every servo BEHIND it with it. Ids 5 and 6 missing together is one cable between
4 and 5, not two dead servos.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def ping(label, port, factory):
    print(f"{label}  {port}")
    try:
        obj = factory(port)
    except Exception as exc:
        print(f"  could not build the bus: {exc}")
        return False

    bus = obj.bus
    expected = {name: m.id for name, m in bus.motors.items()}

    try:
        # handshake=False keeps this to a bus open. The handshake path reads
        # model numbers back from every servo, which is exactly the thing that
        # fails when a servo is intermittent -- and then we would report a
        # connect failure instead of the per-id answer we came here for.
        try:
            bus.connect(handshake=False)
        except TypeError:
            bus.connect()
    except Exception as exc:
        print(f"  could not open the port: {exc}")
        print("  -> wrong path in devices.env, arm unplugged, or no dialout group")
        return False

    try:
        answered = set(bus.broadcast_ping() or {})
    except Exception as exc:
        print(f"  ping failed: {exc}")
        answered = set()
    finally:
        try:
            bus.disconnect()
        except Exception:
            pass

    missing = []
    for name, mid in sorted(expected.items(), key=lambda kv: kv[1]):
        ok = mid in answered
        print(f"  id {mid}  {name:<14} {'ok' if ok else 'NO ANSWER'}")
        if not ok:
            missing.append((mid, name))

    extra = answered - set(expected.values())
    if extra:
        print(f"  also answering, not in the motor list: {sorted(extra)}")
        print("  -> another device on this bus, or the wrong arm on this port")

    if not missing:
        print(f"  all {len(expected)} answering")
        return True

    first = min(mid for mid, _ in missing)
    print(f"  {len(missing)}/{len(expected)} not answering: "
          + ", ".join(f"{mid}({name})" for mid, name in missing))
    if len(missing) > 1 and [mid for mid, _ in missing] == list(
            range(first, first + len(missing))):
        print(f"  -> they are consecutive from id {first}. Suspect the cable "
              f"INTO id {first}, not {len(missing)} dead servos")
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["leader", "follower", "both"], default="both")
    ap.add_argument("--leader-port", default=os.environ.get("LEADER"))
    ap.add_argument("--follower-port", default=os.environ.get("FOLLOWER"))
    ap.add_argument("--leader-id", default="my_leader")
    ap.add_argument("--follower-id", default="my_follower")
    ap.add_argument("--repeat", type=int, default=1,
                    help="ping N times. Use this for an intermittent fault: a "
                         "servo that answers 20/20 is a different problem from "
                         "one that answers 19/20.")
    args = ap.parse_args()

    for label, port in (("LEADER", args.leader_port),
                        ("FOLLOWER", args.follower_port)):
        if args.arm in (label.lower(), "both") and not port:
            print(f"{label}: no port. It is normally set in devices.env "
                  f"(check with: so101 env), or pass --{label.lower()}-port",
                  file=sys.stderr)
            return 2

    # Imported here, not at the top, so --help works on a machine with no
    # lerobot -- and so a missing lerobot is one sentence rather than a
    # traceback, since the SOP sends people here when something is already
    # going wrong.
    try:
        from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
        from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
    except ImportError as exc:
        print(f"lerobot is not installed in this python ({exc}).", file=sys.stderr)
        print("Check which one you are using with:  so101 env", file=sys.stderr)
        print("Install it with:                     bash setup/install.sh", file=sys.stderr)
        return 2

    targets = []
    if args.arm in ("follower", "both"):
        targets.append(("FOLLOWER", args.follower_port, lambda p: SOFollower(
            SOFollowerRobotConfig(port=p, id=args.follower_id, use_degrees=True))))
    if args.arm in ("leader", "both"):
        targets.append(("LEADER", args.leader_port, lambda p: SOLeader(
            SOLeaderTeleopConfig(port=p, id=args.leader_id, use_degrees=True))))

    fails = 0
    for n in range(args.repeat):
        if args.repeat > 1:
            print(f"--- pass {n + 1}/{args.repeat} ---")
        for label, port, factory in targets:
            if not ping(label, port, factory):
                fails += 1
        if n + 1 < args.repeat:
            print()

    if fails:
        print(f"\n{fails} arm-pass(es) with a missing servo.")
        return 1
    print("\nEvery servo answered." + (
        "" if args.repeat > 1 else
        "  For an intermittent fault, run:  so101 ping --repeat 20"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
