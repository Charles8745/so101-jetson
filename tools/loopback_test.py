#!/usr/bin/env python3
"""Run the whole program-3 bridge against itself, over real UDP, in one process.

Proves, without an arm, without Isaac and without a second machine:
  * hello / episode-start / episode-end / bye all round-trip
  * every command is acknowledged and correlated to its own sequence number
  * a slow simulator produces SUPERSEDED commands, not lost ones
  * a second operator is refused
  * the RTT decomposes into network / waiting for the sim's tick / stepping

Run it on the Jetson before plugging anything in. If this passes and p3 then
fails on hardware, the fault is in the arm or the map -- not the bridge.

    python3 tools/loopback_test.py
    python3 tools/loopback_test.py --sim-fps 10     (make the sim the slow one)
"""
import argparse
import os
import statistics as st
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from programs.p3_teleop_sim import SimLink            # noqa: E402
from sim.backend import make_backend                  # noqa: E402
from sim.receiver import Receiver                     # noqa: E402

PORT = 19871           # not the real ports, so this never disturbs a live run
ACK = 19872
ACK2 = 19873


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim-fps", type=float, default=30.0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--steps", type=int, default=90)
    ap.add_argument("--out", default="./logs/loopback")
    args = ap.parse_args()

    rx = Receiver(make_backend("echo"), port=PORT, bind="127.0.0.1",
                  fps=args.sim_fps, out=args.out)
    t = threading.Thread(target=rx.run, daemon=True)
    t.start()
    time.sleep(0.3)

    fails = []

    def check(name, cond, detail=""):
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}"
              + (f"  -- {detail}" if detail else ""))
        if not cond:
            fails.append(name)

    link = SimLink("127.0.0.1", PORT, ACK)
    ok, detail = link.request("hello", map_sha256="loopback", timeout_s=3.0)
    check("hello acknowledged", ok is True, detail)
    if ok is not True:
        rx.stop = True
        return 1

    other = SimLink("127.0.0.1", PORT, ACK2)
    ok2, d2 = other.request("hello", map_sha256="loopback", timeout_s=2.0)
    check("second operator refused", ok2 is False, d2)
    other.close()

    ok, detail = link.request("episode_start", episode=1, timeout_s=2.0)
    check("episode_start acknowledged", ok is True, detail)

    period = 1.0 / args.fps
    for seq in range(args.steps):
        t0 = time.monotonic()
        joints = {"shoulder_pan": 0.01 * seq, "gripper": 0.001 * seq}
        from net.sim_protocol import encode_cmd
        link.send(encode_cmd(seq, t0, joints, "loopback", episode=1,
                             recording=True))
        link.stats.on_send(seq, t0, sent_rad=joints)
        link.pump_until(t0 + period, want_seq=seq)

    ok, detail = link.request("episode_end", episode=1, steps=args.steps,
                              timeout_s=2.0)
    check("episode_end acknowledged", ok is True, detail)

    link.request("bye", timeout_s=1.0)

    time.sleep(0.3)
    s = link.stats.summary()
    print()
    print("  link:", s)
    check("nothing lost", s["lost"] == 0, f"lost={s['lost']}")
    check("no mismatch (the sim applied exactly what we asked)",
          s["mismatched"] == 0, f"mismatched={s['mismatched']}")
    check("every command accounted for",
          s["acked"] + s["superseded"] + s["lost"] + s["in_flight"] == s["sent"],
          f"{s['acked']}+{s['superseded']}+{s['lost']}+{s['in_flight']}"
          f" vs {s['sent']}")
    if args.sim_fps < args.fps:
        check("a slow sim shows up as superseded, not lost",
              s["superseded"] > 0 and s["lost"] == 0,
              f"superseded={s['superseded']}")
    else:
        check("a keeping-up sim supersedes almost nothing",
              s["superseded"] <= 0.05 * s["sent"],
              f"superseded={s['superseded']}/{s['sent']}")

    # ---- the two ways a KEPT episode can be a lie ------------------------
    # (a) ** A RESTARTED p3 IS NOT A RETRANSMISSION. **
    #     p3 binds a fixed source port and its ctl_seq always restarts at 1, so
    #     a second run's `hello` and `episode_start` are byte-for-byte the first
    #     run's. Answered from the idempotency cache, the operator performs a
    #     whole demonstration, p3 prints KEPT, and the simulator started
    #     nothing, recorded nothing and saved nothing.
    link.close()
    time.sleep(0.2)
    n_before = len(rx.backend.episodes)
    again = SimLink("127.0.0.1", PORT, ACK)          # SAME port, fresh tracker
    ok, d = again.request("hello", map_sha256="loopback", timeout_s=2.0)
    check("a restarted operator is accepted as a new run", ok is True, d)
    ok, d = again.request("episode_start", episode=1, timeout_s=2.0)
    check("...and its episode_start really reaches the simulator",
          ok is True and len(rx.backend.episodes) > n_before,
          f"ok={ok}, backend calls {n_before}->{len(rx.backend.episodes)}, {d}")

    # (b) the receiver discarded the episode itself (the operator's machine went
    #     silent). `episode_end` must NOT come back as success, or p3 prints
    #     KEPT for a demonstration that exists nowhere.
    rx.backend.on_episode("discard", 1, {"reason": "test"})
    rx._mark_discarded(1, "test: sender went silent")
    ok, d = again.request("episode_end", episode=1, timeout_s=2.0)
    check("ending a receiver-discarded episode is refused",
          ok is False and "DISCARD" in d.upper(), f"ok={ok}, {d}")
    again.request("bye", timeout_s=1.0)
    again.close()

    rx.stop = True
    t.join(timeout=3.0)
    check("receiver applied the episode", rx.n_applied > 0,
          f"applied={rx.n_applied}")
    check("nothing was left recording", not rx.recording, str(rx.episode_result))
    check("receiver dropped the intruder", rx.n_foreign >= 1,
          f"foreign={rx.n_foreign}")

    if s.get("rtt_ms_median") is not None:
        print(f"\n  rtt median {s['rtt_ms_median']:.2f} ms  "
              f"p95 {s['rtt_ms_p95']:.2f}  max {s['rtt_ms_max']:.2f}")
        print("  (on loopback this is almost entirely the wait for the sim's "
              "next tick; see rows.jsonl wait_ms for the split)")
    print()
    if fails:
        print(f"FAILED: {fails}")
        return 1
    print("LOOPBACK OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
