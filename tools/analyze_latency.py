#!/usr/bin/env python3
"""Offline latency analysis for a p1/p3/p4 run (reads rows.jsonl).

Handles both row shapes: p1 (`leader`/`follower`, one machine) and p3
(`so101.simstep.v1`, with the network leg and the RTT split into waiting
for the simulator's tick, the step itself, and the network). Rows from
DISCARDED episodes are excluded by default -- an episode thrown away for
having a gap in it is exactly the one that would skew a latency figure.

Two different things are called "latency"; this reports both.

  * LOOP latency -- how long our software takes per step (read leader, read
    follower, write goal). Pure software, measured directly in the loop.

  * FOLLOW latency -- how long the physical follower takes to reach where the
    leader already is. This is the number that matters, and it is NOT measurable
    inside one step: it is a property of the servo's internal PID.

We recover FOLLOW latency by cross-correlation: the follower's measured angle
series is (approximately) the leader's series delayed by some lag. We slide one
against the other and take the lag with the highest correlation. Doing it
offline keeps the control loop free of this arithmetic (rule 90: land the data
first, compute afterwards).

This number is the BASELINE. When Isaac (program 3) and the VLA (program 5) are
wired up, their latency is measured the same way and compared against it.
"""
import argparse
import json
import os
import math
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm.units import JOINTS, unit_of   # noqa: E402


def _demean(xs):
    m = sum(xs) / len(xs)
    return [x - m for x in xs]


def best_lag(a, b, max_lag):
    """Lag k (in samples) where b[k:] best matches a[:-k]. Pure Python."""
    n = min(len(a), len(b))
    if n < max_lag + 8:
        return None, 0.0
    a = _demean(a[:n])
    b = _demean(b[:n])
    best_k, best_r = 0, float("-inf")
    for k in range(0, max_lag + 1):
        m = n - k
        num = sum(a[i] * b[i + k] for i in range(m))
        da = sum(a[i] * a[i] for i in range(m)) ** 0.5
        db = sum(b[i + k] * b[i + k] for i in range(m)) ** 0.5
        if da == 0 or db == 0:
            continue
        r = num / (da * db)
        if r > best_r:
            best_k, best_r = k, r
    return best_k, best_r


def pct(xs, p):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    i = min(len(xs) - 1, max(0, int(round(p / 100.0 * (len(xs) - 1)))))
    return xs[i]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rows", help="path to rows.jsonl")
    ap.add_argument("--max-lag", type=int, default=30, help="samples to search")
    ap.add_argument("--min-motion-deg", type=float, default=3.0,
                    help="skip joints that barely moved (lag is meaningless there)")
    ap.add_argument("--include-discarded", action="store_true",
                    help="analyse rows from episodes that were discarded")
    args = ap.parse_args()

    recs = []
    with open(args.rows) as fh:
        for line in fh:
            line = line.strip()
            if line:
                recs.append(json.loads(line))
    # p1 rows carry leader+follower; p3 rows carry leader_deg and a network
    # leg instead. Both are control-step rows and both deserve this analysis.
    kind = "p1"
    step = [r for r in recs if "leader" in r and "follower" in r]
    if not step:
        step = [r for r in recs if r.get("schema") == "so101.simstep.v1"]
        kind = "p3"
    episodes = [r for r in recs if r.get("schema") == "so101.episode.v1"]
    recs = step
    if len(recs) < 20:
        print(f"only {len(recs)} usable rows -- need at least 20")
        return 1
    print(f"file kind         : {kind}")
    if episodes:
        kept = [e for e in episodes if e.get("result") == "kept"]
        dropped = [e for e in episodes if e.get("result") == "discarded"]
        print(f"episodes          : {len(kept)} kept, {len(dropped)} discarded"
              + (f"  (discarded: {[e['episode'] for e in dropped]})"
                 if dropped else ""))
        if dropped and args.include_discarded:
            print("                    INCLUDING discarded episodes, as asked")
        elif dropped:
            drop_ids = {e["episode"] for e in dropped}
            before = len(recs)
            recs = [r for r in recs if r.get("episode") not in drop_ids]
            print(f"                    excluded {before - len(recs)} rows from "
                  f"discarded episodes (--include-discarded to keep them)")
            if len(recs) < 20:
                print(f"only {len(recs)} rows left -- need at least 20")
                return 1

    ts = [r["t_mono"] for r in recs]
    dts = [b - a for a, b in zip(ts, ts[1:])]
    dt_med = st.median(dts)
    print(f"rows              : {len(recs)}")
    print(f"duration          : {ts[-1] - ts[0]:.1f} s")
    print(f"loop rate         : {1.0 / dt_med:.1f} Hz median, "
          f"{1.0 / pct(dts, 95):.1f} Hz at p95-slowest")
    print()
    print("loop breakdown (ms, median / p95)")
    for key, label in (("dt_read_leader_ms", "  read leader   "),
                       ("dt_read_ms", "  read leader   "),
                       ("dt_read_follower_ms", "  read follower "),
                       ("dt_map_ms", "  map to radians"),
                       ("dt_write_ms", "  write goal    "),
                       ("dt_send_wait_ms", "  send + wait   "),
                       ("dt_loop_ms", "  TOTAL         ")):
        v = [r[key] for r in recs if r.get(key) is not None]
        if v:
            print(f"{label}: {st.median(v):6.2f} / {pct(v, 95):6.2f}")

    if kind == "p3":
        rtt = [r["rtt_ms"] for r in recs if r.get("rtt_ms") is not None]
        print()
        print(f"round trip to the simulator  ({len(rtt)}/{len(recs)} steps got "
              f"their own ack inside the tick)")
        if rtt:
            print(f"  rtt total     : {st.median(rtt):6.2f} / {pct(rtt, 95):6.2f}")
            pairs = [(r["rtt_ms"], r.get("sim_wait_ms"), r.get("sim_apply_ms"))
                     for r in recs
                     if r.get("rtt_ms") is not None
                     and r.get("sim_wait_ms") is not None
                     and r.get("sim_apply_ms") is not None]
            if pairs:
                w = [p[1] for p in pairs]
                a = [p[2] for p in pairs]
                n = [p[0] - p[1] - p[2] for p in pairs]
                print(f"  ... waiting for the sim's tick : "
                      f"{st.median(w):6.2f} / {pct(w, 95):6.2f}")
                print(f"  ... the sim step itself        : "
                      f"{st.median(a):6.2f} / {pct(a, 95):6.2f}")
                print(f"  ... network + our own handling : "
                      f"{st.median(n):6.2f} / {pct(n, 95):6.2f}")
                print("  (the split matters: tick phase is not a slow network, "
                      "and chasing the wrong one wastes days)")
            if len(rtt) < 0.9 * len(recs):
                print(f"  NOTE: {len(recs) - len(rtt)} steps had no ack within "
                      f"their tick, so this distribution is truncated -- it "
                      f"under-reports the tail. Check `lost`/`superseded` in "
                      f"events.jsonl.")
        clip_l = sum(1 for r in recs if r.get("clipped_local"))
        clip_s = sum(1 for r in recs if r.get("clipped_sim"))
        print(f"\nclipped by our map : {clip_l}   clipped by the sim : {clip_s}")

    clamped = sum(r.get("clamped", 0) for r in recs)
    tw = [r["track_worst_deg"] for r in recs if "track_worst_deg" in r]
    print()
    print(f"rate-limit clamps : {clamped}")
    if tw:
        print(f"tracking error    : {st.median(tw):.1f} deg median, "
              f"{max(tw):.1f} deg worst")

    print()
    # p1 compares the leader against the REAL follower. p3 has no real
    # follower; the meaningful pair there is what we asked the simulator for
    # against what it measured -- both in radians, so the correlation is over
    # like units, and the lag it reports is the whole path: network, the wait
    # for the sim's tick, and the joint drive settling.
    if kind == "p3":
        src_key, dst_key = "joints_rad", "applied_rad"
        min_motion = math.radians(args.min_motion_deg)
        print("SIM-FOLLOW latency by joint: what we asked vs what the "
              "simulator measured")
        print(f"  (cross-correlation, {dt_med * 1e3:.1f} ms/sample)")
        recs = [r for r in recs if isinstance(r.get(dst_key), dict)]
        if not recs:
            print("  no rows carry applied_rad -- was this a --no-sim run, or "
                  "did no ack arrive inside its tick?")
            return 0
    else:
        src_key, dst_key = "leader", "follower"
        min_motion = args.min_motion_deg
        print(f"FOLLOW latency by joint (cross-correlation, "
              f"{dt_med * 1e3:.1f} ms/sample)")
    lags = []
    for j in JOINTS:
        a = [r[src_key].get(j) for r in recs]
        b = [r[dst_key].get(j) for r in recs]
        if any(x is None for x in a) or any(x is None for x in b):
            continue
        span = max(a) - min(a)
        u = "rad" if kind == "p3" else unit_of(j)
        if span < min_motion:
            print(f"  {j:<15}: skipped (moved only {span:.2f} {u})")
            continue
        k, r = best_lag(a, b, args.max_lag)
        if k is None:
            continue
        lags.append(k * dt_med * 1e3)
        print(f"  {j:<15}: {k * dt_med * 1e3:6.1f} ms  ({k} samples, r={r:.3f})")
    if lags:
        print(f"  {'MEDIAN':<15}: {st.median(lags):6.1f} ms   <-- baseline")
    else:
        print("  (no joint moved enough -- move the arm more next run)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
