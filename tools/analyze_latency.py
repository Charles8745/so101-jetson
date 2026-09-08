#!/usr/bin/env python3
"""Offline latency analysis for a p1/p3/p4 run (reads rows.jsonl).

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
import statistics as st
import sys

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


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
    args = ap.parse_args()

    recs = []
    with open(args.rows) as fh:
        for line in fh:
            line = line.strip()
            if line:
                recs.append(json.loads(line))
    recs = [r for r in recs if "leader" in r and "follower" in r]
    if len(recs) < 20:
        print(f"only {len(recs)} usable rows -- need at least 20")
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
                       ("dt_read_follower_ms", "  read follower "),
                       ("dt_write_ms", "  write goal    "),
                       ("dt_loop_ms", "  TOTAL         ")):
        v = [r[key] for r in recs if key in r]
        if v:
            print(f"{label}: {st.median(v):6.2f} / {pct(v, 95):6.2f}")

    clamped = sum(r.get("clamped", 0) for r in recs)
    tw = [r["track_worst_deg"] for r in recs if "track_worst_deg" in r]
    print()
    print(f"rate-limit clamps : {clamped}")
    if tw:
        print(f"tracking error    : {st.median(tw):.1f} deg median, "
              f"{max(tw):.1f} deg worst")

    print()
    print(f"FOLLOW latency by joint (cross-correlation, {dt_med * 1e3:.1f} ms/sample)")
    lags = []
    for j in JOINTS:
        a = [r["leader"].get(j) for r in recs]
        b = [r["follower"].get(j) for r in recs]
        if any(x is None for x in a) or any(x is None for x in b):
            continue
        span = max(a) - min(a)
        if span < args.min_motion_deg:
            print(f"  {j:<15}: skipped (moved only {span:.1f} deg)")
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
