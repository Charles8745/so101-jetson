#!/usr/bin/env python3
"""Build, verify and inspect the real->sim joint map (arm/sim_mapping.py).

    fit     produce a map from an arm calibration + the model's joint limits
    verify  record that a human checked the map, and how
    check   guard an existing map against an arm, exactly as p3 would
    show    print a map in human terms

The one thing this tool CANNOT work out for you
-----------------------------------------------
Whether the real arm's `range_min` corresponds to the model's LOWER limit or its
UPPER limit. That is the sign of the joint, and no amount of arithmetic can
recover it -- both choices produce a smooth, plausible-looking mapping, one of
which drives the virtual arm backwards. It has to be observed:

    1. fit with no --flip
    2. run p3 with --allow-unverified and move ONE joint at a time
    3. any joint that moves the wrong way in the sim goes in --flip
    4. refit, re-check, then `verify --by <you> --method visual`

That is five minutes of looking, once per model. It is also the difference
between a dataset and a pile of numbers.

Typical use
-----------
    python3 tools/simmap_init.py fit \\
        --arm-role leader --arm-id my_leader \\
        --sim-limits sim_limits.json \\
        --out configs/simmap_leader.json

    python3 tools/simmap_init.py verify --map configs/simmap_leader.json \\
        --by charles --method visual \\
        --note "swept all 6 joints one at a time; sim matched direction and
                approximate angle at both ends and mid-travel"
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm.sim_mapping import (FIT_ENDPOINTS, FIT_IDENTITY, SPAN_FAIL,  # noqa: E402
                             SPAN_WARN, SimMap, build_map, residuals)
from arm.units import BODY_JOINTS, GRIPPER, deg_range_from_calibration  # noqa: E402

# lerobot's on-disk calibration layout. Overridable with --calibration, and the
# tool prints what it resolved so a wrong guess is visible rather than silent.
CAL_ROOT = os.path.expanduser(
    os.environ.get("HF_LEROBOT_CALIBRATION",
                   "~/.cache/huggingface/lerobot/calibration"))
ROLE_DIR = {"leader": ("teleoperators", "so_leader"),
            "follower": ("robots", "so_follower")}


def resolve_calibration(role, arm_id, explicit=None):
    if explicit:
        return explicit
    sub, kind = ROLE_DIR[role]
    return os.path.join(CAL_ROOT, sub, kind, f"{arm_id}.json")


def load_calibration(path):
    with open(path) as fh:
        cal = json.load(fh)
    missing = [j for j in BODY_JOINTS + (GRIPPER,) if j not in cal]
    if missing:
        raise SystemExit(f"calibration {path} is missing joints: {missing}")
    return cal


# ------------------------------------------------------------------- fit
def cmd_fit(args):
    cal_path = resolve_calibration(args.arm_role, args.arm_id, args.calibration)
    print(f"calibration: {cal_path}")
    if not os.path.isfile(cal_path):
        raise SystemExit("no such calibration file. Pass --calibration with the "
                         "path lerobot actually uses (ask the object: "
                         "SOLeader(cfg).calibration_fpath)")
    cal = load_calibration(cal_path)

    with open(args.sim_limits) as fh:
        sim = json.load(fh)
    limits = dict(sim["limits_rad"])

    # ** A joint with no entry here does not get "no mapping". It gets a
    # ** CONSTANT, and a constant mapping is a simulated arm that never moves
    # ** while p3's every statistic stays green.
    # On 2026-09-11 sim_limits.json was produced without --joints, so
    # limits_rad was {}. This tool printed "-- not in the limits file --" on
    # all five body joints, wrote the map, and exited 0. sim/probe_isaac.py's
    # own message had promised that this tool "will refuse a map missing any
    # joint". It did not. Now it does.
    absent = [j for j in BODY_JOINTS if j not in limits]
    if absent:
        raise SystemExit(
            f"limits file {args.sim_limits} has no entry for: "
            f"{', '.join(absent)}.\n"
            f"Those joints would map to a constant, which is a simulated arm "
            f"that cannot move.\n"
            f"Produce the file with --joints, e.g.\n"
            f"  python3 sim/probe_isaac.py --stage limits --usd <the usd> "
            f"--joints identity --out {args.sim_limits}")
    flip = set(x.strip() for x in (args.flip or "").split(",") if x.strip())
    unknown = flip - set(limits) - {GRIPPER}
    if unknown:
        raise SystemExit(f"--flip names joints that are not in the limits file: "
                         f"{sorted(unknown)}")
    for j in list(limits):
        lo, hi = limits[j]
        limits[j] = [hi, lo] if j in flip else [lo, hi]

    grip = list(sim.get("gripper_rad") or [0.0, 0.0])
    if GRIPPER in flip:
        grip = [grip[1], grip[0]]
    if grip[0] == grip[1]:
        if not args.no_gripper:
            raise SystemExit(
                f"gripper_rad is empty or degenerate in {args.sim_limits}, so "
                f"the gripper would map to a\nconstant: 0% and 100% would "
                f"both send the same angle, and nothing downstream\nwould "
                f"say so. Produce the file with --joints, or pass "
                f"--no-gripper if this\nmodel genuinely has no gripper joint.")
        print("--no-gripper: the gripper maps to a constant ON PURPOSE. "
              "Recorded in the map.")

    sim_target = {k: sim[k] for k in ("usd", "dof_names", "isaac", "stage_path")
                  if k in sim}
    m = build_map(cal, limits, tuple(grip), sim_target=sim_target,
                  fit_mode=args.fit_mode,
                  source_arm={"role": args.arm_role, "id": args.arm_id,
                              "calibration_path": os.path.abspath(cal_path)})
    m.doc["fit"]["flipped"] = sorted(flip)
    m.doc["fit"]["no_gripper"] = bool(args.no_gripper)
    m.doc["fit"]["sim_limits_file"] = os.path.abspath(args.sim_limits)

    print()
    print_span_table(m)
    worst, wj = m.worst_span_ratio()
    unreach = m.unreachable_deg()
    if args.fit_mode == FIT_IDENTITY:
        print("\nfit: IDENTITY -- one degree of real rotation is one degree of "
              "model rotation.\n     The scale is exactly pi/180 and cannot be "
              "wrong; only the SIGN and the\n     OFFSET are assumptions, and "
              "both are what the visual check looks at.")
        if unreach:
            print("\n  travel the model cannot reach (the sim will CLIP, and "
                  "say so every step):")
            for j, d in sorted(unreach.items(), key=lambda kv: -kv[1]):
                print(f"    {j:<16}{d:6.1f} deg of real travel, "
                      f"{d / m.joints[j]['span_deg_real'] * 100:4.1f}% of its range")
            print("  Poses using that last stretch are not reachable in sim. "
                  "Either avoid them\n  when demonstrating, or widen the model's "
                  "limits and refit.")
        else:
            print("\n  the model can reach the arm's whole calibrated travel.")
    else:
        if worst is not None and worst > SPAN_FAIL:
            print(f"\n*** {wj} is off by {worst*100:.1f}% (limit "
                  f"{SPAN_FAIL*100:.0f}%). An endpoint fit spreads that error "
                  f"across every angle in between, so the map is wrong "
                  f"everywhere but the two ends. p3 will refuse it. "
                  f"Use --fit-mode identity. ***")
        elif worst is not None and worst > SPAN_WARN:
            print(f"\nWARNING: {wj} is off by {worst*100:.1f}%, and an endpoint "
                  f"fit turns that into an error at every intermediate angle -- "
                  f"below the refusal threshold, so nothing else will stop it. "
                  f"--fit-mode identity does not have this failure mode.")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    m.save(args.out)
    print(f"\nwrote {args.out}")
    print(f"sha  {m.sha256()[:12]}   verified: NO")
    print("\nNext: run p3 with --allow-unverified, move one joint at a time,")
    print("check every direction, then re-run this tool with `verify`.")
    return 0


def print_span_table(m):
    print(f"fit mode: {m.fit_mode()}")
    print(f"{'joint':<16}{'real span':>12}{'model span':>13}{'ratio':>9}"
          f"{'scale':>12}{'offset':>11}")
    print("-" * 73)
    for j in BODY_JOINTS:
        s = m.joints.get(j)
        if not s:
            print(f"{j:<16}{'-- not in the limits file --':>45}")
            continue
        r = s.get("span_ratio")
        flag = ""
        ident = s.get("fit_mode") == FIT_IDENTITY
        if r is not None and abs(r - 1) > SPAN_FAIL:
            flag = "  <-- will clip" if ident else "  <-- FAIL"
        elif r is not None and abs(r - 1) > SPAN_WARN:
            flag = "  <-- will clip" if ident else "  <-- warn"
        print(f"{j:<16}{s['span_deg_real']:>10.1f}d{s['span_deg_model']:>11.1f}d"
              f"{r:>9.3f}{s['scale']:>12.6f}{s['offset']:>11.4f}{flag}")
    g = m.joints.get(GRIPPER)
    if g:
        d0, d1 = g["dst_range_rad"]
        print(f"{GRIPPER:<16}{'0..100%':>12}"
              f"{math.degrees(d1 - d0):>11.1f}d{'':>9}"
              f"  {d0:+.4f} rad at 0%, {d1:+.4f} rad at 100%")


# ---------------------------------------------------------------- verify
def cmd_verify(args):
    m = SimMap.from_file(args.map)
    if len(args.note.split()) < 5:
        raise SystemExit("--note must actually say what you checked. A "
                         "verification nobody can audit is not a verification.")
    # Bind the attestation to the FIT, so editing the map voids it.
    block = {"by": args.by, "method": args.method, "note": args.note,
             "when_unix": __import__("time").time(),
             "fit_sha256": m.fit_sha256()}

    if args.method == "measured":
        if not (args.pose and args.expected):
            raise SystemExit("--method measured needs --pose and --expected")
        with open(args.pose) as fh:
            pose = json.load(fh)
        with open(args.expected) as fh:
            exp = json.load(fh)
        res, clipped = residuals(m, pose, exp)
        print(f"{'joint':<16}{'want rad':>11}{'got rad':>11}{'err deg':>10}")
        print("-" * 58)
        bad = []
        for j, r in sorted(res.items()):
            mark = ""
            if r["clipped"]:
                mark, bad = "  <-- CLIPPED", bad + [j]
            elif abs(r["err_deg"]) > args.tol_deg:
                mark, bad = "  <-- OVER", bad + [j]
            print(f"{j:<16}{r['want_rad']:>11.4f}{r['got_rad']:>11.4f}"
                  f"{r['err_deg']:>10.2f}{mark}")
        block["residuals"] = res
        block["tol_deg"] = args.tol_deg
        if clipped:
            raise SystemExit(
                f"\nFAILED: {clipped} clipped at the model limit. The residual "
                f"for a clipped joint is measured against the clamped value, "
                f"not against what the map would have produced -- it is bounded "
                f"by construction and cannot fail. Verify at a pose inside the "
                f"fitted travel.")
        if bad:
            raise SystemExit(f"\nFAILED: {bad} exceed {args.tol_deg} deg. "
                             f"Not writing a verified block.")
        print("\nall joints within tolerance, none clipped")
    else:
        print("Recording a VISUAL verification. This attests that a human "
              "watched the sim follow the real arm and found the direction and "
              "rough magnitude right on every joint. It does not measure "
              "anything, and the map records that.")

    m.doc["verified"] = block
    m.save(args.map)
    print(f"\n{args.map} is now verified by {args.by} ({args.method})")
    print(f"sha {m.sha256()[:12]}")
    return 0


# ----------------------------------------------------------------- check
def cmd_check(args):
    m = SimMap.from_file(args.map)
    cal_path = resolve_calibration(args.arm_role, args.arm_id, args.calibration)
    print(f"map          {args.map}  sha {m.sha256()[:12]}")
    print(f"fitted for   {m.role()} / {m.arm_id()}")
    print(f"checking vs  {cal_path}")
    cal = load_calibration(cal_path)
    ok, reasons = m.guard(cal, expect_role=args.arm_role)
    print()
    if ok:
        print("PASS -- p3 would accept this map")
        return 0
    print("REFUSED:")
    for r in reasons:
        print("  - " + r)
    return 1


# ------------------------------------------------------------------ show
def cmd_show(args):
    m = SimMap.from_file(args.map)
    print(f"schema   {m.doc['schema']}")
    print(f"sha      {m.sha256()}")
    print(f"arm      {m.doc.get('source_arm')}")
    print(f"cal sha  {m.doc.get('source_calibration_sha256', '')[:16]}")
    print(f"target   {json.dumps(m.doc.get('sim_target', {}))[:300]}")
    print(f"fit      {json.dumps(m.doc.get('fit', {}))[:300]}")
    print(f"verified {m.verification_note()}")
    print()
    print_span_table(m)
    if args.at is not None:
        print(f"\nmapping the pose in {args.at}:")
        with open(args.at) as fh:
            pose = json.load(fh)
        out, clipped = m.apply(pose)
        for j in sorted(out):
            print(f"  {j:<16}{pose[j]:>10.2f} -> {out[j]:>9.4f} rad "
                  f"({math.degrees(out[j]):>8.2f} deg)"
                  + ("   CLIPPED" if j in clipped else ""))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    f = sub.add_parser("fit", help="build a map")
    f.add_argument("--arm-role", choices=("leader", "follower"), default="leader")
    f.add_argument("--arm-id", default="my_leader")
    f.add_argument("--calibration", default=None)
    f.add_argument("--sim-limits", required=True,
                   help="JSON from sim/probe_isaac.py, run on the sim machine")
    f.add_argument("--flip", default="",
                   help="comma-separated joints whose sim axis runs opposite to "
                        "the real one. Determined by LOOKING, not by arithmetic")
    f.add_argument("--fit-mode", choices=(FIT_IDENTITY, FIT_ENDPOINTS),
                   default=FIT_IDENTITY,
                   help="identity (default): one degree of real rotation is one "
                        "degree of model rotation, and travel the model cannot "
                        "reach CLIPS and is reported. endpoints: stretch the "
                        "calibrated travel onto the declared travel -- only "
                        "correct when the two denote the same physical extremes")
    f.add_argument("--out", required=True)
    f.add_argument("--no-gripper", action="store_true",
                   help="this model really has no gripper joint; map it to a "
                        "constant deliberately instead of being refused")
    f.set_defaults(fn=cmd_fit)

    v = sub.add_parser("verify", help="record a human check")
    v.add_argument("--map", required=True)
    v.add_argument("--by", required=True)
    v.add_argument("--method", choices=("visual", "measured"), default="visual")
    v.add_argument("--note", required=True)
    v.add_argument("--pose", default=None, help="measured: leader degrees JSON")
    v.add_argument("--expected", default=None, help="measured: sim radians JSON")
    v.add_argument("--tol-deg", type=float, default=5.0)
    v.set_defaults(fn=cmd_verify)

    c = sub.add_parser("check", help="guard a map against an arm, as p3 would")
    c.add_argument("--map", required=True)
    c.add_argument("--arm-role", choices=("leader", "follower"), default="leader")
    c.add_argument("--arm-id", default="my_leader")
    c.add_argument("--calibration", default=None)
    c.set_defaults(fn=cmd_check)

    s = sub.add_parser("show", help="print a map")
    s.add_argument("--map", required=True)
    s.add_argument("--at", default=None, help="also map this pose (JSON of degrees)")
    s.set_defaults(fn=cmd_show)

    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
