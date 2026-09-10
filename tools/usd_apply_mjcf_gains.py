#!/usr/bin/env python3
"""Write a MuJoCo model's actuator values into a USD asset, with the units
converted once, by the same code that checks them afterwards.

The problem this solves
-----------------------
The SO-101 USD was imported from a URDF. URDF has no `<dynamics>`, so the
importer created the drives and the PhysX joint attributes and set them all to
zero -- except maxForce, which it took from the URDF's `effort="10"`. Measured
2026-09-10 on the asset in use:

    stiffness 0   damping 0   maxForce 10   armature 0   jointFriction 0

A drive whose stiffness and damping are both zero cannot follow a target: the
joint free-swings. And 10 N*m is three times the real STS3215's 3.35, so the
simulated arm lifts things the real one cannot -- which is worse than a wrong
trajectory, because it makes "the task succeeded" mean different things in the
two domains.

Why not just type the numbers in
--------------------------------
Because so101_new_calib.xml carries three sets of them and only one is live: a
childclass every element overrides, a class-default forcerange of 2.94 that
every actuator overrides with 3.35, and the values that actually apply. Reading
that by eye is how 2026-09-08 used a setting that does not exist in the model.

So the resolution is done by usd_joint_report.expected_from_mjcf -- the same
function that verifies the result. There is exactly one place in this repo
where a MuJoCo gain becomes a USD gain, and both writing and checking go
through it. Run the check afterwards; it is not optional:

    tools/usd_apply_mjcf_gains.py --usd in.usda --mjcf model.xml --out new.usda
    tools/usd_joint_report.py --usd new.usda --mjcf model.xml     # must be ok

What it refuses to do
---------------------
    - write anywhere but the input's own directory. The asset references its
      payloads by relative path; exporting elsewhere silently breaks them.
    - touch a prismatic joint. Its gains are per metre, not per degree.
    - invent an attribute name. If armature or jointFriction is not already on
      the prim, it says which name it would have used and stops, unless you
      pass --create-missing-attrs and accept that.
    - overwrite the input. The old asset is the evidence of what was wrong.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common.usd_env import close_pxr, ensure_pxr, flat  # noqa: E402
import usd_joint_report as report                        # noqa: E402

DEFAULT_ARMATURE_ATTR = "physxJoint:armature"
DEFAULT_FRICTION_ATTR = "physxJoint:jointFriction"


def _fail(msg):
    print(f"usd_apply_mjcf_gains: {flat(msg)}", file=sys.stderr)
    return 2


def apply_gains(usd_in, mjcf, usd_out, create_missing=False, dry_run=False):
    from pxr import Usd, UsdPhysics, Sdf

    exp = report.expected_from_mjcf(report.read_mjcf(mjcf))
    stage = Usd.Stage.Open(usd_in)
    if stage is None:
        raise RuntimeError(f"could not open {usd_in}")

    touched, problems = [], []
    for prim in stage.Traverse():
        is_rev = prim.IsA(UsdPhysics.RevoluteJoint)
        is_pri = prim.IsA(UsdPhysics.PrismaticJoint)
        if not is_rev and not is_pri:
            continue
        name = prim.GetName()

        if name not in exp:
            problems.append(f"{name}: no actuator of that name in {mjcf}")
            continue
        if is_pri:
            problems.append(
                f"{name}: prismatic. Its gains are per metre and its limits are"
                " a length; the degree conversion here would be meaningless.")
            continue
        e = exp[name]
        if e["unresolved"]:
            problems.append(f"{name}: unresolved in the MJCF (no class=)")
            continue
        for key in ("usd_stiffness_deg", "maxForce", "armature", "frictionloss"):
            if e[key] is None:
                problems.append(f"{name}: the MJCF gives no {key}")

        row = {"joint": name, "before": {}, "after": {}}

        d = UsdPhysics.DriveAPI.Apply(prim, "angular")
        for label, attr_fn, value in (
                ("stiffness", d.CreateStiffnessAttr, e["usd_stiffness_deg"]),
                ("damping", d.CreateDampingAttr, e["usd_damping_deg"]),
                ("maxForce", d.CreateMaxForceAttr, e["maxForce"])):
            a = attr_fn()
            row["before"][label] = a.Get()
            row["after"][label] = value
            if not dry_run and value is not None:
                a.Set(float(value))
        if not dry_run:
            d.CreateTypeAttr().Set("force")

        for label, hints, default_name, value in (
                ("armature", report.ARMATURE_HINTS, DEFAULT_ARMATURE_ATTR,
                 e["armature"]),
                ("friction", report.FRICTION_HINTS, DEFAULT_FRICTION_ATTR,
                 e["frictionloss"])):
            found = report._find_attrs(prim, hints)
            if found:
                attr_name = found[0][0]
                row["before"][label] = found[0][1]
            elif create_missing:
                attr_name = default_name
                row["before"][label] = None
            else:
                problems.append(
                    f"{name}: no {label} attribute on the prim. It would be "
                    f"`{default_name}`, but guessing a schema name is how the "
                    "57.29578 bug happened. Pass --create-missing-attrs if "
                    "that name is right for this Isaac version.")
                continue
            row["after"][label] = value
            if not dry_run and value is not None:
                prim.CreateAttribute(
                    attr_name, Sdf.ValueTypeNames.Float).Set(float(value))

        touched.append(row)

    if problems:
        raise RuntimeError("refusing to write:\n  " + "\n  ".join(problems))
    if not touched:
        raise RuntimeError(f"no joints in {usd_in} matched an actuator in {mjcf}")

    if not dry_run:
        stage.GetRootLayer().Export(usd_out)
    return touched


def print_table(rows, dry_run):
    head = "would set" if dry_run else "set"
    print(f"{head} {len(rows)} joint(s); every angular gain below is PER DEGREE"
          f"\n(1 rad = {report.DEG_PER_RAD:.9g} deg)\n")
    for r in rows:
        print(r["joint"])
        for k in ("stiffness", "damping", "maxForce", "armature", "friction"):
            if k not in r["after"]:
                continue
            b, a = r["before"].get(k), r["after"][k]
            bs = "MISSING" if b is None else f"{b:.6g}"
            note = ""
            if k in ("stiffness", "damping"):
                note = f"   (= {a * report.DEG_PER_RAD:.6g} per radian)"
            print(f"  {k:<11s} {bs:>12s}  ->  {a:<12.6g}{note}")
        print()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd", required=True)
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("--out", help="required unless --dry-run")
    ap.add_argument("--via", choices=("auto", "direct", "kit"), default="auto")
    ap.add_argument("--dry-run", action="store_true",
                    help="resolve and print, write nothing")
    ap.add_argument("--create-missing-attrs", action="store_true")
    ap.add_argument("--force", action="store_true",
                    help="allow overwriting an existing --out")
    args = ap.parse_args()

    if not args.dry_run:
        if not args.out:
            return _fail("--out is required unless --dry-run")
        a, b = os.path.abspath(args.usd), os.path.abspath(args.out)
        if a == b:
            return _fail("--out must not be --usd. The old asset is the "
                         "evidence of what was wrong; keep it.")
        if os.path.dirname(a) != os.path.dirname(b):
            return _fail(
                "--out must sit in the same directory as --usd. The asset "
                "references its payloads by relative path, and exporting "
                "elsewhere breaks them without saying so.")
        if os.path.exists(b) and not args.force:
            return _fail(f"{args.out} exists; pass --force to overwrite")

    try:
        route, where = ensure_pxr(args.via)
        print(f"pxr via {route}: {', '.join(where)}\n")
        rows = apply_gains(args.usd, args.mjcf, args.out,
                           args.create_missing_attrs, args.dry_run)
    except Exception as e:
        close_pxr()
        return _fail(f"{type(e).__name__}: {e}")

    print_table(rows, args.dry_run)
    if args.dry_run:
        print("--dry-run: nothing written")
    else:
        print(f"wrote {args.out}\n\nNow verify it. This is not optional:\n"
              f"  tools/usd_joint_report.py --usd {args.out} "
              f"--mjcf {args.mjcf}")
    close_pxr()
    return 0


if __name__ == "__main__":
    sys.exit(main())
