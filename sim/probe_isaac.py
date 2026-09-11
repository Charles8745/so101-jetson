#!/usr/bin/env python3
"""Run this ON SPARK. Reports what Isaac is installed, and dumps joint limits.

Two stages, deliberately separated:

    --stage env      (default) Which Isaac, which Python, which module layout.
                     Imports NOTHING heavyweight -- it only asks the import
                     system whether modules exist. It cannot hang and cannot
                     crash a running simulator.

    --stage limits   Read the SO-101 model's joint limits out of a USD file and
                     write the JSON that tools/simmap_init.py needs.

Why stage `limits` does not touch Isaac
---------------------------------------
Joint limits are a property of the USD file, not of the simulator, and the
UsdPhysics schema is stable across Isaac versions in a way the Python API is
not. So we read them with `pxr` directly rather than through any Isaac API.

** Getting pxr is the awkward part, and on Spark both obvious answers are
** wrong: usd-core has no Linux aarch64 wheel, and python.sh on its own does
** not provide pxr either -- there is no pxr directory in the Isaac tree until
** Kit is running. See common/usd_env.py for the measured table. --via kit
** starts a headless SimulationApp for about 17 s purely to get the import
** path; --via direct refuses to.

** UsdPhysics stores revolute limits in DEGREES. ** (`physics:lowerLimit` /
`physics:upperLimit`, UsdPhysicsRevoluteJoint.) We convert to radians here and
say so in the output, because silently mixing the two is exactly the failure
arm/sim_mapping.py exists to prevent.

    python3 sim/probe_isaac.py
    python3 sim/probe_isaac.py --stage limits --usd /path/to/so101.usd \
        --out sim_limits.json
"""
import argparse
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm.units import BODY_JOINTS, GRIPPER  # noqa: E402
from common.usd_env import close_pxr, ensure_pxr, flat  # noqa: E402

CANDIDATE_MODULES = [
    # Isaac Sim >= 4.5 layout
    "isaacsim", "isaacsim.simulation_app", "isaacsim.core.api",
    "isaacsim.core.prims", "isaacsim.core.utils",
    # Isaac Sim <= 4.2 layout
    "omni.isaac.kit", "omni.isaac.core", "omni.isaac.core.articulations",
    # Isaac Lab, both names
    "isaaclab", "isaaclab.assets", "omni.isaac.lab",
    # USD
    "pxr", "pxr.Usd", "pxr.UsdPhysics",
]

DIST_PREFIXES = ("isaacsim", "isaac-sim", "isaaclab", "isaac-lab", "omniverse",
                 "usd-core", "warp-lang", "torch")


def stage_env(args):
    import importlib.util
    import importlib.metadata as md

    print("python           ", sys.version.replace("\n", " "))
    print("executable       ", sys.executable)
    print("platform         ", sys.platform)
    print()

    print("--- modules present (spec lookup only, nothing imported) ---")
    for name in CANDIDATE_MODULES:
        try:
            spec = importlib.util.find_spec(name)
        except (ImportError, ValueError, AttributeError, ModuleNotFoundError) as e:
            print(f"  {name:<32} ERROR {type(e).__name__}")
            continue
        where = ""
        if spec is not None:
            where = (spec.origin or (spec.submodule_search_locations and
                                     list(spec.submodule_search_locations)[0]) or "")
        print(f"  {name:<32} {'yes' if spec else 'no ':<4} {where}")
    print()

    print("--- installed distributions of interest ---")
    seen = []
    try:
        for d in md.distributions():
            n = (d.metadata["Name"] or "").lower()
            if any(n.startswith(p) for p in DIST_PREFIXES):
                seen.append(f"  {d.metadata['Name']:<32} {d.version}")
    except Exception as e:
        print(f"  (could not enumerate: {type(e).__name__}: {e})")
    print("\n".join(sorted(seen)) or "  (none matched)")
    print()

    print("--- environment ---")
    for k in ("ISAAC_PATH", "EXP_PATH", "CARB_APP_PATH", "ISAACSIM_PATH",
              "OMNI_KIT_ACCEPT_EULA", "CONDA_DEFAULT_ENV", "VIRTUAL_ENV"):
        if os.environ.get(k):
            print(f"  {k}={os.environ[k]}")
    print()

    print("--- looking for an SO-101 asset ---")
    pats = ["so101", "so_101", "so-101", "so100", "so_100"]
    roots = [os.path.expanduser("~"), "/isaac-sim", "/opt", "/workspace"]
    if os.environ.get("ISAAC_PATH"):
        roots.append(os.environ["ISAAC_PATH"])
    hits = []
    for root in roots:
        if not os.path.isdir(root):
            continue
        for ext in ("usd", "usda", "usdc", "urdf", "xml"):
            for p in glob.glob(os.path.join(root, "**", f"*.{ext}"),
                               recursive=True)[:20000]:
                b = os.path.basename(p).lower()
                if any(k in b for k in pats):
                    hits.append(p)
    for h in sorted(set(hits))[:40]:
        print("  " + h)
    if not hits:
        print("  (none found -- say so, it decides whether we import a URDF)")
    return 0


def stage_limits(args):
    # ** The advice that used to be here was wrong on the machine this runs
    # ** on. `python.sh` alone does NOT provide pxr on Spark -- there is no
    # ** pxr directory in the Isaac tree at all -- and usd-core has no Linux
    # ** aarch64 wheel. common/usd_env.py has the measured table; it starts
    # ** Kit only if it has to.
    try:
        route, where = ensure_pxr(args.via)
        print(f"pxr via {route}: {', '.join(where)}")
        from pxr import Usd, UsdPhysics
    except Exception as e:
        print(f"cannot get pxr: {flat(e)}")
        return 1

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print(f"could not open {args.usd}")
        return 1

    rows = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint) and \
                not prim.IsA(UsdPhysics.PrismaticJoint):
            continue
        j = (UsdPhysics.RevoluteJoint(prim) if prim.IsA(UsdPhysics.RevoluteJoint)
             else UsdPhysics.PrismaticJoint(prim))
        kind = "revolute" if prim.IsA(UsdPhysics.RevoluteJoint) else "prismatic"
        lo = j.GetLowerLimitAttr().Get()
        hi = j.GetUpperLimitAttr().Get()
        axis = j.GetAxisAttr().Get()
        # ** Only a revolute joint has radians. ** A prismatic joint's limits
        # are a LENGTH. Copying them into a field called `lower_rad` would put
        # metres into the map's radian slot with nothing to catch it -- the
        # exact category of error this whole subsystem exists to prevent, and
        # gripper jaws are frequently prismatic. So the converted fields are
        # written only for revolute joints, and left null otherwise.
        rev = (kind == "revolute" and lo is not None and hi is not None)
        rows.append({"name": prim.GetName(), "path": str(prim.GetPath()),
                     "type": kind, "axis": str(axis),
                     "native_units": ("deg" if kind == "revolute" else "length"),
                     "lower_native": lo, "upper_native": hi,
                     "limited": lo is not None and hi is not None,
                     "lower_rad": (math.radians(lo) if rev else None),
                     "upper_rad": (math.radians(hi) if rev else None)})

    print(f"{'joint':<28}{'type':<11}{'axis':<6}{'lower':>12}{'upper':>12}"
          f"{'  (native units)'}")
    print("-" * 84)
    for r in rows:
        print(f"{r['name']:<28}{r['type']:<11}{r['axis']:<6}"
              f"{str(r['lower_native']):>12}{str(r['upper_native']):>12}")
    print(f"\n{len(rows)} joints. Revolute limits above are DEGREES (UsdPhysics "
          f"schema); the JSON below is in RADIANS.")

    out = {"usd": os.path.abspath(args.usd),
           "note": "limits_rad is [rad_at_the_arm's_range_min, "
                   "rad_at_the_arm's_range_max]. The probe writes the model's "
                   "own [lower, upper]; if a joint turns out to run the other "
                   "way, pass it to `simmap_init.py fit --flip`.",
           "all_joints": rows, "dof_names": {}, "limits_rad": {},
           "gripper_rad": None}

    wanted_all = list(BODY_JOINTS) + [GRIPPER]
    problems = []

    if args.joints == "identity":
        # Not a default: an assertion. Every name is checked below, and a
        # missing one is an error rather than a joint quietly left unmapped.
        want = {n: n for n in wanted_all}
        print("\n--joints identity: the USD is asserted to name the joints "
              "exactly as lerobot does. Each one is checked below.")
    elif args.joints:
        want = dict(p.split("=", 1) for p in args.joints.split(","))
    else:
        want = None

    if want is not None:
        by_name = {r["name"]: r for r in rows}
        for our, theirs in want.items():
            r = by_name.get(theirs)
            if r is None:
                problems.append(f"no joint named {theirs!r} in the USD")
                continue
            out["dof_names"][our] = theirs
            if not r["limited"]:
                problems.append(f"{theirs} has no limits (free-spinning?) -- "
                                f"a two-point fit needs both ends")
                continue
            if r["type"] != "revolute":
                problems.append(
                    f"{theirs} is {r['type']}, not revolute: its limits are a "
                    f"LENGTH, not an angle. The map is built in radians and "
                    f"would silently treat metres as radians. Decide the "
                    f"conversion deliberately and write it in by hand.")
                continue
            if our == "gripper":
                out["gripper_rad"] = [r["lower_rad"], r["upper_rad"]]
            else:
                out["limits_rad"][our] = [r["lower_rad"], r["upper_rad"]]
        for j in wanted_all:
            if j not in want:
                problems.append(f"{j} was not named in --joints, so it has no "
                                f"entry in the limits file")
    else:
        problems.append(
            "no --joints given, so limits_rad and gripper_rad are EMPTY and "
            "this file cannot build a map")
        print("\nPass --joints to name which USD joint is which SO-101 joint. "
              "If the USD\nuses lerobot's own names, say so explicitly:"
              "\n  --joints identity"
              "\nOtherwise spell it out, e.g."
              "\n  --joints shoulder_pan=Rotation,shoulder_lift=Pitch,"
              "elbow_flex=Elbow,wrist_flex=Wrist_Pitch,wrist_roll=Wrist_Roll,"
              "gripper=Jaw")

    if problems:
        out["problems"] = problems
        print("\n!! this limits file is INCOMPLETE:")
        for x in problems:
            print("   - " + x)

    if args.out:
        with open(args.out, "w") as fh:
            json.dump(out, fh, indent=2)
        print(f"\nwrote {args.out}")

    # ** A file that cannot build a map must not report success. ** On
    # 2026-09-11 this returned 0 with an empty limits_rad, the caller checked
    # only the exit code, and simmap fit then wrote a map in which all six
    # joints were constants -- which would have driven the simulated arm to a
    # fixed pose while every statistic stayed green.
    if problems:
        print("\nexit 1: the file was written, but it cannot build a map.")
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", choices=("env", "limits"), default="env")
    ap.add_argument("--usd", default=None)
    ap.add_argument("--joints", default=None,
                    help="our_name=usd_joint_name, comma separated")
    ap.add_argument("--out", default=None)
    ap.add_argument("--via", choices=("auto", "direct", "kit"), default="auto",
                    help="how to get pxr for --stage limits (default auto)")
    args = ap.parse_args()
    if args.stage == "limits" and not args.usd:
        ap.error("--stage limits needs --usd")
    try:
        return stage_env(args) if args.stage == "env" else stage_limits(args)
    finally:
        close_pxr()


if __name__ == "__main__":
    sys.exit(main())
