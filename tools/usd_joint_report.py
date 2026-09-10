#!/usr/bin/env python3
"""What a USD asset ACTUALLY contains, per joint, in BOTH unit spellings.

Needs only `pxr` (usd-core). No SimulationApp, no GPU, no five minute startup,
and nothing that changes when Isaac is upgraded.

Why this tool exists
--------------------
On 2026-09-08 a gain sweep wrote MuJoCo's `kp` straight into the USD drive and
concluded things from the result. The number was wrong by a factor of 57.29578
and three other parameters were never written at all. Nothing failed, nothing
warned, and the run produced a clean-looking table of numbers that decided
nothing. Two days of conclusions had to be thrown away.

    ** UsdPhysics stores an ANGULAR drive's stiffness per DEGREE. **
    ** PhysX, and everything that reads gains back at runtime, uses RADIANS. **
    ** They differ by 180/pi = 57.29578, and BOTH are called `kp`. **

So this tool never prints one number where two are meant. Every angular gain is
printed in both spellings, on the same line, always. A number you cannot pin a
unit to is not a measurement.

Prismatic joints are the mirror-image trap: their limits are a LENGTH and their
drive gains are per METRE. There is no degree/radian conversion to do, and
printing one would put metres in a slot labelled radians. This tool refuses,
the same way sim/probe_isaac.py refuses.

Three modes
-----------
    report            one asset -> per joint: type, axis, limits, drive gains,
                      armature, joint friction. MISSING is called MISSING.

    --compare B.usd   two assets -> structural diff. Use this before swapping
                      the asset under a fitted simmap: if a joint name, type or
                      limit moved, the map silently stops meaning what it meant.
                      Exits 1 on any structural difference.

    --mjcf M.xml      resolve a MuJoCo model's EFFECTIVE actuator values (its
                      class defaults, its per-element overrides) and print what
                      the USD should therefore contain -- next to what it does.
                      Written because so101_new_calib.xml carries three sets of
                      numbers and only one of them is live.

Usage
-----
    python3 tools/usd_joint_report.py --usd so101.usda
    python3 tools/usd_joint_report.py --usd new.usda --compare old.usda
    python3 tools/usd_joint_report.py --usd new.usda --mjcf so101_new_calib.xml
    python3 tools/usd_joint_report.py --usd so101.usda --json report.json
"""
import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

DEG_PER_RAD = 180.0 / math.pi          # 57.29577951308232

# Attribute-name fragments we hunt for rather than importing PhysxSchema.
# PhysX schemas ship with Omniverse, not with usd-core, and the attribute has
# lived under more than one schema name across releases. Searching the prim's
# own authored attributes works in both places and tells us what is really
# there instead of what we assumed would be.
ARMATURE_HINTS = ("armature",)
FRICTION_HINTS = ("jointfriction", "friction")


def _fail(msg):
    # pxr raises multi-line exceptions with embedded file paths and tabs. A
    # failure message that scrolls is a failure message nobody reads.
    flat = " ".join(str(msg).split())
    print(f"usd_joint_report: {flat}", file=sys.stderr)
    return 2


def _attr(prim, name):
    a = prim.GetAttribute(name)
    if not a or not a.IsValid():
        return None
    return a.Get()


def _find_attrs(prim, hints):
    """Authored attributes whose name contains any hint. Reported verbatim."""
    out = []
    for a in prim.GetAttributes():
        n = a.GetName()
        low = n.lower()
        if any(h in low for h in hints):
            v = a.Get()
            if v is not None:
                out.append((n, v))
    return out


def read_usd(path):
    from pxr import Usd, UsdPhysics

    if not os.path.isfile(path):
        raise RuntimeError(f"no such file: {path}")
    stage = Usd.Stage.Open(path)
    if stage is None:
        raise RuntimeError(f"could not open {path}")

    joints = []
    for prim in stage.Traverse():
        is_rev = prim.IsA(UsdPhysics.RevoluteJoint)
        is_pri = prim.IsA(UsdPhysics.PrismaticJoint)
        if not is_rev and not is_pri:
            continue
        kind = "revolute" if is_rev else "prismatic"
        j = (UsdPhysics.RevoluteJoint(prim) if is_rev
             else UsdPhysics.PrismaticJoint(prim))

        lo = j.GetLowerLimitAttr().Get()
        hi = j.GetUpperLimitAttr().Get()
        axis = j.GetAxisAttr().Get()

        # Which drive did somebody actually apply? Ask, do not assume: a
        # revolute joint with a `linear` drive is a real mistake we want to see.
        drives = {}
        for token in ("angular", "linear"):
            st = _attr(prim, f"drive:{token}:physics:stiffness")
            da = _attr(prim, f"drive:{token}:physics:damping")
            mf = _attr(prim, f"drive:{token}:physics:maxForce")
            tp = _attr(prim, f"drive:{token}:physics:targetPosition")
            ty = _attr(prim, f"drive:{token}:physics:type")
            if any(v is not None for v in (st, da, mf, tp, ty)):
                drives[token] = {"stiffness": st, "damping": da,
                                 "maxForce": mf, "targetPosition": tp,
                                 "type": str(ty) if ty is not None else None}

        joints.append({
            "name": prim.GetName(),
            "path": str(prim.GetPath()),
            "type": kind,
            "axis": str(axis) if axis is not None else None,
            "lower": lo, "upper": hi,
            "drives": drives,
            "armature": _find_attrs(prim, ARMATURE_HINTS),
            "friction": _find_attrs(prim, FRICTION_HINTS),
        })
    return {"usd": path, "joints": joints}


def _fmt_pair(v, per_deg_label, per_rad_label):
    """One gain, both spellings, on one line. Never one without the other."""
    if v is None:
        return f"MISSING        ({per_deg_label} / {per_rad_label})"
    return (f"{v:<14.6g} {per_deg_label:<16s} = "
            f"{v * DEG_PER_RAD:<14.6g} {per_rad_label}")


def print_report(rep):
    print(f"=== {rep['usd']}")
    print(f"    {len(rep['joints'])} joint(s); 1 rad = {DEG_PER_RAD:.9g} deg")
    missing = {"drive": [], "armature": [], "friction": []}

    for j in rep["joints"]:
        print()
        print(f"{j['name']:<22s} {j['type']:<10s} axis {j['axis']}")
        print(f"  path          {j['path']}")

        lo, hi = j["lower"], j["upper"]
        if j["type"] == "revolute":
            if lo is None or hi is None:
                print("  limits        MISSING")
            else:
                print(f"  limits        {lo:.4f} .. {hi:.4f} deg"
                      f"   ({math.radians(lo):.6f} .. {math.radians(hi):.6f} rad)")
        else:
            # A prismatic limit is a LENGTH. Converting it to radians would put
            # metres in a slot labelled radians -- the exact error this whole
            # subsystem exists to prevent. So we do not.
            if lo is None or hi is None:
                print("  limits        MISSING")
            else:
                print(f"  limits        {lo:.6f} .. {hi:.6f} (length; NOT an angle)")

        if not j["drives"]:
            print("  drive         MISSING  <- joint will not follow a target")
            missing["drive"].append(j["name"])
        for token, d in j["drives"].items():
            note = ""
            if token == "linear" and j["type"] == "revolute":
                note = "  <- LINEAR drive on a REVOLUTE joint"
            if token == "angular" and j["type"] == "prismatic":
                note = "  <- ANGULAR drive on a PRISMATIC joint"
            print(f"  drive[{token}]  type={d['type']}"
                  f"  target={d['targetPosition']}{note}")
            if token == "angular":
                print("    stiffness   " + _fmt_pair(d["stiffness"],
                                                     "N*m/deg", "N*m/rad"))
                print("    damping     " + _fmt_pair(d["damping"],
                                                     "N*m*s/deg", "N*m*s/rad"))
            else:
                print(f"    stiffness   {d['stiffness']}  N/m   (linear; no deg/rad)")
                print(f"    damping     {d['damping']}  N*s/m (linear; no deg/rad)")
            mf = d["maxForce"]
            unit = "N*m" if token == "angular" else "N"
            print(f"    maxForce    {mf}  {unit}   (a force/torque: NO deg/rad conversion)")

        if j["armature"]:
            for n, v in j["armature"]:
                print(f"  armature      {v:<14.6g} kg*m^2 (no conversion)   attr {n}")
        else:
            print("  armature      MISSING  <- reflected rotor inertia absent;"
                  " the joint is far easier to accelerate than the real one")
            missing["armature"].append(j["name"])

        if j["friction"]:
            for n, v in j["friction"]:
                print(f"  friction      {v:<14.6g} N*m    (no conversion)   attr {n}")
        else:
            print("  friction      MISSING")
            missing["friction"].append(j["name"])

    print()
    print("--- summary")
    for k, names in missing.items():
        if names:
            print(f"  {k} MISSING on {len(names)}: {', '.join(names)}")
    if not any(missing.values()):
        print("  every joint has a drive, an armature and a friction value")
    return missing


def compare(a, b, tol):
    """Structural diff. A fitted simmap is pinned to joint names and limits."""
    an = {j["name"]: j for j in a["joints"]}
    bn = {j["name"]: j for j in b["joints"]}
    print(f"=== compare\n  A {a['usd']}\n  B {b['usd']}")

    diffs = []
    only_a = sorted(set(an) - set(bn))
    only_b = sorted(set(bn) - set(an))
    for n in only_a:
        diffs.append(f"only in A: {n}")
    for n in only_b:
        diffs.append(f"only in B: {n}")

    for n in sorted(set(an) & set(bn)):
        ja, jb = an[n], bn[n]
        if ja["type"] != jb["type"]:
            diffs.append(f"{n}: type {ja['type']} vs {jb['type']}")
        if ja["axis"] != jb["axis"]:
            diffs.append(f"{n}: axis {ja['axis']} vs {jb['axis']}")
        for key in ("lower", "upper"):
            va, vb = ja[key], jb[key]
            if va is None or vb is None:
                if va is not vb:
                    diffs.append(f"{n}: {key} {va} vs {vb}")
            elif abs(va - vb) > tol:
                diffs.append(f"{n}: {key} {va:.6f} vs {vb:.6f}"
                             f"  (delta {vb - va:+.6f})")

    print(f"  {len(set(an) & set(bn))} shared, "
          f"{len(only_a)} only-A, {len(only_b)} only-B")
    if diffs:
        print("  STRUCTURAL DIFFERENCES -- a simmap fitted on one is not valid"
              " on the other:")
        for d in diffs:
            print(f"    {d}")
        return 1
    print("  identical joint names, types, axes and limits.")
    print("  A simmap fitted on one is still valid on the other.")
    return 0


# ---------------------------------------------------------------- MJCF side

def _merge_class_tree(node, inherited, classes):
    """Resolve <default> inheritance into {class_name: {tag: {attr: value}}}."""
    name = node.get("class")
    own = {k: dict(v) for k, v in inherited.items()}
    for child in node:
        if child.tag == "default":
            continue
        own.setdefault(child.tag, {}).update(child.attrib)
    if name:
        classes[name] = own
    for child in node:
        if child.tag == "default":
            _merge_class_tree(child, own, classes)


def read_mjcf(path):
    root = ET.parse(path).getroot()
    comp = root.find("compiler")
    angle = comp.get("angle", "degree") if comp is not None else "degree"
    if angle != "radian":
        raise RuntimeError(
            f"{path}: <compiler angle='{angle}'>. This tool only handles "
            "angle='radian' models; a degree-native MJCF needs different "
            "arithmetic and guessing which one you have is how the 57.29578 "
            "bug happened in the first place.")

    classes = {}
    for d in root.findall("default"):
        _merge_class_tree(d, {}, classes)

    joints = {}
    for jt in root.iter("joint"):
        n = jt.get("name")
        if not n:
            continue
        cls = jt.get("class")
        base = dict(classes.get(cls, {}).get("joint", {})) if cls else {}
        base.update({k: v for k, v in jt.attrib.items()
                     if k not in ("name", "class")})
        joints[n] = {"class": cls, "attrs": base, "resolved": cls is not None}

    acts = {}
    for act in root.iter("position"):
        jn = act.get("joint")
        if not jn:
            continue
        cls = act.get("class")
        base = dict(classes.get(cls, {}).get("position", {})) if cls else {}
        # An attribute written on the element beats the class default. This is
        # the rule that makes forcerange="-2.94 2.94" in class sts3215 dead
        # code: every actuator overrides it with -3.35 3.35.
        base.update({k: v for k, v in act.attrib.items()
                     if k not in ("joint", "class", "name")})
        acts[jn] = {"class": cls, "attrs": base, "resolved": cls is not None}

    return {"mjcf": path, "angle": angle, "classes": sorted(classes),
            "joints": joints, "actuators": acts}


def expected_from_mjcf(mj):
    """MuJoCo effective values -> what the USD fields should hold."""
    out = {}
    for name, a in mj["actuators"].items():
        ja = mj["joints"].get(name, {}).get("attrs", {})
        pa = a["attrs"]

        kp = float(pa["kp"]) if "kp" in pa else None
        kv = float(pa.get("kv", 0.0))
        jd = float(ja.get("damping", 0.0))
        damp_rad = kv + jd

        fr = pa.get("forcerange")
        maxf = abs(float(fr.split()[-1])) if fr else None

        out[name] = {
            "unresolved": not (a["resolved"] and
                               mj["joints"].get(name, {}).get("resolved")),
            "kp_rad": kp,
            "usd_stiffness_deg": None if kp is None else kp / DEG_PER_RAD,
            "damping_rad": damp_rad,
            "usd_damping_deg": damp_rad / DEG_PER_RAD,
            "damping_parts": {"actuator_kv": kv, "joint_damping": jd},
            "maxForce": maxf,
            "armature": float(ja["armature"]) if "armature" in ja else None,
            "frictionloss": (float(ja["frictionloss"])
                             if "frictionloss" in ja else None),
        }
    return out


def print_expected(mj, exp, rep, tol_rel):
    print(f"=== {mj['mjcf']}  (angle={mj['angle']}, "
          f"classes: {', '.join(mj['classes'])})")
    print("    MuJoCo is radian-native. The USD angular drive field is PER "
          "DEGREE.\n    Writing the left column into the right field is the "
          "57.29578x bug.")
    have = {j["name"]: j for j in rep["joints"]} if rep else {}
    bad = 0

    for name, e in sorted(exp.items()):
        if e["unresolved"]:
            print(f"\n{name:<22s} UNRESOLVED -- no class= on the joint or the "
                  "actuator.\n    childclass inheritance is not implemented "
                  "here on purpose: guessing\n    it is how you get a "
                  "plausible number that is wrong. Resolve by hand.")
            bad += 1
            continue

        print(f"\n{name}")
        dp = e["damping_parts"]
        rows = [
            ("stiffness", e["usd_stiffness_deg"], "N*m/deg",
             e["kp_rad"], "N*m/rad  (MuJoCo kp)"),
            ("damping", e["usd_damping_deg"], "N*m*s/deg",
             e["damping_rad"],
             f"N*m*s/rad  (kv {dp['actuator_kv']} + joint damping "
             f"{dp['joint_damping']})"),
        ]
        for label, deg, du, rad, ru in rows:
            if deg is None:
                print(f"  {label:<12s} MISSING in the MJCF")
                continue
            print(f"  {label:<12s} USD should hold {deg:<12.6g} {du:<10s}"
                  f" = {rad:<10.6g} {ru}")
            got = None
            if name in have:
                d = have[name]["drives"].get("angular")
                if d:
                    got = d["stiffness"] if label == "stiffness" else d["damping"]
            if rep is not None:
                if got is None:
                    print(f"  {'':<12s}   asset holds MISSING            <- FAIL")
                    bad += 1
                elif abs(got - deg) <= tol_rel * max(abs(deg), 1e-12):
                    print(f"  {'':<12s}   asset holds {got:<12.6g}  ok")
                else:
                    ratio = got / deg if deg else float("inf")
                    hint = ("  (x57.29578 -- radian value in a degree field)"
                            if abs(ratio - DEG_PER_RAD) < 0.05 * DEG_PER_RAD
                            else "")
                    print(f"  {'':<12s}   asset holds {got:<12.6g}  "
                          f"FAIL x{ratio:.4g}{hint}")
                    bad += 1

        for label, val, unit in (("maxForce", e["maxForce"], "N*m"),
                                 ("armature", e["armature"], "kg*m^2"),
                                 ("frictionloss", e["frictionloss"], "N*m")):
            if val is None:
                print(f"  {label:<12s} MISSING in the MJCF")
                continue
            print(f"  {label:<12s} USD should hold {val:<12.6g} {unit}"
                  f"   (no deg/rad conversion)")
            if rep is not None and name in have:
                if label == "maxForce":
                    d = have[name]["drives"].get("angular")
                    got = d["maxForce"] if d else None
                elif label == "armature":
                    got = have[name]["armature"][0][1] if have[name]["armature"] else None
                else:
                    got = have[name]["friction"][0][1] if have[name]["friction"] else None
                if got is None:
                    print(f"  {'':<12s}   asset holds MISSING            <- FAIL")
                    bad += 1
                elif abs(got - val) <= tol_rel * max(abs(val), 1e-12):
                    print(f"  {'':<12s}   asset holds {got:<12.6g}  ok")
                else:
                    print(f"  {'':<12s}   asset holds {got:<12.6g}  FAIL")
                    bad += 1

    print()
    if rep is None:
        print("--- no --usd given: expectations only, nothing checked")
        return 0
    print(f"--- {bad} mismatch(es)" if bad else "--- asset matches the MJCF")
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd")
    ap.add_argument("--compare", metavar="OTHER.usd")
    ap.add_argument("--mjcf", metavar="MODEL.xml")
    ap.add_argument("--json", metavar="OUT.json")
    ap.add_argument("--tol-deg", type=float, default=1e-6,
                    help="limit-comparison tolerance, degrees (default 1e-6)")
    ap.add_argument("--tol-rel", type=float, default=1e-4,
                    help="gain-comparison relative tolerance (default 1e-4)")
    args = ap.parse_args()

    if not args.usd and not args.mjcf:
        return _fail("nothing to do: give --usd and/or --mjcf")

    rep = None
    if args.usd:
        try:
            rep = read_usd(args.usd)
        except ImportError:
            return _fail("cannot import pxr. Run with Isaac's python.sh, or "
                         "pip install usd-core.")
        except Exception as e:
            return _fail(f"{type(e).__name__}: {e}")

    rc = 0
    if rep is not None and not args.mjcf:
        print_report(rep)

    if args.compare:
        if rep is None:
            return _fail("--compare needs --usd")
        try:
            other = read_usd(args.compare)
        except Exception as e:
            return _fail(f"{type(e).__name__}: {e}")
        print()
        rc |= compare(rep, other, args.tol_deg)

    if args.mjcf:
        try:
            mj = read_mjcf(args.mjcf)
        except Exception as e:
            return _fail(f"{type(e).__name__}: {e}")
        exp = expected_from_mjcf(mj)
        if rep is not None:
            print_report(rep)
            print()
        rc |= print_expected(mj, exp, rep, args.tol_rel)

    if args.json:
        blob = {"deg_per_rad": DEG_PER_RAD, "usd": rep}
        if args.mjcf:
            blob["mjcf"] = {"path": args.mjcf, "expected": exp}
        with open(args.json, "w") as f:
            json.dump(blob, f, indent=2, sort_keys=True)
        print(f"\nwrote {args.json}")

    return rc


if __name__ == "__main__":
    sys.exit(main())
