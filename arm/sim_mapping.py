"""Mapping real SO-101 joint readings onto a simulated SO-101's joints.

** This is the single most dangerous piece of code in the repo. **

A wrong mapping does not crash and does not look wrong: the virtual arm moves,
smoothly, in roughly the right direction, and every dataset collected with it is
quietly worthless. So nothing here is assumed -- the mapping is a FILE, it is
FITTED from measurements, it is VERIFIED on a pose that was not used in the fit,
and it is INTERLOCKED against the arm calibration it was derived from.

What has to be reconciled
-------------------------
Real arm (see arm/units.py):
  body joints  degrees, zero at the MIDPOINT OF THIS ARM'S CALIBRATED TRAVEL
  gripper      percent 0..100 of its calibrated travel
Simulated arm:
  every joint  radians, zero defined by the USD model's joint frame,
               bounded by the model's limits

There is no reason those zeros coincide, and no reason the axis directions
agree. Assuming `radians(degrees)` is the mistake this module exists to prevent.

How the mapping is established
------------------------------
Two-point fit, third-point check:
  1. FIT on the two extremes. The real arm's calibrated range_min/range_max are
     its physical travel limits; the model's joint limits are the modelled ones.
     A two-point affine fit through them also RECOVERS THE SIGN -- if the axis
     runs the other way the fitted scale simply comes out negative.
  2. VERIFY at a third pose that took no part in the fit. If the model is
     faithful the middle must land too. If it does not, the model and the real
     arm are not the same shape and no affine map will save it -- stop.

  3. The two-point fit ASSUMES the real arm's calibrated travel is the same
     travel the model declares. If the arm's mechanical stops and the USD's
     joint limits disagree, an affine fit through the endpoints is exact AT the
     endpoints and wrong everywhere in between -- and a midpoint check cannot
     see it, because an affine fit through two points passes through their
     midpoint by construction. So we compare the SPANS:

         span_ratio = (hi_deg - lo_deg) / degrees(hi_rad - lo_rad)

     1.0 means the real arm and the model have the same range of motion. 1.10
     means the map stretches every intermediate angle by 10 percent. This costs
     nothing, needs no instrument, and is the one check that catches "the model
     is not this arm".

** Two fit modes, and IDENTITY is the right default **
---------------------------------------------------
Measuring the SO-101's own URDF against a real arm's calibration made the point
concrete. shoulder_pan and elbow_flex came out at span ratios of 1.005 and
1.003 -- the endpoint fit lands within half a percent of exactly pi/180. That is
not a coincidence: **the URDF is a faithful geometric model of the same
linkage**, so one degree of real joint rotation IS one degree of model rotation,
by definition.

Once you believe that, the endpoint stretch is the wrong operation. It is only
harmless where the two spans happen to agree, and where they do not it is
DEFINITELY wrong, because it distributes a limits-declaration difference across
every angle in between. On the same arm, wrist_roll's real travel is 360 deg
(the servo turns freely, so calibration recorded the whole encoder range) while
the URDF declares 320 deg -- a deliberate modelling choice, probably to stop
cable wrap. Stretching 360 onto 320 would make every wrist_roll angle in the
dataset wrong by up to 12 percent. Mapping degree-for-degree and letting the
last 40 degrees CLIP is correct, honest, and reported.

  "identity"   scale = +-pi/180 exactly. The sign still has to be observed; the
               offset aligns the two zeros. A span ratio away from 1 then
               predicts CLIPPING, and is not a fit error.       <- the default
  "endpoints"  the two-point stretch. Right only when the arm's calibrated
               travel and the model's declared travel are the same physical
               extremes. A span ratio away from 1 IS a fit error here.

Interlock
---------
The map records the sha256 of the calibration it was fitted against, and WHICH
ARM it was fitted for. Recalibrate the arm and the degrees zero moves, so the
map is automatically invalidated and must be refitted. Point it at the other arm
and the role check refuses: p3 maps from the LEADER (the arm a human moves);
p4/p5 will map from the FOLLOWER. Same sim, different arm, different calibration
midpoint, different map. p3 refuses to record with an unverified, stale or
wrong-role map.
"""
import hashlib
import json
import math
import os
import time

SCHEMA = "so101.simmap.v1"

# Span-ratio thresholds (see the docstring). Above WARN the map is
# suspicious; above FAIL the model is a different linkage and no affine
# map is honest -- fix the USD or the arm calibration, do not force it.
SPAN_WARN = 0.02
SPAN_FAIL = 0.10

DEG_TO_RAD = math.pi / 180.0
FIT_IDENTITY = "identity"
FIT_ENDPOINTS = "endpoints"


def calibration_sha256(calibration):
    """Stable hash of a lerobot calibration dict (order- and format-independent).

    `calibration` may be the raw JSON dict, or lerobot MotorCalibration objects.
    """
    norm = {}
    for name, c in calibration.items():
        d = c if isinstance(c, dict) else {
            "id": c.id, "drive_mode": c.drive_mode,
            "homing_offset": c.homing_offset,
            "range_min": c.range_min, "range_max": c.range_max}
        norm[name] = {k: int(d[k]) for k in
                      ("id", "drive_mode", "homing_offset",
                       "range_min", "range_max")}
    blob = json.dumps(norm, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode()).hexdigest()


def fit_affine(src_a, dst_a, src_b, dst_b):
    """Solve dst = scale*src + offset through two points.

    Recovers a negative scale on its own when the two axes run opposite ways.
    """
    if src_b == src_a:
        raise ValueError("the two fit points have the same source value")
    scale = (dst_b - dst_a) / (src_b - src_a)
    offset = dst_a - scale * src_a
    return scale, offset


class SimMap:
    def __init__(self, doc):
        if doc.get("schema") != SCHEMA:
            raise ValueError(f"expected schema {SCHEMA}, got {doc.get('schema')}")
        self.doc = doc
        self.joints = doc["joints"]

    # ---------- construction ----------
    @staticmethod
    def from_file(path):
        with open(path) as fh:
            return SimMap(json.load(fh))

    def save(self, path):
        """Write via a temporary file and rename, so a crash mid-write cannot
        leave an unparseable map where a working one used to be."""
        tmp = f"{path}.tmp{os.getpid()}"
        with open(tmp, "w") as fh:
            json.dump(self.doc, fh, indent=2, sort_keys=True)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)

    # ---------- guards ----------
    def is_verified(self):
        """A verification counts only if it is BOUND to this exact fit.

        There is deliberately no allowance for a `verified` block without
        `fit_sha256`. An earlier version treated a missing binding as "an old
        map, trust it", which meant the entire interlock could be defeated by
        deleting one field from the JSON -- and nothing on screen would have
        said so. The schema is v1; there are no legacy maps to be kind to.
        """
        v = self.doc.get("verified")
        if not v or not v.get("by"):
            return False
        return v.get("fit_sha256") == self.fit_sha256()

    def verification_note(self):
        v = self.doc.get("verified")
        if not v or not v.get("by"):
            return "NOT VERIFIED"
        bound = v.get("fit_sha256")
        if bound is None:
            return (f"UNBOUND: `verified` claims {v.get('by')} "
                    f"({v.get('method')}) but records no fit_sha256, so the "
                    f"claim cannot be tied to these numbers. Re-run verify.")
        if bound != self.fit_sha256():
            return (f"STALE: verified by {v.get('by')} against fit "
                    f"{bound[:12]}, but the fit is now {self.fit_sha256()[:12]} "
                    f"-- the map was edited after it was checked")
        return f"verified by {v.get('by')} ({v.get('method')})"

    def matches_calibration(self, calibration):
        want = self.doc.get("source_calibration_sha256")
        got = calibration_sha256(calibration)
        return (want == got), want, got

    def sha256(self):
        blob = json.dumps(self.doc, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(blob.encode()).hexdigest()

    def fit_sha256(self):
        """Hash of everything EXCEPT the verified block.

        This is what a verification is bound to. Without it, "verified" is just
        a field somebody wrote once: refit the map, keep the block, and the file
        still claims a human checked it -- while claiming it about numbers that
        no longer exist. Binding the attestation to the fit makes an edited map
        unverified again, automatically.
        """
        d = {k: v for k, v in self.doc.items() if k != "verified"}
        blob = json.dumps(d, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(blob.encode()).hexdigest()

    def role(self):
        return (self.doc.get("source_arm") or {}).get("role")

    def arm_id(self):
        return (self.doc.get("source_arm") or {}).get("id")

    def fit_mode(self):
        for spec in self.joints.values():
            if spec.get("mode") == "affine":
                return spec.get("fit_mode", FIT_ENDPOINTS)
        return None

    def unreachable_deg(self):
        """Per joint, how much real travel the model cannot reach.

        Only meaningful for an identity fit, where the scale is exact: the
        leftover is real rotation the sim will clip. Returned in degrees of
        real travel, so it can be read as "the last N degrees of this joint do
        not exist in the sim".
        """
        out = {}
        for j, spec in self.joints.items():
            r, sr = spec.get("span_deg_real"), spec.get("span_deg_model")
            if r is None or sr is None:
                continue
            if r > sr:
                out[j] = r - sr
        return out

    def worst_span_ratio(self):
        """Largest |span_ratio - 1| over the body joints, or None if unfitted."""
        worst, joint = None, None
        for j, spec in self.joints.items():
            r = spec.get("span_ratio")
            if r is None:
                continue
            e = abs(r - 1.0)
            if worst is None or e > worst:
                worst, joint = e, j
        return (worst, joint)

    def guard(self, calibration, allow_unverified=False, expect_role=None,
              allow_span=False):
        """Returns (ok, [reasons]). p3 must call this before recording."""
        bad = []
        if not self.is_verified() and not allow_unverified:
            bad.append(f"map is not usable as verified: {self.verification_note()}")
        if expect_role is not None and self.role() != expect_role:
            bad.append(f"map was fitted for the {self.role()!r} arm but this "
                       f"program drives from the {expect_role!r} arm -- the two "
                       f"arms have different calibration midpoints, so the map "
                       f"does not transfer. Fit a {expect_role} map.")
        # What a span mismatch MEANS depends on how the map was fitted.
        #   endpoints : the stretch is spread over every intermediate angle, so
        #               a big mismatch makes the map wrong everywhere but the
        #               two ends. That is a fit error -> refuse.
        #   identity  : the scale is exactly pi/180 by construction and cannot
        #               be wrong. A mismatch only predicts CLIPPING at the ends,
        #               which apply() already detects and reports per step.
        #               Refusing would be refusing a correct map.
        worst, wj = self.worst_span_ratio()
        if (worst is not None and worst > SPAN_FAIL and not allow_span
                and self.fit_mode() == FIT_ENDPOINTS):
            bad.append(f"{wj} span ratio is off by {worst*100:.1f}% (limit "
                       f"{SPAN_FAIL*100:.0f}%) -- the real arm and the model do "
                       f"not have the same range of motion, so an endpoint fit "
                       f"is wrong everywhere between the ends. Refit with "
                       f"fit_mode=identity, which maps degree for degree and "
                       f"clips instead of stretching.")
        ok, want, got = self.matches_calibration(calibration)
        if not ok:
            if not want:
                bad.append("map records no source_calibration_sha256, so there "
                           "is nothing tying it to an arm. Refit it.")
            else:
                bad.append(f"map was fitted against calibration {want[:12]}... "
                           f"but the arm now reports {got[:12]}... -- the "
                           f"degrees zero has moved, refit the map")
        missing = [j for j in self.joints if j not in calibration]
        if missing:
            bad.append(f"map mentions joints the arm does not have: {missing}")
        # ...and the reverse, which is the dangerous direction: apply() skips
        # joints the map has never heard of, so a map missing the gripper simply
        # records a gripper that never moves. Nothing errors, nothing is
        # clipped, and the dataset looks complete.
        unmapped = [j for j in calibration if j not in self.joints]
        if unmapped:
            bad.append(f"the arm has joints the map does not cover: {unmapped} "
                       f"-- those joints would silently never move in the sim. "
                       f"Add them to the limits file and refit.")
        # A mapping can also be present and useless: scale 0, or a destination
        # range of zero width (the usual cause is a limits file with no
        # gripper_rad). The joint then maps to a constant and never moves --
        # same silent failure, one step further in.
        frozen = []
        for j, spec in self.joints.items():
            if spec.get("mode") == "affine" and spec.get("scale") == 0:
                frozen.append(j)
            elif spec.get("mode") == "range":
                d0, d1 = spec.get("dst_range_rad", (0.0, 0.0))
                if d0 == d1:
                    frozen.append(j)
        if frozen:
            bad.append(f"these joints map to a constant and can never move: "
                       f"{frozen} -- check the limits file (gripper_rad is the "
                       f"usual culprit) and refit.")
        return (not bad), bad

    # ---------- the actual mapping ----------
    def apply_joint(self, joint, value):
        """One joint, real units -> radians. Returns (radians, was_clipped)."""
        spec = self.joints[joint]
        mode = spec["mode"]
        if mode == "affine":
            q = spec["scale"] * value + spec["offset"]
        elif mode == "range":
            s0, s1 = spec["src_range"]
            d0, d1 = spec["dst_range_rad"]
            if s1 == s0:
                raise ValueError(f"{joint}: empty src_range")
            t = (value - s0) / (s1 - s0)
            q = d0 + t * (d1 - d0)
        else:
            raise ValueError(f"{joint}: unknown mode {mode!r}")
        lo, hi = sorted(spec["clip_rad"])
        if q < lo:
            return lo, True
        if q > hi:
            return hi, True
        return q, False

    def apply(self, values):
        """Whole pose. Returns (radians dict, list of clipped joint names).

        A joint that clips is the runtime symptom of a wrong map: the sim could
        not do what we asked. p3 logs it and the operator must be told.
        """
        out, clipped = {}, []
        for j, v in values.items():
            if j not in self.joints:
                continue
            q, was = self.apply_joint(j, float(v))
            out[j] = q
            if was:
                clipped.append(j)
        return out, clipped


def build_map(calibration, sim_limits_rad, gripper_dst_rad,
              sim_target=None, deg_range_fn=None, source_arm=None,
              fit_mode=FIT_IDENTITY):
    """Fit a map from the arm's calibration and the model's joint limits.

    calibration      : lerobot calibration (dict of dicts or MotorCalibration)
    sim_limits_rad   : {joint: (lo_rad, hi_rad)} from the USD, for the 5 body joints.
                       Order matters and encodes the axis direction: the first
                       element must correspond to the arm's range_min.
    gripper_dst_rad  : (rad_at_0_percent, rad_at_100_percent)
    fit_mode         : "identity" (default, degree-for-degree) or "endpoints"
                       (stretch the calibrated travel onto the declared travel).
                       See the module docstring -- identity is right whenever the
                       model is a faithful model of the same linkage, which is
                       the normal case and the one you can check with the span
                       ratio.
    """
    from .units import BODY_JOINTS, GRIPPER
    if deg_range_fn is None:
        from .units import deg_range_from_calibration as deg_range_fn

    def as_dict(c):
        return c if isinstance(c, dict) else {
            "range_min": c.range_min, "range_max": c.range_max}

    joints = {}
    for j in BODY_JOINTS:
        if j not in calibration or j not in sim_limits_rad:
            continue
        lo_deg, hi_deg = deg_range_fn(as_dict(calibration[j]))
        lo_rad, hi_rad = sim_limits_rad[j]
        span_deg = abs(hi_deg - lo_deg)
        span_model_deg = abs(math.degrees(hi_rad - lo_rad))
        if fit_mode == FIT_IDENTITY:
            # Degree for degree. The SIGN still comes from the caller's
            # ordering of sim_limits_rad (that is what --flip edits); the
            # OFFSET aligns the two zeros -- the real zero is the midpoint of
            # the calibrated travel by construction of MotorNormMode.DEGREES,
            # so it maps to the midpoint of the model's declared travel.
            scale = DEG_TO_RAD if hi_rad >= lo_rad else -DEG_TO_RAD
            offset = (lo_rad + hi_rad) / 2.0 - scale * ((lo_deg + hi_deg) / 2.0)
        elif fit_mode == FIT_ENDPOINTS:
            scale, offset = fit_affine(lo_deg, lo_rad, hi_deg, hi_rad)
        else:
            raise ValueError(f"unknown fit_mode {fit_mode!r}")
        joints[j] = {"mode": "affine", "fit_mode": fit_mode,
                     "scale": scale, "offset": offset,
                     "clip_rad": [min(lo_rad, hi_rad), max(lo_rad, hi_rad)],
                     "fit_points_deg": [lo_deg, hi_deg],
                     "fit_points_rad": [lo_rad, hi_rad],
                     "span_deg_real": span_deg,
                     "span_deg_model": span_model_deg,
                     "span_ratio": (span_deg / span_model_deg
                                    if span_model_deg else None)}
    if GRIPPER in calibration and gripper_dst_rad is not None:
        d0, d1 = gripper_dst_rad
        joints[GRIPPER] = {"mode": "range", "src_range": [0.0, 100.0],
                           "dst_range_rad": [d0, d1],
                           "clip_rad": [min(d0, d1), max(d0, d1)]}
    return SimMap({
        "schema": SCHEMA,
        "created_unix": time.time(),
        "source_calibration_sha256": calibration_sha256(calibration),
        "source_arm": source_arm or {},
        "sim_target": sim_target or {},
        "fit": {"method": fit_mode,
                "note": "fitted on the two travel extremes; the third-pose "
                        "check is an INDEPENDENT verification, never refit on it"},
        "joints": joints,
    })


def residuals(simmap, pose_values, expected_rad):
    """Third-pose check: how far off is the map at a pose it was not fitted on?

    Returns (residuals, clipped). ** The clipped list must not be thrown away. **
    Body joints are reported in DEGREES mode, which lerobot does NOT clamp to
    the calibrated range, so a verification pose slightly outside the fitted
    travel maps outside clip_rad, gets clamped to the model limit, and the
    residual is then computed against the clamped number -- bounded, and biased
    towards passing. A verification that clipped is not a verification.
    """
    got, clipped = simmap.apply(pose_values)
    out = {}
    for j, want in expected_rad.items():
        if j in got:
            out[j] = {"want_rad": want, "got_rad": got[j],
                      "err_rad": got[j] - want,
                      "err_deg": math.degrees(got[j] - want),
                      "clipped": j in clipped}
    return out, clipped
