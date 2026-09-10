"""Pure control-loop helpers. No hardware, no I/O -- so they are unit-testable.

Two safety devices live here:

1. rate_limit()  -- clamp how far a joint command may move in ONE step,
   measured against the PREVIOUS COMMAND. lerobot's own `max_relative_target`
   clamps against the follower's *measured* position instead, which forces an
   extra `sync_read` on the servo bus every step. Ours costs no bus traffic.

2. TrackingWatchdog -- what ours cannot see: if the follower is stuck or
   unpowered, the commands keep sailing ahead of reality. We already read the
   follower's real position each step (for the latency log), so we watch the gap
   and call a fault when it stays too large for too long.

Together these cover both failure shapes; neither costs an extra bus read.
"""

# ---------------------------------------------------------------------------
# How fast the follower is allowed to move, as a SPEED.
#
# rate_limit() takes a limit PER STEP, but what is actually safe is a speed:
# 8 degrees per step is 240 deg/s at 30 Hz and 960 deg/s at 120 Hz -- the same
# number means four different machines. Every caller therefore derives its
# per-step limit from its own loop rate, and nobody types the two numbers
# side by side hoping to keep them consistent.
#
# 240 deg/s and 450 %/s are the values the station has been run at: they are
# what `--fps 30 --max-step-deg 8` and `--fps 120 --max-step-deg 2` both mean.
MAX_JOINT_DEG_PER_S = 240.0
MAX_GRIPPER_PCT_PER_S = 450.0


def step_limits_for(fps, max_step_deg=None, max_step_gripper_pct=None):
    """Per-step limits for a loop running at `fps`, as (body_deg, gripper_pct).

    Pass either override to pin it; whatever is left None is derived so that
    the SPEED stays constant no matter what rate the loop runs at.

    Returns (body_deg, gripper_pct, derived) where `derived` is the tuple of
    names that were computed rather than given -- callers log it, so a run's
    own record says where its limits came from.
    """
    if not fps or fps <= 0:
        raise ValueError(f"fps must be positive, got {fps!r}")
    derived = []
    if max_step_deg is None:
        max_step_deg = MAX_JOINT_DEG_PER_S / fps
        derived.append("max_step_deg")
    if max_step_gripper_pct is None:
        max_step_gripper_pct = MAX_GRIPPER_PCT_PER_S / fps
        derived.append("max_step_gripper_pct")
    return max_step_deg, max_step_gripper_pct, tuple(derived)


def _limit_for(limits, joint):
    """`limits` may be a scalar (same for every joint) or a per-joint dict.

    Per-joint is the correct form: the five body joints are in DEGREES and the
    gripper is in PERCENT of travel (see arm/units.py), so one number cannot be
    right for both.
    """
    if limits is None:
        return None
    if isinstance(limits, dict):
        return limits.get(joint)
    return limits


def rate_limit(target, prev_cmd, limits):
    """Clamp each joint of `target` to within its limit of `prev_cmd`.

    Returns (clamped, n_clamped). `prev_cmd` None -> first step, pass through.
    """
    if prev_cmd is None:
        return dict(target), 0
    out = {}
    n = 0
    for k, v in target.items():
        lim = _limit_for(limits, k)
        p = prev_cmd.get(k)
        if p is None or lim is None or lim <= 0:
            out[k] = v
            continue
        d = v - p
        if d > lim:
            out[k] = p + lim
            n += 1
        elif d < -lim:
            out[k] = p - lim
            n += 1
        else:
            out[k] = v
    return out, n


def pose_diff(a, b):
    """Per-joint absolute difference between two joint dicts.

    Returns (dict of abs diffs, worst_joint_name, worst_value).
    Missing joints on either side are skipped.
    """
    diffs = {k: abs(a[k] - b[k]) for k in a if k in b}
    if not diffs:
        return {}, None, 0.0
    worst = max(diffs, key=diffs.get)
    return diffs, worst, diffs[worst]


class TrackingWatchdog:
    """Fault when |command - measured| exceeds a joint's tolerance for
    `n_strikes` consecutive steps.

    `tol` is per-joint (dict) or scalar. Per-joint matters: 25 degrees and
    25 percent-of-gripper-travel are not the same amount of wrong.
    """

    def __init__(self, tol=25.0, n_strikes=15):
        self.tol = tol
        self.n_strikes = n_strikes
        self.strikes = 0
        self.worst_joint = None
        self.worst_value = 0.0
        self.worst_tol = None

    def in_tolerance(self, commanded, measured):
        """Is every joint within ITS OWN tolerance right now?

        Callers need this to decide when to ARM the watchdog -- the follower is
        legitimately behind while it walks to the leader, and a fault during
        that catch-up means nothing. Doing it by hand as
        `worst_gap <= max(tolerances)` looks equivalent and is not: the worst
        gap is over all joints, and max() of the tolerances is the GRIPPER's,
        which is a percentage. That compares five joints' degrees against a
        percentage and arms far too late (or, with a small gripper tolerance,
        far too early). Same unit trap as everywhere else in this file.
        """
        for k, c in commanded.items():
            m = measured.get(k)
            lim = _limit_for(self.tol, k)
            if m is None or lim is None:
                continue
            if abs(c - m) > lim:
                return False
        return True

    def update(self, commanded, measured):
        """Returns True when the watchdog trips."""
        over_joint, over_val, over_tol = None, 0.0, None
        for k, c in commanded.items():
            m = measured.get(k)
            lim = _limit_for(self.tol, k)
            if m is None or lim is None:
                continue
            d = abs(c - m)
            # rank by how far past its own tolerance the joint is
            if d > lim and (over_tol is None or d / lim > over_val / over_tol):
                over_joint, over_val, over_tol = k, d, lim
        if over_joint is not None:
            self.strikes += 1
            self.worst_joint = over_joint
            self.worst_value = over_val
            self.worst_tol = over_tol
        else:
            self.strikes = 0
        return self.strikes >= self.n_strikes

    def reason(self):
        from .units import unit_of
        u = unit_of(self.worst_joint) if self.worst_joint else "?"
        return (f"tracking error {self.worst_value:.1f} {u} on "
                f"{self.worst_joint} for {self.strikes} consecutive steps "
                f"(tol {self.worst_tol} {u})")
