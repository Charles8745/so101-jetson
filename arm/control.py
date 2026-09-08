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


def rate_limit(target, prev_cmd, max_step_deg):
    """Clamp each joint of `target` to within max_step_deg of `prev_cmd`.

    Returns (clamped, n_clamped). `prev_cmd` None -> first step, pass through.
    """
    if prev_cmd is None or max_step_deg is None or max_step_deg <= 0:
        return dict(target), 0
    out = {}
    n = 0
    for k, v in target.items():
        p = prev_cmd.get(k)
        if p is None:
            out[k] = v
            continue
        d = v - p
        if d > max_step_deg:
            out[k] = p + max_step_deg
            n += 1
        elif d < -max_step_deg:
            out[k] = p - max_step_deg
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
    """Fault when |command - measured| stays above `tol_deg` for `n_strikes`
    consecutive steps on any joint."""

    def __init__(self, tol_deg=25.0, n_strikes=15):
        self.tol_deg = tol_deg
        self.n_strikes = n_strikes
        self.strikes = 0
        self.worst_joint = None
        self.worst_value = 0.0

    def update(self, commanded, measured):
        """Returns True when the watchdog trips."""
        _, worst, val = pose_diff(commanded, measured)
        if val > self.tol_deg:
            self.strikes += 1
            self.worst_joint, self.worst_value = worst, val
        else:
            self.strikes = 0
        return self.strikes >= self.n_strikes

    def reason(self):
        return (f"tracking error {self.worst_value:.1f} deg on "
                f"{self.worst_joint} for {self.strikes} consecutive steps "
                f"(tol {self.tol_deg} deg)")
