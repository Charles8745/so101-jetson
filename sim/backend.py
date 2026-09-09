"""What the receiver needs from a simulator, and nothing else.

The receiver (sim/receiver.py) contains no Isaac code at all. Everything
Isaac-specific lives behind this five-method interface, for two reasons:

  * Isaac's Python API has moved twice (omni.isaac.core -> isaacsim.core.api,
    omni.isaac.lab -> isaaclab). Version churn must not be able to reach the
    networking, the episode state machine or the logging.
  * ** EchoBackend lets the ENTIRE pipeline be tested without Isaac, without
    Spark, and without a network. ** Run the receiver with --backend echo on
    localhost and p3 talks to it exactly as it would to the real thing. If
    something breaks with Isaac attached but works against echo, the fault is
    in the Isaac adapter and nowhere else. That is the whole point of having
    this seam.

Contract
--------
apply(joints_rad) -> (targets_written, clipped)
    `clipped` names joints the simulator refused to place where we asked.
step()
    Advance the simulator. A position target only takes effect here.
read() -> measured joints, or None
    MEASURED state, AFTER the step. The receiver calls apply -> step -> read
    and puts read()'s answer in the ack, so what the Jetson checks against is
    what the simulator actually did -- not our own input echoed back. A backend
    that cannot measure returns None and sets `readback = False`, and the log
    records that rather than implying a verification that never happened.
"""


class SimBackend:
    name = "abstract"
    readback = False        # does apply() return MEASURED state, or our input?

    def joint_names(self):
        """SO-101 joint names this backend can drive."""
        raise NotImplementedError

    def apply(self, joints_rad):
        """Set joint targets. Returns (applied_rad dict, clipped list)."""
        raise NotImplementedError

    def step(self):
        """Advance the simulator one tick. May be a no-op."""

    def read(self):
        """Measured joint positions in radians after the step, or None."""
        return None

    def sim_time(self):
        """Simulator clock in seconds, or None if the backend has none."""
        return None

    def on_episode(self, action, episode, meta=None):
        """'start' / 'end' / 'discard'. Returns (ok, detail).

        A backend that records images or sim state does it here. The default
        accepts and does nothing, so a backend with no recorder is still valid.
        """
        return True, ""

    def close(self):
        pass


class EchoBackend(SimBackend):
    """No simulator. Applies targets exactly, optionally with fake limits.

    Deliberately honest about what it is: `readback = False`, because it returns
    our own numbers back. It can therefore prove the LINK works and cannot prove
    the MAPPING works -- do not let a green echo run be mistaken for a verified
    map.
    """
    name = "echo"
    readback = False

    def __init__(self, joints=None, limits_rad=None, hold_s=0.0):
        from arm.units import JOINTS
        self._joints = tuple(joints or JOINTS)
        self._limits = dict(limits_rad or {})
        self._hold_s = float(hold_s)     # fake per-step work, to test supersede
        self._state = {j: 0.0 for j in self._joints}
        self._t = 0.0
        self.episodes = []

    def joint_names(self):
        return list(self._joints)

    def apply(self, joints_rad):
        import time
        if self._hold_s > 0:
            time.sleep(self._hold_s)
        applied, clipped = {}, []
        for j, v in joints_rad.items():
            if j not in self._joints:
                continue
            lim = self._limits.get(j)
            if lim is not None:
                lo, hi = sorted(lim)
                if v < lo:
                    v, was = lo, True
                elif v > hi:
                    v, was = hi, True
                else:
                    was = False
                if was:
                    clipped.append(j)
            applied[j] = float(v)
            self._state[j] = float(v)
        return applied, clipped

    def step(self):
        self._t += 1.0 / 30.0

    def sim_time(self):
        return self._t

    def on_episode(self, action, episode, meta=None):
        self.episodes.append((action, episode))
        return True, f"echo recorded {action} for episode {episode}"


def make_backend(kind, **kw):
    if kind == "echo":
        return EchoBackend(**kw)
    if kind == "isaac":
        from .isaac_adapter import IsaacBackend
        return IsaacBackend(**kw)
    raise ValueError(f"unknown backend {kind!r} (have: echo, isaac)")
