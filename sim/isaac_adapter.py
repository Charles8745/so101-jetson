"""Isaac backend for sim/receiver.py. THE ONLY VERSION-SPECIFIC FILE IN THE REPO.

Written against **Isaac Sim 6.0.0-rc.22** (source build, linux-aarch64), Isaac
Lab 2.3.2, Kit python 3.12.12 -- the exact build on Spark. Every API name and
signature below was read off that machine by `sim/probe_isaac.py`, not recalled:

    isaacsim.core.prims.Articulation(prim_paths_expr, name=...)
        .set_joint_position_targets(positions, indices=, joint_indices=, joint_names=)
        .get_joint_positions(indices=, joint_indices=, joint_names=, clone=)
        .get_dof_limits() -> array
        .get_dof_index(dof_name) -> int
        .dof_names / .joint_names / .num_dof
    isaacsim.core.api.World(physics_dt=, rendering_dt=, stage_units_in_meters=,
                            backend=, device=, ...)
    isaacsim.core.utils.stage.add_reference_to_stage(usd_path, prim_path, prim_type=)

`omni.isaac.*` does not exist in 6.0 at all (ModuleNotFoundError, checked), so
the old layout is gone rather than deprecated.

** Articulation, not SingleArticulation. ** Two reasons, both measured:
  * `SingleArticulation` has NO `set_joint_position_targets`. Its only route to
    a drive target is `apply_action(ArticulationAction(joint_positions=...))`,
    which goes through the articulation controller. `Articulation` writes the
    target directly.
  * `SingleArticulation` has no `get_dof_limits`, so it cannot report clipping.

** set_joint_positions is NOT an acceptable fallback. ** It TELEPORTS the joint:
no drive, no force, no inertia, no tracking error. A dataset recorded that way
looks perfect and is physically meaningless, because the action and the motion
are unrelated. A Feetech STS3215 is a position servo -- you write a target and
its loop drives there -- so the sim must be driven the same way or an action
recorded in sim means something different from the same action on hardware.
If the target methods ever go missing, this module refuses to start.

** joint_names, not joint_indices. ** Both accessors take an explicit
`joint_names` list, so we never compute an index for reading or writing and a
whole class of ordering bugs cannot occur. Indices are used only for
`get_dof_limits()`, which returns one row per DOF.
"""
import math

from .backend import SimBackend

WRITE_METHOD = "set_joint_position_targets"
READ_METHOD = "get_joint_positions"


def _probe_api():
    """Every import that differs between Isaac versions lives here.

    Isaac Sim 6.0 layout. If a future version moves these again, this is the
    only function to edit -- and `sim/probe_isaac.py` tells you what to put in it.
    """
    tried = []
    try:
        from isaacsim import SimulationApp
    except Exception as e:
        tried.append(f"isaacsim.SimulationApp: {type(e).__name__}: {e}")
        try:
            from isaacsim.simulation_app import SimulationApp
        except Exception as e2:
            tried.append(f"isaacsim.simulation_app: {type(e2).__name__}: {e2}")
            raise RuntimeError(
                "no Isaac Sim entry point found. Tried:\n  " + "\n  ".join(tried) +
                "\n\nRun sim/probe_isaac.py and send its output -- this is the "
                "one place in the repo that has to match your Isaac version.")
    return {
        "label": "isaacsim 6.x",
        "SimulationApp": SimulationApp,
        # These must be resolved AFTER SimulationApp exists: Kit puts the
        # extension modules on sys.path when it boots, not before. Before that,
        # `isaacsim` is a bare namespace package and every submodule below
        # raises ModuleNotFoundError (measured -- it is what made the first
        # probe on Spark come back empty).
        "world": lambda **kw: __import__(
            "isaacsim.core.api", fromlist=["World"]).World(**kw),
        "add_ref": lambda usd, prim: __import__(
            "isaacsim.core.utils.stage", fromlist=["add_reference_to_stage"]
        ).add_reference_to_stage(usd_path=usd, prim_path=prim),
        "articulation": lambda prim: __import__(
            "isaacsim.core.prims", fromlist=["Articulation"]
        ).Articulation(prim_paths_expr=prim, name="so101"),
    }


def _require(obj, name, why):
    if not hasattr(obj, name):
        have = sorted(a for a in dir(obj)
                      if "joint" in a.lower() or "dof" in a.lower())
        raise RuntimeError(
            f"the articulation object has no {name!r}, which is needed to "
            f"{why}.\nIt does have: {have}\n"
            f"Isaac's API has moved. Run sim/probe_isaac.py, then fix "
            f"sim/isaac_adapter.py -- do NOT substitute set_joint_positions, "
            f"which teleports the joint instead of driving it.")
    return getattr(obj, name)


class IsaacBackend(SimBackend):
    name = "isaac"
    readback = True          # we return MEASURED joint positions, not our input

    def __init__(self, usd, dof_names, prim_path="/World/so101",
                 headless=False, fps=30.0, physics_dt=None, render_every=1):
        """`physics_dt` defaults to 1/fps, and fps is the RECEIVER's tick rate.

        Not a style choice: the receiver calls step() once per tick, so the
        simulated clock advances by one physics_dt per tick. Leave physics_dt at
        an unrelated value -- 1/60 against a 30 Hz receiver -- and simulated time
        runs at half wall-clock: `sim_time` wrong by 2x in every ack and every
        row, and the virtual arm responding at half the speed of the operator's
        hand, with nothing reporting it. step() checks the two against real time.
        """
        if not usd:
            raise RuntimeError("no USD given: pass --usd, or put it in the "
                               "map's sim_target.usd")
        if not dof_names:
            raise RuntimeError(
                "no dof_names in the map. The receiver has to know which USD "
                "joint is which SO-101 joint. sim/probe_isaac.py --joints "
                "writes that into the limits file; simmap_init.py carries it "
                "into the map. (For the stock SO-101 URDF the two sets of names "
                "are identical, but that is a fact to check, not to assume.)")
        if physics_dt is None:
            physics_dt = 1.0 / float(fps)
        self.api = _probe_api()
        print(f"[isaac] API: {self.api['label']}  physics_dt={physics_dt:.5f}s "
              f"({1.0 / physics_dt:.1f} Hz, matching the receiver's tick)")

        self.app = self.api["SimulationApp"]({"headless": bool(headless)})
        self.world = self.api["world"](physics_dt=physics_dt,
                                       rendering_dt=physics_dt * render_every)
        self.api["add_ref"](usd, prim_path)
        self.art = self.api["articulation"](prim_path)
        self.world.scene.add(self.art)
        self.world.reset()

        self._write = _require(self.art, WRITE_METHOD,
                               "command a joint POSITION TARGET")
        self._read = _require(self.art, READ_METHOD,
                              "read the joint positions back")

        sim_names = list(getattr(self.art, "dof_names", None)
                         or getattr(self.art, "joint_names"))
        self.dof_names = dict(dof_names)
        self.order = []
        missing = []
        for ours, theirs in self.dof_names.items():
            if theirs in sim_names:
                self.order.append((ours, theirs))
            else:
                missing.append((ours, theirs))
        if missing:
            raise RuntimeError(
                f"these joints are not in the articulation: {missing}\n"
                f"the model has: {sim_names}\n"
                f"Fix dof_names in the limits file and refit the map. Do NOT "
                f"run with a partial mapping -- the missing joint would simply "
                f"never move and nothing would say so.")
        self.our_names = [o for o, _ in self.order]
        self.sim_names = [t for _, t in self.order]
        print("[isaac] bound " + ", ".join(f"{o}->{t}" for o, t in self.order))

        self.limits = self._read_limits()
        if self.limits is None:
            print("[isaac] NOTE: could not read the model's joint limits, so "
                  "this backend cannot report clipping. The Jetson-side map "
                  "still clips against the same limits (clipped_local).")
        self._t = 0.0
        self._dt = physics_dt
        self._t0_wall = None
        self._drift_warned = False

    # ------------------------------------------------------------------
    def _read_limits(self):
        """{our_joint: (lo_rad, hi_rad)} from the model, or None.

        get_dof_limits() returns one row per DOF, so this is the one place an
        index is still needed; get_dof_index() maps a name to it.
        """
        try:
            lim = self.art.get_dof_limits()
            idx = self.art.get_dof_index
            out = {}
            for ours, theirs in self.order:
                row = lim[idx(theirs)]
                while hasattr(row, "__len__") and len(row) and \
                        hasattr(row[0], "__len__"):
                    row = row[0]            # peel a leading batch dimension
                out[ours] = (float(row[0]), float(row[1]))
            return out
        except Exception as e:
            print(f"[isaac] joint limits unavailable: {type(e).__name__}: {e}")
            return None

    def joint_names(self):
        return list(self.our_names)

    def apply(self, joints_rad):
        """Write position targets. What the ack reports comes from read() AFTER
        the step -- see sim/receiver.py."""
        import numpy as np
        names, vals, clipped = [], [], []
        for ours, theirs in self.order:
            if ours not in joints_rad:
                continue
            v = float(joints_rad[ours])
            if self.limits and ours in self.limits:
                lo, hi = self.limits[ours]
                if v < lo:
                    v, _ = lo, clipped.append(ours)
                elif v > hi:
                    v, _ = hi, clipped.append(ours)
            names.append(theirs)
            vals.append(v)
        if not names:
            return {}, []
        # Articulation is a VIEW over N prims, so positions is (N, K); ours
        # matches exactly one prim, hence the leading 1.
        arr = np.asarray([vals], dtype=np.float32)
        self._write(arr, joint_names=names)
        return {o: v for (o, _), v in zip(
            [x for x in self.order if x[0] in joints_rad], vals)}, clipped

    def step(self):
        import time as _time
        self.world.step(render=True)
        self._t += self._dt
        if self._t0_wall is None:
            self._t0_wall = _time.monotonic()
        elif not self._drift_warned and self._t > 10.0:
            wall = _time.monotonic() - self._t0_wall
            if wall > 0 and abs(self._t / wall - 1.0) > 0.10:
                self._drift_warned = True
                print(f"[isaac] WARNING: simulated time is running at "
                      f"{self._t / wall:.2f}x wall clock ({self._t:.1f}s sim in "
                      f"{wall:.1f}s real). sim_time in the logs is not real "
                      f"time -- check --fps against physics_dt.")

    def read(self):
        pos = self._read(joint_names=self.sim_names)
        try:
            pos = pos.numpy()
        except Exception:
            pass
        row = pos
        while hasattr(row, "__len__") and len(row) and hasattr(row[0], "__len__"):
            row = row[0]                    # peel the leading batch dimension
        return {o: float(row[i]) for i, o in enumerate(self.our_names)}

    def sim_time(self):
        return self._t

    def close(self):
        try:
            self.app.close()
        except Exception:
            pass
