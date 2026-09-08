"""Pre-flight self-check for the SO-101 pair.

Answers one question before any motor is commanded: **is the thing plugged into
this port actually the arm this program thinks it is?**

The expensive failure this prevents is not a crash -- it is swapping leader and
follower. That produces no error at all: the program runs, the arms move, and
every recorded episode has its fields the wrong way round.

Checks, in order (cheap and non-moving first):

  [1] both device paths exist and are readable
  [2] the two ports are not the same physical device
  [3] each USB serial matches the one registered in devices.env  <-- anti-swap
  [4] calibration file present with 6 motors, for both arms
  [5] both connect with calibrate=False, and report is_calibrated
  [6] all six motors answer a read (id 1..6 present)
  [7] leader and follower poses agree to within a tolerance   <-- anti-snap
  [8] read-rate probe: measured Hz and failure count

Nothing here commands a motor. [5] powers the bus but sends no Goal_Position.
"""
import glob
import os
import re
import time

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")

_SERIAL_RE = re.compile(r"_([A-Za-z0-9]+)-if\d+")


def parse_serial_from_byid(name):
    """'usb-1a86_USB_Single_Serial_5B79050417-if00' -> '5B79050417'."""
    m = _SERIAL_RE.search(os.path.basename(name))
    return m.group(1) if m else None


def serial_of_port(port, byid_dir="/dev/serial/by-id"):
    """USB serial for a device path, whether given as by-id or as /dev/ttyACMn."""
    s = parse_serial_from_byid(port)
    if s:
        return s
    try:
        target = os.path.realpath(port)
    except OSError:
        return None
    for link in glob.glob(os.path.join(byid_dir, "*")):
        try:
            if os.path.realpath(link) == target:
                return parse_serial_from_byid(link)
        except OSError:
            continue
    return None


def strip_pos(d):
    """lerobot returns 'shoulder_pan.pos'; we want 'shoulder_pan'."""
    return {(k[:-4] if k.endswith(".pos") else k): float(v)
            for k, v in d.items()}


class CheckResult:
    def __init__(self):
        self.rows = []
        self.ok = True

    def add(self, name, ok, detail="", fatal=True):
        self.rows.append({"check": name, "ok": bool(ok), "detail": str(detail)})
        if not ok and fatal:
            self.ok = False
        return ok

    def render(self):
        out = []
        for r in self.rows:
            out.append(f"  [{'PASS' if r['ok'] else 'FAIL'}] {r['check']}"
                       + (f"  -- {r['detail']}" if r["detail"] else ""))
        return "\n".join(out)


def run_preflight(leader, follower, leader_port, follower_port,
                  expect_leader_serial=None, expect_follower_serial=None,
                  max_pose_diff_deg=15.0, probe_reads=30, force=False):
    """`leader` / `follower` are already-constructed (not connected) lerobot objects.

    Returns (CheckResult, info dict). Connects both arms on success.
    """
    r = CheckResult()
    info = {}

    for label, port in (("leader", leader_port), ("follower", follower_port)):
        r.add(f"[1] {label} port exists", os.path.exists(port), port)

    same = os.path.realpath(leader_port) == os.path.realpath(follower_port)
    r.add("[2] ports are two different devices", not same,
          f"both resolve to {os.path.realpath(leader_port)}" if same else "")

    for label, port, expect in (("leader", leader_port, expect_leader_serial),
                                ("follower", follower_port, expect_follower_serial)):
        got = serial_of_port(port)
        info[f"{label}_serial"] = got
        if expect is None:
            r.add(f"[3] {label} serial", True,
                  f"{got} (not registered -- set {label.upper()}_SERIAL in devices.env)",
                  fatal=False)
        else:
            r.add(f"[3] {label} serial matches registry", got == expect,
                  f"expected {expect}, got {got}"
                  + ("  <-- ARMS MAY BE SWAPPED" if got and got != expect else ""))

    if not r.ok:
        return r, info

    for label, obj in (("leader", leader), ("follower", follower)):
        n = len(obj.calibration)
        r.add(f"[4] {label} calibration file", obj.calibration_fpath.is_file() and n == 6,
              f"{obj.calibration_fpath} ({n} motors)")
    if not r.ok:
        return r, info

    try:
        leader.connect(calibrate=False)
        follower.connect(calibrate=False)
    except Exception as e:
        r.add("[5] connect", False, f"{type(e).__name__}: {e}")
        return r, info
    r.add("[5] leader is_calibrated", leader.is_calibrated,
          "motor values disagree with the file -- NOT auto-recalibrating")
    r.add("[5] follower is_calibrated", follower.is_calibrated,
          "motor values disagree with the file -- NOT auto-recalibrating")
    if not r.ok:
        return r, info

    try:
        lead = strip_pos(leader.get_action())
        foll = strip_pos(follower.get_observation())
    except Exception as e:
        r.add("[6] read all motors", False, f"{type(e).__name__}: {e}")
        return r, info
    missing_l = [j for j in JOINTS if j not in lead]
    missing_f = [j for j in JOINTS if j not in foll]
    r.add("[6] leader reports 6 motors", not missing_l, f"missing {missing_l}")
    r.add("[6] follower reports 6 motors", not missing_f, f"missing {missing_f}")
    if not r.ok:
        return r, info

    from .control import pose_diff
    diffs, worst, val = pose_diff(lead, foll)
    info["pose_diff"] = diffs
    aligned = val <= max_pose_diff_deg
    detail = (f"worst {worst} {val:.1f} deg (limit {max_pose_diff_deg})"
              + ("" if aligned else
                 "  <-- move the LEADER to match the FOLLOWER, or pass --force"))
    r.add("[7] leader/follower poses aligned", aligned or force, detail,
          fatal=not force)
    if force and not aligned:
        r.rows[-1]["detail"] += "  [FORCED]"

    t0 = time.monotonic()
    fails = 0
    for _ in range(probe_reads):
        try:
            leader.get_action()
            follower.get_observation()
        except Exception:
            fails += 1
    dt = time.monotonic() - t0
    hz = probe_reads / dt if dt > 0 else 0.0
    info["probe_hz"] = hz
    info["probe_fails"] = fails
    r.add("[8] read-rate probe", fails == 0,
          f"{hz:.1f} Hz over {probe_reads} paired reads, {fails} failures",
          fatal=False)

    return r, info
