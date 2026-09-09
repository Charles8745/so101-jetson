"""What the six numbers actually MEAN. Read this before touching any of them.

lerobot does NOT report all six joints in the same unit, and it is not a config
choice -- look at SOFollower.__init__:

    "shoulder_pan" ... "wrist_roll" : Motor(..., norm_mode_body)   # follows use_degrees
    "gripper"                       : Motor(6, "sts3215", RANGE_0_100)   # HARD-CODED

So with use_degrees=True you get:

  BODY (5 joints)  degrees, from MotorNormMode.DEGREES:
        deg = (present_ticks - (range_min + range_max)/2) * 360 / 4095
      * zero is the MIDPOINT OF THE CALIBRATED TRAVEL of that particular arm --
        a mechanical reference that depends on how this arm was assembled and
        calibrated. It is NOT a URDF zero and NOT the servo's 2048.
      * `present_ticks` already has Homing_Offset subtracted BY THE SERVO
        (lerobot writes it into the servo EEPROM at calibration time).
      * NOT clamped to the calibrated range (the other norm modes are), and
        drive_mode is ignored in this mode (ours are all 0, so it does not bite).

  GRIPPER  percent of calibrated travel, from MotorNormMode.RANGE_0_100:
        pct = (ticks - range_min) / (range_max - range_min) * 100
      * 0 = the closed end of the recorded travel, 100 = the open end.
      * clamped, and drive_mode DOES invert it.

Consequences: a single "max step in degrees" is meaningless for the gripper, and
labelling a record `"units": "deg"` for all six is simply false. Anything that
maps these onto another system (Isaac, a policy, a dataset) must branch here.
"""

BODY_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll")
GRIPPER = "gripper"
JOINTS = BODY_JOINTS + (GRIPPER,)

UNIT_DEG = "deg"
UNIT_PCT = "percent_0_100"

UNITS = {j: UNIT_DEG for j in BODY_JOINTS}
UNITS[GRIPPER] = UNIT_PCT

# Feetech STS3215: 4096 counts, and lerobot divides by (resolution - 1).
TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / (TICKS_PER_REV - 1)


def unit_of(joint):
    return UNITS.get(joint, UNIT_DEG)


def is_gripper(joint):
    return joint == GRIPPER


def per_joint(body_value, gripper_value):
    """Build a per-joint limit dict with the right unit for each joint."""
    d = {j: body_value for j in BODY_JOINTS}
    d[GRIPPER] = gripper_value
    return d


def deg_range_from_calibration(cal_entry):
    """The travel of a BODY joint in degrees, from its calibration entry.

    cal_entry: dict with range_min / range_max (raw ticks).
    Returns (lo_deg, hi_deg) about the same zero lerobot reports.
    """
    lo, hi = cal_entry["range_min"], cal_entry["range_max"]
    mid = (lo + hi) / 2.0
    return ((lo - mid) * DEG_PER_TICK, (hi - mid) * DEG_PER_TICK)
