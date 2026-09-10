#!/usr/bin/env python3
"""Dependency-free self-tests: no OpenCV, no lerobot, no hardware.

Covers every piece of logic that does not touch a device, so a broken refactor
is caught on any machine before it reaches the arm. Hardware behaviour is not
covered here and never claimed to be.
"""
import json
import os
import pathlib
import sys
import tempfile
import time
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def test_clock():
    from common.clock import epoch, stamp
    s = stamp()
    assert set(s) == {"t_mono", "t_unix"}, s
    assert isinstance(s["t_mono"], float) and isinstance(s["t_unix"], float)
    assert set(epoch()) == {"t_mono", "t_unix"}
    print("ok  clock")


def test_signal_jsonl():
    from arm.signal_pub import SCHEMA, SignalPublisher
    p = tempfile.mktemp(suffix=".jsonl")
    pub = SignalPublisher(udp_addr=None, jsonl_path=p)
    pub.publish({"schema": SCHEMA, "seq": 0, "leader": {"shoulder_pan": 1.0}})
    pub.publish({"schema": SCHEMA, "seq": 1, "leader": {"shoulder_pan": 2.0}})
    pub.close()
    lines = open(p).read().strip().splitlines()
    assert len(lines) == 2, lines
    assert json.loads(lines[0])["leader"]["shoulder_pan"] == 1.0
    print("ok  signal jsonl")


def test_signal_udp():
    import socket
    from arm.signal_pub import SignalPublisher
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    port = rx.getsockname()[1]
    pub = SignalPublisher(udp_addr=f"127.0.0.1:{port}")
    pub.publish({"seq": 7})
    rx.settimeout(1.0)
    data, _ = rx.recvfrom(4096)
    pub.close()
    rx.close()
    assert json.loads(data)["seq"] == 7
    print("ok  signal udp")


def test_jsonl_writer():
    from common.jsonl import JsonlWriter
    p = os.path.join(tempfile.mkdtemp(), "sub", "x.jsonl")
    w = JsonlWriter(p)
    w.write({"a": 1})
    w.event("start", note="hi")
    w.close()
    lines = [json.loads(x) for x in open(p).read().strip().splitlines()]
    assert lines[0]["a"] == 1
    assert lines[1]["event"] == "start" and lines[1]["note"] == "hi"
    assert "t_mono" in lines[1]
    print("ok  jsonl writer")


def test_rate_limit():
    from arm.control import rate_limit
    prev = {"a": 0.0, "b": 0.0}
    out, n = rate_limit({"a": 100.0, "b": 1.0}, prev, 8.0)
    assert out["a"] == 8.0 and out["b"] == 1.0 and n == 1, (out, n)
    out, n = rate_limit({"a": -100.0}, {"a": 0.0}, 8.0)
    assert out["a"] == -8.0 and n == 1
    out, n = rate_limit({"a": 999.0}, None, 8.0)
    assert out["a"] == 999.0 and n == 0, "first step passes through"
    out, n = rate_limit({"a": 999.0}, prev, 0)
    assert out["a"] == 999.0 and n == 0, "0 disables the limiter"
    print("ok  rate_limit")


class _stub_imaging:
    """Import our camera/program modules with no OpenCV or numpy installed.

    resilient_camera imports cv2 at module level and p2 imports numpy too, so
    touching either needs *something* under those names. Everything else in
    this file runs on a bare python and that is the point of it: `so101
    selftest` has to be able to say "the code is fine, go and look at the
    hardware" on a machine where the install itself is what went wrong.

    Pass any of our own modules that must be re-imported inside the block --
    `from x import y` hands back a cached module still bound to the real cv2,
    and the stub would be silently ignored.
    """

    def __init__(self, *evict):
        self._evict = ("camera.resilient_camera",) + evict

    def __enter__(self):
        # Ask BEFORE evicting anything. The real OpenCV cannot survive being
        # dropped from sys.modules and re-imported: its __init__ pulls in
        # cv2.typing and cv2.mat_wrapper, which are still cached and refer back
        # to the half-built parent, so the retry dies on a circular import.
        # Whatever is really installed is therefore left completely alone.
        self._saved = {}
        for name, build in (("cv2", self._fake_cv2), ("numpy", self._fake_numpy)):
            try:
                __import__(name)
            except ImportError:
                self._saved[name] = sys.modules.get(name)
                sys.modules[name] = build()
        for name in self._evict:
            self._saved[name] = sys.modules.get(name)
            sys.modules.pop(name, None)
        return self

    @staticmethod
    def _fake_cv2():
        fake = types.ModuleType("cv2")
        for i, name in enumerate((
                "CAP_V4L2", "CAP_PROP_FOURCC", "CAP_PROP_FRAME_WIDTH",
                "CAP_PROP_FRAME_HEIGHT", "CAP_PROP_FPS", "CAP_PROP_AUTO_EXPOSURE",
                "ROTATE_90_CLOCKWISE", "ROTATE_180", "ROTATE_90_COUNTERCLOCKWISE",
                "INTER_AREA", "FONT_HERSHEY_SIMPLEX", "WINDOW_NORMAL")):
            setattr(fake, name, i)
        fake.rotate = lambda f, code: f
        fake.VideoWriter_fourcc = lambda *a: 0
        fake.VideoWriter = lambda *a, **k: None
        fake.VideoCapture = lambda *a, **k: None
        fake.getBuildInformation = lambda: "GUI: NONE"
        return fake

    @staticmethod
    def _fake_numpy():
        fake = types.ModuleType("numpy")
        fake.uint8 = "uint8"
        fake.ndarray = type("ndarray", (), {})
        fake.zeros = lambda *a, **k: None
        fake.concatenate = lambda *a, **k: None
        return fake

    def __exit__(self, *exc):
        for k, v in self._saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
        return False


def test_camera_rotation_is_applied_at_the_source():
    """A camera mounted upside down is corrected in the capture thread, so the
    video file and the live view get the same picture. Correcting it only at
    display leaves the RECORDING upside down, which nobody notices until a
    policy is trained on it. 90/270 also swap the frame size, so the writer has
    to be sized for the output, not for the sensor mode."""
    with _stub_imaging():
        from camera.resilient_camera import ResilientCamera
        c0 = ResilientCamera("a", "/dev/null", 1024, 768, 30, rotate=0)
        c180 = ResilientCamera("b", "/dev/null", 1024, 768, 30, rotate=180)
        c90 = ResilientCamera("c", "/dev/null", 1024, 768, 30, rotate=90)
        assert c0.out_size() == (1024, 768)
        assert c180.out_size() == (1024, 768)
        assert c90.out_size() == (768, 1024), "90 deg swaps width and height"
        try:
            ResilientCamera("d", "/dev/null", 640, 480, 30, rotate=45)
            raise AssertionError("45 deg should be refused")
        except ValueError:
            pass

    # This next part checks OpenCV's own behaviour, not ours, so it is optional:
    # everything else in this file runs with no numpy and no OpenCV installed,
    # which is what lets `so101 selftest` separate "the code is broken" from
    # "the machine is broken". Losing that to a third-party sanity check would
    # be a bad trade.
    rotation_checked = False
    try:
        import cv2 as _cv
        import numpy as np
    except ImportError:
        pass
    else:  # noqa: E301
        f = np.zeros((4, 6, 3), dtype=np.uint8)
        f[0, 0] = (1, 2, 3)
        r = _cv.rotate(f, _cv.ROTATE_180)
        assert tuple(r[-1, -1]) == (1, 2, 3), "180 puts the corner opposite"
        rotation_checked = True

    src = open(os.path.join(ROOT, "camera", "resilient_camera.py")).read()
    assert "frame = cv2.rotate(frame, self._rot)" in src, \
        "rotation must happen in the capture loop, not at display"
    p2 = open(os.path.join(ROOT, "programs", "p2_record_cameras.py")).read()
    assert "out_size[name]" in p2, "the writer must be sized for the OUTPUT"
    print("ok  camera rotation is applied at the source, and resizes the writer"
          + ("" if rotation_checked else "  (pixel check skipped: no cv2/numpy)"))


def test_exposure_and_mains_frequency_are_independent():
    """Asking for anti-flicker must not lock the exposure as a side effect, and
    "auto" must be SET rather than assumed: V4L2 controls live on the device, so
    a previous locked run carries over into every later run silently."""
    import inspect
    from camera import controls as cc
    assert "power_line_hz" not in inspect.signature(cc.lock_exposure).parameters, \
        "lock_exposure must not also set the mains frequency"
    assert hasattr(cc, "set_power_line") and hasattr(cc, "unlock_exposure")
    src = open(os.path.join(ROOT, "programs", "p2_record_cameras.py")).read()
    assert "args.lock_exposure or args.power_line_hz is not None" not in src, \
        "p2 must not trigger the exposure lock from --power-line-hz"
    assert "camctl.unlock_exposure(dev)" in src, \
        "p2 must actively restore auto, not just decline to lock"

    calls = []
    cc_set = cc.set_ctrl
    cc_list = cc.list_controls
    cc.list_controls = lambda d: ({"auto_exposure": {"value": 1, "default": 3},
                                   "white_balance_automatic": {"value": 0,
                                                               "default": 1},
                                   "power_line_frequency": {"value": 0,
                                                            "default": 2}}, None)
    cc.set_ctrl = lambda d, n, v: (calls.append((n, v)) or (True, ""))
    try:
        cc.unlock_exposure("/dev/null")
        assert calls == [("auto_exposure", 3),
                         ("white_balance_automatic", 1)], calls
        calls.clear()
        cc.set_power_line("/dev/null", 60)
        assert calls == [("power_line_frequency", 2)], calls
    finally:
        cc.set_ctrl, cc.list_controls = cc_set, cc_list
    print("ok  exposure and mains frequency are separate, auto is set not assumed")


def test_watchdog_arming_must_not_mix_units():
    """Arming with `worst_gap <= max(tolerances)` looks equivalent to asking
    "is tracking good" and is not: max() of the per-joint tolerances is the
    GRIPPER's, which is a percentage, so five joints' degrees get compared
    against it."""
    from arm.control import TrackingWatchdog
    from arm.units import per_joint
    tol = per_joint(25.0, 30.0)          # 25 deg body, 30 percent gripper
    w = TrackingWatchdog(tol, 15)
    cmd = {"shoulder_pan": 0.0, "gripper": 0.0}

    meas = {"shoulder_pan": 28.0, "gripper": 0.0}     # 28 deg out, over its 25
    assert not w.in_tolerance(cmd, meas), "28 deg is over the 25 deg tolerance"
    worst = max(abs(cmd[k] - meas[k]) for k in cmd)
    assert worst <= max(tol.values()), \
        "...but the naive test would have called this in tolerance, because " \
        "it compares 28 degrees against the gripper's 30 percent"

    assert w.in_tolerance(cmd, {"shoulder_pan": 20.0, "gripper": 25.0})
    assert not w.in_tolerance(cmd, {"shoulder_pan": 0.0, "gripper": 31.0})
    for src in ("programs/p1_follow_leader.py", "programs/p2_record_cameras.py"):
        body = open(os.path.join(ROOT, src)).read()
        assert "in_tolerance(cmd, meas)" in body, src
        assert "max(args.track_tol_deg" not in body, src
    print("ok  watchdog arming asks each joint about its own tolerance")


def test_rate_limit_first_step_is_the_dangerous_one():
    """rate_limit() passes a joint straight through when it has no previous
    command for it. That makes the FIRST step unclamped -- and step one is
    exactly when the two arms are furthest apart. p1 therefore seeds prev_cmd
    with the follower's MEASURED pose so the first command is limited like
    every other one."""
    from arm.control import rate_limit
    from arm.units import per_joint
    lim = per_joint(8.0, 15.0)
    leader = {"shoulder_pan": 90.0, "gripper": 100.0}

    unseeded, n = rate_limit(leader, None, lim)
    assert unseeded["shoulder_pan"] == 90.0 and n == 0, \
        "unseeded, the follower is told to go the whole 90 degrees at once"

    follower_now = {"shoulder_pan": 0.0, "gripper": 0.0}
    seeded, n = rate_limit(leader, follower_now, lim)
    assert seeded["shoulder_pan"] == 8.0, seeded
    assert seeded["gripper"] == 15.0, seeded
    assert n == 2
    src = open(os.path.join(ROOT, "programs", "p1_follow_leader.py")).read()
    assert "prev_cmd = strip_pos(follower.get_observation())" in src, \
        "p1 must seed the rate limiter from the follower's real position"
    assert "watchdog_armed" in src, \
        "the watchdog must not fire while the follower is still catching up"
    print("ok  rate limiter is seeded, so step one is clamped too")


def test_pose_diff_and_watchdog():
    from arm.control import TrackingWatchdog, pose_diff
    _, worst, val = pose_diff({"a": 10.0, "b": 0.0}, {"a": 1.0, "b": 0.5})
    assert worst == "a" and abs(val - 9.0) < 1e-9
    w = TrackingWatchdog(tol=5.0, n_strikes=3)
    assert not w.update({"a": 0.0}, {"a": 0.0})
    assert not w.update({"a": 100.0}, {"a": 0.0})
    assert not w.update({"a": 100.0}, {"a": 0.0})
    assert w.update({"a": 100.0}, {"a": 0.0}), "trips on the 3rd strike"
    assert "tracking error" in w.reason()
    w2 = TrackingWatchdog(tol=5.0, n_strikes=3)
    w2.update({"a": 100.0}, {"a": 0.0})
    assert not w2.update({"a": 0.0}, {"a": 0.0}), "in-tolerance resets strikes"
    print("ok  pose_diff + tracking watchdog")


def test_units_are_not_all_degrees():
    """The gripper is percent, not degrees. lerobot hard-codes RANGE_0_100 for
    it whatever use_degrees says, so one scalar limit cannot serve both."""
    from arm.control import TrackingWatchdog, rate_limit
    from arm.units import (BODY_JOINTS, DEG_PER_TICK, GRIPPER, UNIT_DEG,
                           UNIT_PCT, deg_range_from_calibration, per_joint,
                           unit_of)
    assert unit_of("shoulder_pan") == UNIT_DEG
    assert unit_of(GRIPPER) == UNIT_PCT, "gripper must not be labelled degrees"
    assert len(BODY_JOINTS) == 5
    assert abs(DEG_PER_TICK - 360.0 / 4095) < 1e-12

    lim = per_joint(8.0, 15.0)
    out, n = rate_limit({"shoulder_pan": 100.0, GRIPPER: 100.0},
                        {"shoulder_pan": 0.0, GRIPPER: 0.0}, lim)
    assert out["shoulder_pan"] == 8.0 and out[GRIPPER] == 15.0, out
    assert n == 2

    # a gripper move inside its own percent tolerance must not trip a watchdog
    # whose body tolerance is smaller
    w = TrackingWatchdog(tol=per_joint(25.0, 30.0), n_strikes=1)
    assert not w.update({GRIPPER: 28.0}, {GRIPPER: 0.0}), "28 pct < 30 pct tol"
    assert w.update({"shoulder_pan": 28.0}, {"shoulder_pan": 0.0}), "28 deg > 25 deg tol"

    # the real follower calibration must reproduce the ranges we measured
    lo, hi = deg_range_from_calibration({"range_min": 851, "range_max": 3185})
    assert abs(hi - 102.6) < 0.1 and abs(lo + 102.6) < 0.1, (lo, hi)
    print("ok  unit semantics (gripper is percent, body is degrees)")


def test_serial_parsing():
    from arm.preflight import parse_serial_from_byid
    assert parse_serial_from_byid(
        "usb-1a86_USB_Single_Serial_5B79050417-if00") == "5B79050417"
    assert parse_serial_from_byid(
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79050450-if00") == "5B79050450"
    assert parse_serial_from_byid("/dev/ttyACM0") is None
    print("ok  serial parsing")


def test_best_lag():
    import math
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    from analyze_latency import best_lag
    a = [math.sin(i * 0.15) for i in range(400)]
    for true_lag in (0, 3, 11):
        b = [0.0] * true_lag + a[:len(a) - true_lag]
        k, r = best_lag(a, b, 30)
        assert k == true_lag, f"expected {true_lag}, got {k}"
        assert r > 0.99
    print("ok  cross-correlation lag")


def test_camera_bandwidth_budget():
    from camera.preflight import bandwidth_mbps, budget_report, mode_matches
    assert round(bandwidth_mbps(640, 480, 30, "YUYV")) == 147
    assert round(bandwidth_mbps(1024, 768, 30, "MJPG")) == 28
    cams = [("a", "/dev/null"), ("b", "/dev/null")]
    ok, unproven, total, _ = budget_report(cams, 1024, 768, 30, "MJPG")
    assert ok and not unproven, (ok, unproven, total)
    ok, unproven, _, _ = budget_report(cams, 640, 480, 30, "YUYV")
    assert ok and unproven, "294 Mbps is within spec but beyond what we measured"
    asked = {"width": 1024, "height": 768, "fps": 30.0, "fourcc": "MJPG"}
    assert mode_matches(asked, dict(asked))[0]
    same, why = mode_matches(asked, {**asked, "fps": 120.1})
    assert not same and "fps" in why, why
    print("ok  camera bandwidth budget + mode negotiation check")


def test_camera_controls():
    from camera.controls import parse_ctrls, resolve
    text = """
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=157 flags=inactive
"""
    c = parse_ctrls(text)
    assert set(c) == {"white_balance_automatic", "auto_exposure",
                      "exposure_time_absolute"}, sorted(c)
    assert c["auto_exposure"]["value"] == 3 and c["auto_exposure"]["type"] == "menu"
    assert c["exposure_time_absolute"]["flags"] == "inactive"
    assert resolve(c, "auto_exposure") == "auto_exposure"
    assert resolve(c, "auto_white_balance") == "white_balance_automatic"
    old = parse_ctrls("  exposure_auto 0x009a0901 (menu) : min=0 max=3 default=3 value=3")
    assert resolve(old, "auto_exposure") == "exposure_auto", "old kernel naming"
    assert resolve(c, "power_line_frequency") is None
    print("ok  v4l2 control parsing + alias resolution")


def test_resilient_reconnect():
    # This test swaps a fake cv2 into sys.modules and then imports
    # camera.resilient_camera. That only works if the module has NOT already
    # been imported -- otherwise `from ... import` hands back the cached module,
    # still bound to the real cv2, and the fake is silently ignored. It used to
    # rely on being the first test to touch it, which is not a property a test
    # should depend on. So: evict the module, install the fake, import fresh,
    # and put everything back afterwards.
    real_cv2 = sys.modules.get("cv2")
    cached = {k: v for k, v in sys.modules.items()
              if k == "camera.resilient_camera"}
    for k in cached:
        del sys.modules[k]

    fake = types.ModuleType("cv2")
    fake.CAP_V4L2 = 200
    fake.CAP_PROP_FOURCC = 6
    fake.CAP_PROP_FRAME_WIDTH = 3
    fake.CAP_PROP_FRAME_HEIGHT = 4
    fake.CAP_PROP_FPS = 5
    fake.ROTATE_90_CLOCKWISE = 0
    fake.ROTATE_180 = 1
    fake.ROTATE_90_COUNTERCLOCKWISE = 2
    fake.rotate = lambda f, code: f
    fake.VideoWriter_fourcc = lambda *a: 0
    st = {"reads": 0, "opened": 0}

    class FakeCap:
        def __init__(self, dev, backend=None):
            st["opened"] += 1

        def isOpened(self):
            return True

        def set(self, *a):
            return True

        def get(self, prop):
            return {3: 1024, 4: 768, 5: 30.0, 6: 1196444237}.get(prop, 0)

        def read(self):
            st["reads"] += 1
            if 3 < st["reads"] <= 40:      # device "gone", then it comes back
                return False, None
            return True, "FRAME"

        def release(self):
            pass

    fake.VideoCapture = FakeCap
    sys.modules["cv2"] = fake
    from camera.resilient_camera import ResilientCamera
    rc = ResilientCamera("t", "/dev/null", 1024, 768, 30,
                         reconnect_backoff=(0.01, 0.02), max_read_fail=5).start()
    deadline = time.time() + 3.0
    f = None
    while time.time() < deadline:
        f, _stale, state = rc.read_latest()
        if f == "FRAME" and state == "streaming" and st["opened"] >= 2:
            break
        time.sleep(0.05)
    rc.stop()
    assert st["opened"] >= 2, f"should have reopened; opened={st['opened']}"
    assert f == "FRAME", f"should have recovered a frame; got {f}"
    assert rc.stats["reconnects"] >= 1, rc.stats
    assert rc.stats["total_down_s"] > 0, rc.stats
    assert rc.stats["dropped"] > 0, rc.stats
    assert rc.stats["mode_changes"] == 0, "mode was stable; must not report a change"
    if real_cv2 is not None:
        sys.modules["cv2"] = real_cv2
    else:
        sys.modules.pop("cv2", None)
    sys.modules.pop("camera.resilient_camera", None)
    sys.modules.update(cached)
    assert rc.mode["width"] == 1024 and rc.mode["fourcc"] == "MJPG", rc.mode
    print(f"ok  resilient reconnect ({rc.stats['reconnects']} reconnects, "
          f"{rc.stats['total_down_s']:.2f}s down, mode {rc.mode['fourcc']} "
          f"{rc.mode['width']}x{rc.mode['height']})")



# ---------------------------------------------------------------- program 3
def _fake_calibration():
    ranges = {"shoulder_pan": (791, 3305), "shoulder_lift": (881, 3215),
              "elbow_flex": (943, 3153), "wrist_flex": (874, 3222),
              "wrist_roll": (1, 4095), "gripper": (2000, 3400)}
    return {j: {"id": i + 1, "drive_mode": 0, "homing_offset": 0,
                "range_min": lo, "range_max": hi}
            for i, (j, (lo, hi)) in enumerate(ranges.items())}


def _faithful_limits(cal, flip=()):
    """Model limits that exactly match the arm's travel, with a non-zero zero."""
    import math as _m
    from arm.units import BODY_JOINTS, deg_range_from_calibration
    out = {}
    for k, j in enumerate(BODY_JOINTS):
        lo_deg, hi_deg = deg_range_from_calibration(cal[j])
        span = _m.radians(hi_deg - lo_deg)
        a = -span / 2 + 0.1 * k
        b = a + span
        out[j] = [b, a] if j in flip else [a, b]
    return out


def test_simmap_fit_recovers_sign_and_scale():
    import math
    from arm.sim_mapping import build_map
    cal = _fake_calibration()
    from arm.sim_mapping import FIT_ENDPOINTS
    m = build_map(cal, _faithful_limits(cal, flip=("elbow_flex",)),
                  (0.0, 0.6109), fit_mode=FIT_ENDPOINTS,
                  source_arm={"role": "leader", "id": "x"})
    for j, spec in m.joints.items():
        if spec["mode"] != "affine":
            continue
        # a faithful model must fit at exactly one degree per degree
        assert abs(abs(spec["scale"]) - math.pi / 180) < 1e-9, (j, spec["scale"])
        assert abs(spec["span_ratio"] - 1.0) < 1e-9, (j, spec["span_ratio"])
    assert m.joints["elbow_flex"]["scale"] < 0, "a backwards model axis must " \
        "come out as a NEGATIVE scale, not as a silently wrong mapping"
    assert m.joints["shoulder_pan"]["scale"] > 0
    # the gripper is percent, not degrees, and must map by range
    assert m.joints["gripper"]["mode"] == "range"
    q, clipped = m.apply_joint("gripper", 50.0)
    assert abs(q - 0.30545) < 1e-4 and not clipped, q
    print("ok  simmap fit (sign recovered, scale exact, gripper by range)")


def test_identity_fit_beats_endpoint_fit_when_spans_disagree():
    """Measured against the real SO-101 URDF, this is not hypothetical.

    The arm's wrist_flex sweeps 206.4 deg; the URDF declares 190. An endpoint
    fit spreads that 8.6% across every angle in between -- at mid-travel it is
    4 degrees wrong, reports no clipping, and 8.6% sits UNDER the refusal
    threshold, so nothing stops it. Identity maps degree for degree and clips
    the part the model genuinely cannot reach, saying so every step.
    """
    import math
    from arm.sim_mapping import FIT_ENDPOINTS, FIT_IDENTITY, build_map
    cal = _fake_calibration()
    urdf = {"shoulder_pan": (-1.91986, 1.91986), "shoulder_lift": (-1.74533, 1.74533),
            "elbow_flex": (-1.69, 1.69), "wrist_flex": (-1.65806, 1.65806),
            "wrist_roll": (-2.74385, 2.84121)}
    arm = {"role": "leader", "id": "x"}
    ident = build_map(cal, urdf, (-0.174533, 1.74533), source_arm=arm,
                      fit_mode=FIT_IDENTITY)
    ends = build_map(cal, urdf, (-0.174533, 1.74533), source_arm=arm,
                     fit_mode=FIT_ENDPOINTS)

    for j, spec in ident.joints.items():
        if spec["mode"] == "affine":
            assert abs(abs(spec["scale"]) - math.pi / 180) < 1e-12, j

    q, clipped = ident.apply_joint("wrist_flex", 50.0)
    assert abs(math.degrees(q) - 50.0) < 1e-9 and not clipped, math.degrees(q)
    q2, clipped2 = ends.apply_joint("wrist_flex", 50.0)
    err = 50.0 - math.degrees(q2)
    assert err > 3.5 and not clipped2, (err, clipped2)

    q3, clipped3 = ident.apply_joint("wrist_roll", 170.0)
    assert clipped3 and abs(math.degrees(q3) - 162.8) < 0.1, math.degrees(q3)
    q4, clipped4 = ends.apply_joint("wrist_roll", 170.0)
    assert not clipped4 and 170.0 - math.degrees(q4) > 15.0, math.degrees(q4)

    ok, bad = ident.guard(cal, allow_unverified=True, expect_role="leader")
    assert ok, bad
    ok, bad = ends.guard(cal, allow_unverified=True, expect_role="leader")
    assert not ok and any("endpoint fit" in b for b in bad), bad

    un = ident.unreachable_deg()
    assert round(un["wrist_roll"]) == 40 and round(un["wrist_flex"]) == 16, un
    print("ok  identity fit is exact where the endpoint fit is quietly 4 deg out")


def test_simmap_span_check_catches_a_different_linkage():
    from arm.sim_mapping import SPAN_FAIL, build_map
    cal = _fake_calibration()
    lim = _faithful_limits(cal)
    lo, hi = lim["elbow_flex"]                       # model 15% short
    mid, half = (lo + hi) / 2, (hi - lo) / 2 * 0.85
    lim["elbow_flex"] = [mid - half, mid + half]
    from arm.sim_mapping import FIT_ENDPOINTS
    m = build_map(cal, lim, (0.0, 0.6109), fit_mode=FIT_ENDPOINTS,
                  source_arm={"role": "leader", "id": "x"})
    worst, wj = m.worst_span_ratio()
    assert wj == "elbow_flex" and worst > SPAN_FAIL, (wj, worst)
    ok, bad = m.guard(cal, allow_unverified=True, expect_role="leader")
    assert not ok and any("span ratio" in b for b in bad), bad
    # ... and the midpoint check that a naive implementation would use is
    # VACUOUS here: an affine fit through two points hits their midpoint exactly.
    q, _ = m.apply_joint("elbow_flex", 0.0)
    mid_model = (lim["elbow_flex"][0] + lim["elbow_flex"][1]) / 2
    assert abs(q - mid_model) < 1e-12, "midpoint residual cannot detect this"
    print("ok  simmap span check (catches what a midpoint check cannot)")


def test_simmap_guards_role_calibration_and_tampering():
    from arm.sim_mapping import build_map
    cal = _fake_calibration()
    m = build_map(cal, _faithful_limits(cal), (0.0, 0.6109),
                  source_arm={"role": "leader", "id": "my_leader"})
    ok, bad = m.guard(cal, expect_role="leader")
    assert not ok and any("not usable as verified" in b for b in bad), bad

    m.doc["verified"] = {"by": "tester", "method": "visual", "note": "n",
                         "fit_sha256": m.fit_sha256()}
    ok, bad = m.guard(cal, expect_role="leader")
    assert ok, bad

    ok, bad = m.guard(cal, expect_role="follower")
    assert not ok and any("fitted for" in b for b in bad), bad

    del m.doc["verified"]["fit_sha256"]                  # the one-field attack
    assert not m.is_verified(), "an unbound claim is not a verification"
    assert "UNBOUND" in m.verification_note()
    m.doc["verified"]["fit_sha256"] = m.fit_sha256()

    m.joints["wrist_roll"]["scale"] *= 1.02              # edit after verifying
    assert not m.is_verified(), "editing the fit must void the verification"
    assert "STALE" in m.verification_note()
    m.joints["wrist_roll"]["scale"] /= 1.02

    moved = _fake_calibration()
    moved["shoulder_pan"]["range_min"] += 1              # one tick
    ok, bad = m.guard(moved, expect_role="leader")
    assert not ok and any("degrees zero" in b for b in bad), bad
    print("ok  simmap guards (verified / role / tamper / recalibration)")


def test_simmap_refuses_a_map_that_cannot_move_a_joint():
    """Two ways a joint goes missing without anything erroring."""
    from arm.sim_mapping import build_map
    cal = _fake_calibration()
    lim = _faithful_limits(cal)

    # (a) the joint is absent from the map entirely -> apply() skips it
    partial = build_map(cal, {k: v for k, v in lim.items() if k != "wrist_roll"},
                        (0.0, 0.6109), source_arm={"role": "leader", "id": "x"})
    out, _ = partial.apply({j: 0.0 for j in cal})
    assert "wrist_roll" not in out, "apply() silently drops unmapped joints"
    ok, bad = partial.guard(cal, allow_unverified=True, expect_role="leader")
    assert not ok and any("does not cover" in b for b in bad), bad

    # (b) the joint is present but maps to a constant (empty gripper_rad)
    frozen = build_map(cal, lim, (0.0, 0.0),
                       source_arm={"role": "leader", "id": "x"})
    a, _ = frozen.apply_joint("gripper", 0.0)
    b, _ = frozen.apply_joint("gripper", 100.0)
    assert a == b, "this is the failure: open and shut map to the same radian"
    ok, bad = frozen.guard(cal, allow_unverified=True, expect_role="leader")
    assert not ok and any("never move" in x for x in bad), bad
    print("ok  simmap refuses maps with a joint that cannot move")


def test_linkstats_correlates_acks_with_their_own_command():
    """The bug the end-to-end test found: at 30 Hz the ack for step N arrives
    during step N+1, so comparing it against the CURRENT command marks every
    single step as a mismatch."""
    from net.sim_protocol import LinkStats, encode_ack, encode_cmd
    st = LinkStats()
    cmds = []
    for i in range(3):
        c = encode_cmd(i, 100.0 + i, {"shoulder_pan": 0.1 * i}, "sha")
        st.on_send(c["seq"], c["t_send_mono"], sent_rad=c["joints_rad"])
        cmds.append(c)
    for i, c in enumerate(cmds):                 # ack each against its OWN cmd
        st.on_ack(encode_ack(c, c["joints_rad"], []), 100.0 + i + 0.004)
    assert st.mismatched == 0, st.summary()
    assert st.acked == 3 and st.lost == 0
    assert abs(st.summary()["rtt_ms_median"] - 4.0) < 1e-6, st.summary()

    st2 = LinkStats()
    c = encode_cmd(9, 200.0, {"shoulder_pan": 0.5}, "sha")
    st2.on_send(9, 200.0, sent_rad=c["joints_rad"])
    st2.on_ack(encode_ack(c, {"shoulder_pan": 0.9}, []), 200.004)
    assert st2.mismatched == 1, "the sim applying something else, undeclared, " \
        "must be caught"
    print("ok  linkstats correlates each ack with its own command")


def test_linkstats_supersede_is_not_loss():
    from net.sim_protocol import LinkStats, encode_ack, encode_cmd
    st = LinkStats(lost_after_s=1.0)
    for i in range(4):
        st.on_send(i, 300.0, sent_rad={"a": 0.0})
    c = encode_cmd(3, 300.0, {"a": 0.0}, "sha")
    st.on_ack(encode_ack(c, {"a": 0.0}, [], superseded=[0, 1, 2]), 300.005)
    st.expire(302.0)
    s = st.summary()
    assert s["superseded"] == 3 and s["lost"] == 0 and s["acked"] == 1, s
    assert s["loss_rate"] == 0.0 and s["supersede_rate"] == 0.75, s

    st2 = LinkStats(lost_after_s=1.0)
    for i in range(4):
        st2.on_send(i, 300.0, sent_rad={"a": 0.0})
    st2.expire(302.0)
    assert st2.summary()["lost"] == 4, "a genuinely unanswered command IS lost"
    print("ok  linkstats keeps 'the sim was slow' apart from 'the net dropped it'")


def test_linkstats_counts_a_send_that_never_left():
    """A sendto() that fails is neither acked nor lost. Uncounted, it puts a
    hole in the sim's motion while loss_rate still reads zero."""
    from net.sim_protocol import LinkStats
    st = LinkStats()
    st.on_send(0, 100.0, sent_rad={"a": 0.0})
    st.on_send_failed(1)
    st.expire(200.0)
    s = st.summary()
    assert s["sent"] == 2 and s["send_failed"] == 1 and s["lost"] == 1, s
    print("ok  linkstats counts a command that never reached the wire")


def test_mismatch_tolerance_is_a_tracking_error_not_an_epsilon():
    """Against real Isaac the ack carries MEASURED joint positions, which never
    land exactly on the target. A 1e-6 tolerance would flag every step and the
    warning would be trained out of the operator within a day."""
    from net.sim_protocol import LinkStats, encode_ack, encode_cmd
    st = LinkStats(mismatch_tol_rad=0.10)
    c = encode_cmd(0, 1.0, {"shoulder_pan": 1.0}, "sha")
    st.on_send(0, 1.0, sent_rad=c["joints_rad"])
    st.on_ack(encode_ack(c, {"shoulder_pan": 1.008}, []), 1.01)   # normal lag
    assert st.mismatched == 0, "8 mrad of tracking error is not a wrong map"

    c = encode_cmd(1, 2.0, {"shoulder_pan": 1.0}, "sha")
    st.on_send(1, 2.0, sent_rad=c["joints_rad"])
    st.on_ack(encode_ack(c, {"shoulder_pan": -0.4}, []), 2.01)    # wrong sign
    assert st.mismatched == 1, "a flipped axis must still be caught"
    s = st.summary()
    assert abs(s["max_dev_deg"] - 80.21) < 0.1, s
    assert s["max_dev_joint"] == "shoulder_pan"
    print("ok  mismatch tolerance is sized to catch a wrong map, not lag")


def test_ctl_tracker_retransmits_then_gives_up():
    from net.sim_protocol import CtlTracker, encode_ctl_ack
    t = CtlTracker(timeout_s=0.1, max_tries=3)
    seq, msg = t.start("episode_start", episode=1)
    assert len(t.due(0.0)) == 1 and not t.due(0.0), "no resend before timeout"
    assert len(t.due(0.15)) == 1 and len(t.due(0.30)) == 1
    assert t.due(0.45) == [] and t.is_settled(seq), "must give up after max_tries"
    ok, detail = t.result(seq)
    assert ok is False and "no ack" in detail, detail

    t2 = CtlTracker(timeout_s=0.1, max_tries=5)
    seq2, msg2 = t2.start("episode_end", episode=2)
    t2.due(0.0)
    t2.on_ack(encode_ctl_ack(msg2, True, "saved"))
    assert t2.result(seq2) == (True, "saved") and t2.due(1.0) == []
    print("ok  ctl tracker (retransmit until acked, then fail loudly)")


def test_receiver_ctl_cache_is_per_sender_and_per_run():
    """Two ways the idempotency cache can answer the wrong question.

    (a) Every operator's CtlTracker starts at ctl_seq 1, so a cache keyed on the
        sequence alone hands a SECOND operator the first one's 'you are the
        owner' ack -- they see 'connected' and drive nothing.
    (b) p3 binds a fixed source port and also starts at ctl_seq 1, so a
        RESTARTED p3 is byte-for-byte a retransmission of the old one. Without
        the run nonce the receiver replays 'episode 1 started' and 'episode 1
        ended', p3 prints KEPT, and the sim recorded nothing at all.
    """
    src = open(os.path.join(ROOT, "sim", "receiver.py")).read()
    assert "key = (src, sess, seq)" in src and "self.seen_ctl[key]" in src, \
        "seen_ctl must be keyed by sender AND run AND sequence"
    assert "if seq in self.seen_ctl" not in src

    from net.sim_protocol import CtlTracker, encode_ctl, encode_ctl_ack
    a, b = CtlTracker(), CtlTracker()
    assert a.session != b.session, "each run needs its own nonce"
    sa, ma = a.start("episode_start", episode=1)
    sb, mb = b.start("episode_start", episode=1)
    assert sa == sb == 1, "the sequence alone cannot tell the runs apart"
    assert ma["session"] != mb["session"], "the nonce can"

    # b's ack must not settle a's request
    a.due(0.0)
    assert a.on_ack(encode_ctl_ack(mb, True, "ok")) is None
    assert a.result(sa) == (None, "pending") and a.rejected == 1
    # nor may an ack for a different episode
    a.on_ack(encode_ctl_ack(encode_ctl(1, "episode_start", session=a.session,
                                       episode=7), True, "ok"))
    assert a.result(sa) == (None, "pending"), "wrong episode must be rejected"
    # the right one settles it
    a.on_ack(encode_ctl_ack(ma, True, "started"))
    assert a.result(sa) == (True, "started")
    print("ok  ctl cache and acks are per sender, per run, per question")


def test_echo_backend_clips_and_reports():
    from sim.backend import make_backend
    b = make_backend("echo", limits_rad={"shoulder_pan": (-1.0, 1.0)})
    applied, clipped = b.apply({"shoulder_pan": 2.0, "elbow_flex": 0.3})
    assert applied["shoulder_pan"] == 1.0 and clipped == ["shoulder_pan"]
    assert applied["elbow_flex"] == 0.3
    assert b.readback is False, "echo must never claim to read back real state"
    ok, _ = b.on_episode("start", 1)
    assert ok and b.episodes == [("start", 1)]
    print("ok  echo backend (clips, and is honest about not reading back)")


def test_every_pasteable_command_in_the_docs_actually_parses():
    """A ```sh block in docs/ must survive being pasted into a shell.

    This exists because it has already gone wrong: the SOP hard-coded
    ~/so101venv/bin/python, the machine had ~/step0venv, and every command in
    the handover document failed on the first paste. The prose next to it
    explained the difference -- which helps nobody who is copying the block.

    Two things are checked, and the second is the dangerous one:

      1. `bash -n` parses it. A block that cannot even parse is dead on arrival.

      2. No UNQUOTED <placeholder>. In a shell `<name>` is not a blank to fill
         in, it is a REDIRECTION: `--usd <so101.usd>` reads from one file and
         TRUNCATES another. Sometimes that is a syntax error and you find out;
         sometimes it parses fine and silently creates files. Quoting it --
         `"<so101.usd>"` -- keeps it visibly blank and makes it fail loudly.

    Blocks that are diagrams or sample output stay untagged and are skipped,
    which is what the `sh` tag is for: it marks a block as meant to be pasted.
    """
    import re
    import subprocess

    docs = sorted(pathlib.Path(os.path.join(ROOT, "docs")).glob("*.md"))
    assert docs, "no docs/ to check"
    checked = 0
    for doc in docs:
        text = doc.read_text()
        for m in re.finditer(r"^```sh\n(.*?)^```", text, re.S | re.M):
            block = m.group(1)
            line = text[:m.start()].count("\n") + 1
            where = f"{doc.name}:{line}"

            r = subprocess.run(["bash", "-n"], input=block,
                               text=True, capture_output=True)
            assert r.returncode == 0, f"{where}: will not parse\n{r.stderr}{block}"

            # Strip quoted spans before looking, so a quoted placeholder passes.
            bare = re.sub(r"'[^']*'", "", re.sub(r'"[^"]*"', "", block))
            bad = re.findall(r"<[A-Za-z_][A-Za-z0-9_.-]*>", bare)
            assert not bad, (f"{where}: unquoted placeholder(s) {bad} -- in a "
                             f"shell that is a redirection, not a blank\n{block}")
            checked += 1
    assert checked >= 15, f"only {checked} sh blocks found; did the tags get lost?"

    # The SOP tells the reader how many checks to expect, and that number was
    # already two behind before anyone noticed. A count in a document is a fact
    # about the code, so hold it to the code.
    me = pathlib.Path(__file__).read_text()
    ran = len(re.findall(r"^    test_\w+\(\)$", me, re.M))
    sop = pathlib.Path(os.path.join(ROOT, "docs", "SOP.md")).read_text()
    m = re.search(r"`ALL PASS` and (\d+) checks", sop)
    assert m, "docs/SOP.md no longer says how many checks to expect"
    assert int(m.group(1)) == ran, (
        f"docs/SOP.md promises {m.group(1)} checks, this file runs {ran}")
    print(f"ok  every pasteable command in docs/ parses ({checked} blocks), "
          f"and the SOP's count of {ran} is right")


def test_the_launchers_parse_and_point_at_files_that_exist():
    """bin/ is now the only documented way to run anything, so a typo in it
    breaks every command in the SOP at once. Check two things statically:
    the scripts parse, and every repo-relative path they exec actually exists.
    """
    import re
    import subprocess

    binder = pathlib.Path(os.path.join(ROOT, "bin"))
    scripts = sorted(p for p in binder.iterdir() if p.is_file())
    assert {p.name for p in scripts} >= {"so101", "p1", "p2", "p3"}, scripts

    for script in scripts:
        r = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
        assert r.returncode == 0, f"bin/{script.name}: {r.stderr}"
        assert os.access(script, os.X_OK), f"bin/{script.name} is not executable"

    body = (binder / "so101").read_text()
    targets = re.findall(r'(?:run_program |run_tool |"\$REPO/)'
                         r'((?:programs|tools|sim)/[\w./]+\.py)', body)
    assert len(targets) >= 8, f"only found {len(targets)} script paths: {targets}"
    for rel in targets:
        assert os.path.isfile(os.path.join(ROOT, rel)), \
            f"bin/so101 execs {rel}, which does not exist"

    # Every subcommand in the case statement must appear in the usage text, or
    # it exists and nobody can find it.
    cases = set(re.findall(r"^  ([a-z0-9|]+)\)", body, re.M))
    named = {c for group in cases for c in group.split("|")}
    named -= {"*", "help", "-h", "--help", "leader", "follower"}
    usage = body[body.index("usage() {"):body.index("USAGE\n}")]
    missing = sorted(c for c in named if c not in usage)
    assert not missing, f"bin/so101 subcommands not listed in its own usage: {missing}"
    print(f"ok  launchers parse, and every path they run exists ({len(targets)} scripts)")



def _argparse_defaults(ap):
    import argparse
    return {a.dest: a.default for a in ap._actions
            if not isinstance(a, argparse._HelpAction)}


def test_step_limit_is_a_speed_not_a_number():
    """The per-step limit must follow the loop rate, or it means four things.

    rate_limit() clamps per STEP. 8 degrees per step is 240 deg/s at 30 Hz and
    960 deg/s at 120 Hz. Before this, both programs defaulted to 8.0 whatever
    the rate -- so p2's own default (60 Hz) was already running at twice the
    speed the SOP documents as safe, and `--arm-fps 120` without the matching
    `--max-step-deg 2` would have been four times.

    Checked here: the speed is constant across rates, an explicit flag still
    wins, the two flags are independent, and a bad rate is refused rather than
    dividing by zero into an infinite limit.
    """
    from arm.control import (MAX_GRIPPER_PCT_PER_S, MAX_JOINT_DEG_PER_S,
                             step_limits_for)

    for fps in (10, 30, 60, 120, 200):
        deg, grip, derived = step_limits_for(fps)
        assert abs(deg * fps - MAX_JOINT_DEG_PER_S) < 1e-9, (fps, deg)
        assert abs(grip * fps - MAX_GRIPPER_PCT_PER_S) < 1e-9, (fps, grip)
        assert derived == ("max_step_deg", "max_step_gripper_pct"), derived

    # The table in docs/SOP.md, reproduced from the constants. If someone
    # changes a constant, this is what says the document is now wrong.
    assert step_limits_for(30)[:2] == (8.0, 15.0)
    assert step_limits_for(60)[:2] == (4.0, 7.5)
    assert step_limits_for(120)[:2] == (2.0, 3.75)

    deg, grip, derived = step_limits_for(120, max_step_deg=5.0)
    assert deg == 5.0 and derived == ("max_step_gripper_pct",), (deg, derived)
    deg, grip, derived = step_limits_for(120, max_step_gripper_pct=9.0)
    assert grip == 9.0 and derived == ("max_step_deg",), (grip, derived)
    assert step_limits_for(120, 5.0, 9.0)[2] == (), "nothing was derived"

    # 0 means "off" for a LIMIT, but as a RATE it is a division by zero that
    # would hand back an infinite limit -- the one value that must not be
    # silently accepted.
    for bad in (0, -1, None):
        try:
            step_limits_for(bad)
        except ValueError:
            continue
        raise AssertionError(f"step_limits_for({bad!r}) should have refused")
    print("ok  step limit is a speed, and it scales with the loop rate")


def test_p1_and_p2_agree_on_defaults_that_make_them_comparable():
    """p2 exists to answer "does running the cameras degrade the arm?".

    That comparison is only valid if the arm loop is set up identically in
    both, so the two defaults are checked against each other here rather than
    left to whoever edits one file and not the other.
    """
    import importlib

    # p2 imports cv2 and numpy at module level; this file must still run
    # without them. The helper stubs only what is genuinely missing and puts
    # sys.modules back exactly as it found it -- another test installs its own
    # fake cv2 and would be poisoned by a leftover.
    progs = ("programs.p1_follow_leader", "programs.p2_record_cameras")
    with _stub_imaging(*progs):
        p1 = importlib.import_module(progs[0])
        p2 = importlib.import_module(progs[1])
        d1 = _argparse_defaults(p1.build_args())
        d2 = _argparse_defaults(p2.build_args())

    assert d1["fps"] == d2["arm_fps"], (
        f"p1 --fps {d1['fps']} but p2 --arm-fps {d2['arm_fps']}: a bare p1 and "
        f"a bare p2 would run the arm at different rates and could not be "
        f"compared")
    for k in ("max_step_deg", "max_step_gripper_pct"):
        assert d1[k] is None and d2[k] is None, (
            f"{k} must default to None in both so it is derived from the rate")
    for k in ("track_tol_deg", "track_tol_gripper_pct", "track_strikes"):
        assert d1[k] == d2[k], f"{k}: p1 {d1[k]} vs p2 {d2[k]}"
    print("ok  p1 and p2 default to the same arm loop, so p2's comparison holds")



if __name__ == "__main__":
    test_clock()
    test_signal_jsonl()
    test_signal_udp()
    test_jsonl_writer()
    test_rate_limit()
    test_camera_rotation_is_applied_at_the_source()
    test_exposure_and_mains_frequency_are_independent()
    test_watchdog_arming_must_not_mix_units()
    test_rate_limit_first_step_is_the_dangerous_one()
    test_pose_diff_and_watchdog()
    test_units_are_not_all_degrees()
    test_serial_parsing()
    test_best_lag()
    test_camera_bandwidth_budget()
    test_camera_controls()
    test_resilient_reconnect()
    test_simmap_fit_recovers_sign_and_scale()
    test_identity_fit_beats_endpoint_fit_when_spans_disagree()
    test_simmap_span_check_catches_a_different_linkage()
    test_simmap_guards_role_calibration_and_tampering()
    test_simmap_refuses_a_map_that_cannot_move_a_joint()
    test_linkstats_correlates_acks_with_their_own_command()
    test_linkstats_supersede_is_not_loss()
    test_linkstats_counts_a_send_that_never_left()
    test_mismatch_tolerance_is_a_tracking_error_not_an_epsilon()
    test_ctl_tracker_retransmits_then_gives_up()
    test_receiver_ctl_cache_is_per_sender_and_per_run()
    test_echo_backend_clips_and_reports()
    test_step_limit_is_a_speed_not_a_number()
    test_p1_and_p2_agree_on_defaults_that_make_them_comparable()
    test_every_pasteable_command_in_the_docs_actually_parses()
    test_the_launchers_parse_and_point_at_files_that_exist()
    print("ALL PASS")
