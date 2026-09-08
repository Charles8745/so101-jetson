#!/usr/bin/env python3
"""Dependency-free self-tests (no OpenCV, no lerobot). Runs on any machine.

Covers the logic that does not touch hardware: the shared clock, the signal
publisher's JSONL output, and the camera's reconnect state machine (with a fake
VideoCapture injected in place of cv2).
"""
import json
import os
import sys
import tempfile
import time
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def test_clock():
    from common.clock import stamp, epoch
    s = stamp()
    assert set(s) == {"t_mono", "t_unix"}, s
    assert isinstance(s["t_mono"], float) and isinstance(s["t_unix"], float)
    assert set(epoch()) == {"t_mono", "t_unix"}
    print("ok  clock")


def test_signal_jsonl():
    from arm.signal_pub import SignalPublisher, SCHEMA
    p = tempfile.mktemp(suffix=".jsonl")
    pub = SignalPublisher(udp_addr=None, jsonl_path=p)
    pub.publish({"schema": SCHEMA, "seq": 0, "leader": {"shoulder_pan": 1.0}})
    pub.publish({"schema": SCHEMA, "seq": 1, "leader": {"shoulder_pan": 2.0}})
    pub.close()
    lines = open(p).read().strip().splitlines()
    assert len(lines) == 2, lines
    r0 = json.loads(lines[0])
    assert r0["seq"] == 0 and r0["leader"]["shoulder_pan"] == 1.0
    print("ok  signal jsonl")


def test_signal_udp():
    # send to a real local UDP socket and read it back
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


def test_resilient_reconnect():
    # inject a fake cv2 so we can import resilient_camera without OpenCV
    fake = types.ModuleType("cv2")
    fake.CAP_V4L2 = 200
    fake.CAP_PROP_FOURCC = 6
    fake.CAP_PROP_FRAME_WIDTH = 3
    fake.CAP_PROP_FRAME_HEIGHT = 4
    fake.CAP_PROP_FPS = 5
    fake.VideoWriter_fourcc = lambda *a: 0
    st = {"reads": 0, "opened": 0}

    class FakeCap:
        def __init__(self, dev, backend=None):
            st["opened"] += 1

        def isOpened(self):
            return True

        def set(self, *a):
            return True

        def read(self):
            st["reads"] += 1
            # reads 4..40 fail (device "gone"), then it recovers
            if 3 < st["reads"] <= 40:
                return False, None
            return True, "FRAME"

        def release(self):
            pass

    fake.VideoCapture = FakeCap
    sys.modules["cv2"] = fake
    from camera.resilient_camera import ResilientCamera
    rc = ResilientCamera("t", "/dev/null", 8, 8, 30,
                         reconnect_backoff=(0.01, 0.02), max_read_fail=5).start()
    deadline = time.time() + 3.0
    f = None
    while time.time() < deadline:
        f, stale, state = rc.read_latest()
        if f == "FRAME" and state == "streaming" and st["opened"] >= 2:
            break
        time.sleep(0.05)
    rc.stop()
    assert st["opened"] >= 2, f"should have reopened; opened={st['opened']}"
    assert f == "FRAME", f"should have recovered a frame; got {f}"
    print(f"ok  resilient reconnect (reopened {st['opened']}x)")




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


def test_pose_diff_and_watchdog():
    from arm.control import TrackingWatchdog, pose_diff
    d, worst, val = pose_diff({"a": 10.0, "b": 0.0}, {"a": 1.0, "b": 0.5})
    assert worst == "a" and abs(val - 9.0) < 1e-9, (d, worst, val)
    w = TrackingWatchdog(tol_deg=5.0, n_strikes=3)
    assert not w.update({"a": 0.0}, {"a": 0.0})
    assert not w.update({"a": 100.0}, {"a": 0.0})
    assert not w.update({"a": 100.0}, {"a": 0.0})
    assert w.update({"a": 100.0}, {"a": 0.0}), "should trip on 3rd strike"
    assert "tracking error" in w.reason()
    w2 = TrackingWatchdog(tol_deg=5.0, n_strikes=3)
    w2.update({"a": 100.0}, {"a": 0.0})
    assert not w2.update({"a": 0.0}, {"a": 0.0}), "in-tolerance resets strikes"
    print("ok  pose_diff + tracking watchdog")


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


if __name__ == "__main__":
    test_clock()
    test_signal_jsonl()
    test_signal_udp()
    test_jsonl_writer()
    test_rate_limit()
    test_pose_diff_and_watchdog()
    test_serial_parsing()
    test_best_lag()
    test_resilient_reconnect()
    print("ALL PASS")
