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


if __name__ == "__main__":
    test_clock()
    test_signal_jsonl()
    test_signal_udp()
    test_resilient_reconnect()
    print("ALL PASS")
