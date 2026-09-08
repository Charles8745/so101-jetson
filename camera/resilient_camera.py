#!/usr/bin/env python3
"""A single USB camera that survives disconnects.

Wraps cv2.VideoCapture opened by a STABLE by-path device node. A background
thread reads frames; on read failure it retries and, if the device drops off the
USB bus (the wrist-cable fault we hit on 09-05), it reopens the SAME physical
camera by its by-path -- NOT /dev/videoN, which renumbers on reconnect.

Contract:
  * read_latest() NEVER raises. It returns (frame_or_None, stale_s, state).
  * Every fault is logged as JSON (device by-path, consecutive fails, down_s) so
    a demonstration episode overlapping a fault can be flagged/dropped later.
  * It reports MAGNITUDE, not a boolean (down_s, dropped-frame count) -- rule 99.
"""
import json
import threading
import time

import cv2


class ResilientCamera:
    def __init__(self, name, device, width, height, fps, fourcc="MJPG",
                 fault_log=None, reconnect_backoff=(0.2, 2.0), max_read_fail=30):
        self.name = name
        self.device = device            # MUST be a by-path node for reliable reopen
        self.width, self.height, self.fps = width, height, fps
        self.fourcc = fourcc
        self._cap = None
        self._frame = None
        self._frame_mono = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._fault_fh = open(fault_log, "a", buffering=1) if fault_log else None
        self._bmin, self._bmax = reconnect_backoff
        self._max_read_fail = max_read_fail
        self.state = "init"             # init|streaming|reconnecting|down
        self.dropped = 0

    def _log(self, event, **kw):
        rec = {"t_unix": time.time(), "t_mono": time.monotonic(),
               "cam": self.name, "device": self.device, "event": event, **kw}
        if self._fault_fh:
            self._fault_fh.write(json.dumps(rec, separators=(",", ":")) + "\n")

    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        return cap

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def _loop(self):
        backoff = self._bmin
        consecutive = 0
        down_since = None
        while not self._stop.is_set():
            if self._cap is None:
                cap = self._open()
                if cap is None:
                    self.state = "reconnecting"
                    time.sleep(backoff)
                    backoff = min(backoff * 2, self._bmax)
                    continue
                self._cap = cap
                backoff = self._bmin
                if down_since is not None:
                    self._log("reconnected",
                              down_s=round(time.monotonic() - down_since, 3))
                    down_since = None
                self.state = "streaming"
                consecutive = 0
            ok, frame = self._cap.read()
            if ok:
                with self._lock:
                    self._frame = frame
                    self._frame_mono = time.monotonic()
                consecutive = 0
                self.state = "streaming"
            else:
                consecutive += 1
                self.dropped += 1
                if consecutive == 1:
                    self._log("read_fail")
                if consecutive >= self._max_read_fail:
                    if down_since is None:
                        down_since = time.monotonic()
                    self._log("down", consecutive=consecutive)
                    self.state = "down"
                    try:
                        self._cap.release()
                    except Exception:
                        pass
                    self._cap = None
                    time.sleep(backoff)
                    backoff = min(backoff * 2, self._bmax)
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass

    def read_latest(self):
        with self._lock:
            f = self._frame
            m = self._frame_mono
        stale = (time.monotonic() - m) if m else float("inf")
        return f, stale, self.state

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._fault_fh:
            try:
                self._fault_fh.flush()
                self._fault_fh.close()
            except Exception:
                pass
