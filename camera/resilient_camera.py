#!/usr/bin/env python3
"""A single USB camera that survives disconnects.

Opens cv2.VideoCapture on a STABLE by-path node. A background thread reads
frames; on failure it retries, and if the device drops off the USB bus it
reopens the SAME physical camera by that same by-path -- never /dev/videoN,
which renumbers on reconnect and can land you on the OTHER camera (data records
fine, with the wrong label).

Policy here is fail-continue-and-mark, which is right for a camera and wrong for
an arm: a dropped frame is a data-quality problem, not a physical hazard. The
caller decides what to do with the marks -- p2 keeps going, p4 will abort the
episode.

  * read_latest() NEVER raises: it returns (frame_or_None, stale_s, state).
  * Every fault is logged as JSON with MAGNITUDE, not a boolean -- how many
    frames, how many seconds down, how many reconnects (rule 99).
  * ** After every reconnect the negotiated mode is re-checked. ** A camera that
    comes back at a different resolution and is not noticed is the worst failure
    in this family: recording continues and looks fine.
"""
import json
import threading
import time

import cv2


def _fourcc_str(v):
    return "".join(chr((int(v) >> (8 * i)) & 0xFF) for i in range(4)).strip("\x00")


class ResilientCamera:
    def __init__(self, name, device, width, height, fps, fourcc="MJPG",
                 fault_log=None, reconnect_backoff=(0.2, 2.0),
                 max_read_fail=30, log_every_n_attempts=10):
        self.name = name
        self.device = device
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
        self._log_every = max(1, log_every_n_attempts)
        self.state = "init"              # init|streaming|reconnecting|down
        self.mode = None                 # negotiated mode of the current open
        self.stats = {"frames": 0, "dropped": 0, "reconnects": 0,
                      "open_attempts": 0, "total_down_s": 0.0,
                      "mode_changes": 0}

    # ---------- logging ----------
    def _log(self, event, **kw):
        rec = {"t_unix": time.time(), "t_mono": time.monotonic(),
               "cam": self.name, "device": self.device, "event": event, **kw}
        if self._fault_fh:
            self._fault_fh.write(json.dumps(rec, separators=(",", ":")) + "\n")

    # ---------- device ----------
    def _open(self):
        self.stats["open_attempts"] += 1
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        mode = {"width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                "fps": round(float(cap.get(cv2.CAP_PROP_FPS)), 3),
                "fourcc": _fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))}
        if self.mode is None:
            self.mode = mode
        elif mode != self.mode:
            self.stats["mode_changes"] += 1
            self._log("mode_changed", was=self.mode, now=mode)
            self.mode = mode
        return cap

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def _loop(self):
        backoff = self._bmin
        consecutive = 0
        down_since = None
        attempts_since_down = 0
        while not self._stop.is_set():
            if self._cap is None:
                cap = self._open()
                if cap is None:
                    attempts_since_down += 1
                    if attempts_since_down == 1 or attempts_since_down % self._log_every == 0:
                        self._log("reconnect_attempt", attempt=attempts_since_down)
                    self.state = "reconnecting"
                    time.sleep(backoff)
                    backoff = min(backoff * 2, self._bmax)
                    continue
                self._cap = cap
                backoff = self._bmin
                if down_since is not None:
                    down = time.monotonic() - down_since
                    self.stats["reconnects"] += 1
                    self.stats["total_down_s"] += down
                    self._log("reconnected", down_s=round(down, 3),
                              attempts=attempts_since_down,
                              reconnects_total=self.stats["reconnects"])
                    down_since = None
                attempts_since_down = 0
                self.state = "streaming"
                consecutive = 0

            ok, frame = self._cap.read()
            if ok:
                with self._lock:
                    self._frame = frame
                    self._frame_mono = time.monotonic()
                self.stats["frames"] += 1
                consecutive = 0
                self.state = "streaming"
            else:
                consecutive += 1
                self.stats["dropped"] += 1
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

    # ---------- consumer ----------
    def read_latest(self):
        with self._lock:
            f = self._frame
            m = self._frame_mono
        stale = (time.monotonic() - m) if m else float("inf")
        return f, stale, self.state

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        self._log("summary", **self.stats)
        if self._fault_fh:
            try:
                self._fault_fh.flush()
                self._fault_fh.close()
            except Exception:
                pass
