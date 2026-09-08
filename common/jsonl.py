"""Durable line-oriented JSON logging.

Every program writes two streams:
  * a ROW stream  -- one record per control step (high rate)
  * an EVENT stream -- start / pre-flight result / fault / stop (low rate)

Both are JSONL (one JSON object per line). Line-buffered so a killed process
still leaves everything written up to the last completed line -- the lesson from
`features.npz`, which only wrote in `finally` and lost a whole run on SIGHUP.
"""
import json
import os
import time


class JsonlWriter:
    def __init__(self, path):
        self.path = path
        d = os.path.dirname(os.path.abspath(path))
        if d:
            os.makedirs(d, exist_ok=True)
        self._fh = open(path, "a", buffering=1)  # line-buffered

    def write(self, record):
        self._fh.write(json.dumps(record, separators=(",", ":"),
                                  default=str) + "\n")

    def event(self, kind, **kw):
        self.write({"t_unix": time.time(), "t_mono": time.monotonic(),
                    "event": kind, **kw})

    def close(self):
        try:
            self._fh.flush()
            os.fsync(self._fh.fileno())
        except OSError:
            pass
        try:
            self._fh.close()
        except OSError:
            pass
