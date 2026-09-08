"""Emit SO-101 joint states as a signal stream.

Transport (default): UDP datagrams of JSON, plus an optional JSONL file on disk.
Both use only the standard library (socket, json) so this has ZERO extra
dependencies and can run anywhere. A later Isaac Sim bridge just subscribes to
the same UDP port and mirrors the joints onto the virtual SO-101.

Schema: see docs/SIGNAL_SCHEMA.md ("so101.joints.v1").
"""
import json
import socket

SCHEMA = "so101.joints.v1"


class SignalPublisher:
    def __init__(self, udp_addr=None, jsonl_path=None):
        # udp_addr: "host:port" (e.g. "192.168.0.9:9870") or None
        self._sock = None
        self._dst = None
        self._fh = None
        if udp_addr:
            host, port = udp_addr.rsplit(":", 1)
            self._dst = (host, int(port))
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if jsonl_path:
            self._fh = open(jsonl_path, "a", buffering=1)  # line-buffered

    def publish(self, record):
        line = json.dumps(record, separators=(",", ":"))
        if self._sock is not None:
            try:
                self._sock.sendto(line.encode(), self._dst)
            except OSError:
                pass  # UDP is best-effort; the network must never kill teleop
        if self._fh is not None:
            self._fh.write(line + "\n")

    def close(self):
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except OSError:
                pass
        if self._sock is not None:
            self._sock.close()
