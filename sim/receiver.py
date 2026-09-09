#!/usr/bin/env python3
"""Sim-side half of program 3. Runs on Spark. Contains no Isaac code.

    Jetson  --UDP :9871-->  THIS  -->  SimBackend  -->  Isaac
            <--UDP  ack---

Responsibilities, in the order they matter:

  1. ** Conflate, never queue. ** Drain everything waiting in the socket each
     tick and apply only the NEWEST command. A receiver that applies them in
     order falls further behind for as long as it is slow, and the latency you
     then measure is your own bug, not the network's. The skipped seqs are
     named in the ack as `superseded` so the Jetson counts them separately from
     packet loss -- different problem, different fix.

  2. ** One owner at a time. ** The first sender to say `hello` claims the sim;
     anything from a second sender is dropped and counted. Two operators driving
     one virtual arm produces a recording that looks fine and is meaningless.

  3. ** Ack AFTER stepping, with read-back where the backend can. ** The ack
     reports what the simulator actually did, so a wrong joint map shows up as a
     mismatch on the Jetson within one step instead of at training time.

  4. ** Never leave the sim recording. ** If the Jetson dies mid-episode the
     receiver discards the open episode after --episode-timeout-s. An episode
     that ends because the operator's machine crashed is not a demonstration.

  5. Control messages (episode boundaries) are idempotent on `ctl_seq`: a
     retransmission is re-acked, never re-executed.

Test it with no simulator and no second machine:

    python3 sim/receiver.py --backend echo
    python3 programs/p3_teleop_sim.py --sim 127.0.0.1 ...      (other terminal)
"""
import argparse
import errno
import json
import os
import select
import signal
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.clock import epoch, stamp                                # noqa: E402
from common.jsonl import JsonlWriter                                 # noqa: E402
from net.sim_protocol import (ACK_SCHEMA, CMD_SCHEMA, CTL_SCHEMA,    # noqa: E402
                              DEFAULT_CMD_PORT, dumps, encode_ack,
                              encode_ctl_ack, loads)

DRAIN_CAP = 256          # datagrams per tick; a flood must not stall the loop
RCVBUF = 1 << 20

# Written into rows.jsonl itself, not only into events.jsonl. A loader that
# groups rows by `episode` -- the obvious way to build a dataset -- would
# otherwise happily train on episodes that were discarded, because discarding
# writes a line to a different file and removes nothing.
EPISODE_SCHEMA = "so101.episode.v1"


class Receiver:
    def __init__(self, backend, port=DEFAULT_CMD_PORT, bind="0.0.0.0",
                 fps=30.0, expect_map_sha=None, episode_timeout_s=5.0,
                 stale_ms=500.0, out="./logs/p3_sim", takeover=False,
                 allow_anonymous=False):
        self.backend = backend
        self.fps = float(fps)
        self.period = 1.0 / self.fps if self.fps > 0 else 0.0
        self.expect_map_sha = expect_map_sha
        self.episode_timeout_s = float(episode_timeout_s)
        self.stale_s = float(stale_ms) / 1e3
        self.takeover = bool(takeover)
        self.allow_anonymous = bool(allow_anonymous)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF)
        self.sock.bind((bind, int(port)))
        self.sock.setblocking(False)

        run_dir = os.path.join(out, time.strftime("%Y%m%d-%H%M%S"))
        self.run_dir = run_dir
        self.rows = JsonlWriter(os.path.join(run_dir, "rows.jsonl"))
        self.events = JsonlWriter(os.path.join(run_dir, "events.jsonl"))

        self.owner = None                 # (ip, port) of the claiming sender
        self.session = None               # the owner's run nonce
        # What actually happened to each episode. `episode_end` must not be
        # able to return success for an episode the receiver already threw
        # away: p3 would print KEPT for a demonstration that exists nowhere.
        self.episode_result = {}          # episode -> "kept" | "discarded"
        # (sender, ctl_seq) -> ack. Keyed by SENDER as well as sequence: every
        # operator's CtlTracker starts at ctl_seq 1, so a cache keyed on the
        # sequence alone hands a second operator the FIRST one's "you are the
        # owner" acknowledgement. They then see "connected", move their leader,
        # and nothing happens, with no clue why. Found by the two-operator test.
        self.seen_ctl = {}
        self.episode = None
        self.recording = False
        self.last_applied = None

        self.n_cmd = 0
        self.n_applied = 0
        self.n_superseded = 0
        self.n_foreign = 0
        self.n_bad_map = 0
        self.n_stale = 0
        self.n_malformed = 0
        self.max_seq = -1
        self.ep_applied = 0        # steps in the CURRENT episode, not lifetime
        self.t_last_cmd = None
        self.stop = False

    def _mark_discarded(self, ep, reason):
        self.episode_result[ep] = "discarded"
        self.recording = False
        self.events.event("episode_discard", episode=ep, reason=reason,
                          steps=self.ep_applied)
        self.rows.write({"schema": EPISODE_SCHEMA, **stamp(), "episode": ep,
                         "result": "discarded", "steps": self.ep_applied,
                         "detail": reason})

    # ------------------------------------------------------------------ net
    def _drain(self):
        """Returns (newest_cmd, t_arrive, superseded_seqs, [ctl msgs], addr).

        `t_arrive` is when the newest command was READ off the socket. We drain
        continuously but only apply at tick boundaries, so a command can sit
        here for up to one tick. Reporting that wait back in the ack is what
        lets the Jetson split its round-trip time into network / waiting for the
        sim's tick / actually stepping. Without the split, a 20 ms RTT looks
        like a slow network when it is really our own 30 Hz phase.
        """
        newest, superseded, ctls, addr = None, [], [], None
        t_arrive = None
        for _ in range(DRAIN_CAP):
            try:
                buf, src = self.sock.recvfrom(65535)
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise
            try:
                msg = loads(buf)
            except Exception:
                self.n_malformed += 1
                continue
            schema = msg.get("schema")
            if schema == CTL_SCHEMA:
                ctls.append((msg, src))
                continue
            if schema != CMD_SCHEMA:
                self.n_malformed += 1
                continue
            if self.owner is not None and src != self.owner:
                self.n_foreign += 1
                continue
            addr = src
            self.n_cmd += 1
            if newest is None:
                newest, t_arrive = msg, time.monotonic()
            elif msg["seq"] > newest["seq"]:
                superseded.append(newest["seq"])
                newest, t_arrive = msg, time.monotonic()
            else:
                # UDP reordered: this one is older than what we already hold
                superseded.append(msg["seq"])
        return newest, t_arrive, superseded, ctls, addr

    def _send(self, obj, addr):
        if addr is None:
            return
        try:
            self.sock.sendto(dumps(obj), addr)
        except (OSError, TypeError):
            pass          # an ack is best-effort; the loop must never die on it

    # -------------------------------------------------------------- control
    def _handle_ctl(self, msg, src):
        seq = int(msg.get("ctl_seq", -1))
        action = msg.get("action")
        sess = msg.get("session")
        # Keyed by SENDER and by the sender's RUN, as well as by sequence.
        # Every operator's CtlTracker starts at ctl_seq 1 and p3 binds a fixed
        # source port, so without the session a restarted p3 is indistinguish-
        # able from a retransmission: the receiver would replay "you are the
        # owner" and "episode 1 started" from the previous run, p3 would print
        # KEPT, and the simulator would have recorded nothing whatsoever.
        key = (src, sess, seq)

        if key in self.seen_ctl:                      # retransmission
            self._send(self.seen_ctl[key], src)
            return

        if action == "hello":
            same_session = (src == self.owner and sess == self.session)
            if self.owner is None or same_session or self.takeover:
                changed = (self.owner != src) or (self.session != sess)
                if self.recording and changed:
                    # A new run showed up while an episode was open. Nobody is
                    # going to close it, so close it here -- otherwise the
                    # backend keeps recording into an episode the operator has
                    # already walked away from. ("Never leave the sim
                    # recording" is only true if this branch does it too.)
                    self.backend.on_episode("discard", self.episode,
                                            {"reason": "a new session connected"})
                    self.episode_result[self.episode] = "discarded"
                    self.events.event("episode_discard", episode=self.episode,
                                      reason="a new session connected")
                    print(f"[rx] DISCARDED open episode {self.episode}: "
                          f"a new session connected")
                self.owner = src
                self.session = sess
                self.episode, self.recording = None, False
                ack = encode_ctl_ack(msg, True, f"owner={src[0]}:{src[1]}",
                                     backend=self.backend.name,
                                     session_accepted=sess,
                                     readback=self.backend.readback,
                                     joints=self.backend.joint_names(),
                                     expect_map_sha=self.expect_map_sha)
                if changed:
                    self.events.event("owner_claimed", addr=list(src),
                                      peer_map_sha=msg.get("map_sha256"))
                    print(f"[rx] owner {src[0]}:{src[1]}  "
                          f"map {str(msg.get('map_sha256'))[:12]}")
                if self.expect_map_sha and msg.get("map_sha256") != self.expect_map_sha:
                    ack["ok"] = False
                    ack["detail"] = (f"map sha mismatch: sim expects "
                                     f"{self.expect_map_sha[:12]}, sender has "
                                     f"{str(msg.get('map_sha256'))[:12]}")
                    self.owner = None
                    print("[rx] REFUSED: " + ack["detail"])
            else:
                self.n_foreign += 1
                self.events.event("hello_refused", addr=list(src),
                                  owner=list(self.owner))
                print(f"[rx] refused a second operator at {src[0]}:{src[1]}")
                ack = encode_ctl_ack(msg, False,
                                     f"busy: owned by {self.owner[0]}:{self.owner[1]}")
        elif self.owner is not None and (src != self.owner
                                         or sess != self.session):
            self.n_foreign += 1
            ack = encode_ctl_ack(msg, False, "not the owner (say hello first)")
        elif self.owner is None:
            ack = encode_ctl_ack(msg, False, "nobody owns this sim -- say hello")
        elif action == "episode_start":
            ep = msg.get("episode")
            if self.recording:
                ack = encode_ctl_ack(msg, False,
                                     f"episode {self.episode} is still open")
            else:
                ok, detail = self.backend.on_episode("start", ep, msg.get("meta"))
                if ok:
                    self.episode, self.recording = ep, True
                    self.ep_applied = 0
                    self.episode_result.pop(ep, None)
                    self.events.event("episode_start", episode=ep,
                                      meta=msg.get("meta"))
                    self.rows.write({"schema": EPISODE_SCHEMA, **stamp(),
                                     "episode": ep, "result": "open"})
                    print(f"[rx] episode {ep} START")
                ack = encode_ctl_ack(msg, ok, detail, episode=ep)
        elif action in ("episode_end", "episode_discard"):
            ep = msg.get("episode")
            kind = "end" if action == "episode_end" else "discard"
            if not self.recording:
                # NOT blanket-idempotent. Saying "sure, that worked" for an
                # episode we already threw away makes p3 print KEPT for a
                # demonstration that exists nowhere -- which is the exact
                # failure this program is built to prevent. Only a repeat of
                # an end we already performed is a success.
                prior = self.episode_result.get(ep)
                if prior == "kept" and kind == "end":
                    ack = encode_ctl_ack(msg, True, "already ended", episode=ep)
                elif prior == "discarded":
                    ack = encode_ctl_ack(
                        msg, kind == "discard",
                        f"episode {ep} was DISCARDED by the receiver -- it was "
                        f"not saved", episode=ep, receiver_discarded=True)
                else:
                    ack = encode_ctl_ack(msg, kind == "discard",
                                         f"no episode {ep} is open here",
                                         episode=ep)
            elif ep != self.episode:
                ack = encode_ctl_ack(msg, False,
                                     f"open episode is {self.episode}, not {ep}")
            else:
                ok, detail = self.backend.on_episode(kind, ep, msg.get("meta"))
                self.recording = False
                result = "kept" if (ok and kind == "end") else "discarded"
                self.episode_result[ep] = result
                self.events.event(f"episode_{kind}", episode=ep, ok=ok,
                                  detail=detail, steps=self.ep_applied,
                                  total_applied=self.n_applied)
                self.rows.write({"schema": EPISODE_SCHEMA, **stamp(),
                                 "episode": ep, "result": result,
                                 "steps": self.ep_applied, "detail": detail})
                print(f"[rx] episode {ep} {kind.upper()}  "
                      f"({self.ep_applied} steps, {detail})")
                ack = encode_ctl_ack(msg, ok, detail, episode=ep)
        elif action == "ping":
            ack = encode_ctl_ack(msg, True, "pong", sim_time=self.backend.sim_time())
        elif action == "bye":
            if self.recording:
                self.backend.on_episode("discard", self.episode, {"reason": "bye"})
                self._mark_discarded(self.episode, "sender said bye mid-episode")
            self.owner = None
            self.session = None
            self.events.event("owner_released", addr=list(src))
            print("[rx] owner released")
            ack = encode_ctl_ack(msg, True, "released")
        else:
            ack = encode_ctl_ack(msg, False, f"unknown action {action!r}")

        self.seen_ctl[key] = ack
        if len(self.seen_ctl) > 4096:
            for k in list(self.seen_ctl)[:2048]:
                del self.seen_ctl[k]
        self._send(ack, src)

    # ----------------------------------------------------------------- loop
    def run(self):
        # Only the main thread may install handlers; tools/loopback_test.py
        # runs the receiver in a worker and stops it by setting .stop directly.
        if threading.current_thread() is threading.main_thread():
            for s in (signal.SIGINT, signal.SIGTERM):
                signal.signal(s, lambda *_: setattr(self, "stop", True))
        self.events.event("start", epoch=epoch(), backend=self.backend.name,
                          readback=self.backend.readback,
                          joints=self.backend.joint_names(),
                          expect_map_sha=self.expect_map_sha,
                          port=self.sock.getsockname()[1], fps=self.fps)
        print(f"[rx] listening on {self.sock.getsockname()[0]}:"
              f"{self.sock.getsockname()[1]}  backend={self.backend.name} "
              f"readback={self.backend.readback}")
        print(f"[rx] logging to {self.run_dir}   Ctrl+C to stop")

        next_tick = time.monotonic()
        pending, pend_t, pend_sup, pend_addr = None, None, [], None
        while not self.stop:
            now = time.monotonic()
            timeout = max(0.0, next_tick - now)
            try:
                select.select([self.sock], [], [], timeout)
            except (OSError, ValueError):
                pass

            # Drain on EVERY wake, not only on the tick, so arrival time is
            # real. Control messages are handled the moment they land: an
            # episode boundary must not wait for a physics tick.
            newest, t_arrive, superseded, ctls, addr = self._drain()
            for msg, src in ctls:
                self._handle_ctl(msg, src)
            if newest is not None:
                if pending is not None:
                    older, newer = ((pending, newest)
                                    if newest["seq"] > pending["seq"]
                                    else (newest, pending))
                    pend_sup.append(older["seq"])
                    if newer is newest:
                        pending, pend_t, pend_addr = newest, t_arrive, addr
                else:
                    pending, pend_t, pend_addr = newest, t_arrive, addr
                pend_sup.extend(superseded)

            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick += self.period
            if next_tick < now:          # backend overran; do not busy-spin
                next_tick = now + self.period
            newest, t_arrive, superseded, addr = (pending, pend_t,
                                                  pend_sup, pend_addr)
            pending, pend_t, pend_sup, pend_addr = None, None, [], None

            if newest is not None and self.owner is None:
                # A command stream with no `hello` behind it. Allowed only on
                # request: silently accepting one means a p3 with a stale or
                # unverified map can drive and record with nothing on this side
                # checked at all.
                if self.allow_anonymous and addr is not None:
                    self.owner = addr
                    self.events.event("owner_claimed", addr=list(addr),
                                      implicit=True)
                else:
                    self.n_foreign += 1
                    if self.n_foreign in (1, 100) or self.n_foreign % 900 == 0:
                        print("[rx] ignoring commands from an unannounced "
                              "sender (no hello). Start p3 normally, or pass "
                              "--allow-anonymous.")
                    newest = None
            if newest is not None and self.expect_map_sha and \
                    newest.get("map_sha256") != self.expect_map_sha:
                self.n_bad_map += 1
                if self.n_bad_map == 1:
                    self.events.event("map_sha_mismatch",
                                      expect=self.expect_map_sha,
                                      got=newest.get("map_sha256"))
                    print("[rx] REFUSING commands: map sha does not match "
                          f"(expect {self.expect_map_sha[:12]}, got "
                          f"{str(newest.get('map_sha256'))[:12]})")
                newest = None

            t_apply0 = time.monotonic()
            if newest is not None:
                self.n_superseded += len(superseded)
                self.t_last_cmd = t_apply0
                if newest["seq"] > self.max_seq:
                    self.max_seq = newest["seq"]
                applied, clipped = self.backend.apply(newest.get("joints_rad", {}))
                self.backend.step()
                # The ack must carry MEASURED state wherever the backend has
                # it: a target written before the step is still just our own
                # number handed back, which verifies nothing.
                measured = self.backend.read()
                if measured:
                    applied = measured
                t_apply1 = time.monotonic()
                self.last_applied = applied
                self.n_applied += 1
                if self.recording:
                    self.ep_applied += 1
                wait_ms = ((t_apply0 - t_arrive) * 1e3
                           if t_arrive is not None else None)
                ack = encode_ack(newest, applied, clipped,
                                 sim_time=self.backend.sim_time(),
                                 superseded=superseded)
                ack["wait_ms"] = (round(wait_ms, 3) if wait_ms is not None else None)
                ack["apply_ms"] = round((t_apply1 - t_apply0) * 1e3, 3)
                self._send(ack, self.owner or addr)
                self.rows.write({
                    "schema": ACK_SCHEMA, "seq": newest["seq"], **stamp(),
                    "episode": self.episode, "recording": self.recording,
                    "sender_episode": newest.get("episode"),
                    "sender_recording": newest.get("recording"),
                    "requested_rad": newest.get("joints_rad", {}),
                    "applied_rad": applied, "clipped": clipped,
                    "clipped_local": newest.get("clipped_local", []),
                    "readback": self.backend.readback,
                    "sim_time": self.backend.sim_time(),
                    "n_superseded": len(superseded),
                    "wait_ms": ack["wait_ms"],
                    "dt_apply_ms": round((t_apply1 - t_apply0) * 1e3, 3)})
            else:
                self.backend.step()
                if self.t_last_cmd is not None:
                    gap = now - self.t_last_cmd
                    if self.recording and gap > self.stale_s:
                        self.n_stale += 1
                        if self.n_stale in (1, 10, 100) or self.n_stale % 300 == 0:
                            self.events.event("stale", gap_s=round(gap, 3),
                                              episode=self.episode)
                            print(f"[rx] no command for {gap:.2f}s while "
                                  f"recording episode {self.episode}")
                    if self.recording and gap > self.episode_timeout_s:
                        self.backend.on_episode("discard", self.episode,
                                                {"reason": "sender went silent"})
                        print(f"[rx] DISCARDED episode {self.episode}: sender "
                              f"silent for {gap:.1f}s")
                        self._mark_discarded(self.episode,
                                             f"no command for {gap:.1f}s")
        self._shutdown()

    def _shutdown(self):
        if self.recording:
            self.backend.on_episode("discard", self.episode,
                                    {"reason": "receiver stopped"})
            print(f"[rx] DISCARDED open episode {self.episode} (receiver stopped)")
            self._mark_discarded(self.episode, "receiver stopped mid-episode")
        summary = {"cmds_received": self.n_cmd, "applied": self.n_applied,
                   "superseded": self.n_superseded, "foreign_dropped": self.n_foreign,
                   "map_sha_refused": self.n_bad_map, "malformed": self.n_malformed,
                   "stale_ticks": self.n_stale, "max_seq": self.max_seq,
                   "episodes": dict(self.episode_result)}
        self.events.event("stop", **summary)
        print("\n[rx] " + json.dumps(summary))
        print(f"[rx] logs in {self.run_dir}")
        try:
            self.backend.close()
        finally:
            self.rows.close()
            self.events.close()
            self.sock.close()


def build_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", default="echo", choices=("echo", "isaac"))
    ap.add_argument("--port", type=int, default=DEFAULT_CMD_PORT)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--fps", type=float, default=30.0,
                    help="tick rate; the backend may still be slower")
    ap.add_argument("--map", default=None,
                    help="the simmap JSON the Jetson is using. Used to pin the "
                         "expected map sha and to read sim_target.dof_names")
    ap.add_argument("--usd", default=None, help="isaac backend: USD to load")
    ap.add_argument("--episode-timeout-s", type=float, default=5.0)
    ap.add_argument("--stale-ms", type=float, default=500.0)
    ap.add_argument("--out", default="./logs/p3_sim")
    ap.add_argument("--takeover", action="store_true",
                    help="let a new sender seize the sim from the current owner")
    ap.add_argument("--allow-anonymous", action="store_true",
                    help="accept a command stream that never said hello")
    ap.add_argument("--echo-hold-ms", type=float, default=0.0,
                    help="echo backend only: fake per-step work, to exercise "
                         "the supersede path without a real simulator")
    return ap


def main():
    args = build_args().parse_args()

    expect_sha, dof_names, sim_target = None, None, {}
    if args.map:
        from arm.sim_mapping import SimMap
        m = SimMap.from_file(args.map)
        expect_sha = m.sha256()
        sim_target = m.doc.get("sim_target") or {}
        dof_names = sim_target.get("dof_names")
        print(f"[rx] map {args.map}  sha {expect_sha[:12]}  "
              f"role {m.role()}  verified {m.is_verified()}")

    if args.fps <= 0:
        raise SystemExit("--fps must be > 0")
    if not args.map:
        print("=" * 70)
        print("[rx] NO --map GIVEN. Nothing on this side checks which mapping")
        print("     the sender is using, so a stale, unverified or wrong-arm")
        print("     map would drive and record with no complaint. Pass --map")
        print("     unless you are deliberately testing the transport.")
        print("=" * 70)

    from sim.backend import make_backend
    if args.backend == "echo":
        backend = make_backend("echo", hold_s=args.echo_hold_ms / 1e3)
    else:
        backend = make_backend("isaac", usd=args.usd or sim_target.get("usd"),
                               dof_names=dof_names, fps=args.fps)

    Receiver(backend, port=args.port, bind=args.bind, fps=args.fps,
             expect_map_sha=expect_sha, episode_timeout_s=args.episode_timeout_s,
             stale_ms=args.stale_ms, out=args.out, takeover=args.takeover,
             allow_anonymous=args.allow_anonymous).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
