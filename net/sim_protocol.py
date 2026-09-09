"""Wire format for driving a simulated SO-101 from the Jetson.

    Jetson (leader, real)  --UDP 9871-->  Spark (Isaac Sim, virtual SO-101)
                           <--UDP 9872--  acknowledgement

Four design decisions worth stating, because each prevents a specific bug.

1. ** The ack echoes t_send_mono verbatim. **
   Round-trip time is therefore computed entirely on the JETSON's clock. No NTP,
   no clock discipline between machines, nothing to drift. Cross-machine
   timestamps are a classic source of numbers that look fine and are wrong.

2. ** The ack carries back what the sim ACTUALLY applied, and what it clipped. **
   Without it a wrong joint mapping is invisible: the virtual arm moves, nothing
   errors, and the dataset is quietly worthless. With it, every step is checked
   against what we asked for, and a clipped joint is reported the moment it
   happens.

3. ** Superseded is not the same as lost. **
   Isaac will not always keep up with 30 Hz. A receiver that queues commands
   grows its own latency without bound, so ours drains the socket and applies
   only the NEWEST command each tick. The ones it skipped are named in
   `superseded` so they are counted as "we sent faster than the sim ran",
   NOT as "the network dropped packets". Two different problems, two different
   fixes; conflating them would send us debugging the wrong one.

4. ** Episode boundaries are RELIABLE, joint commands are not. **
   Losing one joint command at 30 Hz means the sim holds its target for 33 ms --
   harmless. Losing "episode 7 ended" means episode 7 and episode 8 are welded
   into one file and nobody finds out until training. So control messages carry
   their own sequence, are retransmitted until acknowledged, and are idempotent
   on the receiver (`ctl_seq` already seen -> re-ack, do not re-execute).

5. ** Every run carries a fresh `session` nonce. **
   Idempotency and a program restart look identical without one: p3 binds a
   fixed source port and its ctl_seq always starts at 1, so a restarted p3's
   `hello` is byte-for-byte a retransmission of the old one. The receiver would
   replay the cached "you are the owner" and "episode 1 started" answers, p3
   would print KEPT, and the simulator would have recorded nothing at all. The
   nonce makes a new run a new run.

UDP is deliberate. Sequence numbers make the loss rate MEASURABLE rather than
assumed, which is the property that actually matters.
"""
import json
import math
import statistics as st
import time
import uuid

CMD_SCHEMA = "so101.simcmd.v1"
ACK_SCHEMA = "so101.simack.v1"
CTL_SCHEMA = "so101.simctl.v1"
CTLACK_SCHEMA = "so101.simctlack.v1"

DEFAULT_CMD_PORT = 9871
DEFAULT_ACK_PORT = 9872

# Control actions. The receiver must treat every one of these as idempotent.
CTL_ACTIONS = ("hello", "episode_start", "episode_end", "episode_discard",
               "ping", "bye")


def encode_cmd(seq, t_send_mono, joints_rad, map_sha256, clipped_local=None,
               episode=None, recording=False):
    """One joint command.

    `episode` / `recording` are repeated in EVERY step on purpose. The episode
    boundary messages are the authority, but stamping each step too means a row
    can be attributed to an episode from the row alone, without replaying the
    control stream. Belt and braces, and it costs 20 bytes.
    """
    return {
        "schema": CMD_SCHEMA,
        "seq": int(seq),
        "t_send_mono": float(t_send_mono),
        "map_sha256": map_sha256,
        "joints_rad": {k: float(v) for k, v in joints_rad.items()},
        "clipped_local": list(clipped_local or ()),
        "episode": episode,
        "recording": bool(recording),
    }


def encode_ack(cmd, applied_rad, clipped, sim_time=None, superseded=None):
    """Built on the SIM side. `t_send_mono` is echoed untouched -- do not
    replace it with a local time, that is the whole point.

    `superseded`: seqs that arrived but were skipped because a newer command was
    already in the socket buffer when we drained it.
    """
    return {
        "schema": ACK_SCHEMA,
        "seq": int(cmd["seq"]),
        "t_send_mono": float(cmd["t_send_mono"]),
        "applied_rad": {k: float(v) for k, v in applied_rad.items()},
        "clipped": list(clipped or ()),
        "sim_time": sim_time,
        "superseded": [int(s) for s in (superseded or ())],
    }


def encode_ctl(ctl_seq, action, session=None, **fields):
    if action not in CTL_ACTIONS:
        raise ValueError(f"unknown control action {action!r}")
    d = {"schema": CTL_SCHEMA, "ctl_seq": int(ctl_seq), "action": action,
         "session": session, "t_send_mono": time.monotonic()}
    d.update(fields)
    return d


def encode_ctl_ack(ctl, ok, detail="", **fields):
    """The ack echoes session / action / episode so the sender can prove the
    answer belongs to the question it asked."""
    d = {"schema": CTLACK_SCHEMA, "ctl_seq": int(ctl["ctl_seq"]),
         "session": ctl.get("session"), "action": ctl.get("action"),
         "episode": ctl.get("episode"),
         "ok": bool(ok), "detail": str(detail)}
    d.update(fields)
    return d


def dumps(obj):
    return json.dumps(obj, separators=(",", ":")).encode()


def loads(buf):
    return json.loads(buf.decode())


class LinkStats:
    """Sequence, loss and round-trip accounting for the command link."""

    def __init__(self, lost_after_s=1.0, keep_rtt=2000, mismatch_tol_rad=0.10):
        """`mismatch_tol_rad` is how far the simulator's MEASURED joint may sit
        from the target before we call it a mismatch.

        It is not a numerical epsilon. A position-driven articulation never
        lands exactly on its target -- finite drive stiffness plus a step of lag
        -- so a 1e-6 tolerance would flag every single step against real Isaac
        and the warning would be trained out of the operator within a day. What
        we are trying to catch is a WRONG JOINT MAP or a wrong DOF name, and
        those are wrong by tens of degrees, not by 0.1 rad (5.7 deg).
        """
        self.lost_after_s = lost_after_s
        self.mismatch_tol_rad = float(mismatch_tol_rad)
        self.sent = 0
        self.acked = 0
        self.lost = 0
        self.send_failed = 0       # sendto() refused; never reached the wire
        self.superseded = 0        # sim could not keep up; NOT a network loss
        self.out_of_order = 0
        self.mismatched = 0        # sim applied something we did not ask for
        self.max_dev_rad = 0.0
        self.max_dev_joint = None
        self.clipped_steps = 0
        self.rtt_ms = []
        self._keep = keep_rtt
        self._inflight = {}        # seq -> t_send_mono
        self._max_ack_seq = -1

    def on_send(self, seq, t_send_mono, sent_rad=None):
        """Stash what was sent WITH this seq.

        The mismatch check has to compare an ack against the command that
        produced it. Comparing it against whatever we happen to be sending when
        it arrives flags every step as a mismatch, because at 30 Hz the ack for
        step N lands during step N+1 and the arm has moved. (Caught by the
        end-to-end test, which is the only reason it is not still in here.)
        """
        self.sent += 1
        self._inflight[seq] = (t_send_mono, sent_rad)

    def on_send_failed(self, seq):
        """A datagram that sendto() refused. Counted, because a command that
        never reached the wire is neither acked nor lost, and would otherwise
        vanish from the books entirely -- taking a hole in the sim's motion with
        it while loss_rate still read 0."""
        self.sent += 1
        self.send_failed += 1

    def on_ack(self, ack, now_mono, sent_rad=None, tol_rad=None):
        self.on_superseded(ack.get("superseded"))
        seq = ack["seq"]
        ent = self._inflight.pop(seq, None)
        if ent is None:
            return None                      # duplicate, or already expired
        t0, stashed = ent
        if sent_rad is None:
            sent_rad = stashed
        self.acked += 1
        rtt = (now_mono - t0) * 1e3
        self.rtt_ms.append(rtt)
        if len(self.rtt_ms) > self._keep:
            del self.rtt_ms[0]
        if seq < self._max_ack_seq:
            self.out_of_order += 1
        else:
            self._max_ack_seq = seq
        if ack.get("clipped"):
            self.clipped_steps += 1
        if sent_rad is not None:
            tol = self.mismatch_tol_rad if tol_rad is None else tol_rad
            applied = ack.get("applied_rad", {})
            clipped = ack.get("clipped", ())
            worst_j, worst = None, 0.0
            for j, v in sent_rad.items():
                if j not in applied or j in clipped:
                    continue
                d = abs(applied[j] - v)
                if d > worst:
                    worst_j, worst = j, d
            if worst > self.max_dev_rad:
                self.max_dev_rad, self.max_dev_joint = worst, worst_j
            if worst > tol:
                self.mismatched += 1
        return rtt

    def on_superseded(self, seqs):
        """Retire seqs the sim admits it skipped, so they never age into `lost`."""
        n = 0
        for s in (seqs or ()):
            if self._inflight.pop(int(s), None) is not None:
                n += 1
        self.superseded += n
        return n

    def expire(self, now_mono):
        """Anything unanswered for longer than lost_after_s counts as lost."""
        dead = [s for s, (t, _r) in self._inflight.items()
                if now_mono - t > self.lost_after_s]
        for s in dead:
            del self._inflight[s]
        self.lost += len(dead)
        return len(dead)

    def summary(self):
        d = {"sent": self.sent, "acked": self.acked, "lost": self.lost,
             "send_failed": self.send_failed,
             "superseded": self.superseded,
             "in_flight": len(self._inflight),
             "out_of_order": self.out_of_order,
             "mismatched": self.mismatched,
             "max_dev_deg": round(math.degrees(self.max_dev_rad), 3),
             "max_dev_joint": self.max_dev_joint,
             "clipped_steps": self.clipped_steps}
        if self.rtt_ms:
            xs = sorted(self.rtt_ms)
            d["rtt_ms_median"] = round(st.median(xs), 3)
            d["rtt_ms_p95"] = round(xs[min(len(xs) - 1, int(0.95 * (len(xs) - 1)))], 3)
            d["rtt_ms_max"] = round(xs[-1], 3)
        d["loss_rate"] = (round(self.lost / self.sent, 5) if self.sent else None)
        d["supersede_rate"] = (round(self.superseded / self.sent, 5)
                               if self.sent else None)
        return d


class CtlTracker:
    """Retransmit-until-acked bookkeeping for control messages. Owns no socket.

    The caller drives it: `start()` a request, then call `due()` each loop and
    resend whatever it hands back, and `on_ack()` when an ack arrives. Kept
    socket-free so it is unit-testable without a network.
    """

    def __init__(self, timeout_s=0.15, max_tries=20, session=None):
        self.timeout_s = timeout_s
        self.max_tries = max_tries
        self.session = session or uuid.uuid4().hex[:12]
        self._pending = {}          # ctl_seq -> [msg, tries, t_last]
        self.acked = {}             # ctl_seq -> ack
        self.failed = []            # ctl_seqs that ran out of tries
        self.rejected = 0           # acks that did not match what we asked
        self._next = 1

    def start(self, action, **fields):
        seq = self._next
        self._next += 1
        msg = encode_ctl(seq, action, session=self.session, **fields)
        self._pending[seq] = [msg, 0, 0.0]
        return seq, msg

    def cancel(self, seq):
        """Stop retransmitting a request the caller has given up on.

        Without this a retransmit can still be answered AFTER `request()` has
        already told the operator the message failed -- so the sim would start
        or end an episode that p3 has reported as not saved."""
        if seq in self._pending:
            del self._pending[seq]
            if seq not in self.acked:
                self.failed.append(seq)

    def due(self, now_mono):
        """Messages that should be (re)sent right now. Marks them as sent."""
        out = []
        for seq, ent in list(self._pending.items()):
            msg, tries, t_last = ent
            if tries == 0 or (now_mono - t_last) >= self.timeout_s:
                if tries >= self.max_tries:
                    del self._pending[seq]
                    self.failed.append(seq)
                    continue
                ent[1] = tries + 1
                ent[2] = now_mono
                out.append(msg)
        return out

    def on_ack(self, ack):
        """Accept an ack only if it answers the question we actually asked."""
        seq = int(ack.get("ctl_seq", -1))
        if ack.get("session") != self.session:
            self.rejected += 1          # someone else's run, or a stale replay
            return None
        want = self._pending.get(seq)
        if want is not None:
            msg = want[0]
            if (ack.get("action") != msg.get("action")
                    or ack.get("episode") != msg.get("episode")):
                self.rejected += 1
                return None
            del self._pending[seq]
        elif seq in self.acked:
            return seq                  # duplicate ack for something settled
        else:
            self.rejected += 1
            return None
        self.acked[seq] = ack
        if len(self.acked) > 4096:
            for k in sorted(self.acked)[:2048]:
                del self.acked[k]
        return seq

    def is_settled(self, seq):
        return seq in self.acked or seq in self.failed

    def result(self, seq):
        """(ok, detail). A message that ran out of tries is a hard failure."""
        if seq in self.acked:
            a = self.acked[seq]
            return bool(a.get("ok")), a.get("detail", "")
        if seq in self.failed:
            return False, f"no ack after {self.max_tries} attempts"
        return None, "pending"

    def pending_count(self):
        return len(self._pending)
