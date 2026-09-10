#!/usr/bin/env python3
"""Program 3 -- drive the SIMULATED SO-101 with the REAL leader arm.

    [Jetson]  leader (real, moved by hand)
        read six joints  ->  rate limit  ->  simmap  ->  radians
        --UDP 9871-->  [Spark]  sim/receiver.py  ->  Isaac  ->  virtual SO-101
        <--UDP 9872--  ack: what the sim ACTUALLY applied

Why this program exists at all: every off-the-shelf teleop-into-simulator script
assumes the leader is plugged into the machine running the simulator. Ours is
not -- the leader is on the Jetson and Isaac is on Spark. That network hop is
the entire reason for program 3, and it is also the thing that has to be
measured rather than assumed.

** The real follower is not touched. ** p3 runs the leader alone (`solo`
pre-flight). If the follower is plugged in and still powered from a previous
run, it simply holds wherever it was; nothing here sends it a goal.

The three ways this can silently produce a worthless dataset, and the guard for
each:

  1. Wrong joint mapping.  -> The map is a FITTED, VERIFIED, calibration-locked
     file (arm/sim_mapping.py), and every step is checked against what the sim
     says it applied. A map that is stale, unverified, or fitted for the other
     arm is refused before the first command.
  2. A gap in the middle of an episode.  -> Link loss, a leader read failure or
     a stalled sim while recording DISCARDS the episode. A demonstration with a
     hole in it is not a demonstration. (This is the p4 tier of the fault
     policy: p1 stops and waits because a stuck arm is a hazard; p2 continues
     and marks because a dropped camera frame is recoverable; p3 and p4 discard
     because a corrupt episode poisons training and nothing later can find it.)
  3. The sim silently running behind.  -> Superseded commands are counted apart
     from lost ones, so "Isaac cannot keep up" never gets misread as "the
     network is dropping packets".

Keyboard (type the letter, then Enter):
    s   start an episode          e   end and KEEP the episode
    d   discard the open episode  q   quit  (q twice if an episode is open)
    Enter  status
"""
import argparse
import math
import os
import queue
import select
import signal
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm.control import rate_limit, step_limits_for                    # noqa: E402
from arm.preflight import JOINTS, run_preflight, strip_pos                     # noqa: E402
from arm.sim_mapping import SPAN_WARN, SimMap                          # noqa: E402
from arm.signal_pub import SignalPublisher                             # noqa: E402
from arm.units import UNITS, per_joint                                 # noqa: E402
from common.clock import epoch, stamp                                  # noqa: E402
from common.jsonl import JsonlWriter                                   # noqa: E402
from net.sim_protocol import (ACK_SCHEMA, CTLACK_SCHEMA,               # noqa: E402
                              DEFAULT_ACK_PORT, DEFAULT_CMD_PORT,
                              CtlTracker, LinkStats, dumps, encode_cmd,
                              loads)

ROW_SCHEMA = "so101.simstep.v1"
# Written into rows.jsonl itself, not only into events.jsonl. A loader that
# groups rows by `episode` -- the obvious way to build a dataset -- would
# otherwise train on discarded episodes, because discarding writes a line to a
# different file and removes nothing.
EPISODE_SCHEMA = "so101.episode.v1"


# --------------------------------------------------------------------- input
class Console(threading.Thread):
    """Line-oriented keyboard commands on a queue. No termios, no curses --
    this has to work in a plain AnyDesk terminal with no tty tricks.

    ** What end-of-input means depends on where stdin came from. ** At a
    terminal, EOF is Ctrl+D and means quit. Under `nohup ... &` or with stdin
    redirected from /dev/null, EOF arrives instantly and means only that there
    is no keyboard -- reading THAT as "the operator pressed quit" made p3 exit
    after zero steps and print a tidy summary of nothing, which looks exactly
    like a working program that had nothing to do. A pipe is a third case: not
    a terminal, but something may well be feeding it, so the keyboard works
    right up until it closes.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.q = queue.Queue()
        self.died = None
        self.eof = False

    def run(self):
        try:
            tty = bool(sys.stdin) and sys.stdin.isatty()
        except (ValueError, AttributeError):
            tty = False
        while True:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                if tty:
                    self.q.put("q")        # Ctrl+D at a real terminal
                self.eof = True
                return
            except Exception as e:
                # e.g. backgrounded and the shell denies the read (EIO/SIGTTIN).
                # Give up on the keyboard; do NOT interpret it as a command.
                self.died = repr(e)
                return
            self.q.put(line.strip().lower())

    def get(self):
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return None


# ----------------------------------------------------------------------- net
class SimLink:
    """One UDP socket, bound to the ack port, used for both directions.

    Binding the SEND socket to the ack port is deliberate: the receiver then
    simply replies to the datagram's source address and never has to be told the
    Jetson's IP. One less thing to configure wrongly, and it works through a
    changed DHCP lease.
    """

    def __init__(self, host, cmd_port=DEFAULT_CMD_PORT,
                 bind_port=DEFAULT_ACK_PORT, mismatch_tol_rad=0.10):
        self.dst = (host, int(cmd_port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(("0.0.0.0", int(bind_port)))
        self.sock.setblocking(False)
        self.ctl = CtlTracker()
        self.stats = LinkStats(mismatch_tol_rad=mismatch_tol_rad)
        self.t_last_ack = None
        self.last_ack = None

    def send(self, obj):
        try:
            self.sock.sendto(dumps(obj), self.dst)
            return True
        except OSError:
            return False

    def pump(self, now, cap=64):
        """Drain the socket: acks into LinkStats, ctl-acks into the tracker.

        Returns (ack, rtt) pairs. The rtt must travel WITH its ack: reading
        `stats.rtt_ms[-1]` afterwards gives the last round trip processed in
        the batch, which is a different command whenever two acks arrive in one
        window, an ack comes back out of order, or an expired one is dropped.
        The row would then carry a latency belonging to another step -- and
        latency is a deliverable here, not a debug print.
        """
        got = []
        for _ in range(cap):
            try:
                buf, _src = self.sock.recvfrom(65535)
            except OSError:
                break
            try:
                msg = loads(buf)
            except Exception:
                continue
            schema = msg.get("schema")
            if schema == ACK_SCHEMA:
                rtt = self.stats.on_ack(msg, now)
                self.t_last_ack = now
                got.append((msg, rtt))
            elif schema == CTLACK_SCHEMA:
                self.ctl.on_ack(msg)
                self.t_last_ack = now
        for msg in self.ctl.due(now):
            self.send(msg)
        self.stats.expire(now)
        return got

    def pump_until(self, deadline, want_seq=None):
        """Collect acks until `deadline`, sleeping in select() rather than in
        time.sleep().

        This costs nothing -- that time was going to be spent waiting for the
        next tick anyway -- and it buys a HONEST round-trip time. Polling once
        per tick instead would report the RTT as one whole tick no matter how
        fast the simulator actually answered, because the ack for step N would
        not be read until step N+1. The number would look plausible and would be
        measuring our own loop, which is exactly the kind of latency figure this
        program exists to avoid producing.
        """
        acks, want_ack, want_rtt = [], None, None

        def take(pairs):
            nonlocal want_ack, want_rtt
            for msg, rtt in pairs:
                acks.append(msg)
                if want_seq is not None and msg.get("seq") == want_seq:
                    want_ack, want_rtt = msg, rtt

        while True:
            now = time.monotonic()
            left = deadline - now
            if left <= 0:
                break
            try:
                ready, _, _ = select.select([self.sock], [], [], left)
            except (OSError, ValueError):
                break
            if not ready:
                break
            take(self.pump(time.monotonic()))
        # one last non-blocking sweep so retransmits and expiry still run
        take(self.pump(time.monotonic()))
        return acks, want_ack, want_rtt

    def request(self, action, timeout_s=3.0, **fields):
        """Fire a control message and block until it is acked or gives up."""
        seq, msg = self.ctl.start(action, **fields)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            now = time.monotonic()
            self.pump(now)
            if self.ctl.is_settled(seq):
                self.last_ack = self.ctl.acked.get(seq)
                return self.ctl.result(seq)
            time.sleep(0.01)
        # Stop retransmitting: otherwise a late retry could still be answered
        # AFTER we have told the operator this message failed, and the sim
        # would start or end an episode p3 has already reported as not saved.
        self.ctl.cancel(seq)
        self.last_ack = None
        return None, f"timed out after {timeout_s}s"

    def close(self):
        self.sock.close()


# ---------------------------------------------------------------------- args
def build_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--leader-port", default=os.environ.get("LEADER"))
    ap.add_argument("--leader-id", default="my_leader")
    ap.add_argument("--leader-serial", default=os.environ.get("LEADER_SERIAL"))
    ap.add_argument("--map", required=True, help="simmap JSON (tools/simmap_init.py)")
    ap.add_argument("--sim", default=None,
                    help="host running sim/receiver.py. Omit with --no-sim")
    ap.add_argument("--no-sim", action="store_true",
                    help="map and log locally with no network at all -- checks "
                         "the arm and the map on the Jetson alone")
    ap.add_argument("--cmd-port", type=int, default=DEFAULT_CMD_PORT)
    ap.add_argument("--ack-port", type=int, default=DEFAULT_ACK_PORT)
    # 30 Hz, not p1/p2's 120: there is a simulator on the other end and it
    # applies at ITS tick. Sending faster than the sim ticks does not move the
    # arm sooner, it just makes more commands arrive between ticks and get
    # superseded -- which p3 counts, so you can see it. Raise it once the sim's
    # own rate is known; the step limit follows automatically.
    ap.add_argument("--fps", type=float, default=30.0,
                    help="leader sampling rate. Match the simulator's tick "
                         "rather than raising it blindly")
    ap.add_argument("--max-step-deg", type=float, default=None,
                    help="max BODY joint move per step. Derived from --fps to "
                         "hold a constant deg/s if not given")
    ap.add_argument("--max-step-gripper-pct", type=float, default=None)
    ap.add_argument("--link-timeout-ms", type=float, default=1000.0,
                    help="no ack for this long while recording -> discard episode")
    ap.add_argument("--read-strikes", type=int, default=5,
                    help="consecutive leader read failures before a hard fault")
    ap.add_argument("--allow-unverified", action="store_true",
                    help="run with a map that has no verified block. Allowed for "
                         "a shakedown; NEVER for a dataset")
    ap.add_argument("--allow-span", action="store_true",
                    help="run with a map whose span ratio failed. Almost always "
                         "the wrong answer -- fix the USD instead")
    ap.add_argument("--out", default="./logs/p3")
    ap.add_argument("--udp", default=None,
                    help='also stream rows to "host:port" for a live monitor')
    ap.add_argument("--duration", type=float, default=0.0)
    ap.add_argument("--mismatch-tol-deg", type=float, default=5.7,
                    help="how far the sim's MEASURED joint may sit from the "
                         "target before it counts as a mismatch. Sized to "
                         "catch a wrong map, not normal drive tracking error")
    ap.add_argument("--no-console", action="store_true",
                    help="never read stdin, even at a terminal")
    return ap


# ---------------------------------------------------------------------- main
def main():
    ap = build_args()
    args = ap.parse_args()
    if not args.leader_port:
        ap.error("need --leader-port (or the LEADER env var)")
    if not args.sim and not args.no_sim:
        ap.error("need --sim <host>, or --no-sim to run without a simulator")

    simmap = SimMap.from_file(args.map)
    from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
    leader = SOLeader(SOLeaderTeleopConfig(
        port=args.leader_port, id=args.leader_id, use_degrees=True))

    run_dir = os.path.join(args.out, time.strftime("%Y%m%d-%H%M%S"))
    rows = JsonlWriter(os.path.join(run_dir, "rows.jsonl"))
    events = JsonlWriter(os.path.join(run_dir, "events.jsonl"))
    pub = SignalPublisher(udp_addr=args.udp)
    map_sha = simmap.sha256()
    events.event("start", argv=sys.argv[1:], epoch=epoch(), run_dir=run_dir,
                 units=UNITS, map_path=os.path.abspath(args.map),
                 map_sha256=map_sha, map_role=simmap.role(),
                 map_arm_id=simmap.arm_id(), map_verified=simmap.is_verified(),
                 map_verification=simmap.verification_note(),
                 sim=args.sim, no_sim=bool(args.no_sim))
    print(f"[p3] logging to {run_dir}")
    print(f"[p3] map {args.map}")
    print(f"[p3]     sha {map_sha[:12]}  role {simmap.role()}  "
          f"arm {simmap.arm_id()}")
    print(f"[p3]     {simmap.verification_note()}")

    def bail(code, *msg):
        for m in msg:
            print(m)
        try:
            leader.disconnect()
        except Exception:
            pass
        events.event("stop", reason="preflight", code=code)
        rows.close()
        events.close()
        pub.close()
        return code

    # ---- guard the MAP against the LEADER's calibration, before any I/O ----
    ok, reasons = simmap.guard(leader.calibration, expect_role="leader",
                               allow_unverified=args.allow_unverified,
                               allow_span=args.allow_span)
    events.event("map_guard", ok=ok, reasons=reasons,
                 span=simmap.worst_span_ratio())
    if not ok:
        print("\n[p3] MAP REFUSED:")
        for r in reasons:
            print("      - " + r)
        return bail(1, "\n[p3] refit with tools/simmap_init.py, then retry.")
    worst, wj = simmap.worst_span_ratio()
    if worst is not None and worst > SPAN_WARN:
        print(f"[p3] WARNING: {wj} span ratio off by {worst*100:.1f}% -- the "
              f"real arm and the model do not quite agree on range of motion")
    if not simmap.is_verified():
        print(f"[p3] WARNING: {simmap.verification_note()}. "
              f"Shakedown only, not a dataset.")

    # ---- pre-flight, leader only ------------------------------------------
    print("[p3] pre-flight (leader only; the real follower is not touched) ...")
    report, info = run_preflight(leader, None, args.leader_port, None,
                                 expect_leader_serial=args.leader_serial)
    print(report.render())
    events.event("preflight", ok=report.ok, checks=report.rows, info=info)
    if not report.ok:
        return bail(1, "\n[p3] PRE-FLIGHT FAILED -- not starting.")
    if not args.leader_serial:
        print("[p3] NOTE: LEADER_SERIAL is not set, so check [3] cannot prove "
              "this is the arm the map was fitted for. `source devices.env` to "
              "arm it. (is_calibrated would still catch a different arm.)")



    # ---- handshake ---------------------------------------------------------
    link = None
    if not args.no_sim:
        try:
            link = SimLink(args.sim, args.cmd_port, args.ack_port,
                           mismatch_tol_rad=math.radians(args.mismatch_tol_deg))
        except OSError as e:
            return bail(1, f"\n[p3] cannot bind UDP :{args.ack_port} -- "
                           f"another p3 already running? ({e})")
        print(f"[p3] hello -> {args.sim}:{args.cmd_port} ...")
        ok, detail = link.request("hello", map_sha256=map_sha,
                                  map_role=simmap.role(),
                                  joints=list(simmap.joints))
        events.event("handshake", ok=ok, detail=detail)
        if ok is None:
            link.close()
            return bail(1, f"\n[p3] no answer from {args.sim}:{args.cmd_port}.",
                        "      Is sim/receiver.py running there? Firewall?",
                        f"      ({detail})")
        if not ok:
            link.close()
            return bail(1, f"\n[p3] the simulator refused us: {detail}")
        ack = link.last_ack or {}
        print(f"[p3] connected. backend={ack.get('backend')} "
              f"readback={ack.get('readback')}")
        if ack.get("readback") is False:
            print("[p3] NOTE: this backend echoes our own numbers back. The link "
                  "is proven; the MAPPING is not.")

    # ---- run ---------------------------------------------------------------
    stop = {"why": None}

    def _sig(signum, _f):
        stop["why"] = f"signal:{signal.Signals(signum).name}"
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    try:
        has_tty = bool(sys.stdin) and sys.stdin.isatty()
    except (ValueError, AttributeError):
        has_tty = False
    console = None
    if not args.no_console:
        console = Console()
        console.start()

    print("\n[p3] RUNNING. Move the LEADER; the VIRTUAL arm follows.")
    if console is not None and has_tty:
        print("[p3]   s = start episode   e = end+keep   d = discard   q = quit")
        print("[p3]   Enter = status\n")
    elif console is not None:
        print("[p3]   stdin is not a terminal. Episode commands (s/e/d/q) still "
              "work if something is feeding stdin; if nothing is, this run has "
              "no episodes.")
        if args.duration > 0:
            print(f"[p3]   stopping by itself after {args.duration:.0f}s.\n")
        else:
            print("[p3]   and nothing else will stop it -- use SIGTERM, or pass "
                  "--duration.\n")
        events.event("no_tty", duration=args.duration)
    else:
        print("[p3]   keyboard control is OFF (--no-console): no episodes.")
        if args.duration <= 0:
            print("[p3]   stop it with SIGINT/SIGTERM.\n")
        events.event("console_disabled", duration=args.duration)

    # Same derivation as p1 and p2: the limit is per STEP, so a fixed number is
    # a different SPEED at every rate. p3's default of 30 Hz happened to land
    # on the right one; raising --fps without this would not have.
    step_deg, step_grip, derived = step_limits_for(
        args.fps, args.max_step_deg, args.max_step_gripper_pct)
    step_limits = per_joint(step_deg, step_grip)
    print(f"[p3] {args.fps:g} Hz, limit {step_deg:g} deg/step "
          f"({step_deg * args.fps:.0f} deg/s), {step_grip:g} %/step"
          + (f"  [{', '.join(derived)} derived from --fps]" if derived else
             "  [given on the command line]"))
    events.event("step_limits", fps=args.fps, max_step_deg=step_deg,
                 max_step_gripper_pct=step_grip, derived=list(derived))
    period = 1.0 / args.fps
    prev_cmd = None
    seq = 0
    episode = 0
    recording = False
    ep_steps = 0
    ep_clip = 0
    clamped_total = 0
    clip_total = 0
    read_fails = 0
    kept, discarded = [], []
    fault = None
    console_warned = False
    quit_armed_until = 0.0
    t_end = time.monotonic() + args.duration if args.duration > 0 else None

    def episode_ctl(action, ep, **meta):
        """Reliable episode boundary. Returns (ok, detail)."""
        if link is None:
            return True, "no-sim"
        return link.request(action, episode=ep, meta=meta or None)

    def mark_episode(ep, result, steps, detail=""):
        rows.write({"schema": EPISODE_SCHEMA, **stamp(), "episode": ep,
                    "result": result, "steps": steps, "detail": detail,
                    "run_dir": run_dir})

    def drop_episode(reason):
        nonlocal recording, ep_steps, ep_clip
        print(f"\n[p3] EPISODE {episode} DISCARDED -- {reason}\n")
        events.event("episode_discard", episode=episode, reason=reason,
                     steps=ep_steps)
        mark_episode(episode, "discarded", ep_steps, reason)
        episode_ctl("episode_discard", episode, reason=reason)
        discarded.append((episode, reason))
        recording, ep_steps, ep_clip = False, 0, 0

    try:
        while stop["why"] is None:
            t0 = time.monotonic()

            # ---------------- keyboard ----------------
            if console is not None and console.died and not console_warned:
                console_warned = True
                events.event("console_died", err=console.died)
                print(f"\n[p3] the keyboard reader stopped ({console.died}). "
                      f"No more episode commands can be given -- stop with "
                      f"SIGTERM.\n")
            cmd_key = console.get() if console is not None else None
            if cmd_key is not None:
                if cmd_key == "s":
                    if recording:
                        print(f"[p3] episode {episode} is already open")
                    else:
                        episode += 1
                        ok, detail = episode_ctl("episode_start", episode)
                        if ok:
                            recording, ep_steps, ep_clip = True, 0, 0
                            events.event("episode_start", episode=episode)
                            mark_episode(episode, "open", 0)
                            print(f"[p3] episode {episode} RECORDING")
                        else:
                            episode -= 1
                            print(f"[p3] could not start: {detail}")
                elif cmd_key in ("e", "d"):
                    if not recording:
                        print("[p3] no episode open")
                    elif cmd_key == "d":
                        drop_episode("operator")
                    else:
                        ok, detail = episode_ctl("episode_end", episode,
                                                 steps=ep_steps)
                        events.event("episode_end", episode=episode,
                                     steps=ep_steps, clipped_steps=ep_clip,
                                     ok=ok, detail=detail)
                        if ok:
                            kept.append((episode, ep_steps))
                            mark_episode(episode, "kept", ep_steps, detail)
                            print(f"[p3] episode {episode} KEPT "
                                  f"({ep_steps} steps, {ep_clip} clipped)")
                        else:
                            # Do not just walk away: the receiver still has the
                            # episode OPEN, and it would be closed later by
                            # whatever happens to arrive next. Say so explicitly
                            # so both sides agree on what happened.
                            why = f"sim refused the save: {detail}"
                            discarded.append((episode, why))
                            mark_episode(episode, "discarded", ep_steps, why)
                            episode_ctl("episode_discard", episode, reason=why)
                            print(f"[p3] episode {episode} NOT SAVED: {detail}")
                            print("[p3]   told the simulator to discard it.")
                        recording, ep_steps, ep_clip = False, 0, 0
                elif cmd_key == "q":
                    if recording and time.monotonic() > quit_armed_until:
                        quit_armed_until = time.monotonic() + 5.0
                        print(f"[p3] episode {episode} is OPEN. Press e to keep "
                              f"it, or q again within 5s to quit and DISCARD it.")
                    else:
                        if recording:
                            drop_episode("quit with an open episode")
                        stop["why"] = "quit"
                        continue
                else:
                    s = link.stats.summary() if link else {}
                    print(f"[p3] seq {seq}  episode {episode} "
                          f"{'RECORDING' if recording else 'idle'}  "
                          f"clamped {clamped_total}  clipped {clip_total}  {s}")
                if cmd_key != "q":
                    quit_armed_until = 0.0

            # ---------------- read the leader ----------------
            try:
                target = strip_pos(leader.get_action())
                short = [j for j in JOINTS if j not in target]
                if short:
                    # rate_limit() passes a joint through unclamped when it has
                    # no previous command for it, and simmap.apply() silently
                    # skips joints it was not given. One short read therefore
                    # lets the missing joint make an unlimited jump on the next
                    # step, with nothing logged. Treat it as a read failure.
                    raise RuntimeError(f"leader returned only "
                                       f"{len(target)}/6 joints, missing {short}")
                read_fails = 0
            except Exception as e:
                read_fails += 1
                events.event("leader_read_fail", n=read_fails, err=repr(e))
                if recording:
                    drop_episode(f"leader read failed: {type(e).__name__}")
                if read_fails >= args.read_strikes:
                    fault = (f"leader read failed {read_fails}x in a row: "
                             f"{type(e).__name__}: {e}")
                    break
                time.sleep(period)
                continue
            t1 = time.monotonic()

            # ---------------- limit, then map ----------------
            cmd_deg, n_clamped = rate_limit(target, prev_cmd, step_limits)
            prev_cmd = cmd_deg
            clamped_total += n_clamped
            joints_rad, clipped = simmap.apply(cmd_deg)
            if clipped:
                clip_total += 1
                ep_clip += 1
            t2 = time.monotonic()

            # ---------------- send, then wait out the tick on the socket ----
            rtt = None
            applied = None
            sim_time = None
            sim_clipped = []
            ack_seq = None
            sim_wait_ms = None
            sim_apply_ms = None
            if link is not None:
                msg = encode_cmd(seq, t2, joints_rad, map_sha,
                                 clipped_local=clipped,
                                 episode=episode if recording else None,
                                 recording=recording)
                if link.send(msg):
                    link.stats.on_send(seq, t2, sent_rad=joints_rad)
                else:
                    link.stats.on_send_failed(seq)
                _acks, mine, rtt = link.pump_until(t0 + period, want_seq=seq)
                if mine is not None:
                    ack_seq = mine.get("seq")
                    applied = mine.get("applied_rad")
                    sim_time = mine.get("sim_time")
                    sim_clipped = mine.get("clipped", [])
                    sim_wait_ms = mine.get("wait_ms")
                    sim_apply_ms = mine.get("apply_ms")
                if recording and link.t_last_ack is not None and \
                        (time.monotonic() - link.t_last_ack) * 1e3 > args.link_timeout_ms:
                    drop_episode(f"no ack from the simulator for "
                                 f"{args.link_timeout_ms:.0f} ms")
            t3 = time.monotonic()

            rec = {"schema": ROW_SCHEMA, "seq": seq, **stamp(),
                   "episode": episode if recording else None,
                   "recording": recording,
                   "leader_deg": target, "command_deg": cmd_deg, "units": UNITS,
                   "joints_rad": joints_rad, "clipped_local": clipped,
                   "applied_rad": applied, "clipped_sim": sim_clipped,
                   "ack_seq": ack_seq, "sim_time": sim_time,
                   "sim_wait_ms": sim_wait_ms, "sim_apply_ms": sim_apply_ms,
                   "map_sha256": map_sha,
                   "clamped": n_clamped,
                   "rtt_ms": (round(rtt, 3) if rtt is not None else None),
                   "dt_read_ms": round((t1 - t0) * 1e3, 3),
                   "dt_map_ms": round((t2 - t1) * 1e3, 3),
                   # send + the rest of the tick spent waiting on the socket,
                   # NOT a network cost. The network number is rtt_ms minus
                   # sim_wait_ms minus sim_apply_ms.
                   "dt_send_wait_ms": round((t3 - t2) * 1e3, 3),
                   "dt_loop_ms": round((t3 - t0) * 1e3, 3)}
            rows.write(rec)
            pub.publish(rec)
            seq += 1
            if recording:
                ep_steps += 1

            if t_end and time.monotonic() >= t_end:
                stop["why"] = "duration"
                break
            if link is None:                 # --no-sim: nothing to wait on
                sleep = period - (time.monotonic() - t0)
                if sleep > 0:
                    time.sleep(sleep)
    except Exception as e:
        fault = f"{type(e).__name__}: {e}"

    # ---- shutdown ----------------------------------------------------------
    if recording:
        drop_episode(fault or stop["why"] or "shutdown")
    reason = fault or stop["why"] or "unknown"
    if fault:
        print(f"\n[p3] FAULT: {fault}")

    if link is not None:
        link.request("bye", timeout_s=0.5)
        stats = link.stats.summary()
        link.close()
    else:
        stats = {}

    try:
        leader.disconnect()
    except Exception as e:
        events.event("disconnect_error", err=repr(e))

    clamp_rate = (clamped_total / seq) if seq else 0.0
    events.event("stop", reason=reason, steps=seq, clamped_total=clamped_total,
                 clipped_steps=clip_total, link=stats,
                 episodes_kept=kept, episodes_discarded=discarded)
    print(f"\n[p3] {seq} steps, {clamped_total} rate-clamped "
          f"({clamp_rate*100:.1f}%), {clip_total} steps hit a map limit")
    if clamp_rate > 0.01:
        print("[p3] NOTE: the rate limiter was active on more than 1% of steps. "
              "You moved the leader faster than the sim was allowed to follow, "
              "so the recorded action stream lags your hand. Raise "
              "--max-step-deg or move more slowly.")
    if clip_total:
        print(f"[p3] NOTE: {clip_total} steps were clipped by the map -- the "
              "leader went outside its fitted range. Those steps do not "
              "correspond to a reachable sim pose.")
    if stats:
        print(f"[p3] link: {stats}")
        if stats.get("send_failed"):
            print(f"[p3] NOTE: {stats['send_failed']} commands never reached "
                  f"the wire (sendto failed). Those are holes in the sim's "
                  f"motion, not network loss.")
        if stats.get("max_dev_joint"):
            print(f"[p3] worst sim/target gap: {stats['max_dev_deg']:.2f} deg "
                  f"on {stats['max_dev_joint']}"
                  + ("" if not stats.get("mismatched") else
                     f"  ({stats['mismatched']}/{stats['acked']} steps over "
                     f"{args.mismatch_tol_deg} deg)"))
        if stats.get("mismatched"):
            rate = stats["mismatched"] / max(1, stats["acked"])
            print("[p3] *** the simulator's measured joints sat further from "
                  "our targets than a position drive should. A wrong joint map "
                  "or a wrong DOF name looks exactly like this. ***")
            if rate > 0.5:
                print("[p3]     on more than half the steps -- treat this run "
                      "as suspect and re-check the map before recording more.")
            else:
                print("[p3]     intermittently; could also be the arm being "
                      "moved faster than the sim's drive can follow.")
        if stats.get("superseded", 0) > 0.05 * max(1, stats.get("sent", 1)):
            print("[p3] NOTE: the simulator skipped more than 5% of commands. "
                  "It is running slower than --fps. Lower --fps rather than "
                  "pretending the data was collected at 30 Hz.")
    print(f"[p3] kept {len(kept)} episode(s), discarded {len(discarded)}")
    print(f"[p3] logs in {run_dir}")
    rows.close()
    events.close()
    pub.close()
    return 2 if fault else 0


if __name__ == "__main__":
    sys.exit(main())
