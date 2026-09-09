# Program 3 -- real leader, virtual arm

```
[Jetson]  leader (real, moved by hand)
   read 6 joints -> rate limit -> simmap -> radians
   --UDP 9871-->  [Spark]  sim/receiver.py -> SimBackend -> Isaac
   <--UDP 9872--  ack: what the sim ACTUALLY applied, and what it clipped
```

**The real follower is never touched.** p3 runs the leader alone. If the
follower is plugged in and still powered from a p1 run it just holds where it
is; nothing here sends it a goal.

## Why this program exists

Every off-the-shelf "teleoperate into a simulator" script assumes the leader is
plugged into the machine running the simulator. Ours is not: the leader is on
the Jetson, Isaac is on Spark. That network hop **is** program 3, and it is the
thing that has to be measured rather than assumed.

Second reason: nothing in lerobot, Isaac Lab, LeKiwi or the SO-101 docs records
the **unit convention** for the joint mapping. Four sources checked, none states
it. Undocumented conventions are where errors hide, so this pair of programs
treats the mapping as the primary hazard.

## The three silent failures, and the guard for each

| Failure | What it looks like | Guard |
|---|---|---|
| Wrong joint mapping | virtual arm moves smoothly in roughly the right direction; every episode worthless | the map is a fitted, verified, calibration-locked **file**; the sim echoes back what it applied and every step is checked |
| Gap inside an episode | one episode silently contains a jump | link loss, leader read failure or a stalled sim while recording **discards the episode** |
| Sim quietly running behind | latency creeps up; data claims 30 Hz and is not | skipped commands are counted as `superseded`, never as `lost` |

### Fault policy, and why it differs per program
- **p1 stop-and-wait** -- a stuck arm is a physical hazard; freeze and let a
  human look.
- **p2 continue-and-mark** -- a dropped camera frame is recoverable; note it and
  keep going.
- **p3 / p4 discard the episode** -- a corrupt demonstration poisons training and
  nothing downstream can detect it. Throwing it away is cheaper than finding it
  later.

## Units: the whole point

Real arm (`arm/units.py`): five body joints in **degrees measured from the
midpoint of THIS arm's calibrated travel**, gripper in **percent of its
travel** (lerobot hard-codes `RANGE_0_100` for the gripper whatever
`use_degrees` says). Sim: **radians**, zero defined by the USD's joint frames.

There is no reason those zeros coincide and no reason the axes agree.
`radians(degrees)` is the mistake this whole subsystem exists to prevent.

## The map, and how it is established

`arm/sim_mapping.py`, built by `tools/simmap_init.py`.

1. **Identity fit, degree for degree** (the default). If the model is a
   faithful model of the same linkage, one degree of real joint rotation *is*
   one degree of model rotation, so the scale is exactly π/180 and cannot be
   wrong. Only the **sign** and the **offset** are assumptions, and those are
   what the visual check looks at.

   The alternative, `--fit-mode endpoints`, stretches the arm's calibrated
   travel onto the model's declared travel. **Measured against the real SO-101
   URDF, that is actively harmful**: the arm's `wrist_flex` sweeps 206.4° while
   the URDF declares 190°, so the stretch is 4° wrong at mid-travel, reports no
   clipping, and 8.6% sits *under* the refusal threshold — nothing would have
   stopped it. `wrist_roll` is worse: 360° real against 320° declared, 16° wrong
   at +170° and silent. Identity maps degree for degree and lets the part the
   model genuinely cannot reach **clip**, which `apply()` reports every step.
   Use `endpoints` only when the two ranges denote the same physical extremes.
2. **Span check, automatic, needs no instrument.**
   `span_ratio = real_span_deg / model_span_deg`. 1.0 means the arm and the
   model have the same range of motion. It is the only cheap check that can
   catch "the model is not this arm": an affine fit through two points passes
   through their midpoint **by construction**, so a midpoint residual check is
   vacuous.

   What a mismatch *means* depends on the fit mode, and the guard branches on it:
   - `endpoints` — the stretch is spread over every intermediate angle, so a big
     mismatch makes the map wrong everywhere but the two ends. Refused above 10%.
   - `identity` — the scale is exact by construction and cannot be wrong. A
     mismatch only predicts **clipping** at the extremes, which is detected and
     reported per step. `simmap_init fit` prints how many degrees of real travel
     the model cannot reach, per joint.

   For the real SO-101 URDF against a real arm that comes out as: `wrist_roll`
   39.9° unreachable (11.1% of its range), `wrist_flex` 16.4° (8.0%),
   `shoulder_lift` 5.2°, `shoulder_pan` 1.0°, `elbow_flex` 0.6°. Those are poses
   you simply cannot demonstrate into the sim — worth knowing before recording,
   not after.
3. **Sign, per joint, by looking.** Whether the arm's `range_min` corresponds to
   the model's lower or upper limit cannot be recovered by arithmetic -- both
   choices give a smooth plausible mapping, one of them backwards. Run p3 with
   `--allow-unverified`, move one joint at a time, and `--flip` whichever went
   the wrong way.
4. **Human verification, recorded and bound.** `verify` writes who checked it,
   how, and the sha of the fit they checked. Edit the map afterwards and the
   verification is automatically void.
5. **Calibration interlock.** The map stores the sha256 of the calibration it
   was fitted against and **which arm** (leader or follower). Recalibrate by one
   tick and the map is refused. Point a leader map at the follower and it is
   refused -- the two arms have different travel midpoints, so the map does not
   transfer. p3 will need a leader map; p4/p5 will need a follower one.

## Setup, once per model

**On Spark**
```
python3 sim/probe_isaac.py
python3 sim/probe_isaac.py --stage limits --usd <so101.usd> \
    --joints shoulder_pan=<usd_name>,shoulder_lift=<usd_name>,elbow_flex=<usd_name>,wrist_flex=<usd_name>,wrist_roll=<usd_name>,gripper=<usd_name> \
    --out sim_limits.json
```
`--stage limits` reads the limits out of the **USD schema** with `pxr`, not
through any Isaac API, so it does not depend on the Isaac version and does not
need a SimulationApp. UsdPhysics stores revolute limits in degrees; the JSON is
in radians and says so.

**On the Jetson** (copy `sim_limits.json` over)
```
python3 tools/simmap_init.py fit --arm-role leader --arm-id my_leader \
    --sim-limits sim_limits.json --out configs/simmap_leader.json
python3 tools/simmap_init.py show --map configs/simmap_leader.json
```

**Check the signs**, then
```
python3 tools/simmap_init.py fit ... --flip elbow_flex,wrist_roll   # if needed
python3 tools/simmap_init.py verify --map configs/simmap_leader.json \
    --by <you> --method visual --note "<what you actually checked>"
```

## Running

**Spark**
```
python3 sim/receiver.py --backend isaac --map configs/simmap_leader.json --usd <so101.usd>
```
**Jetson**
```
python3 programs/p3_teleop_sim.py --leader-port $LEADER \
    --map configs/simmap_leader.json --sim <spark-ip>
```
Keys: `s` start episode, `e` end and keep, `d` discard, `q` quit, Enter status.

### Without Isaac, without Spark, without an arm
```
python3 tools/loopback_test.py            # the whole bridge, against itself
python3 tools/loopback_test.py --sim-fps 10
```
### Without Isaac, with the real arm (both on the Jetson)
```
python3 sim/receiver.py --backend echo --map configs/simmap_leader.json   # term 1
python3 programs/p3_teleop_sim.py --sim 127.0.0.1 ...                     # term 2
```
The echo backend reports `readback=False` and p3 says so: **the link is proven,
the mapping is not.** Echo hands our own numbers back; it cannot check a map.

### With neither
```
python3 programs/p3_teleop_sim.py --no-sim --leader-port $LEADER --map ...
```
Reads the arm, applies the map, logs locally. Checks the arm and the map with no
network at all.

## Protocol -- `so101.simcmd.v1` / `so101.simack.v1` / `so101.simctl.v1`

One UDP socket on the Jetson, bound to 9872, used for both directions. The
receiver replies to the datagram's source, so it never has to be told the
Jetson's IP -- one less thing to configure wrongly, and it survives a new DHCP
lease.

**Joint commands are unreliable on purpose.** A lost command at 30 Hz means the
sim holds its target for 33 ms. Sequence numbers make the loss rate *measurable*
rather than assumed, which is the property that matters.

**Episode boundaries are reliable.** Losing "episode 7 ended" welds episodes 7
and 8 together and nobody finds out until training. Control messages carry their
own sequence, retransmit until acknowledged, and are idempotent on the receiver
— keyed by **(sender, session, ctl_seq)**.

All three parts of that key are load-bearing:
- **sender**, because every operator's counter starts at 1, so a cache keyed on
  the sequence alone hands a second operator the first one's "you are the owner"
  acknowledgement — they see *connected* and drive nothing.
- **session** (a fresh nonce per run), because p3 binds a fixed source port and
  also restarts its counter at 1, so a *restarted* p3 is byte-for-byte a
  retransmission of the previous one. Without the nonce the receiver replays
  "episode 1 started" and "episode 1 ended" from the dead run: the operator
  performs a whole demonstration, p3 prints KEPT, and the simulator started
  nothing and saved nothing.

**An acknowledgement must answer the question that was asked.** Acks echo
`session`, `action` and `episode`, and the sender rejects any that do not match
its pending request. A request the sender has given up on is cancelled, so a
late retransmit cannot start or end an episode p3 has already reported as failed.

**`episode_end` is not blanket-idempotent.** The receiver keeps a ledger of what
actually happened to each episode. If it discarded episode 4 itself — the
operator's machine went silent — then `episode_end 4` comes back **refused**,
saying so. Answering "sure, that worked" would make p3 print KEPT for a
demonstration that exists nowhere, which is the precise failure this program is
built to prevent.

**One owner at a time.** The first sender to say `hello` claims the sim; a
second is refused, not queued. Two people driving one virtual arm produces a
recording that looks fine and means nothing. A command stream that never said
hello is ignored unless you pass `--allow-anonymous` — silently accepting one
means a p3 with a stale or unverified map can drive and record with nothing on
the Spark side checked at all. A `hello` from a *new* run while an episode is
open discards that episode rather than orphaning it in the backend.

**Conflate, never queue.** The receiver drains the socket each tick and applies
only the newest command. Applying them in order would make it fall further
behind for as long as it is slow, and the latency measured afterwards would be
its own bug. Skipped seqs come back in `superseded`.

### Latency, decomposed
The ack carries `wait_ms` (how long the command sat waiting for the sim's next
tick) and `apply_ms` (the step itself), so

```
rtt = network + wait_for_sim_tick + apply/step
```

is separable. `tools/analyze_latency.py` prints the split. A loopback run at
30/30 measured:

```
rtt total                      : 19.93 / 32.72   (median / p95, ms)
  ... waiting for the sim's tick : 19.44 / 32.23
  ... the sim step itself        :  0.02 /  0.07
  ... network + our own handling :  0.51 /  0.77
```

**The receiver's tick quantisation dominates, not the network.** Without the
split, that 20 ms reads as a slow network and sends you debugging the wrong
thing for a day.

Each row's `rtt_ms` belongs to that row's own sequence number — it is not
"whatever round trip was measured most recently", which would silently attribute
one step's latency to another whenever two acks land in the same window. Rows
whose ack did not arrive inside their own tick carry `rtt_ms: null`, and the
analyser says how many, because a distribution built only from the fast ones is
truncated exactly where the interesting tail is.

### The mismatch alarm is a tracking-error threshold, not an epsilon
With a real simulator the ack carries **measured** joint positions, and a
position-driven joint never lands exactly on its target. A 1e-6 tolerance would
therefore fire on every single step, and the operator would learn to ignore the
one warning designed to catch a wrong joint map. `--mismatch-tol-deg` defaults
to 5.7° (0.1 rad): far above normal drive lag, far below the tens of degrees a
wrong axis or a wrong DOF name produces. The summary reports the worst gap seen
and on which joint, so the number is informative rather than a yes/no.

## Logs

`p3` writes `rows.jsonl` and `events.jsonl`; the receiver writes its own pair.

`rows.jsonl` carries two schemas. `so101.simstep.v1` is one control step.
`so101.episode.v1` is an episode marker — `open`, then `kept` or `discarded`
with the step count and the reason. **The verdict is in the row stream itself,
not only in `events.jsonl`**, because the obvious way to build a dataset is to
group `rows.jsonl` by `episode`, and a discard that only writes to another file
would leave that loader training on the episode we threw away.
`tools/analyze_latency.py` excludes discarded episodes by default.

Step fields:
`leader_deg`, `command_deg` (after rate limiting), `units`, `joints_rad`,
`clipped_local` (our map clipped it), `applied_rad` + `clipped_sim` (what the
sim did), `ack_seq`, `rtt_ms`, `sim_wait_ms`, `sim_apply_ms`, `episode`,
`recording`, `map_sha256`, and the `t_mono` shared with the camera recorder.

`clipped_local` and `clipped_sim` are different facts. Our map clips when the
leader is driven past the travel it was fitted on; the sim clips when it refuses
a target. Either one means those steps do not correspond to a reachable pose.

## Status

| Piece | State |
|---|---|
| `arm/sim_mapping.py`, `net/sim_protocol.py`, `sim/backend.py`, `sim/receiver.py`, `programs/p3_teleop_sim.py`, `tools/simmap_init.py` | tested end to end over real UDP |
| `sim/probe_isaac.py` | runs; the `limits` stage needs a USD to read |
| `sim/isaac_adapter.py` | **written, never run** -- needs Spark's Isaac version |

Reviewed adversarially by a second agent that had not seen it being written;
eighteen findings, all of the top- and high-severity ones fixed and covered by
tests (`tools/selftest.py`, `tools/loopback_test.py`).

`IsaacBackend` defaults `physics_dt` to `1/fps` where fps is the **receiver's
tick rate**, because the receiver steps the world once per tick: leave it at an
unrelated 1/60 with a 30 Hz receiver and simulated time runs at half wall-clock
— `sim_time` wrong by 2× in every ack and every row, and the virtual arm
responding at half the speed of the operator's hand, with nothing reporting it.
`step()` compares simulated against wall time and warns on >10% drift.

`sim/isaac_adapter.py` is the only version-specific file in the repo. Every
Isaac-dependent import is inside `_probe_api()`, and the adapter checks that the
articulation object has the methods it is about to call, printing what it does
have if not. It refuses to start rather than half-work.
