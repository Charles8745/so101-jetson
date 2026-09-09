# Signal schema -- `so101.joints.v1`

`teleop_arm.py` emits one JSON record per control step, over UDP (best-effort)
and/or appended to a JSONL file. A later Isaac Sim bridge subscribes to the UDP
port and mirrors these joints onto the virtual SO-101.

## Per-step record
```json
{
  "schema": "so101.joints.v1",
  "seq": 1234,
  "t_mono": 51234.918,
  "t_unix": 1757068800.123,
  "leader":   {"shoulder_pan": 3.0, "shoulder_lift": -100.7, "elbow_flex": 96.0,
               "wrist_flex": 73.6, "wrist_roll": -5.8, "gripper": 0.0},
  "follower": {"shoulder_pan": 2.9, "shoulder_lift": -100.5, "...": "..."},
  "units": {"shoulder_pan": "deg", "shoulder_lift": "deg",
            "elbow_flex": "deg", "wrist_flex": "deg", "wrist_roll": "deg",
            "gripper": "percent_0_100"}
}
```
### ** The six values are NOT all in the same unit. **
The five body joints are DEGREES; the **gripper is PERCENT of its calibrated
travel (0-100)**, because lerobot hard-codes `MotorNormMode.RANGE_0_100` for it
regardless of `use_degrees`. Degrees are measured from the MIDPOINT OF THIS
ARM'S CALIBRATED TRAVEL -- not a URDF zero, not the servo's 2048. See
`arm/units.py` for the exact formulas. Anything mapping these onto another
system must branch on the unit.

- `leader`   = the commanded joint targets (what drove the follower this step).
- `follower` = the follower's measured joint state this step.
- Isaac should drive the virtual arm from `leader` (the intent). `follower` is
  there for logging / lag analysis.

## Epoch record (first line of the JSONL only)
```json
{"schema": "so101.joints.v1.epoch", "t_mono": 51230.0, "t_unix": 1757068795.0}
```
Pins the monotonic<->wall relationship once, so a reader can convert `t_mono`
(the alignment key) to wall time.

## Alignment with camera data
`t_mono` comes from `time.monotonic()`, which is system-wide on Linux. The camera
recorder stamps the SAME clock in each `<cam>_timestamps.csv`. To build one
synchronized episode, join arm records and camera frames on nearest `t_mono`.

## Transport notes
- UDP is fire-and-forget: a dropped datagram is skipped, never retried, and never
  blocks the control loop. Use the JSONL file as the reliable record.
- Default port suggestion: 9870. Set with `--udp <spark-ip>:9870`.

## Related schemas
| schema | written by | what it is |
|---|---|---|
| `so101.joints.v1` | p1 | above: leader + follower, real units |
| `so101.simstep.v1` | p3 | one step of the real→sim bridge: leader degrees, the rate-limited command, the mapped radians, what the sim applied, and the RTT split into network / waiting for the sim's tick / stepping |
| `so101.simcmd.v1` / `so101.simack.v1` / `so101.simctl.v1` | p3 ↔ receiver | the wire format itself |
| `so101.simmap.v1` | `tools/simmap_init.py` | the fitted, verified, calibration-locked real→sim joint map |

See **[SIM_BRIDGE.md](SIM_BRIDGE.md)**. The `units` field above applies to every
one of them: the gripper is never in degrees.
