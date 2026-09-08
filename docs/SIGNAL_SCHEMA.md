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
  "units": "deg"
}
```
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
