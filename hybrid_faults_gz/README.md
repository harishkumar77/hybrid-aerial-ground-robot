# hybrid_faults_gz: fault scenarios in Gazebo

Separate project. Uses `../hybrid_safety` **read-only** (as a library); nothing there is changed.

The robot follows the real mission route (draft map → mode-typed A\* → local repairs) in a
**50 ms control loop with the real `Supervisor`**: heartbeats every tick, `tick()` every tick,
takeoff/landing through `RequestMode` interlocks. A fault is injected and **the supervisor
decides** the reaction. This layer only turns the decided action into motion for Gazebo.

## Folder layout (both side by side)
```
~/rust_ws/hybrid_safety/       existing project (untouched)
~/rust_ws/hybrid_faults_gz/    this project
```

## Run
```bash
cd ~/rust_ws/hybrid_faults_gz
cargo test                                   # 8 scenario tests
cargo run                                    # writes gazebo/fault_site.sdf + gazebo/scenarios/*.tsv
cargo run -- estop_flying                    # also prints that scenario's narration (no Gazebo needed)

gz sim -r gazebo/fault_site.sdf              # terminal 1
python3 gazebo/fault_replay.py --list        # terminal 2
python3 gazebo/fault_replay.py planner_dies_flying --quiet
python3 gazebo/fault_replay.py estop_flying --pause      # stops at each fault/decision
```
`--quiet` hides the planner/map notes, `--speed 0.5` slows down, `--pause` waits for Enter.

If `cargo run` says it can't read the base world: `cd ../hybrid_safety && cargo run --bin export_gazebo`.

## Robot color = supervisor state
green **AUTO** · amber **TAKEOFF/LANDING** · red **FAIL-SAFE** · blue **MANUAL** · grey **DISARMED**

## Scenarios
| Name | Fault injected | Supervisor decides | Motion | Ends |
|---|---|---|---|---|
| `nominal` | none | interlocks pass at takeoff/landing | full mission | AUTO at goal |
| `planner_dies_driving` | planner killed after the mud | ControlledStop | gentle stop; planner restarts, fault stays latched; operator takes over | MANUAL |
| `planner_dies_flying` | planner killed over the river | HoverThenLand | PX4 Hold 3 s → back to takeoff cell → land* → reset | DISARMED |
| `localization_lost_flying` | SLAM invalid over the river | LandNow | lands in place (in the river); recovers but stays latched; reset on ground | DISARMED |
| `estop_driving` | e-stop pressed while driving | EmergencyBrake | stops immediately; reset refused until e-stop released | DISARMED |
| `estop_flying` | e-stop over the river | LandNow (never cut motors) | takeover refused; lands in place; reset refused until release | DISARMED |
| `takeoff_rejected` | battery 45% at the bank | takeoff interlock rejects (< 50% reserve) | stays on the bank | AUTO, stopped |
| `landing_never_confirmed` | touchdown never confirmed | transition timeout after 8 s → LandNow | hovers low, then lands; reset | DISARMED |

\* The "hold 3 s, then return to last ground-verified cell" escalation is implemented in this
simulation layer, not yet in the supervisor (which only commands Hold). Narration says so.

## Honest scope
- Still a **replay**: positions are computed in Rust and placed in Gazebo with `set_pose`.
  No Gazebo physics, no PX4 SITL, no real sensors.
- The supervisor logic, timing (200 ms heartbeat deadline, 8 s transition timeout) and every
  decision are real and identical to `hybrid_safety`.
- Next: same scenarios closed-loop with the supervisor as a ROS2 node and PX4 SITL.
