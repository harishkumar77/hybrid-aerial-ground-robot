# hybrid_safety

Safety supervisor for a hybrid aerial–ground robot: interlocks, heartbeat watchdog,
arm/disarm, auto/manual handoff, fly↔drive mode transitions, and mode-aware fail-safes.

Pure Rust, no dependencies. Designed to sit between the planner and the actuators
(ROS2 node wrapper and PX4 offboard bridge are the next step).

```bash
cargo test                              # 20 tests
cargo run --bin hybrid_safety           # 6 safety scenarios
cargo run --bin hybrid_safety -- 2      # one scenario
cargo run --bin survey_demo             # survey -> draft map -> plan -> execution
cargo run --bin survey_demo -- --step   # same, pauses between stages (for presenting)
```

## Survey -> draft map -> mode-typed plan -> execution (`src/survey.rs`)

A 2D grid prototype of the full pipeline:

1. **Aerial survey**: a downward elevation sensor model sees large geometry (river, building)
   but can't see under tree canopy (Unknown) and misses sub-resolution or non-geometric
   hazards (rock, mud).
2. **Global plan**: A* over `(cell, mode)`. Steps are typed Drive or Fly; a mode switch costs a
   takeoff/landing penalty and is only allowed on a cell believed Free. Unknown cells are
   drivable at a higher cost.
3. **Execution**: ground sensing (radius 2) corrects the map. If the next few steps become
   invalid, a **local repair** (drive-only A* in a window) detours and rejoins the global plan;
   a full global re-plan is only the fallback.
4. **Safety**: every takeoff and landing goes through the `Supervisor` interlocks
   (landing zone must be verified Free).

Result on the demo site: the river forces a 3-step fly segment, mud and a rock are found by
ground sensing and avoided with 3 local repairs and 0 global replans; executed cost 840 vs
planned 820, vs about 2020 to fly the whole route.

## Gazebo replay (Gazebo Harmonic)

```bash
cargo run --bin export_gazebo          # writes gazebo/hybrid_site.sdf + trail/hazards/events CSVs
gz sim -r gazebo/hybrid_site.sdf       # terminal 1 (-r = start playing)
python3 gazebo/replay.py               # terminal 2; add --pause to stop at each event
python3 gazebo/replay.py --reset       # put the robot back at the start
```

The world is built from the same grid (1 cell = 1 m): blue river, grey building, brown mud,
rocks, a tree grove hiding a fallen trunk and a rock, cyan/magenta tiles for the global plan.
The replay moves `hybrid_bot` along the executed route with `set_pose`, lifts it over the
river at takeoff/landing, and raises a red tile wherever ground sensing contradicted the
aerial draft. It is a visualization of the planner's output, not closed-loop simulation.

## State machine

```
DISARMED --Arm--> ARMED/Manual --EngageAuto--> ARMED/Auto --RequestMode--> TRANSITION
   ^                 ^      ^------ManualTakeover (always)-----|              |
   |                 |                                         <--Complete----+
   +--ResetFault-- FAIL-SAFE <-- watchdog miss / interlock loss / e-stop / transition timeout
       (at rest)   (latched)
```

## Design decisions

| Decision | Why |
|---|---|
| Fail-safe depends on mode *and* cause | Same fault, different safe action: a ground robot brakes, an aerial one holds or lands. |
| Never cut motors in the air | E-stop while flying = land now. Killing motors turns a fault into a crash. |
| Faults are latched | A flickering sensor can't silently re-enable autonomy. Reset only at rest, and only back to DISARMED. |
| Required heartbeats depend on mode | Flight controller matters when flying; drive controller when driving. |
| Unknown source = stale | Fail closed, never open. |
| Rejections list *every* reason | Operators fix everything at once instead of one error at a time. |
| `Permit` token | `Platform::translate` needs a `Permit`, which only the supervisor can mint. Autonomous commands while disarmed don't compile. |
| `Platform` trait | Adding a vehicle = implementing a trait; the safety logic is untouched. |

## Files

- `src/supervisor.rs` – state machine, interlocks, fail-safe policy, tests
- `src/watchdog.rs` – heartbeat deadlines
- `src/platform.rs` – `Platform` trait, DiffDrive and PX4 multicopter, output gate
- `src/main.rs` – safety scenario demo with a simulated 50 ms control loop
- `src/survey.rs` – world, aerial/ground sensor models, belief map, mode-typed A*, mission executor
- `src/bin/survey_demo.rs` – terminal visualization of the survey pipeline

## Limitations / next steps

- The survey demo is a 2D grid with rule-based sensor models, not Gazebo. Next: a downward depth
  camera on the sim drone, an elevation map (e.g. grid_map) and a traversability layer feeding the same planner.

- Runs in-process with a simulated clock. In production the watchdog belongs on
  independent hardware (MCU) so it survives a crash of the main computer.
- Next: rclrs node subscribing to planner/SLAM/LiDAR/`px4_msgs::VehicleStatus` as heartbeats,
  publishing gated commands; then run scenarios in Gazebo + PX4 SITL.
- Debounce (N missed cycles) and degraded modes (e.g. reduced speed on partial LiDAR loss).
