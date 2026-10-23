# Hybrid Aerial–Ground Autonomous Robot

> **Status: work in progress.** This repo collects the whole project. Components are added as
> they mature; the first one published is the safety supervisor and planning prototype.

I'm working on a hypothetical model which is a hybrid robot (quadcopter and quad-wheeled rover  that **drives most of the way and flies only where it must** (for example over a
river). This was build to considering how efficient it can be when it comes to energy use, payload, handling the obstacles like river, ditches, walls and rubble. 

The model starts working by taking a quick aerial survey builds a draft map, a mode-typed planner decides drive vs fly for
every step, ground sensing corrects the map along the way, and a safety supervisor sits between
the planner and the actuators.


**Core idea of the safety layer:** the same fault needs a different safe reaction depending on mode.
If the planner crashes while driving, the robot brakes. If it crashes while flying, it holds
position and lands, because stopping in the air means falling.

## Architecture

```
Aerial survey ─► Draft map ─► Mode-typed A* (drive/fly per step)
                                     │
Perception ─► Localization ─► Planning ─► Control ─► [Safety Supervisor] ─► Actuators
     ▲              ▲             ▲          ▲              │       (wheels / PX4)
     └──────────────┴─────────────┴──────────┴─ heartbeats ─┘
```

## Components

| Component | Folder | Status |
|---|---|---|
| Safety supervisor (interlocks, watchdog, arm/disarm, auto/manual, mode-aware fail-safes) | [`hybrid_safety/`](hybrid_safety/) | ✅ Rust prototype, 20 tests |
| Survey → draft map → mode-typed A\* → local repair | [`hybrid_safety/`](hybrid_safety/) (`src/survey.rs`) | ✅ 2D grid prototype |
| Fault-injection scenarios replayed in Gazebo | [`hybrid_faults_gz/`](hybrid_faults_gz/) | ✅ 8 scenarios, 8 tests |
| ROS2 workspace (Jazzy, Rust nodes, `px4_msgs`) | `ros2_ws/` | 🔜 to be added |
| PX4 SITL + Micro XRCE-DDS bridge, LiDAR via `ros_gz_bridge` | `ros2_ws/` | 🔜 to be added |
| Ground SLAM (`slam_toolbox`) and exploration | `ros2_ws/` | 🔜 to be added |
| Supervisor and planner as ROS2 nodes, closed-loop PX4 SITL | — | 📋 planned |
| Downward depth camera → elevation / traversability map | — | 📋 planned |
| Hardware | — | 📋 planned |

## Safety supervisor (`hybrid_safety/`)
- **Interlocks** before arming, engaging autonomy, takeoff and landing; rejections list every failing reason
- **Heartbeat watchdog**; required components depend on mode; unknown sources fail closed
- **Arm/disarm** gate; no disarm in the air
- **Auto/manual handoff**; operator takeover always wins, except during an e-stop
- **Mode-aware, latched fail-safes**: controlled stop / emergency brake on the ground; hold-then-land / land-now in the air; never cut motors mid-flight
- **`Permit` token**: autonomous actuator commands can't be constructed unless the supervisor allows it
- **`Platform` trait**: the same safety logic drives a diff-drive robot and a PX4 multicopter

## Planning (`hybrid_safety/src/survey.rs`)
- Aerial survey builds a **draft map** (misses canopy, small rocks, mud)
- **A\* over (cell, mode)**: every step typed drive or fly; takeoff/landing cost; switch only on verified-free cells
- Ground sensing corrects the map; **local repair** detours and rejoins the global plan; global re-plan is the fallback


## Implementation results 
![Alt Text](https://github.com/harishkumar77/hybrid-aerial-ground-robot/blob/main/Screenshot%20from%202026-10-23%2013-50-15.png)
![Alt Text](https://github.com/harishkumar77/hybrid-aerial-ground-robot/blob/main/Screenshot%20from%202026-10-23%2013-50-49.png)
![Alt Text](https://github.com/harishkumar77/hybrid-aerial-ground-robot/blob/main/Screenshot%20from%202026-10-23%2013-52-04.png)
![Alt Text](https://github.com/harishkumar77/hybrid-aerial-ground-robot/blob/main/Screenshot%20from%202026-10-23%2013-53-39.png)

## Quick start

Requires Rust (`cargo`); Gazebo Harmonic for the 3D replay.

```bash
git clone <this repo> && cd hybrid-aerial-ground-robot

cd hybrid_safety
cargo test                                  # 20 tests
cargo run -- 2                              # planner crash: drive vs fly response
cargo run --bin survey_demo -- --step       # draft map -> plan -> local repair (terminal)

cd ../hybrid_faults_gz
cargo test                                  # 8 scenario tests
cargo run                                   # export scenarios
gz sim -r gazebo/fault_site.sdf             # terminal 1
python3 gazebo/fault_replay.py planner_dies_flying --quiet   # terminal 2
```

Robot color in Gazebo = supervisor state: green AUTO · amber TAKEOFF/LANDING · red FAIL-SAFE · blue MANUAL · grey DISARMED.


## Why Rust?

I chose Rust for this project partly to explore the language for robotics, and partly because
it suits safety-critical code:

- **Memory and thread safety at compile time**, without a garbage collector (no pauses in the control loop)
- **The type system can enforce safety rules**: an autonomous actuator command can only be built with a
  `Permit`, which only the supervisor can create, so "command while disarmed" doesn't compile
- **Exhaustive `match`**: every state and fail-safe case must be handled
- **C++-level performance**, and it works with ROS2 (`rclrs`) and `px4_msgs`

The design itself (state machine, interlocks, platform interface) isn't Rust-specific and
would map directly to C++ or Java.

## Current limitations
- Planning uses a 2D grid with rule-based sensor models, not real sensor data
- The Gazebo part is a **replay** of Rust output (`set_pose`), not physics or PX4 SITL
- Supervisor runs in-process with a simulated clock; production would put the watchdog on independent hardware with a hardwired e-stop
- Hand-tuned costs; no debounce or degraded modes yet

## Author
Harish Kumar Kaliyappan · [LinkedIn](https://linkedin.com/in/harish-kumar-kaliyappan)
