//! `cargo run --bin survey_demo`            all stages at once
//! `cargo run --bin survey_demo -- --step`  pause (Enter) between stages, for presenting
//! `--no-color`                             plain ASCII

use hybrid_safety::survey::*;
use hybrid_safety::{Config, Mode, Supervisor};
use std::io::{self, BufRead, Write};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let step = args.iter().any(|a| a == "--step");
    let paint = Paint { color: !args.iter().any(|a| a == "--no-color") };
    let pause = || {
        if step {
            print!("\n[Enter] ");
            io::stdout().flush().ok();
            let mut s = String::new();
            io::stdin().lock().read_line(&mut s).ok();
        }
    };

    let world = World::demo();
    let mut sup = Supervisor::new(Config::default());
    let r = run_mission(&world, &mut sup, &MissionConfig::default());

    println!("\n=== 1. GROUND TRUTH  (the robot never sees this directly) ===");
    print!("{}", paint.truth(&world));
    println!("legend: ~ river   # building   , mud   o rock   % tree canopy   X fallen trunk under canopy");
    pause();

    println!("\n=== 2. AERIAL DRAFT MAP  (one lawnmower survey, downward elevation sensor) ===");
    print!("{}", paint.belief(&r.draft, &[], world.start, world.goal, false));
    println!("legend: . free   # blocked   ? unknown (ground hidden under canopy)");
    for e in &r.events {
        if let MissionEvent::Survey { blocked, unknown } = e {
            println!("  sees river and building ({} blocked cells), {} unknown cells under canopy", blocked, unknown);
            println!("  misses the mud patch and the rock: too small or not geometric from altitude");
        }
    }
    pause();

    println!("\n=== 3. GLOBAL PLAN  (A* over cell x mode, on the draft map) ===");
    print!("{}", paint.belief(&r.draft, &r.global_plan, world.start, world.goal, false));
    println!("legend: = drive   ^ fly   T takeoff   L landing");
    for e in &r.events {
        if let MissionEvent::GlobalPlan { cost, drive_steps, fly_steps, switches } = e {
            println!(
                "  cost {}  |  {} drive steps, {} fly steps, {} mode switches",
                cost, drive_steps, fly_steps, switches
            );
        }
    }
    for (m, a, b) in segments(&r.global_plan) {
        let (s, e) = (r.global_plan[a], r.global_plan[b]);
        println!("  {:<5} ({:>2},{:>2}) -> ({:>2},{:>2})", format!("{:?}", m), s.x, s.y, e.x, e.y);
    }
    pause();

    println!("\n=== 4. EXECUTION LOG  (ground sensing radius 2, local repair, supervisor-gated mode switches) ===");
    for e in &r.events {
        match e {
            MissionEvent::Hazard { step, corrections } => {
                let cells: Vec<String> = corrections
                    .iter()
                    .map(|c| format!("({},{}) {:?}->{:?}", c.x, c.y, c.was, c.now))
                    .collect();
                println!("  step {:>3}  HAZARD   ground sensing contradicts draft: {}", step, cells.join(", "));
            }
            MissionEvent::UnknownsResolved { step, count } => {
                println!("  step {:>3}  resolved {} unknown cell(s) under canopy", step, count);
            }
            MissionEvent::LocalRepair { step, detour_len, rejoined_at } => {
                println!(
                    "  step {:>3}  LOCAL REPAIR  +{} steps detour, rejoined global plan at ({},{})",
                    step, detour_len, rejoined_at.0, rejoined_at.1
                );
            }
            MissionEvent::GlobalReplan { step, cost } => {
                println!("  step {:>3}  GLOBAL REPLAN  new cost {}", step, cost);
            }
            MissionEvent::Supervisor(l) => println!("            supervisor {}", l),
            MissionEvent::Arrived { step } => println!("  step {:>3}  ARRIVED at goal", step),
            MissionEvent::Aborted { step, why } => println!("  step {:>3}  ABORTED: {}", step, why),
            _ => {}
        }
    }
    pause();

    println!("\n=== 5. CORRECTED MAP + PATH ACTUALLY TAKEN ===");
    print!("{}", paint.belief(&r.final_map, &r.trail, world.start, world.goal, true));
    println!("legend: = drove   ^ flew   T/L takeoff/landing   ! hazard found by ground sensing (not in draft)");

    let flown = r.trail.iter().filter(|s| s.mode == Mode::Fly).count();
    let driven = r.trail.len() - flown;
    println!("\n  arrived: {}   steps driven: {}   steps flown: {}", r.arrived, driven, flown);
    println!(
        "  planned cost {}  ->  executed cost {}   ({} local repairs, {} global replans)",
        r.planned_cost, r.executed_cost, r.local_repairs, r.global_replans
    );
    let c = MissionConfig::default().costs;
    let dx = world.start.0.abs_diff(world.goal.0) as u32;
    let fly_all = 2 * c.mode_switch + c.fly * dx;
    println!("  baseline: fly the whole route ~ {}   |   drive only: impossible (river)", fly_all);
    println!("  final supervisor state: {}", sup.state());
}
