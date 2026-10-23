//! Export the demo site and the mission result for Gazebo Harmonic.
//!
//! `cargo run --bin export_gazebo` writes into ./gazebo/:
//!   hybrid_site.sdf  world: river, building, mud, rocks, tree grove, plan tiles,
//!                    hidden hazard markers, and the `hybrid_bot` model
//!   trail.csv        executed path: step,x,y,mode
//!   hazards.csv      hazards found by ground sensing: step,marker,x,y
//!   events.csv       narration: step;text
//!
//! Then: `gz sim gazebo/hybrid_site.sdf` and `python3 gazebo/replay.py`.

use hybrid_safety::survey::*;
use hybrid_safety::{Config, Mode, Supervisor};
use std::fmt::Write as _;
use std::fs;

const CELL: f64 = 1.0;

struct Site {
    ox: f64,
    oy: f64,
}

impl Site {
    /// grid cell -> world metres (x east, y north; grid rows grow southward)
    fn xy(&self, cx: usize, cy: usize) -> (f64, f64) {
        (cx as f64 * CELL - self.ox, self.oy - cy as f64 * CELL)
    }
}

fn mat(r: f64, g: f64, b: f64) -> String {
    format!(
        "<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>"
    )
}

#[allow(clippy::too_many_arguments)]
fn boxm(name: &str, x: f64, y: f64, z: f64, sx: f64, sy: f64, sz: f64, rgb: (f64, f64, f64), transp: f64) -> String {
    format!(
        r#"    <model name="{name}"><static>true</static><pose>{x:.3} {y:.3} {z:.3} 0 0 0</pose>
      <link name="l"><visual name="v"><transparency>{transp}</transparency><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>{m}</visual></link></model>
"#,
        m = mat(rgb.0, rgb.1, rgb.2)
    )
}

fn sphere(name: &str, x: f64, y: f64, z: f64, r: f64, rgb: (f64, f64, f64), transp: f64) -> String {
    format!(
        r#"    <model name="{name}"><static>true</static><pose>{x:.3} {y:.3} {z:.3} 0 0 0</pose>
      <link name="l"><visual name="v"><transparency>{transp}</transparency><geometry><sphere><radius>{r}</radius></sphere></geometry>{m}</visual></link></model>
"#,
        m = mat(rgb.0, rgb.1, rgb.2)
    )
}

#[allow(clippy::too_many_arguments)]
fn cyl(name: &str, x: f64, y: f64, z: f64, roll: f64, pitch: f64, r: f64, len: f64, rgb: (f64, f64, f64)) -> String {
    format!(
        r#"    <model name="{name}"><static>true</static><pose>{x:.3} {y:.3} {z:.3} {roll} {pitch} 0</pose>
      <link name="l"><visual name="v"><geometry><cylinder><radius>{r}</radius><length>{len}</length></cylinder></geometry>{m}</visual></link></model>
"#,
        m = mat(rgb.0, rgb.1, rgb.2)
    )
}

fn robot() -> String {
    // Visual-only, gravity off: the replay script places it with set_pose.
    let body = mat(0.95, 0.45, 0.05);
    let dark = mat(0.1, 0.1, 0.1);
    let mut s = String::from(
        r#"    <model name="hybrid_bot"><pose>0 0 -10 0 0 0</pose>
      <link name="base"><gravity>false</gravity>
        <inertial><mass>1</mass></inertial>
"#,
    );
    let _ = writeln!(s, r#"        <visual name="body"><geometry><box><size>0.7 0.5 0.22</size></box></geometry>{body}</visual>"#);
    for (i, (wx, wy)) in [(0.25, 0.3), (-0.25, 0.3), (0.25, -0.3), (-0.25, -0.3)].iter().enumerate() {
        let _ = writeln!(
            s,
            r#"        <visual name="wheel{i}"><pose>{wx} {wy} -0.08 1.5708 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.08</length></cylinder></geometry>{dark}</visual>"#
        );
    }
    for (i, (ax, ay)) in [(0.42f64, 0.42f64), (-0.42, 0.42), (0.42, -0.42), (-0.42, -0.42)].iter().enumerate() {
        let _ = writeln!(
            s,
            r#"        <visual name="arm{i}"><pose>{} {} 0.12 0 0 {}</pose><geometry><box><size>0.6 0.04 0.03</size></box></geometry>{dark}</visual>"#,
            ax / 2.0,
            ay / 2.0,
            ay.atan2(*ax)
        );
        let _ = writeln!(
            s,
            r#"        <visual name="rotor{i}"><pose>{ax} {ay} 0.15 0 0 0</pose><transparency>0.3</transparency><geometry><cylinder><radius>0.2</radius><length>0.01</length></cylinder></geometry>{dark}</visual>"#
        );
    }
    s.push_str("      </link>\n    </model>\n");
    s
}

fn main() {
    let world = World::demo();
    let mut sup = Supervisor::new(Config::default());
    let r = run_mission(&world, &mut sup, &MissionConfig::default());
    let site = Site { ox: 24.0, oy: 8.0 };

    let mut models = String::new();
    let (w, h) = (world.w, world.h);

    // terrain
    for y in 0..h {
        for x in 0..w {
            let (px, py) = site.xy(x, y);
            match world.terrain(x, y) {
                Terrain::Water => models.push_str(&boxm(&format!("river_{x}_{y}"), px, py, 0.01, 1.0, 1.0, 0.04, (0.1, 0.35, 0.85), 0.0)),
                Terrain::Mud => models.push_str(&boxm(&format!("mud_{x}_{y}"), px, py, 0.01, 1.0, 1.0, 0.03, (0.35, 0.22, 0.08), 0.0)),
                Terrain::Rock => models.push_str(&sphere(&format!("rock_{x}_{y}"), px, py, 0.25, 0.38, (0.45, 0.42, 0.4), 0.0)),
                _ => {}
            }
        }
    }
    // building (cells 10..13 x 1..4)
    let (bx0, by0) = site.xy(10, 1);
    let (bx1, by1) = site.xy(13, 4);
    models.push_str(&boxm("building", (bx0 + bx1) / 2.0, (by0 + by1) / 2.0, 3.0, 4.0, 4.0, 6.0, (0.6, 0.6, 0.62), 0.0));
    // fallen trunk under canopy (cells 34..36, row 6)
    let (tx, ty) = site.xy(35, 6);
    models.push_str(&cyl("fallen_trunk", tx, ty, 0.3, 0.0, 1.5708, 0.3, 3.0, (0.4, 0.26, 0.13)));
    // tree grove: trunks away from the driven row, canopy over the whole grove
    let mut k = 0;
    for (cx, cy) in [(33, 5), (35, 5), (37, 5), (33, 10), (35, 11), (37, 10), (37, 8), (33, 9)] {
        let (px, py) = site.xy(cx, cy);
        models.push_str(&cyl(&format!("tree_trunk_{k}"), px, py, 1.5, 0.0, 0.0, 0.15, 3.0, (0.4, 0.26, 0.13)));
        k += 1;
    }
    k = 0;
    for cy in [5, 7, 9, 11] {
        for cx in [33, 35, 37] {
            let (px, py) = site.xy(cx, cy);
            models.push_str(&sphere(&format!("canopy_{k}"), px, py, 3.4, 1.35, (0.15, 0.5, 0.15), 0.15));
            k += 1;
        }
    }
    // start / goal pads
    let (sx, sy) = site.xy(world.start.0, world.start.1);
    let (gx, gy) = site.xy(world.goal.0, world.goal.1);
    models.push_str(&boxm("start_pad", sx, sy, 0.005, 1.2, 1.2, 0.01, (0.1, 0.8, 0.2), 0.0));
    models.push_str(&boxm("goal_pad", gx, gy, 0.005, 1.2, 1.2, 0.01, (0.95, 0.85, 0.1), 0.0));

    // global plan tiles (what the robot intended from the aerial draft)
    for (i, s) in r.global_plan.iter().enumerate() {
        let (px, py) = site.xy(s.x, s.y);
        let rgb = if s.mode == Mode::Fly { (0.85, 0.2, 0.85) } else { (0.1, 0.8, 0.9) };
        models.push_str(&boxm(&format!("plan_{i}"), px, py, 0.03, 0.35, 0.35, 0.02, rgb, 0.3));
    }

    // hazard markers: hidden underground, raised by replay.py when detected
    let mut hazards = String::from("step,marker,x,y\n");
    let mut events = String::new();
    let mut n = 0;
    for e in &r.events {
        match e {
            MissionEvent::Hazard { step, corrections } => {
                let mut cells = Vec::new();
                for c in corrections {
                    let (px, py) = site.xy(c.x, c.y);
                    let name = format!("hazard_{n}");
                    models.push_str(&boxm(&name, px, py, -5.0, 0.9, 0.9, 0.06, (0.95, 0.1, 0.1), 0.35));
                    let _ = writeln!(hazards, "{step},{name},{px:.3},{py:.3}");
                    cells.push(format!("({},{})", c.x, c.y));
                    n += 1;
                }
                let _ = writeln!(events, "{step};HAZARD: ground sensing contradicts aerial draft at {}", cells.join(" "));
            }
            MissionEvent::LocalRepair { step, detour_len, rejoined_at } => {
                let _ = writeln!(
                    events,
                    "{step};LOCAL REPAIR: +{detour_len} step detour, rejoins global plan at ({},{})",
                    rejoined_at.0, rejoined_at.1
                );
            }
            MissionEvent::GlobalReplan { step, cost } => {
                let _ = writeln!(events, "{step};GLOBAL REPLAN: new cost {cost}");
            }
            MissionEvent::UnknownsResolved { step, count } => {
                let _ = writeln!(events, "{step};resolved {count} unknown cell(s) under canopy");
            }
            MissionEvent::Arrived { step } => {
                let _ = writeln!(events, "{step};ARRIVED at goal");
            }
            _ => {}
        }
    }

    let mut trail = String::from("step,x,y,mode\n");
    for (i, s) in r.trail.iter().enumerate() {
        let (px, py) = site.xy(s.x, s.y);
        let _ = writeln!(trail, "{i},{px:.3},{py:.3},{:?}", s.mode);
    }

    let sdf = format!(
        r#"<?xml version="1.0"?>
<!-- Generated by the export_gazebo binary. Hybrid aerial-ground survey site. -->
<sdf version="1.9">
  <world name="hybrid_site">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.75 0.85 0.95 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -0.9</direction></light>
    <model name="ground"><static>true</static>
      <link name="l">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>80 40</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>{gw} {gh}</size></plane></geometry>{gm}</visual>
      </link><pose>{gcx} {gcy} 0 0 0 0</pose></model>
{models}{robot}  </world>
</sdf>
"#,
        gw = w as f64 * CELL + 4.0,
        gh = h as f64 * CELL + 4.0,
        gcx = (w as f64 - 1.0) * CELL / 2.0 - site.ox,
        gcy = site.oy - (h as f64 - 1.0) * CELL / 2.0,
        gm = mat(0.55, 0.68, 0.4),
        robot = robot(),
    );

    fs::create_dir_all("gazebo").expect("create gazebo/");
    fs::write("gazebo/hybrid_site.sdf", sdf).unwrap();
    fs::write("gazebo/trail.csv", trail).unwrap();
    fs::write("gazebo/hazards.csv", hazards).unwrap();
    fs::write("gazebo/events.csv", events).unwrap();
    println!(
        "wrote gazebo/hybrid_site.sdf, trail.csv ({} steps), hazards.csv ({} markers), events.csv",
        r.trail.len(),
        n
    );
    println!("next:  gz sim gazebo/hybrid_site.sdf     (press Play)");
    println!("then:  python3 gazebo/replay.py");
}
