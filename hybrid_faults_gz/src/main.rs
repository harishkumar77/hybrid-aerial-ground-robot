//! `cargo run`                 export every scenario + the Gazebo world into ./gazebo/
//! `cargo run -- <scenario>`   also print that scenario's narration in the terminal
//! `cargo run -- --list`       list scenario names
//!
//! Reads the base world from ../hybrid_safety/gazebo/hybrid_site.sdf (read-only),
//! writes ./gazebo/fault_site.sdf with five colored copies of the robot (one per
//! supervisor state).

use hybrid_faults_gz::*;
use std::fmt::Write as _;
use std::fs;

const BASE_SDF: &str = "../hybrid_safety/gazebo/hybrid_site.sdf";

fn rgb(look: Look) -> (f64, f64, f64) {
    match look {
        Look::Disarmed => (0.55, 0.55, 0.55),
        Look::Manual => (0.15, 0.4, 0.95),
        Look::Auto => (0.1, 0.8, 0.25),
        Look::Transition => (0.98, 0.7, 0.05),
        Look::FailSafe => (0.9, 0.1, 0.1),
    }
}

fn bot(look: Look) -> String {
    let (r, g, b) = rgb(look);
    let body = format!("<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><emissive>{} {} {} 1</emissive></material>", r * 0.3, g * 0.3, b * 0.3);
    let dark = "<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>";
    let mut s = format!(
        "    <model name=\"bot_{}\"><pose>0 0 -20 0 0 0</pose>\n      <link name=\"base\"><gravity>false</gravity><inertial><mass>1</mass></inertial>\n",
        look.name()
    );
    let _ = writeln!(s, "        <visual name=\"body\"><geometry><box><size>0.7 0.5 0.22</size></box></geometry>{body}</visual>");
    let _ = writeln!(s, "        <visual name=\"beacon\"><pose>0 0 0.45 0 0 0</pose><geometry><sphere><radius>0.18</radius></sphere></geometry>{body}</visual>");
    for (i, (wx, wy)) in [(0.25, 0.3), (-0.25, 0.3), (0.25, -0.3), (-0.25, -0.3)].iter().enumerate() {
        let _ = writeln!(s, "        <visual name=\"wheel{i}\"><pose>{wx} {wy} -0.08 1.5708 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.08</length></cylinder></geometry>{dark}</visual>");
    }
    for (i, (ax, ay)) in [(0.42f64, 0.42f64), (-0.42, 0.42), (0.42, -0.42), (-0.42, -0.42)].iter().enumerate() {
        let _ = writeln!(s, "        <visual name=\"arm{i}\"><pose>{} {} 0.12 0 0 {}</pose><geometry><box><size>0.6 0.04 0.03</size></box></geometry>{dark}</visual>", ax / 2.0, ay / 2.0, ay.atan2(*ax));
        let _ = writeln!(s, "        <visual name=\"rotor{i}\"><pose>{ax} {ay} 0.15 0 0 0</pose><transparency>0.3</transparency><geometry><cylinder><radius>0.2</radius><length>0.01</length></cylinder></geometry>{dark}</visual>");
    }
    s.push_str("      </link>\n    </model>\n");
    s
}

fn clean(s: &str) -> String {
    s.replace(['\t', '\n'], " ")
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--list") {
        for f in Fault::all() {
            println!("{:<26} {}", f.name(), f.title());
        }
        return;
    }

    // world
    let base = match fs::read_to_string(BASE_SDF) {
        Ok(s) => s,
        Err(_) => {
            eprintln!("could not read {BASE_SDF}");
            eprintln!("run this first:  cd ../hybrid_safety && cargo run --bin export_gazebo && cd ../hybrid_faults_gz");
            std::process::exit(1);
        }
    };
    let mut bots = String::new();
    for l in Look::all() {
        bots.push_str(&bot(l));
    }
    let sdf = base
        .replace("<world name=\"hybrid_site\">", "<world name=\"fault_site\">")
        .replacen("  </world>", &format!("{bots}  </world>"), 1);
    fs::create_dir_all("gazebo/scenarios").unwrap();
    fs::write("gazebo/fault_site.sdf", sdf).unwrap();

    let mut hz = String::from("marker\tx\ty\n");
    for h in hazards() {
        let _ = writeln!(hz, "{}\t{:.3}\t{:.3}", h.name, h.x, h.y);
    }
    fs::write("gazebo/hazards.tsv", hz).unwrap();

    // scenarios
    println!("{:<26} {:>7} {:>6}  final supervisor state", "scenario", "frames", "secs");
    for f in Fault::all() {
        let s = run(f);
        let mut out = String::from("t_ms\tx\ty\tz\tyaw\tlook\traise\tnotes\n");
        for fr in &s.frames {
            let _ = writeln!(
                out,
                "{}\t{:.3}\t{:.3}\t{:.3}\t{:.4}\t{}\t{}\t{}",
                fr.t_ms, fr.x, fr.y, fr.z, fr.yaw, fr.look.name(),
                fr.raise.join(","),
                fr.notes.iter().map(|n| clean(n)).collect::<Vec<_>>().join(" || ")
            );
        }
        fs::write(format!("gazebo/scenarios/{}.tsv", f.name()), out).unwrap();
        println!(
            "{:<26} {:>7} {:>6.1}  {}",
            f.name(),
            s.frames.len(),
            s.frames.last().unwrap().t_ms as f64 / 1000.0,
            s.final_state
        );

        if args.iter().any(|a| a == f.name()) {
            println!("\n--- {} ---", f.title());
            for fr in &s.frames {
                for n in &fr.notes {
                    println!("  t={:>5.2}s  [{:<10}] {}", fr.t_ms as f64 / 1000.0, fr.look.name(), n);
                }
            }
            println!();
        }
    }
    println!("\nwrote gazebo/fault_site.sdf, gazebo/hazards.tsv, gazebo/scenarios/*.tsv");
    println!("next:  gz sim -r gazebo/fault_site.sdf");
    println!("then:  python3 gazebo/fault_replay.py <scenario>      (--list to see names)");
}
