//! Scenario demo: `cargo run` (all) or `cargo run -- <n>` (one scenario).
//! Simulated clock; every line printed is a real state transition or rejection
//! produced by the library.

use hybrid_safety::*;

const ALL: [Source; 5] = [
    Source::Planner,
    Source::Localization,
    Source::Lidar,
    Source::DriveController,
    Source::FlightController,
];

struct Sim {
    sup: Supervisor,
    ground: DiffDrive,
    air: Multicopter,
    t: u64,
    dead: Vec<Source>,
}

impl Sim {
    fn new() -> Self {
        let mut sup = Supervisor::new(Config::default());
        for s in ALL {
            sup.watchdog.register(s, 200, 0);
        }
        Self {
            sup,
            ground: DiffDrive { wheel_base_m: 0.16, max_speed: 0.22 },
            air: Multicopter { max_vz: 1.0 },
            t: 0,
            dead: vec![],
        }
    }

    /// Advance the clock in 50 ms control cycles; live components send heartbeats.
    fn run(&mut self, ms: u64) {
        let end = self.t + ms;
        while self.t < end {
            self.t += 50;
            for s in ALL {
                if !self.dead.contains(&s) {
                    self.sup.watchdog.heartbeat(s, self.t);
                }
            }
            self.sup.tick(self.t);
            self.flush();
        }
    }

    fn ev(&mut self, e: Event) {
        println!("  >> {:?}", e);
        let _ = self.sup.handle(e, self.t);
        self.flush();
    }

    fn note(&self, s: &str) {
        println!("  -- {}", s);
    }

    fn show_output(&self) {
        let sp = Setpoint { vx: 0.2, vz: 0.5, yaw_rate: 0.3 };
        println!("     actuator cmd: {:?}", self.sup.output(&sp, &self.ground, &self.air));
    }

    fn flush(&mut self) {
        for l in self.sup.drain_log() {
            println!("     {}", l);
        }
    }
}

fn header(n: u32, title: &str) {
    println!("\n=== Scenario {}: {} ===", n, title);
}

fn s1_nominal() {
    header(1, "nominal mission: drive -> take off -> fly -> land -> drive -> hand back");
    let mut s = Sim::new();
    s.ev(Event::Arm);
    s.show_output();
    s.ev(Event::EngageAuto);
    s.show_output();
    s.sup.interlocks.at_rest = false;
    s.run(1000);
    s.note("reached takeoff point, stopping");
    s.sup.interlocks.at_rest = true;
    s.ev(Event::RequestMode(Mode::Fly));
    s.run(2000);
    s.ev(Event::TransitionComplete);
    s.sup.interlocks.at_rest = false;
    s.show_output();
    s.run(1000);
    s.ev(Event::RequestMode(Mode::Drive));
    s.run(3000);
    s.ev(Event::TransitionComplete);
    s.show_output();
    s.ev(Event::ManualTakeover);
    s.show_output();
    s.sup.interlocks.at_rest = true;
    s.ev(Event::Disarm);
}

fn s2_planner_lost() {
    header(2, "planner crashes: same fault, mode-aware response");
    let mut s = Sim::new();
    s.ev(Event::Arm);
    s.ev(Event::EngageAuto);
    s.note("driving; planner process dies");
    s.dead.push(Source::Planner);
    s.run(400);
    s.show_output();

    let mut f = Sim::new();
    f.ev(Event::Arm);
    f.ev(Event::EngageAuto);
    f.ev(Event::RequestMode(Mode::Fly));
    f.ev(Event::TransitionComplete);
    f.note("flying; planner process dies");
    f.dead.push(Source::Planner);
    f.run(400);
    f.show_output();
}

fn s3_localization_in_air() {
    header(3, "localization lost mid-flight; fault is latched");
    let mut s = Sim::new();
    s.ev(Event::Arm);
    s.ev(Event::EngageAuto);
    s.ev(Event::RequestMode(Mode::Fly));
    s.ev(Event::TransitionComplete);
    s.sup.interlocks.at_rest = false;
    s.run(500);
    s.note("SLAM diverges");
    s.sup.interlocks.localization_ok = false;
    s.run(100);
    s.show_output();
    s.ev(Event::ResetFault);
    s.note("localization recovers; fault stays latched");
    s.sup.interlocks.localization_ok = true;
    s.run(200);
    println!("     state is still: {}", s.sup.state());
    s.note("landed");
    s.sup.interlocks.at_rest = true;
    s.ev(Event::ResetFault);
    s.ev(Event::EngageAuto);
    s.ev(Event::Arm);
}

fn s4_estop_in_air() {
    header(4, "e-stop while flying: land, never cut motors; human can't override");
    let mut s = Sim::new();
    s.ev(Event::Arm);
    s.ev(Event::EngageAuto);
    s.ev(Event::RequestMode(Mode::Fly));
    s.ev(Event::TransitionComplete);
    s.sup.interlocks.at_rest = false;
    s.run(300);
    s.note("operator hits the e-stop (hardware line opens)");
    s.sup.interlocks.estop_clear = false;
    s.run(50);
    s.show_output();
    s.ev(Event::ManualTakeover);
}

fn s5_rejections() {
    header(5, "interlocks refuse unsafe requests (and say why)");
    let mut s = Sim::new();
    s.sup.interlocks.battery_pct = 20;
    s.sup.interlocks.localization_ok = false;
    s.ev(Event::Arm);
    s.sup.interlocks.battery_pct = 45;
    s.sup.interlocks.localization_ok = true;
    s.ev(Event::Arm);
    s.ev(Event::Disarm);
    s.ev(Event::Arm);
    s.ev(Event::EngageAuto);
    s.ev(Event::Disarm);
    s.sup.interlocks.airspace_ok = false;
    s.ev(Event::RequestMode(Mode::Fly));
}

fn s6_transition_timeout() {
    header(6, "landing never confirmed: transition watchdog fires");
    let mut s = Sim::new();
    s.ev(Event::Arm);
    s.ev(Event::EngageAuto);
    s.ev(Event::RequestMode(Mode::Fly));
    s.ev(Event::TransitionComplete);
    s.ev(Event::RequestMode(Mode::Drive));
    s.note("touchdown never detected (e.g. landing on soft ground)");
    s.run(8_200);
    s.show_output();
}

fn main() {
    let pick: Option<u32> = std::env::args().nth(1).and_then(|a| a.parse().ok());
    let scenarios: [(u32, fn()); 6] = [
        (1, s1_nominal),
        (2, s2_planner_lost),
        (3, s3_localization_in_air),
        (4, s4_estop_in_air),
        (5, s5_rejections),
        (6, s6_transition_timeout),
    ];
    for (n, f) in scenarios {
        if pick.map_or(true, |p| p == n) {
            f();
        }
    }
}
