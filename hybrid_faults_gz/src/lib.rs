//! Fault-injection scenarios on the hybrid robot's real mission route.
//!
//! The robot follows the route produced by `hybrid_safety::survey::run_mission`
//! (draft map -> mode-typed A* -> local repairs). This crate replays that route
//! in a 50 ms control loop with a **real `Supervisor`**: heartbeats are sent
//! every tick, `tick()` runs every tick, takeoff/landing go through
//! `RequestMode` interlocks. A fault is injected at a chosen moment and
//! **the supervisor decides** the reaction. This layer only turns the decided
//! fail-safe action into motion (brake, hold, land) for Gazebo.
//!
//! One thing is implemented here and not in the supervisor: the escalation of
//! `HoverThenLand` (hold 3 s, then fly back to the last ground-verified cell
//! and land). It is labelled as such in the narration.

use hybrid_safety::survey::{run_mission, Belief, MissionConfig, MissionEvent, Terrain, World};
use hybrid_safety::{Config, Control, Event, FailSafe, Mode, Source, State, Supervisor};

pub const TICK_MS: u64 = 50;
pub const Z_DRIVE: f64 = 0.22;
pub const Z_FLY: f64 = 2.5;
const STEP_TICKS: u32 = 8; // 0.4 s per cell
const SWITCH_TICKS: u32 = 40; // 2 s takeoff / landing
const SOURCES: [Source; 5] = [
    Source::Planner,
    Source::Localization,
    Source::Lidar,
    Source::DriveController,
    Source::FlightController,
];

/// Robot color in Gazebo = supervisor state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Look {
    Disarmed,
    Manual,
    Auto,
    Transition,
    FailSafe,
}

impl Look {
    pub fn name(self) -> &'static str {
        match self {
            Look::Disarmed => "disarmed",
            Look::Manual => "manual",
            Look::Auto => "auto",
            Look::Transition => "transition",
            Look::FailSafe => "failsafe",
        }
    }
    pub fn all() -> [Look; 5] {
        [Look::Disarmed, Look::Manual, Look::Auto, Look::Transition, Look::FailSafe]
    }
    fn of(s: &State) -> Look {
        match s {
            State::Disarmed => Look::Disarmed,
            State::Armed { control: Control::Manual, .. } => Look::Manual,
            State::Armed { control: Control::Auto, .. } => Look::Auto,
            State::Transition { .. } => Look::Transition,
            State::FailSafe { .. } => Look::FailSafe,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Frame {
    pub t_ms: u64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub yaw: f64,
    pub look: Look,
    pub raise: Vec<String>,
    pub notes: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    Nominal,
    PlannerDiesDriving,
    PlannerDiesFlying,
    LocalizationLostFlying,
    EStopDriving,
    EStopFlying,
    TakeoffRejected,
    LandingNeverConfirmed,
}

impl Fault {
    pub fn all() -> [Fault; 8] {
        [
            Fault::Nominal,
            Fault::PlannerDiesDriving,
            Fault::PlannerDiesFlying,
            Fault::LocalizationLostFlying,
            Fault::EStopDriving,
            Fault::EStopFlying,
            Fault::TakeoffRejected,
            Fault::LandingNeverConfirmed,
        ]
    }
    pub fn name(self) -> &'static str {
        match self {
            Fault::Nominal => "nominal",
            Fault::PlannerDiesDriving => "planner_dies_driving",
            Fault::PlannerDiesFlying => "planner_dies_flying",
            Fault::LocalizationLostFlying => "localization_lost_flying",
            Fault::EStopDriving => "estop_driving",
            Fault::EStopFlying => "estop_flying",
            Fault::TakeoffRejected => "takeoff_rejected",
            Fault::LandingNeverConfirmed => "landing_never_confirmed",
        }
    }
    pub fn title(self) -> &'static str {
        match self {
            Fault::Nominal => "Nominal mission, no faults (for comparison)",
            Fault::PlannerDiesDriving => "Planner process dies while driving -> controlled stop, then operator takes over",
            Fault::PlannerDiesFlying => "Planner process dies over the river -> hold, then land at last verified-free cell",
            Fault::LocalizationLostFlying => "Localization lost over the river -> land now; fault latched until reset on the ground",
            Fault::EStopDriving => "E-stop while driving -> emergency brake",
            Fault::EStopFlying => "E-stop over the river -> land now (never cut motors); takeover refused",
            Fault::TakeoffRejected => "Battery 45% at the river bank -> takeoff interlock refuses; robot waits on the bank",
            Fault::LandingNeverConfirmed => "Touchdown never confirmed -> transition watchdog fires after 8 s -> land now",
        }
    }
    pub fn from_name(s: &str) -> Option<Fault> {
        Fault::all().into_iter().find(|f| f.name() == s)
    }
}

pub struct Scenario {
    pub fault: Fault,
    pub frames: Vec<Frame>,
    pub final_state: State,
}

/// Grid cell -> Gazebo metres (same mapping as hybrid_safety's export_gazebo).
pub fn site_xy(cx: usize, cy: usize) -> (f64, f64) {
    (cx as f64 - 24.0, 8.0 - cy as f64)
}
fn cell_of(x: f64, y: f64) -> (usize, usize) {
    ((x + 24.0).round().max(0.0) as usize, (8.0 - y).round().max(0.0) as usize)
}

/// Hazard markers in the same order/names as hybrid_safety's export_gazebo:
/// (trail step, marker name, x, y).
pub struct Hazard {
    pub step: usize,
    pub name: String,
    pub x: f64,
    pub y: f64,
}

struct Route {
    trail: Vec<(usize, usize, Mode)>,
    hazards: Vec<Hazard>,
    step_notes: Vec<(usize, String)>,
}

fn route(world: &World) -> Route {
    let mut sup = Supervisor::new(Config::default());
    let r = run_mission(world, &mut sup, &MissionConfig::default());
    let mut hazards = Vec::new();
    let mut step_notes = Vec::new();
    let mut n = 0;
    for e in &r.events {
        match e {
            MissionEvent::Hazard { step, corrections } => {
                let mut cells = Vec::new();
                for c in corrections {
                    let (x, y) = site_xy(c.x, c.y);
                    hazards.push(Hazard { step: *step, name: format!("hazard_{n}"), x, y });
                    cells.push(format!("({},{})", c.x, c.y));
                    n += 1;
                }
                step_notes.push((*step, format!("planner: ground sensing contradicts aerial draft at {}", cells.join(" "))));
            }
            MissionEvent::LocalRepair { step, detour_len, rejoined_at } => step_notes.push((
                *step,
                format!("planner: local repair, +{} step detour, rejoins plan at ({},{})", detour_len, rejoined_at.0, rejoined_at.1),
            )),
            _ => {}
        }
    }
    Route { trail: r.trail.iter().map(|s| (s.x, s.y, s.mode)).collect(), hazards, step_notes }
}

pub fn hazards() -> Vec<Hazard> {
    route(&World::demo()).hazards
}

struct Sim<'a> {
    sup: Supervisor,
    world: &'a World,
    t: u64,
    x: f64,
    y: f64,
    z: f64,
    yaw: f64,
    dead: Vec<Source>,
    frames: Vec<Frame>,
    notes: Vec<String>,
    raise: Vec<String>,
}

impl<'a> Sim<'a> {
    fn new(world: &'a World, start: (usize, usize)) -> Self {
        let mut sup = Supervisor::new(Config::default());
        for s in SOURCES {
            sup.watchdog.register(s, 200, 0);
        }
        let (x, y) = site_xy(start.0, start.1);
        let mut sim = Sim {
            sup, world, t: 0, x, y, z: Z_DRIVE, yaw: 0.0,
            dead: vec![], frames: vec![], notes: vec![], raise: vec![],
        };
        sim.frame();
        sim
    }

    fn failsafe(&self) -> Option<FailSafe> {
        match self.sup.state() {
            State::FailSafe { action, .. } => Some(*action),
            _ => None,
        }
    }

    fn note(&mut self, s: impl Into<String>) {
        self.notes.push(s.into());
    }

    fn frame(&mut self) {
        let mut notes = std::mem::take(&mut self.notes);
        for l in self.sup.drain_log() {
            notes.push(format!("supervisor {}", l.trim()));
        }
        self.frames.push(Frame {
            t_ms: self.t,
            x: self.x,
            y: self.y,
            z: self.z,
            yaw: self.yaw,
            look: Look::of(self.sup.state()),
            raise: std::mem::take(&mut self.raise),
            notes,
        });
    }

    /// One 50 ms control cycle: live components send heartbeats, supervisor ticks.
    fn tick(&mut self) {
        self.t += TICK_MS;
        for s in SOURCES {
            if !self.dead.contains(&s) {
                self.sup.watchdog.heartbeat(s, self.t);
            }
        }
        self.sup.tick(self.t);
        self.frame();
    }

    fn event(&mut self, e: Event) -> bool {
        let ok = self.sup.handle(e, self.t).is_ok();
        self.frame();
        ok
    }

    fn hold(&mut self, ticks: u32) {
        for _ in 0..ticks {
            self.tick();
        }
    }

    /// Move toward a target over `ticks` cycles. Stops early (returns false)
    /// the moment the supervisor enters fail-safe.
    fn move_to(&mut self, tx: f64, ty: f64, tz: f64, ticks: u32) -> bool {
        let (x0, y0, z0) = (self.x, self.y, self.z);
        if (tx - x0).abs() + (ty - y0).abs() > 1e-6 {
            self.yaw = (ty - y0).atan2(tx - x0);
        }
        for k in 1..=ticks {
            let a = k as f64 / ticks as f64;
            self.x = x0 + (tx - x0) * a;
            self.y = y0 + (ty - y0) * a;
            self.z = z0 + (tz - z0) * a;
            self.tick();
            if self.failsafe().is_some() {
                return false;
            }
        }
        true
    }

    /// Execute the fail-safe the supervisor chose. Returns after the vehicle is safe.
    fn respond(&mut self, takeoff_cell: (usize, usize)) {
        let Some(action) = self.failsafe() else { return };
        match action {
            FailSafe::ControlledStop => {
                self.note("motion: controlled stop (gentle deceleration, brakes held)");
                let (dx, dy) = (self.yaw.cos() * 0.4, self.yaw.sin() * 0.4);
                let (x0, y0) = (self.x, self.y);
                for k in 1..=10 {
                    let a = 1.0 - (1.0 - k as f64 / 10.0).powi(2); // ease out
                    self.x = x0 + dx * a;
                    self.y = y0 + dy * a;
                    self.tick();
                }
                self.sup.interlocks.at_rest = true;
                self.hold(20);
            }
            FailSafe::EmergencyBrake => {
                self.note("motion: emergency brake, stopped immediately");
                self.sup.interlocks.at_rest = true;
                self.hold(20);
            }
            FailSafe::HoverThenLand => {
                self.note("motion: PX4 Hold. Still localized, so it holds position and waits for an operator");
                self.hold(60);
                self.note("motion: no operator response after 3 s -> fly back to last ground-verified cell (takeoff point) and land  [escalation implemented in this sim layer, not yet in the supervisor]");
                let (tx, ty) = site_xy(takeoff_cell.0, takeoff_cell.1);
                let dist = ((tx - self.x).powi(2) + (ty - self.y).powi(2)).sqrt();
                self.move_to_unchecked(tx, ty, Z_FLY, (dist * 10.0).ceil().max(4.0) as u32);
                self.move_to_unchecked(tx, ty, Z_DRIVE, SWITCH_TICKS);
                self.note("motion: landed on verified-free ground");
                self.sup.interlocks.at_rest = true;
                self.hold(10);
            }
            FailSafe::LandNow => {
                self.note("motion: PX4 Land, descending in place");
                let (x, y) = (self.x, self.y);
                self.move_to_unchecked(x, y, 0.05, SWITCH_TICKS);
                let (cx, cy) = cell_of(self.x, self.y);
                if self.world.terrain(cx, cy) == Terrain::Water {
                    self.note("motion: touched down IN THE RIVER. LandNow lands where it is; that is the trade-off of landing immediately (design discussion point)");
                } else {
                    self.note("motion: landed");
                }
                self.sup.interlocks.at_rest = true;
                self.hold(10);
            }
        }
    }

    /// Movement during fail-safe execution (the supervisor is already in fail-safe).
    fn move_to_unchecked(&mut self, tx: f64, ty: f64, tz: f64, ticks: u32) {
        let (x0, y0, z0) = (self.x, self.y, self.z);
        for k in 1..=ticks {
            let a = k as f64 / ticks as f64;
            self.x = x0 + (tx - x0) * a;
            self.y = y0 + (ty - y0) * a;
            self.z = z0 + (tz - z0) * a;
            self.tick();
        }
    }
}

pub fn run(fault: Fault) -> Scenario {
    let world = World::demo();
    let rt = route(&world);
    let mut sim = Sim::new(&world, (rt.trail[0].0, rt.trail[0].1));
    sim.note(format!("scenario: {}", fault.title()));

    sim.sup.interlocks.at_rest = true;
    sim.event(Event::Arm);
    sim.event(Event::EngageAuto);
    sim.hold(4);

    let mut takeoff_cell = (rt.trail[0].0, rt.trail[0].1);
    let mut fly_moves = 0;

    'mission: for i in 0..rt.trail.len() - 1 {
        // things discovered on arrival at step i
        for h in rt.hazards.iter().filter(|h| h.step == i) {
            sim.raise.push(h.name.clone());
        }
        for (_, n) in rt.step_notes.iter().filter(|(s, _)| *s == i) {
            sim.notes.push(n.clone());
        }

        // drive-time fault injection
        if i == 8 && fault == Fault::PlannerDiesDriving {
            sim.note("FAULT INJECTED: planner process killed (stops publishing)");
            sim.dead.push(Source::Planner);
        }
        if i == 18 && fault == Fault::EStopDriving {
            sim.note("FAULT INJECTED: operator presses the e-stop");
            sim.sup.interlocks.estop_clear = false;
        }

        let (ax, ay, am) = rt.trail[i];
        let (bx, by, bm) = rt.trail[i + 1];

        if am != bm {
            // takeoff or landing: stop, ask the supervisor
            sim.sup.interlocks.at_rest = true;
            sim.sup.interlocks.landing_zone_clear = true;
            if bm == Mode::Fly {
                takeoff_cell = (ax, ay);
                if fault == Fault::TakeoffRejected {
                    sim.note("FAULT INJECTED: battery reads 45% at the river bank");
                    sim.sup.interlocks.battery_pct = 45;
                }
            }
            if !sim.event(Event::RequestMode(bm)) {
                sim.note("motion: request refused -> robot stays on the bank, stopped, autonomy still engaged");
                sim.hold(60);
                break 'mission;
            }
            let z_to = if bm == Mode::Fly { Z_FLY } else { Z_DRIVE };
            if bm == Mode::Drive && fault == Fault::LandingNeverConfirmed {
                sim.note("FAULT INJECTED: touchdown sensor never confirms landing");
                let (x, y) = (sim.x, sim.y);
                sim.move_to(x, y, 0.9, 30);
                for _ in 0..400 {
                    if sim.failsafe().is_some() {
                        break;
                    }
                    sim.tick();
                }
                break 'mission;
            }
            let (x, y) = (sim.x, sim.y);
            if !sim.move_to(x, y, z_to, SWITCH_TICKS) {
                break 'mission;
            }
            sim.event(Event::TransitionComplete);
            sim.sup.interlocks.at_rest = false;
            continue;
        }

        // ordinary move between cells
        if am == Mode::Fly {
            fly_moves += 1;
            // second fly move starts over the water
            if fly_moves == 2 {
                match fault {
                    Fault::PlannerDiesFlying => {
                        sim.note("FAULT INJECTED: planner process killed over the river");
                        sim.dead.push(Source::Planner);
                    }
                    Fault::LocalizationLostFlying => {
                        sim.note("FAULT INJECTED: SLAM diverges over the river (localization invalid)");
                        sim.sup.interlocks.localization_ok = false;
                    }
                    Fault::EStopFlying => {
                        sim.note("FAULT INJECTED: operator presses the e-stop over the river");
                        sim.sup.interlocks.estop_clear = false;
                    }
                    _ => {}
                }
            }
        } else {
            sim.sup.interlocks.at_rest = false;
        }
        let (tx, ty) = site_xy(bx, by);
        let tz = if am == Mode::Fly { Z_FLY } else { Z_DRIVE };
        if !sim.move_to(tx, ty, tz, STEP_TICKS) {
            break 'mission;
        }
    }

    // fail-safe execution and recovery, all decided/checked by the supervisor
    if sim.failsafe().is_some() {
        if fault == Fault::EStopFlying {
            sim.hold(10);
            sim.note("operator: tries to take manual control");
            sim.event(Event::ManualTakeover);
        }
        sim.respond(takeoff_cell);
        match fault {
            Fault::PlannerDiesDriving => {
                sim.note("planner restarts and publishes again; fault stays latched");
                sim.dead.clear();
                sim.hold(10);
                sim.note("operator: takes manual control");
                sim.event(Event::ManualTakeover);
                sim.hold(10);
            }
            Fault::PlannerDiesFlying | Fault::LandingNeverConfirmed => {
                sim.note("operator: resets the fault on the ground");
                sim.event(Event::ResetFault);
                sim.hold(10);
            }
            Fault::LocalizationLostFlying => {
                sim.note("localization recovers; fault stays latched");
                sim.sup.interlocks.localization_ok = true;
                sim.hold(10);
                sim.note("operator: resets the fault on the ground");
                sim.event(Event::ResetFault);
                sim.hold(10);
            }
            Fault::EStopFlying | Fault::EStopDriving => {
                sim.note("operator: tries to reset with the e-stop still pressed");
                sim.event(Event::ResetFault);
                sim.hold(10);
                sim.note("operator: releases the e-stop and resets");
                sim.sup.interlocks.estop_clear = true;
                sim.event(Event::ResetFault);
                sim.hold(10);
            }
            _ => {}
        }
    } else if fault == Fault::Nominal || fault == Fault::TakeoffRejected {
        if fault == Fault::Nominal {
            for (_, n) in rt.step_notes.iter().filter(|(s, _)| *s == rt.trail.len() - 1) {
                sim.notes.push(n.clone());
            }
            sim.note("ARRIVED at goal");
        }
        sim.hold(10);
    }

    let final_state = sim.sup.state().clone();
    Scenario { fault, frames: sim.frames, final_state }
}

/// Cells believed free in the aerial draft (for documentation/tests).
pub fn draft_free(x: usize, y: usize) -> bool {
    let world = World::demo();
    hybrid_safety::survey::aerial_observe(&world, x, y) == Belief::Free
}

#[cfg(test)]
mod tests {
    use super::*;

    fn last_action(s: &Scenario) -> Vec<String> {
        s.frames.iter().flat_map(|f| f.notes.clone()).collect()
    }
    fn saw(s: &Scenario, needle: &str) -> bool {
        last_action(s).iter().any(|n| n.contains(needle))
    }

    #[test]
    fn nominal_arrives_in_auto_drive() {
        let s = run(Fault::Nominal);
        assert_eq!(s.final_state, State::Armed { mode: Mode::Drive, control: Control::Auto });
        let f = s.frames.last().unwrap();
        let (gx, gy) = site_xy(45, 8);
        assert!((f.x - gx).abs() < 1e-6 && (f.y - gy).abs() < 1e-6);
        assert!(s.frames.iter().any(|f| f.z > 2.0)); // flew
    }

    #[test]
    fn planner_dies_driving_stops_then_manual() {
        let s = run(Fault::PlannerDiesDriving);
        assert!(saw(&s, "ControlledStop"));
        assert_eq!(s.final_state, State::Armed { mode: Mode::Drive, control: Control::Manual });
        assert!(s.frames.iter().all(|f| f.z < 1.0)); // never flew
    }

    #[test]
    fn planner_dies_flying_holds_then_lands_on_free_ground() {
        let s = run(Fault::PlannerDiesFlying);
        assert!(saw(&s, "HoverThenLand"));
        let f = s.frames.last().unwrap();
        assert!(f.z < 0.3);
        let (cx, cy) = cell_of(f.x, f.y);
        assert!(World::demo().terrain(cx, cy).drivable());
        assert_eq!(s.final_state, State::Disarmed);
    }

    #[test]
    fn localization_lost_flying_lands_now_and_resets_to_disarmed() {
        let s = run(Fault::LocalizationLostFlying);
        assert!(saw(&s, "LandNow"));
        assert_eq!(s.final_state, State::Disarmed);
    }

    #[test]
    fn estop_flying_lands_refuses_takeover_and_needs_release() {
        let s = run(Fault::EStopFlying);
        assert!(saw(&s, "LandNow"));
        assert!(saw(&s, "takeover not allowed"));
        assert!(saw(&s, "e-stop still engaged"));
        assert_eq!(s.final_state, State::Disarmed);
    }

    #[test]
    fn estop_driving_emergency_brakes() {
        let s = run(Fault::EStopDriving);
        assert!(saw(&s, "EmergencyBrake"));
        assert_eq!(s.final_state, State::Disarmed);
    }

    #[test]
    fn takeoff_rejected_stays_on_bank() {
        let s = run(Fault::TakeoffRejected);
        assert!(saw(&s, "takeoff reserve"));
        assert!(s.frames.iter().all(|f| f.z < 1.0));
        assert_eq!(s.final_state, State::Armed { mode: Mode::Drive, control: Control::Auto });
    }

    #[test]
    fn landing_never_confirmed_times_out_and_lands() {
        let s = run(Fault::LandingNeverConfirmed);
        assert!(saw(&s, "timed out"));
        assert!(s.frames.last().unwrap().z < 0.3);
        assert_eq!(s.final_state, State::Disarmed);
    }
}
