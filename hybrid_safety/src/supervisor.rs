//! The safety state machine.
//!
//! ```text
//!            Arm (interlocks)               EngageAuto (interlocks + watchdog)
//!  DISARMED ─────────────────▶ ARMED/MANUAL ───────────────────────────▶ ARMED/AUTO
//!     ▲          Disarm (at rest)   ▲   ▲        ManualTakeover (always)    │   ▲
//!     │                             │   └───────────────────────────────────┘   │
//!     │                             │                                RequestMode │ TransitionComplete
//!     │ ResetFault (at rest,        │ ManualTakeover                            ▼   │
//!     │  e-stop clear)              └──────────────────────────────── MODE TRANSITION
//!     │                                                          (fly↔drive, timed)
//!  FAIL-SAFE ◀── watchdog miss / interlock loss / e-stop / transition timeout
//!   (latched; mode-aware action: brake | hover-then-land | land now)
//! ```

use crate::watchdog::{Source, Watchdog};
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Drive,
    Fly,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Control {
    Manual,
    Auto,
}

/// What the vehicle does when something goes wrong. Chosen by mode *and* cause.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailSafe {
    /// Ground: decelerate to a stop and hold brakes.
    ControlledStop,
    /// Ground: maximum braking (e-stop, actuator fault).
    EmergencyBrake,
    /// Air: hold position, then land (planner lost but we still know where we are).
    HoverThenLand,
    /// Air: land immediately (can't trust position, battery critical, e-stop).
    /// Note: never "cut motors" in the air; that turns a fault into a crash.
    LandNow,
}

#[derive(Debug, Clone, PartialEq)]
pub enum State {
    Disarmed,
    Armed { mode: Mode, control: Control },
    /// Autonomous fly↔drive switch in progress (takeoff or landing).
    Transition { from: Mode, to: Mode, since_ms: u64 },
    /// Latched. Leaves only via ResetFault (at rest) or a pilot takeover.
    FailSafe { mode: Mode, action: FailSafe, reason: String },
}

impl State {
    /// The physical mode the robot is in (for transitions, the conservative one: Fly).
    pub fn mode(&self) -> Option<Mode> {
        match self {
            State::Disarmed => None,
            State::Armed { mode, .. } | State::FailSafe { mode, .. } => Some(*mode),
            State::Transition { .. } => Some(Mode::Fly),
        }
    }
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            State::Disarmed => write!(f, "DISARMED"),
            State::Armed { mode, control } => write!(f, "ARMED/{:?}/{:?}", control, mode),
            State::Transition { from, to, .. } => write!(f, "TRANSITION {:?}->{:?}", from, to),
            State::FailSafe { mode, action, .. } => write!(f, "FAILSAFE[{:?}] {:?}", mode, action),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    Arm,
    Disarm,
    EngageAuto,
    ManualTakeover,
    RequestMode(Mode),
    TransitionComplete,
    EStop,
    ResetFault,
}

/// Inputs from hardware / health monitors. Updated every cycle by the caller.
#[derive(Debug, Clone, Copy)]
pub struct Interlocks {
    pub estop_clear: bool,
    pub localization_ok: bool,
    pub actuators_ok: bool,
    pub battery_pct: u8,
    /// Vehicle is stationary (ground) or landed (air).
    pub at_rest: bool,
    /// Airspace / geofence permits flight here.
    pub airspace_ok: bool,
    /// Landing spot has been checked as drivable terrain.
    pub landing_zone_clear: bool,
}

impl Default for Interlocks {
    fn default() -> Self {
        Self {
            estop_clear: true,
            localization_ok: true,
            actuators_ok: true,
            battery_pct: 100,
            at_rest: true,
            airspace_ok: true,
            landing_zone_clear: true,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub min_battery_arm: u8,
    pub min_battery_takeoff: u8,
    pub critical_battery: u8,
    pub transition_timeout_ms: u64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            min_battery_arm: 30,
            min_battery_takeoff: 50,
            critical_battery: 15,
            transition_timeout_ms: 8_000,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Rejected {
    pub event: Event,
    pub reasons: Vec<String>,
}

impl fmt::Display for Rejected {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?} REJECTED: {}", self.event, self.reasons.join("; "))
    }
}

/// Proof that the supervisor currently allows autonomous actuation.
/// The private field means no code outside this crate can construct one, so
/// "send an autonomous command while disarmed" does not compile.
#[derive(Debug)]
pub struct Permit(());

pub struct Supervisor {
    state: State,
    pub interlocks: Interlocks,
    pub watchdog: Watchdog,
    pub config: Config,
    log: Vec<String>,
}

/// Components that must be alive for autonomy in each mode.
pub fn required_sources(mode: Mode) -> &'static [Source] {
    match mode {
        Mode::Drive => &[
            Source::Planner,
            Source::Localization,
            Source::Lidar,
            Source::DriveController,
        ],
        Mode::Fly => &[
            Source::Planner,
            Source::Localization,
            Source::Lidar,
            Source::FlightController,
        ],
    }
}

impl Supervisor {
    pub fn new(config: Config) -> Self {
        Self {
            state: State::Disarmed,
            interlocks: Interlocks::default(),
            watchdog: Watchdog::new(),
            config,
            log: Vec::new(),
        }
    }

    pub fn state(&self) -> &State {
        &self.state
    }

    pub fn drain_log(&mut self) -> Vec<String> {
        std::mem::take(&mut self.log)
    }

    /// A permit exists only while autonomy is engaged (including mid-transition).
    pub fn permit(&self) -> Option<Permit> {
        match self.state {
            State::Armed { control: Control::Auto, .. } | State::Transition { .. } => {
                Some(Permit(()))
            }
            _ => None,
        }
    }

    fn set(&mut self, next: State, now_ms: u64, why: &str) {
        self.log
            .push(format!("[t={:>6}ms] {} -> {}  ({})", now_ms, self.state, next, why));
        self.state = next;
    }

    fn reject(&mut self, event: Event, reasons: Vec<String>, now_ms: u64) -> Result<(), Rejected> {
        let r = Rejected { event, reasons };
        self.log.push(format!("[t={:>6}ms] {}", now_ms, r));
        Err(r)
    }

    // ---- interlock checks: return every failing reason, not just the first ----

    fn arm_blockers(&self) -> Vec<String> {
        let i = &self.interlocks;
        let mut r = Vec::new();
        if !i.estop_clear {
            r.push("e-stop engaged".into());
        }
        if !i.localization_ok {
            r.push("localization not valid".into());
        }
        if !i.actuators_ok {
            r.push("actuator self-test failed".into());
        }
        if i.battery_pct < self.config.min_battery_arm {
            r.push(format!("battery {}% < {}%", i.battery_pct, self.config.min_battery_arm));
        }
        if !i.at_rest {
            r.push("vehicle not at rest".into());
        }
        r
    }

    fn autonomy_blockers(&self, mode: Mode, now_ms: u64) -> Vec<String> {
        let mut r = self.arm_blockers();
        r.retain(|s| s != "vehicle not at rest"); // can engage while rolling
        for s in self.watchdog.stale_among(required_sources(mode), now_ms) {
            r.push(format!("{} heartbeat stale", s));
        }
        r
    }

    fn mode_switch_blockers(&self, to: Mode, now_ms: u64) -> Vec<String> {
        let i = &self.interlocks;
        let mut r = self.autonomy_blockers(to, now_ms);
        match to {
            Mode::Fly => {
                if !i.airspace_ok {
                    r.push("airspace/geofence does not permit flight".into());
                }
                if i.battery_pct < self.config.min_battery_takeoff {
                    r.push(format!(
                        "battery {}% < {}% takeoff reserve",
                        i.battery_pct, self.config.min_battery_takeoff
                    ));
                }
                if !i.at_rest {
                    r.push("must be stopped before takeoff".into());
                }
            }
            Mode::Drive => {
                if !i.landing_zone_clear {
                    r.push("landing zone not verified drivable".into());
                }
            }
        }
        r
    }

    /// Fail-safe policy: same fault, different response depending on mode.
    fn failsafe_for(&self, mode: Mode, cause: Cause) -> FailSafe {
        match (mode, cause) {
            (Mode::Drive, Cause::EStop | Cause::Actuator) => FailSafe::EmergencyBrake,
            (Mode::Drive, _) => FailSafe::ControlledStop,
            (Mode::Fly, Cause::PlannerLost) => FailSafe::HoverThenLand,
            (Mode::Fly, _) => FailSafe::LandNow,
        }
    }

    fn enter_failsafe(&mut self, cause: Cause, reason: String, now_ms: u64) {
        let mode = self.state.mode().unwrap_or(Mode::Drive);
        let action = self.failsafe_for(mode, cause);
        self.set(State::FailSafe { mode, action, reason: reason.clone() }, now_ms, &reason);
    }

    // ---- commanded events ----

    pub fn handle(&mut self, event: Event, now_ms: u64) -> Result<(), Rejected> {
        use Event::*;
        match (self.state.clone(), event) {
            // E-stop wins from any powered state.
            (State::Disarmed, EStop) => Ok(()),
            (_, EStop) => {
                self.enter_failsafe(Cause::EStop, "e-stop pressed".into(), now_ms);
                Ok(())
            }

            (State::Disarmed, Arm) => {
                let b = self.arm_blockers();
                if !b.is_empty() {
                    return self.reject(event, b, now_ms);
                }
                self.set(State::Armed { mode: Mode::Drive, control: Control::Manual }, now_ms, "arm checks passed");
                Ok(())
            }

            (State::Armed { control: Control::Manual, mode }, Disarm) => {
                if !self.interlocks.at_rest {
                    let why = if mode == Mode::Fly { "cannot disarm in the air" } else { "vehicle still moving" };
                    return self.reject(event, vec![why.into()], now_ms);
                }
                self.set(State::Disarmed, now_ms, "disarmed at rest");
                Ok(())
            }
            (State::Armed { control: Control::Auto, .. }, Disarm) => {
                self.reject(event, vec!["take manual control before disarming".into()], now_ms)
            }

            (State::Armed { control: Control::Manual, mode }, EngageAuto) => {
                let b = self.autonomy_blockers(mode, now_ms);
                if !b.is_empty() {
                    return self.reject(event, b, now_ms);
                }
                self.set(State::Armed { mode, control: Control::Auto }, now_ms, "autonomy engaged");
                Ok(())
            }

            // The human always wins (except over an e-stop, see below).
            (State::Armed { control: Control::Auto, mode }, ManualTakeover) => {
                self.set(State::Armed { mode, control: Control::Manual }, now_ms, "pilot/driver takeover");
                Ok(())
            }
            (State::Transition { .. }, ManualTakeover) => {
                // Mid takeoff/landing: hand over as a flying vehicle; the pilot lands it.
                self.set(State::Armed { mode: Mode::Fly, control: Control::Manual }, now_ms, "takeover during transition");
                Ok(())
            }
            (State::FailSafe { action, mode, .. }, ManualTakeover) => {
                if !self.interlocks.estop_clear {
                    return self.reject(event, vec!["e-stop engaged; takeover not allowed".into()], now_ms);
                }
                let _ = action;
                self.set(State::Armed { mode, control: Control::Manual }, now_ms, "pilot recovers from fail-safe");
                Ok(())
            }

            (State::Armed { control: Control::Auto, mode }, RequestMode(to)) if to != mode => {
                let b = self.mode_switch_blockers(to, now_ms);
                if !b.is_empty() {
                    return self.reject(event, b, now_ms);
                }
                self.set(State::Transition { from: mode, to, since_ms: now_ms }, now_ms, "mode switch interlocks passed");
                Ok(())
            }

            (State::Transition { to, .. }, TransitionComplete) => {
                self.set(State::Armed { mode: to, control: Control::Auto }, now_ms, "transition confirmed by platform");
                Ok(())
            }

            (State::FailSafe { .. }, ResetFault) => {
                let mut b = Vec::new();
                if !self.interlocks.at_rest {
                    b.push("vehicle not at rest / not landed".to_string());
                }
                if !self.interlocks.estop_clear {
                    b.push("e-stop still engaged".to_string());
                }
                if !b.is_empty() {
                    return self.reject(event, b, now_ms);
                }
                // Back to DISARMED, not ARMED: re-arming re-runs every check.
                self.set(State::Disarmed, now_ms, "fault reset; re-arm required");
                Ok(())
            }

            (s, e) => self.reject(e, vec![format!("not valid in state {}", s)], now_ms),
        }
    }

    // ---- continuous monitoring, called every control cycle ----

    pub fn tick(&mut self, now_ms: u64) {
        // E-stop is monitored in every powered state, manual included.
        if !matches!(self.state, State::Disarmed | State::FailSafe { .. }) && !self.interlocks.estop_clear {
            self.enter_failsafe(Cause::EStop, "e-stop line opened".into(), now_ms);
            return;
        }

        // Autonomy-only monitoring: in manual the human is the safety layer.
        let mode = match self.state {
            State::Armed { control: Control::Auto, mode } => mode,
            State::Transition { from, to, since_ms } => {
                if now_ms.saturating_sub(since_ms) > self.config.transition_timeout_ms {
                    let why = format!("{:?}->{:?} transition timed out", from, to);
                    self.enter_failsafe(Cause::Timeout, why, now_ms);
                    return;
                }
                Mode::Fly
            }
            _ => return,
        };

        let i = self.interlocks;
        if !i.actuators_ok {
            self.enter_failsafe(Cause::Actuator, "actuator fault".into(), now_ms);
            return;
        }
        if !i.localization_ok {
            self.enter_failsafe(Cause::Localization, "localization lost".into(), now_ms);
            return;
        }
        if i.battery_pct <= self.config.critical_battery {
            self.enter_failsafe(Cause::Battery, format!("battery critical ({}%)", i.battery_pct), now_ms);
            return;
        }

        let stale = self.watchdog.stale_among(required_sources(mode), now_ms);
        if let Some(first) = stale.first() {
            let cause = if *first == Source::Planner { Cause::PlannerLost } else { Cause::Heartbeat };
            let names: Vec<String> = stale.iter().map(|s| s.to_string()).collect();
            self.enter_failsafe(cause, format!("heartbeat missed: {}", names.join(", ")), now_ms);
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum Cause {
    EStop,
    Actuator,
    Localization,
    Battery,
    PlannerLost,
    Heartbeat,
    Timeout,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy(now: u64) -> Supervisor {
        let mut s = Supervisor::new(Config::default());
        for src in [
            Source::Planner,
            Source::Localization,
            Source::Lidar,
            Source::DriveController,
            Source::FlightController,
        ] {
            s.watchdog.register(src, 200, now);
        }
        s
    }

    fn beat_all(s: &mut Supervisor, now: u64) {
        for src in [
            Source::Planner,
            Source::Localization,
            Source::Lidar,
            Source::DriveController,
            Source::FlightController,
        ] {
            s.watchdog.heartbeat(src, now);
        }
    }

    fn to_auto_drive(s: &mut Supervisor) {
        s.handle(Event::Arm, 0).unwrap();
        s.handle(Event::EngageAuto, 0).unwrap();
    }

    fn to_auto_fly(s: &mut Supervisor) {
        to_auto_drive(s);
        s.handle(Event::RequestMode(Mode::Fly), 0).unwrap();
        s.handle(Event::TransitionComplete, 0).unwrap();
        s.interlocks.at_rest = false;
    }

    #[test]
    fn cannot_arm_with_estop_and_reports_all_reasons() {
        let mut s = healthy(0);
        s.interlocks.estop_clear = false;
        s.interlocks.battery_pct = 10;
        let err = s.handle(Event::Arm, 0).unwrap_err();
        assert_eq!(err.reasons.len(), 2);
        assert_eq!(*s.state(), State::Disarmed);
    }

    #[test]
    fn no_permit_unless_auto() {
        let mut s = healthy(0);
        assert!(s.permit().is_none());
        s.handle(Event::Arm, 0).unwrap();
        assert!(s.permit().is_none());
        s.handle(Event::EngageAuto, 0).unwrap();
        assert!(s.permit().is_some());
    }

    #[test]
    fn engage_rejected_when_required_heartbeat_stale() {
        let mut s = healthy(0);
        s.handle(Event::Arm, 0).unwrap();
        let err = s.handle(Event::EngageAuto, 500).unwrap_err();
        assert!(err.reasons.iter().any(|r| r.contains("planner")));
    }

    #[test]
    fn manual_takeover_always_wins_over_auto() {
        let mut s = healthy(0);
        to_auto_drive(&mut s);
        s.interlocks.localization_ok = false; // even with a bad sensor
        s.handle(Event::ManualTakeover, 0).unwrap();
        assert_eq!(*s.state(), State::Armed { mode: Mode::Drive, control: Control::Manual });
    }

    #[test]
    fn planner_loss_drive_vs_fly_gives_different_failsafe() {
        let mut d = healthy(0);
        to_auto_drive(&mut d);
        d.tick(300);
        assert!(matches!(d.state(), State::FailSafe { action: FailSafe::ControlledStop, .. }));

        let mut f = healthy(0);
        to_auto_fly(&mut f);
        // keep everything but the planner alive
        for src in [Source::Localization, Source::Lidar, Source::FlightController] {
            f.watchdog.heartbeat(src, 300);
        }
        f.tick(300);
        assert!(matches!(f.state(), State::FailSafe { action: FailSafe::HoverThenLand, .. }));
    }

    #[test]
    fn localization_loss_in_air_lands_now() {
        let mut s = healthy(0);
        to_auto_fly(&mut s);
        s.interlocks.localization_ok = false;
        s.tick(50);
        assert!(matches!(s.state(), State::FailSafe { action: FailSafe::LandNow, mode: Mode::Fly, .. }));
    }

    #[test]
    fn estop_in_air_lands_instead_of_cutting_motors_and_blocks_takeover() {
        let mut s = healthy(0);
        to_auto_fly(&mut s);
        s.interlocks.estop_clear = false;
        s.handle(Event::EStop, 10).unwrap();
        assert!(matches!(s.state(), State::FailSafe { action: FailSafe::LandNow, .. }));
        assert!(s.handle(Event::ManualTakeover, 20).is_err());
    }

    #[test]
    fn cannot_disarm_in_air() {
        let mut s = healthy(0);
        to_auto_fly(&mut s);
        s.handle(Event::ManualTakeover, 0).unwrap();
        let err = s.handle(Event::Disarm, 0).unwrap_err();
        assert!(err.reasons[0].contains("air"));
    }

    #[test]
    fn takeoff_blocked_by_low_battery_and_geofence() {
        let mut s = healthy(0);
        to_auto_drive(&mut s);
        s.interlocks.battery_pct = 40;
        s.interlocks.airspace_ok = false;
        let err = s.handle(Event::RequestMode(Mode::Fly), 0).unwrap_err();
        assert_eq!(err.reasons.len(), 2);
    }

    #[test]
    fn landing_blocked_when_zone_not_drivable() {
        let mut s = healthy(0);
        to_auto_fly(&mut s);
        s.interlocks.landing_zone_clear = false;
        assert!(s.handle(Event::RequestMode(Mode::Drive), 0).is_err());
    }

    #[test]
    fn transition_timeout_lands() {
        let mut s = healthy(0);
        to_auto_fly(&mut s);
        beat_all(&mut s, 1_000);
        s.handle(Event::RequestMode(Mode::Drive), 1_000).unwrap();
        beat_all(&mut s, 9_500);
        s.tick(9_500);
        assert!(matches!(s.state(), State::FailSafe { action: FailSafe::LandNow, .. }));
    }

    #[test]
    fn fault_is_latched_and_reset_requires_rest_then_rearm() {
        let mut s = healthy(0);
        to_auto_fly(&mut s);
        s.interlocks.localization_ok = false;
        s.tick(10);
        // still flying: reset refused
        assert!(s.handle(Event::ResetFault, 20).is_err());
        // fault stays latched even after the sensor recovers
        s.interlocks.localization_ok = true;
        s.tick(30);
        assert!(matches!(s.state(), State::FailSafe { .. }));
        s.interlocks.at_rest = true; // landed
        s.handle(Event::ResetFault, 40).unwrap();
        assert_eq!(*s.state(), State::Disarmed);
        assert!(s.permit().is_none());
    }

    #[test]
    fn estop_line_monitored_in_manual_too() {
        let mut s = healthy(0);
        s.handle(Event::Arm, 0).unwrap();
        s.interlocks.estop_clear = false;
        s.tick(10);
        assert!(matches!(s.state(), State::FailSafe { action: FailSafe::EmergencyBrake, .. }));
    }
}
