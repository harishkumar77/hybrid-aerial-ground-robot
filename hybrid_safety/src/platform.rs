//! Hardware abstraction: the safety logic above is shared; only the mapping
//! from "setpoint" / "fail-safe action" to actuator commands is per platform.
//! Adding a new vehicle means implementing this trait, not touching the
//! state machine.

use crate::supervisor::{FailSafe, Mode, Permit, State, Supervisor};

/// Generic body-frame setpoint from the planner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Setpoint {
    /// Forward velocity (m/s).
    pub vx: f32,
    /// Vertical velocity (m/s), ignored on the ground.
    pub vz: f32,
    /// Yaw rate (rad/s).
    pub yaw_rate: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ActuatorCmd {
    /// Nothing is commanded; motors not energized.
    Idle,
    /// Human has control; autonomy outputs are ignored.
    PilotControl,
    Wheels { left: f32, right: f32 },
    Brake { hard: bool },
    /// PX4 offboard velocity setpoint (TrajectorySetpoint).
    Px4Velocity { vx: f32, vz: f32, yaw_rate: f32 },
    /// PX4 hold / loiter.
    Px4Hold,
    /// PX4 land.
    Px4Land,
}

pub trait Platform {
    fn name(&self) -> &'static str;
    fn mode(&self) -> Mode;
    /// Requires a `Permit`: only reachable when the supervisor allows autonomy.
    fn translate(&self, sp: &Setpoint, permit: &Permit) -> ActuatorCmd;
    fn failsafe(&self, action: FailSafe) -> ActuatorCmd;
}

/// Differential-drive ground robot (e.g. TurtleBot3-class).
pub struct DiffDrive {
    pub wheel_base_m: f32,
    pub max_speed: f32,
}

impl Platform for DiffDrive {
    fn name(&self) -> &'static str {
        "diff-drive ground"
    }
    fn mode(&self) -> Mode {
        Mode::Drive
    }
    fn translate(&self, sp: &Setpoint, _permit: &Permit) -> ActuatorCmd {
        let half = self.wheel_base_m / 2.0;
        let clamp = |v: f32| v.clamp(-self.max_speed, self.max_speed);
        ActuatorCmd::Wheels {
            left: clamp(sp.vx - sp.yaw_rate * half),
            right: clamp(sp.vx + sp.yaw_rate * half),
        }
    }
    fn failsafe(&self, action: FailSafe) -> ActuatorCmd {
        match action {
            FailSafe::ControlledStop => ActuatorCmd::Brake { hard: false },
            _ => ActuatorCmd::Brake { hard: true },
        }
    }
}

/// PX4 multicopter (offboard control over Micro XRCE-DDS).
pub struct Multicopter {
    pub max_vz: f32,
}

impl Platform for Multicopter {
    fn name(&self) -> &'static str {
        "PX4 multicopter"
    }
    fn mode(&self) -> Mode {
        Mode::Fly
    }
    fn translate(&self, sp: &Setpoint, _permit: &Permit) -> ActuatorCmd {
        ActuatorCmd::Px4Velocity {
            vx: sp.vx,
            vz: sp.vz.clamp(-self.max_vz, self.max_vz),
            yaw_rate: sp.yaw_rate,
        }
    }
    fn failsafe(&self, action: FailSafe) -> ActuatorCmd {
        match action {
            FailSafe::HoverThenLand | FailSafe::ControlledStop => ActuatorCmd::Px4Hold,
            // Never a motor kill in the air, even on e-stop.
            FailSafe::LandNow | FailSafe::EmergencyBrake => ActuatorCmd::Px4Land,
        }
    }
}

impl Supervisor {
    /// The single gate between planner output and hardware.
    pub fn output(&self, sp: &Setpoint, ground: &dyn Platform, air: &dyn Platform) -> ActuatorCmd {
        let pick = |m: Mode| if m == ground.mode() { ground } else { air };
        match self.state() {
            State::Disarmed => ActuatorCmd::Idle,
            State::Armed { control: crate::Control::Manual, .. } => ActuatorCmd::PilotControl,
            State::FailSafe { mode, action, .. } => pick(*mode).failsafe(*action),
            State::Armed { mode, .. } => {
                let permit = self.permit().expect("auto state always has a permit");
                pick(*mode).translate(sp, &permit)
            }
            State::Transition { .. } => {
                let permit = self.permit().expect("transition always has a permit");
                air.translate(sp, &permit)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Config, Event, Source};

    #[test]
    fn same_failsafe_different_hardware_commands() {
        let g = DiffDrive { wheel_base_m: 0.16, max_speed: 0.22 };
        let a = Multicopter { max_vz: 1.0 };
        assert_eq!(g.failsafe(FailSafe::EmergencyBrake), ActuatorCmd::Brake { hard: true });
        assert_eq!(a.failsafe(FailSafe::EmergencyBrake), ActuatorCmd::Px4Land);
    }

    #[test]
    fn output_is_idle_or_pilot_when_not_auto() {
        let g = DiffDrive { wheel_base_m: 0.16, max_speed: 0.22 };
        let a = Multicopter { max_vz: 1.0 };
        let sp = Setpoint { vx: 0.2, vz: 0.0, yaw_rate: 0.0 };
        let mut s = Supervisor::new(Config::default());
        for src in [Source::Planner, Source::Localization, Source::Lidar, Source::DriveController] {
            s.watchdog.register(src, 200, 0);
        }
        assert_eq!(s.output(&sp, &g, &a), ActuatorCmd::Idle);
        s.handle(Event::Arm, 0).unwrap();
        assert_eq!(s.output(&sp, &g, &a), ActuatorCmd::PilotControl);
        s.handle(Event::EngageAuto, 0).unwrap();
        assert!(matches!(s.output(&sp, &g, &a), ActuatorCmd::Wheels { .. }));
    }
}
