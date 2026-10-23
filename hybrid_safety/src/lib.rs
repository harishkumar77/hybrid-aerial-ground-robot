//! Safety supervisor for a hybrid aerial–ground robot.
//!
//! The autonomy stack (planner, SLAM) decides *what* the robot should do.
//! This crate decides whether it is *allowed* to do it:
//!
//! * **Interlocks** – preconditions that must hold before arming, engaging
//!   autonomy, or switching between flying and driving.
//! * **Watchdog** – every safety-relevant component must publish a heartbeat
//!   within its deadline; which components are required depends on the mode.
//! * **Arm / disarm** and **auto / manual** – an explicit state machine with
//!   guarded transitions. A manual takeover always wins (except over an e-stop).
//! * **Fail-safe** – a mode-aware response: a ground robot brakes, an aerial
//!   robot holds or lands. Motors are never cut in the air.
//! * **Platform abstraction** – the same safety logic drives different
//!   hardware through the [`platform::Platform`] trait.
//! * **Permit** – an autonomous actuator command can only be built with a
//!   [`supervisor::Permit`], and only the supervisor can mint one.

pub mod platform;
pub mod supervisor;
pub mod survey;
pub mod watchdog;

pub use platform::{ActuatorCmd, DiffDrive, Multicopter, Platform, Setpoint};
pub use supervisor::{
    Config, Control, Event, FailSafe, Interlocks, Mode, Permit, Rejected, State, Supervisor,
};
pub use watchdog::{Source, Watchdog};
