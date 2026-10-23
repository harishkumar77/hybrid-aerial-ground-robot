//! Heartbeat watchdog.
//!
//! Each monitored component must report in before its deadline expires.
//! In a ROS2 deployment the heartbeat is "a message arrived on this topic";
//! the same idea as a QoS deadline / liveliness policy, but evaluated by the
//! supervisor so the *response* to a missed deadline is mode-aware.

use std::collections::BTreeMap;
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Source {
    /// Global/local planner publishing setpoints.
    Planner,
    /// Localization (SLAM / EKF) publishing a pose.
    Localization,
    /// LiDAR scans (obstacle awareness).
    Lidar,
    /// Ground drive controller (wheel odometry / motor driver).
    DriveController,
    /// PX4 flight controller (vehicle_status over Micro XRCE-DDS).
    FlightController,
}

impl fmt::Display for Source {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Source::Planner => "planner",
            Source::Localization => "localization",
            Source::Lidar => "lidar",
            Source::DriveController => "drive_controller",
            Source::FlightController => "flight_controller",
        };
        f.write_str(s)
    }
}

#[derive(Debug, Default)]
pub struct Watchdog {
    deadline_ms: BTreeMap<Source, u64>,
    last_seen_ms: BTreeMap<Source, u64>,
}

impl Watchdog {
    pub fn new() -> Self {
        Self::default()
    }

    /// Start monitoring `source`. The clock starts now, so a component that
    /// never reports will go stale after one deadline.
    pub fn register(&mut self, source: Source, deadline_ms: u64, now_ms: u64) {
        self.deadline_ms.insert(source, deadline_ms);
        self.last_seen_ms.insert(source, now_ms);
    }

    pub fn heartbeat(&mut self, source: Source, now_ms: u64) {
        if self.deadline_ms.contains_key(&source) {
            self.last_seen_ms.insert(source, now_ms);
        }
    }

    /// Whether `source` has missed its deadline.
    pub fn is_stale(&self, source: Source, now_ms: u64) -> bool {
        match (self.deadline_ms.get(&source), self.last_seen_ms.get(&source)) {
            (Some(&deadline), Some(&last)) => now_ms.saturating_sub(last) > deadline,
            // Not registered: treat as stale. Fail closed, never open.
            _ => true,
        }
    }

    /// The stale sources among `required`.
    pub fn stale_among(&self, required: &[Source], now_ms: u64) -> Vec<Source> {
        required
            .iter()
            .copied()
            .filter(|s| self.is_stale(*s, now_ms))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn goes_stale_after_deadline_and_recovers_on_heartbeat() {
        let mut wd = Watchdog::new();
        wd.register(Source::Planner, 100, 0);
        assert!(!wd.is_stale(Source::Planner, 100));
        assert!(wd.is_stale(Source::Planner, 101));
        wd.heartbeat(Source::Planner, 150);
        assert!(!wd.is_stale(Source::Planner, 200));
    }

    #[test]
    fn unregistered_source_fails_closed() {
        let wd = Watchdog::new();
        assert!(wd.is_stale(Source::Lidar, 0));
    }
}
