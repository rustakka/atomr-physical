//! Clock-source selection for ROS2 message timestamps.

use serde::{Deserialize, Serialize};

/// Which clock the bridge stamps outbound ROS2 messages from.
///
/// The bridge is **not** a time source. When a [`Reading`] already
/// carries a `timestamp_ms`, the codec copies it onto the message
/// header; otherwise the bridge reads the configured clock. This enum
/// records that choice as plain data so it can be planned and asserted
/// on offline.
///
/// [`Reading`]: atomr_physical_core::Reading
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum Ros2ClockSource {
    /// The host wall clock. Matches ROS2 with `use_sim_time = false` —
    /// the default.
    #[default]
    Wall,
    /// The ROS2 graph's time, as published on `/clock` by the node.
    RosTime,
    /// Simulation time, as published on `/clock` by a simulator. Matches
    /// ROS2 with `use_sim_time = true`.
    SimTime,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_clock_is_wall() {
        assert_eq!(Ros2ClockSource::default(), Ros2ClockSource::Wall);
    }

    #[test]
    fn clock_source_round_trips_through_json() {
        for src in [
            Ros2ClockSource::Wall,
            Ros2ClockSource::RosTime,
            Ros2ClockSource::SimTime,
        ] {
            let json = serde_json::to_string(&src).unwrap();
            let back: Ros2ClockSource = serde_json::from_str(&json).unwrap();
            assert_eq!(src, back);
        }
    }
}
