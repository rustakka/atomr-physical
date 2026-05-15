//! ROS2 Quality-of-Service profiles, as pure data.
//!
//! A [`QosProfile`] mirrors the QoS settings the bridge actually applies
//! to a publisher or subscription. It is plain serde data — buildable,
//! diffable, and unit-testable with no ROS2 toolchain — and is attached
//! to a [`Ros2Endpoint`](crate::Ros2Endpoint) so the topic plan records
//! the intended delivery semantics offline.

use serde::{Deserialize, Serialize};

use crate::endpoint::Ros2Direction;

/// The delivery guarantee for a ROS2 endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Reliability {
    /// Every message is delivered, retrying as needed.
    Reliable,
    /// Messages may be dropped under load — lowest latency.
    BestEffort,
}

/// Whether late-joining subscribers receive previously published
/// messages.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Durability {
    /// Only messages published after a subscriber joins are delivered.
    Volatile,
    /// A late-joining subscriber receives the last published messages.
    TransientLocal,
}

/// How many past messages an endpoint retains.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum History {
    /// Keep only the most recent `depth` messages.
    KeepLast,
    /// Keep every message (bounded by middleware resource limits).
    KeepAll,
}

/// A ROS2 Quality-of-Service profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct QosProfile {
    /// The delivery guarantee.
    pub reliability: Reliability,
    /// Whether late joiners see prior messages.
    pub durability: Durability,
    /// The history policy.
    pub history: History,
    /// The history depth — the queue size when `history` is
    /// [`History::KeepLast`].
    pub depth: u32,
}

impl QosProfile {
    /// The sensor-data profile: best-effort, volatile, keep-last(5).
    ///
    /// Mirrors the intent of ROS2's `rmw_qos_profile_sensor_data` —
    /// favours latency over guaranteed delivery for high-rate sensor
    /// streams. The per-direction default for [`Ros2Direction::Publish`].
    pub fn sensor_data() -> Self {
        Self {
            reliability: Reliability::BestEffort,
            durability: Durability::Volatile,
            history: History::KeepLast,
            depth: 5,
        }
    }

    /// The command profile: reliable, volatile, keep-last(10).
    ///
    /// Commands must not be silently dropped. The per-direction default
    /// for [`Ros2Direction::Subscribe`].
    pub fn command() -> Self {
        Self {
            reliability: Reliability::Reliable,
            durability: Durability::Volatile,
            history: History::KeepLast,
            depth: 10,
        }
    }

    /// The QoS profile a direction defaults to when an endpoint does not
    /// set one explicitly.
    pub fn default_for(direction: Ros2Direction) -> Self {
        match direction {
            Ros2Direction::Publish => Self::sensor_data(),
            Ros2Direction::Subscribe => Self::command(),
        }
    }
}

impl Default for QosProfile {
    /// The generic default profile: reliable, volatile, keep-last(10) —
    /// matching ROS2's default QoS for a topic.
    fn default() -> Self {
        Self::command()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_data_is_best_effort() {
        let qos = QosProfile::sensor_data();
        assert_eq!(qos.reliability, Reliability::BestEffort);
        assert_eq!(qos.history, History::KeepLast);
        assert_eq!(qos.depth, 5);
    }

    #[test]
    fn command_is_reliable() {
        assert_eq!(QosProfile::command().reliability, Reliability::Reliable);
    }

    #[test]
    fn default_for_direction_matches_intent() {
        assert_eq!(
            QosProfile::default_for(Ros2Direction::Publish),
            QosProfile::sensor_data()
        );
        assert_eq!(
            QosProfile::default_for(Ros2Direction::Subscribe),
            QosProfile::command()
        );
    }

    #[test]
    fn qos_round_trips_through_json() {
        let qos = QosProfile::sensor_data();
        let json = serde_json::to_string(&qos).unwrap();
        let back: QosProfile = serde_json::from_str(&json).unwrap();
        assert_eq!(qos, back);
    }
}
