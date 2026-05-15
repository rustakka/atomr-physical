//! ROS2 endpoints — the binding between an atomr-physical device and a
//! single point on the ROS2 graph.

use serde::{Deserialize, Serialize};

use crate::qos::QosProfile;

/// Which way data flows across a ROS2 endpoint, from atomr-physical's
/// point of view.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Ros2Direction {
    /// atomr-physical publishes to the topic (sensor readings out).
    Publish,
    /// atomr-physical subscribes to the topic (commands in).
    Subscribe,
}

/// A single ROS2 endpoint bound to an atomr-physical device.
///
/// The endpoint records the topic name, the ROS2 message type, the
/// direction data flows, and an optional [`QosProfile`]. When no QoS is
/// set, the bridge applies the per-direction default
/// ([`QosProfile::default_for`]) — see [`Ros2Endpoint::effective_qos`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ros2Endpoint {
    /// The fully-qualified ROS2 topic name, e.g. `/robot/joint_states`.
    pub topic: String,
    /// The ROS2 message type, e.g. `sensor_msgs/msg/JointState`.
    pub message_type: String,
    /// The direction data flows across this endpoint.
    pub direction: Ros2Direction,
    /// The Quality-of-Service profile, if set explicitly. When `None`,
    /// the bridge uses [`QosProfile::default_for`] the direction.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub qos: Option<QosProfile>,
}

impl Ros2Endpoint {
    /// A publishing endpoint (sensor side).
    pub fn publish(topic: impl Into<String>, message_type: impl Into<String>) -> Self {
        Self {
            topic: topic.into(),
            message_type: message_type.into(),
            direction: Ros2Direction::Publish,
            qos: None,
        }
    }

    /// A subscribing endpoint (actuator side).
    pub fn subscribe(topic: impl Into<String>, message_type: impl Into<String>) -> Self {
        Self {
            topic: topic.into(),
            message_type: message_type.into(),
            direction: Ros2Direction::Subscribe,
            qos: None,
        }
    }

    /// Builder-style: attach an explicit [`QosProfile`] to this endpoint.
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.qos = Some(qos);
        self
    }

    /// The QoS profile this endpoint resolves to: its explicit
    /// [`QosProfile`] if set, otherwise the per-direction default.
    pub fn effective_qos(&self) -> QosProfile {
        self.qos
            .unwrap_or_else(|| QosProfile::default_for(self.direction))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::qos::Reliability;

    #[test]
    fn factories_set_direction_and_no_qos() {
        let pub_ep = Ros2Endpoint::publish("/t", "std_msgs/msg/Float64");
        assert_eq!(pub_ep.direction, Ros2Direction::Publish);
        assert!(pub_ep.qos.is_none());

        let sub_ep = Ros2Endpoint::subscribe("/c", "std_msgs/msg/Float64");
        assert_eq!(sub_ep.direction, Ros2Direction::Subscribe);
    }

    #[test]
    fn effective_qos_falls_back_to_direction_default() {
        let ep = Ros2Endpoint::publish("/t", "sensor_msgs/msg/Temperature");
        assert_eq!(ep.effective_qos(), QosProfile::sensor_data());
    }

    #[test]
    fn with_qos_overrides_the_default() {
        let ep = Ros2Endpoint::publish("/t", "sensor_msgs/msg/Temperature").with_qos(QosProfile::command());
        assert_eq!(ep.effective_qos().reliability, Reliability::Reliable);
    }

    #[test]
    fn endpoint_without_qos_round_trips_and_omits_the_field() {
        let ep = Ros2Endpoint::publish("/t", "std_msgs/msg/Float64");
        let json = serde_json::to_string(&ep).unwrap();
        assert!(!json.contains("qos"), "None qos must be skipped: {json}");
        let back: Ros2Endpoint = serde_json::from_str(&json).unwrap();
        assert_eq!(ep, back);
    }

    #[test]
    fn endpoint_with_qos_round_trips() {
        let ep = Ros2Endpoint::subscribe("/c", "std_msgs/msg/Float64").with_qos(QosProfile::sensor_data());
        let json = serde_json::to_string(&ep).unwrap();
        let back: Ros2Endpoint = serde_json::from_str(&json).unwrap();
        assert_eq!(ep, back);
    }
}
