//! ROS2 bridge for atomr-physical.
//!
//! This crate maps atomr-physical's actor world onto ROS2's
//! topic / service / action graph: a `SensorActor`'s reading stream
//! becomes a published topic, an `ActuatorActor`'s command mailbox
//! becomes a subscription, and a `RobotActor` becomes a ROS2 node.
//!
//! The bridge surface here is transport-agnostic and builds with **no
//! ROS2 installation**. The `rclrs` feature (Phase 2) links the
//! [`rclrs`](https://github.com/ros2-rust/ros2_rust) client library and
//! implements [`Ros2Bridge::spin`] against a live ROS2 graph; until
//! then the bridge records its topic mappings so a node graph can be
//! planned, inspected, and unit-tested offline. See
//! [`docs/ros2-bridge.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/ros2-bridge.md).

use std::collections::HashMap;

use atomr_physical_core::{ActuatorId, PhysicalError, Result, RobotId, SensorId};
use serde::{Deserialize, Serialize};

/// Re-export of the atomr actor runtime this crate builds on.
pub use atomr_core as actor;

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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ros2Endpoint {
    /// The fully-qualified ROS2 topic name, e.g. `/robot/joint_states`.
    pub topic: String,
    /// The ROS2 message type, e.g. `sensor_msgs/msg/JointState`.
    pub message_type: String,
    /// The direction data flows across this endpoint.
    pub direction: Ros2Direction,
}

impl Ros2Endpoint {
    /// A publishing endpoint (sensor side).
    pub fn publish(topic: impl Into<String>, message_type: impl Into<String>) -> Self {
        Self {
            topic: topic.into(),
            message_type: message_type.into(),
            direction: Ros2Direction::Publish,
        }
    }

    /// A subscribing endpoint (actuator side).
    pub fn subscribe(topic: impl Into<String>, message_type: impl Into<String>) -> Self {
        Self {
            topic: topic.into(),
            message_type: message_type.into(),
            direction: Ros2Direction::Subscribe,
        }
    }
}

/// The topic-graph plan for one robot: which device maps to which ROS2
/// endpoint.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TopicMap {
    sensor_topics: HashMap<SensorId, Ros2Endpoint>,
    actuator_topics: HashMap<ActuatorId, Ros2Endpoint>,
}

impl TopicMap {
    /// An empty topic map.
    pub fn new() -> Self {
        Self::default()
    }

    /// Bind a sensor's reading stream to a published topic.
    pub fn bind_sensor(&mut self, sensor: SensorId, endpoint: Ros2Endpoint) {
        self.sensor_topics.insert(sensor, endpoint);
    }

    /// Bind an actuator's command mailbox to a subscribed topic.
    pub fn bind_actuator(&mut self, actuator: ActuatorId, endpoint: Ros2Endpoint) {
        self.actuator_topics.insert(actuator, endpoint);
    }

    /// The endpoint a sensor publishes to, if bound.
    pub fn sensor_endpoint(&self, sensor: &SensorId) -> Option<&Ros2Endpoint> {
        self.sensor_topics.get(sensor)
    }

    /// The endpoint an actuator subscribes from, if bound.
    pub fn actuator_endpoint(&self, actuator: &ActuatorId) -> Option<&Ros2Endpoint> {
        self.actuator_topics.get(actuator)
    }

    /// Total number of bound endpoints.
    pub fn len(&self) -> usize {
        self.sensor_topics.len() + self.actuator_topics.len()
    }

    /// Returns `true` if no endpoints are bound.
    pub fn is_empty(&self) -> bool {
        self.sensor_topics.is_empty() && self.actuator_topics.is_empty()
    }
}

/// Bridges a `RobotActor`'s actor graph onto a ROS2 node.
///
/// **Phase 2** (the `rclrs` feature) implements [`Ros2Bridge::spin`]
/// against the `rclrs` client library: it creates an `rclrs` node,
/// wires real publishers / subscriptions from the [`TopicMap`], and
/// drives them on the atomr runtime. The current form owns the node
/// name and topic plan so the ROS2 graph can be declared and validated
/// offline.
pub struct Ros2Bridge {
    node_name: String,
    robot: RobotId,
    topics: TopicMap,
}

impl Ros2Bridge {
    /// Construct a bridge for a robot, naming the ROS2 node.
    pub fn new(node_name: impl Into<String>, robot: RobotId) -> Self {
        Self {
            node_name: node_name.into(),
            robot,
            topics: TopicMap::new(),
        }
    }

    /// The ROS2 node name this bridge will register.
    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    /// The robot this bridge serves.
    pub fn robot(&self) -> &RobotId {
        &self.robot
    }

    /// The bridge's topic plan.
    pub fn topics(&self) -> &TopicMap {
        &self.topics
    }

    /// The bridge's topic plan — mutate it to bind devices to endpoints.
    pub fn topics_mut(&mut self) -> &mut TopicMap {
        &mut self.topics
    }

    /// Drive the bridge against a live ROS2 graph.
    ///
    /// **Phase 2** (the `rclrs` feature) implements this against the
    /// `rclrs` client library. Until then it returns
    /// [`PhysicalError::Ros2Bridge`] so callers fail fast rather than
    /// silently no-op.
    pub async fn spin(&self) -> Result<()> {
        Err(PhysicalError::Ros2Bridge(format!(
            "rclrs feature not enabled — node {:?} ({} endpoints) planned but not spun; \
             see docs/ros2-bridge.md",
            self.node_name,
            self.topics.len(),
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn topic_map_binds_both_directions() {
        let mut map = TopicMap::new();
        map.bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::publish("/robot/temp", "sensor_msgs/msg/Temperature"),
        );
        map.bind_actuator(
            ActuatorId::from("a1"),
            Ros2Endpoint::subscribe("/robot/cmd", "std_msgs/msg/Float64"),
        );
        assert_eq!(map.len(), 2);
        assert_eq!(
            map.sensor_endpoint(&SensorId::from("s1")).unwrap().direction,
            Ros2Direction::Publish
        );
        assert_eq!(
            map.actuator_endpoint(&ActuatorId::from("a1")).unwrap().direction,
            Ros2Direction::Subscribe
        );
    }

    #[tokio::test]
    async fn spin_without_rclrs_feature_errors() {
        let bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("r1"));
        assert!(bridge.spin().await.is_err());
    }
}
