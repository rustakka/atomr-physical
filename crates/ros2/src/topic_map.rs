//! The offline topic-graph plan: which device maps to which ROS2
//! endpoint.

use std::collections::HashMap;

use atomr_physical_core::{ActuatorId, SensorId};
use serde::{Deserialize, Serialize};

use crate::endpoint::Ros2Endpoint;

/// The topic-graph plan for one robot: which device maps to which ROS2
/// endpoint.
///
/// `TopicMap` is the topic slice of the full [`Ros2Plan`](crate::Ros2Plan)
/// — kept as a distinct type for the common topic-only case. It is plain
/// serde data: build it, serialise it, diff it, and assert on it in
/// tests with no ROS2 toolchain.
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

    /// Every sensor binding — `(SensorId, endpoint)` pairs.
    ///
    /// The bridge consumes this to spawn one publisher actor (and one
    /// `rclrs` publisher) per bound sensor. Iteration order is
    /// unspecified.
    pub fn sensor_bindings(&self) -> impl Iterator<Item = (&SensorId, &Ros2Endpoint)> {
        self.sensor_topics.iter()
    }

    /// Every actuator binding — `(ActuatorId, endpoint)` pairs.
    ///
    /// The bridge consumes this to spawn one subscriber actor (and one
    /// `rclrs` subscription) per bound actuator. Iteration order is
    /// unspecified.
    pub fn actuator_bindings(&self) -> impl Iterator<Item = (&ActuatorId, &Ros2Endpoint)> {
        self.actuator_topics.iter()
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::endpoint::Ros2Direction;

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

    #[test]
    fn bindings_iterators_yield_every_endpoint() {
        let mut map = TopicMap::new();
        map.bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::publish("/robot/temp", "sensor_msgs/msg/Temperature"),
        );
        map.bind_sensor(
            SensorId::from("s2"),
            Ros2Endpoint::publish("/robot/imu", "sensor_msgs/msg/Imu"),
        );
        map.bind_actuator(
            ActuatorId::from("a1"),
            Ros2Endpoint::subscribe("/robot/cmd", "std_msgs/msg/Float64"),
        );
        assert_eq!(map.sensor_bindings().count(), 2);
        assert_eq!(map.actuator_bindings().count(), 1);
        assert!(map
            .sensor_bindings()
            .all(|(_, ep)| ep.direction == Ros2Direction::Publish));
    }

    #[test]
    fn empty_map_reports_empty() {
        let map = TopicMap::new();
        assert!(map.is_empty());
        assert_eq!(map.len(), 0);
        assert_eq!(map.sensor_bindings().count(), 0);
    }
}
