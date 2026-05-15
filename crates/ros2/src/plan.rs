//! [`Ros2Plan`] — the full offline plan for one robot's ROS2 node:
//! topics, services, actions, and parameters.

use serde::{Deserialize, Serialize};

use crate::action::Ros2ActionEndpoint;
use crate::param::Ros2ParamDecl;
use crate::service::Ros2ServiceEndpoint;
use crate::topic_map::TopicMap;

/// The complete ROS2 graph plan for one robot.
///
/// `Ros2Plan` aggregates the four binding kinds the bridge orchestrates:
/// the [`TopicMap`] of pub/sub endpoints, the service endpoints, the
/// action endpoints, and the parameter declarations. It is plain serde
/// data — the entire plan can be built, serialised, diffed, validated,
/// and asserted on with no ROS2 toolchain.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Ros2Plan {
    topics: TopicMap,
    services: Vec<Ros2ServiceEndpoint>,
    actions: Vec<Ros2ActionEndpoint>,
    params: Vec<Ros2ParamDecl>,
}

impl Ros2Plan {
    /// An empty plan.
    pub fn new() -> Self {
        Self::default()
    }

    /// The topic plan — pub/sub endpoints.
    pub fn topics(&self) -> &TopicMap {
        &self.topics
    }

    /// The topic plan — mutate it to bind devices to topics.
    pub fn topics_mut(&mut self) -> &mut TopicMap {
        &mut self.topics
    }

    /// Add a service endpoint to the plan.
    pub fn add_service(&mut self, endpoint: Ros2ServiceEndpoint) {
        self.services.push(endpoint);
    }

    /// The service endpoints in the plan.
    pub fn services(&self) -> &[Ros2ServiceEndpoint] {
        &self.services
    }

    /// Add an action endpoint to the plan.
    pub fn add_action(&mut self, endpoint: Ros2ActionEndpoint) {
        self.actions.push(endpoint);
    }

    /// The action endpoints in the plan.
    pub fn actions(&self) -> &[Ros2ActionEndpoint] {
        &self.actions
    }

    /// Declare a parameter the bridge mirrors.
    pub fn declare_param(&mut self, decl: Ros2ParamDecl) {
        self.params.push(decl);
    }

    /// The parameter declarations in the plan.
    pub fn params(&self) -> &[Ros2ParamDecl] {
        &self.params
    }

    /// Total number of bound endpoints across every kind — topics,
    /// services, actions, and parameters.
    pub fn len(&self) -> usize {
        self.topics.len() + self.services.len() + self.actions.len() + self.params.len()
    }

    /// Returns `true` if the plan binds nothing at all.
    pub fn is_empty(&self) -> bool {
        self.topics.is_empty()
            && self.services.is_empty()
            && self.actions.is_empty()
            && self.params.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action::ActionRole;
    use crate::endpoint::Ros2Endpoint;
    use crate::param::ParamValue;
    use crate::service::ServiceRole;
    use atomr_physical_core::{ActuatorId, SensorId};

    fn populated_plan() -> Ros2Plan {
        let mut plan = Ros2Plan::new();
        plan.topics_mut().bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::publish("/arm/temp", "sensor_msgs/msg/Temperature"),
        );
        plan.topics_mut().bind_actuator(
            ActuatorId::from("a1"),
            Ros2Endpoint::subscribe("/arm/cmd", "std_msgs/msg/Float64"),
        );
        plan.add_service(Ros2ServiceEndpoint::server("/arm/home", "std_srvs/srv/Trigger"));
        plan.add_action(Ros2ActionEndpoint::server(
            "/arm/traj",
            "control_msgs/action/FollowJointTrajectory",
        ));
        plan.declare_param(Ros2ParamDecl::new("shoulder.period_ms", ParamValue::Int(100)));
        plan
    }

    #[test]
    fn empty_plan_reports_empty() {
        let plan = Ros2Plan::new();
        assert!(plan.is_empty());
        assert_eq!(plan.len(), 0);
    }

    #[test]
    fn len_counts_every_kind() {
        let plan = populated_plan();
        // 2 topics + 1 service + 1 action + 1 param.
        assert_eq!(plan.len(), 5);
        assert!(!plan.is_empty());
        assert_eq!(plan.services()[0].role, ServiceRole::Server);
        assert_eq!(plan.actions()[0].role, ActionRole::Server);
        assert_eq!(plan.params()[0].name, "shoulder.period_ms");
    }

    #[test]
    fn plan_round_trips_through_json() {
        let plan = populated_plan();
        let json = serde_json::to_string(&plan).unwrap();
        let back: Ros2Plan = serde_json::from_str(&json).unwrap();
        assert_eq!(plan, back);
    }
}
