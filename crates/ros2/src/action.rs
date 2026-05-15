//! ROS2 action endpoints — goal / feedback / result bindings.
//!
//! A ROS2 action is a long-running request with streamed feedback and a
//! final result. It maps onto atomr's `ask` plus a feedback stream
//! channel. atomr-physical's core has no "goal" concept, so the action
//! payloads stay ros2-crate-local and generic — the bridge orchestrates
//! the goal lifecycle (accept, feedback, result, cancel) and delegates
//! the action semantics to a user-supplied handler actor.
//!
//! This module defines the offline endpoint type and the [`GoalId`] that
//! identifies an in-flight goal; the payload-carrying lifecycle types
//! land with the codec layer.

use serde::{Deserialize, Serialize};

/// Whether atomr-physical hosts an action or calls it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ActionRole {
    /// atomr-physical **serves** the action — external clients send
    /// goals, and a handler actor drives them to a result.
    Server,
    /// atomr-physical **calls** the action — it lives on an external
    /// ROS2 node, and an atomr actor sends it goals.
    Client,
}

/// Identifies a single in-flight action goal across its feedback stream.
///
/// On a live graph this carries the ROS2 goal UUID; offline it is an
/// opaque string so plans and tests can name goals without DDS.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct GoalId(String);

impl GoalId {
    /// Borrow the goal id as a string slice.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<String> for GoalId {
    fn from(s: String) -> Self {
        Self(s)
    }
}

impl From<&str> for GoalId {
    fn from(s: &str) -> Self {
        Self(s.to_string())
    }
}

impl std::fmt::Display for GoalId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// A ROS2 action endpoint bound to an atomr-physical handler.
///
/// Records the action name, the action type (`package/action/Type`),
/// and the [`ActionRole`]. Plain serde data — buildable and assertable
/// with no ROS2 toolchain.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ros2ActionEndpoint {
    /// The fully-qualified ROS2 action name, e.g. `/arm/follow_traj`.
    pub action: String,
    /// The ROS2 action type, e.g.
    /// `control_msgs/action/FollowJointTrajectory`.
    pub action_type: String,
    /// Whether atomr-physical serves or calls this action.
    pub role: ActionRole,
}

impl Ros2ActionEndpoint {
    /// An action atomr-physical **serves** — external clients send goals.
    pub fn server(action: impl Into<String>, action_type: impl Into<String>) -> Self {
        Self {
            action: action.into(),
            action_type: action_type.into(),
            role: ActionRole::Server,
        }
    }

    /// An action atomr-physical **calls** — it is an external server.
    pub fn client(action: impl Into<String>, action_type: impl Into<String>) -> Self {
        Self {
            action: action.into(),
            action_type: action_type.into(),
            role: ActionRole::Client,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn factories_set_role() {
        assert_eq!(
            Ros2ActionEndpoint::server("/arm/traj", "control_msgs/action/FollowJointTrajectory").role,
            ActionRole::Server
        );
        assert_eq!(
            Ros2ActionEndpoint::client("/arm/dock", "nav2_msgs/action/NavigateToPose").role,
            ActionRole::Client
        );
    }

    #[test]
    fn goal_id_round_trips_as_a_string() {
        let id = GoalId::from("goal-7f3a");
        assert_eq!(id.as_str(), "goal-7f3a");
        assert_eq!(id.to_string(), "goal-7f3a");
        let json = serde_json::to_string(&id).unwrap();
        let back: GoalId = serde_json::from_str(&json).unwrap();
        assert_eq!(id, back);
    }

    #[test]
    fn action_endpoint_round_trips_through_json() {
        let ep = Ros2ActionEndpoint::server("/arm/traj", "control_msgs/action/FollowJointTrajectory");
        let json = serde_json::to_string(&ep).unwrap();
        let back: Ros2ActionEndpoint = serde_json::from_str(&json).unwrap();
        assert_eq!(ep, back);
    }
}
