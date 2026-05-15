//! Offline validation of a ROS2 plan.
//!
//! These lints catch malformed endpoints — empty or unqualified topic
//! names, malformed message-type strings, a sensor bound to a
//! subscription — *before* the bridge is spun, so a bad plan surfaces as
//! inspectable data rather than as an `rclrs` panic on a live graph. All
//! checks are pure and run with no ROS2 toolchain.

use crate::action::Ros2ActionEndpoint;
use crate::endpoint::{Ros2Direction, Ros2Endpoint};
use crate::param::Ros2ParamDecl;
use crate::plan::Ros2Plan;
use crate::service::Ros2ServiceEndpoint;
use crate::topic_map::TopicMap;

/// A single problem found in a plan.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ValidationError {
    /// The endpoint the problem was found on — the topic / service /
    /// action / parameter name, or a `<placeholder>` when that name is
    /// itself empty.
    pub endpoint: String,
    /// What is wrong.
    pub issue: ValidationIssue,
}

/// The kind of problem a [`ValidationError`] reports.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum ValidationIssue {
    /// The topic name is empty.
    EmptyTopic,
    /// The topic name is not fully qualified (does not start with `/`).
    UnqualifiedTopic,
    /// The topic name contains a character ROS2 does not allow in a
    /// fully-qualified name.
    IllegalTopicChar(char),
    /// The topic name contains an empty segment (`//`) or a trailing `/`.
    EmptyTopicSegment,
    /// The message type string is empty.
    EmptyMessageType,
    /// The message type is not in `package/<kind>/Type` form, where
    /// `<kind>` is `msg`, `srv`, or `action`.
    MalformedMessageType,
    /// A device was bound to an endpoint pointing the wrong way — a
    /// sensor to a non-[`Publish`](Ros2Direction::Publish) endpoint, or
    /// an actuator to a non-[`Subscribe`](Ros2Direction::Subscribe) one.
    DirectionMismatch {
        /// The direction the binding requires.
        expected: Ros2Direction,
        /// The direction the endpoint actually carries.
        found: Ros2Direction,
    },
    /// A service type is not in `package/srv/Type` form.
    MalformedServiceType,
    /// An action type is not in `package/action/Type` form.
    MalformedActionType,
    /// A parameter name is empty.
    EmptyParamName,
    /// A parameter name has an empty `.`-separated segment or an illegal
    /// character.
    MalformedParamName,
}

/// Lint a single topic endpoint's topic name and message type.
///
/// Returns every issue found — an empty `Vec` means the endpoint is
/// well-formed. This does **not** check direction; that needs the
/// binding context and is covered by [`validate_topic_map`].
pub fn validate_endpoint(endpoint: &Ros2Endpoint) -> Vec<ValidationIssue> {
    let mut issues = Vec::new();
    issues.extend(topic_name_issues(&endpoint.topic));
    issues.extend(message_type_issues(&endpoint.message_type));
    issues
}

/// Lint a service endpoint's service name and service type.
pub fn validate_service_endpoint(endpoint: &Ros2ServiceEndpoint) -> Vec<ValidationIssue> {
    let mut issues = topic_name_issues(&endpoint.service);
    if !is_typed_as(&endpoint.service_type, "srv") {
        issues.push(ValidationIssue::MalformedServiceType);
    }
    issues
}

/// Lint an action endpoint's action name and action type.
pub fn validate_action_endpoint(endpoint: &Ros2ActionEndpoint) -> Vec<ValidationIssue> {
    let mut issues = topic_name_issues(&endpoint.action);
    if !is_typed_as(&endpoint.action_type, "action") {
        issues.push(ValidationIssue::MalformedActionType);
    }
    issues
}

/// Lint a parameter declaration's name.
pub fn validate_param_decl(decl: &Ros2ParamDecl) -> Vec<ValidationIssue> {
    param_name_issues(&decl.name)
}

/// Lint every endpoint in a [`TopicMap`], including direction.
///
/// Returns every [`ValidationError`] found — an empty `Vec` means the
/// topic plan is well-formed.
pub fn validate_topic_map(map: &TopicMap) -> Vec<ValidationError> {
    let mut errors = Vec::new();

    for (sensor, endpoint) in map.sensor_bindings() {
        let where_ = endpoint_label(&endpoint.topic, sensor.as_str());
        for issue in validate_endpoint(endpoint) {
            errors.push(ValidationError {
                endpoint: where_.clone(),
                issue,
            });
        }
        if endpoint.direction != Ros2Direction::Publish {
            errors.push(ValidationError {
                endpoint: where_,
                issue: ValidationIssue::DirectionMismatch {
                    expected: Ros2Direction::Publish,
                    found: endpoint.direction,
                },
            });
        }
    }

    for (actuator, endpoint) in map.actuator_bindings() {
        let where_ = endpoint_label(&endpoint.topic, actuator.as_str());
        for issue in validate_endpoint(endpoint) {
            errors.push(ValidationError {
                endpoint: where_.clone(),
                issue,
            });
        }
        if endpoint.direction != Ros2Direction::Subscribe {
            errors.push(ValidationError {
                endpoint: where_,
                issue: ValidationIssue::DirectionMismatch {
                    expected: Ros2Direction::Subscribe,
                    found: endpoint.direction,
                },
            });
        }
    }

    errors
}

/// Lint a whole [`Ros2Plan`] — topics, services, actions, and
/// parameters.
///
/// Returns every [`ValidationError`] found — an empty `Vec` means the
/// plan is well-formed.
pub fn validate_plan(plan: &Ros2Plan) -> Vec<ValidationError> {
    let mut errors = validate_topic_map(plan.topics());

    for endpoint in plan.services() {
        let where_ = endpoint_label(&endpoint.service, "service");
        for issue in validate_service_endpoint(endpoint) {
            errors.push(ValidationError {
                endpoint: where_.clone(),
                issue,
            });
        }
    }

    for endpoint in plan.actions() {
        let where_ = endpoint_label(&endpoint.action, "action");
        for issue in validate_action_endpoint(endpoint) {
            errors.push(ValidationError {
                endpoint: where_.clone(),
                issue,
            });
        }
    }

    for decl in plan.params() {
        let where_ = endpoint_label(&decl.name, "param");
        for issue in validate_param_decl(decl) {
            errors.push(ValidationError {
                endpoint: where_.clone(),
                issue,
            });
        }
    }

    errors
}

/// The label a [`ValidationError`] reports — the name itself, falling
/// back to `<fallback>` when the name is empty.
fn endpoint_label(name: &str, fallback: &str) -> String {
    if name.is_empty() {
        format!("<{fallback}>")
    } else {
        name.to_string()
    }
}

/// Topic-name lints, in the order a reader would diagnose them.
fn topic_name_issues(topic: &str) -> Vec<ValidationIssue> {
    let mut issues = Vec::new();
    if topic.is_empty() {
        issues.push(ValidationIssue::EmptyTopic);
        return issues;
    }
    if !topic.starts_with('/') {
        issues.push(ValidationIssue::UnqualifiedTopic);
    }
    if let Some(bad) = topic
        .chars()
        .find(|c| !(c.is_ascii_alphanumeric() || *c == '_' || *c == '/'))
    {
        issues.push(ValidationIssue::IllegalTopicChar(bad));
    }
    if topic.contains("//") || (topic.len() > 1 && topic.ends_with('/')) {
        issues.push(ValidationIssue::EmptyTopicSegment);
    }
    issues
}

/// Message-type lints — the type must be `package/<kind>/Type` where
/// `<kind>` is `msg`, `srv`, or `action`.
fn message_type_issues(message_type: &str) -> Vec<ValidationIssue> {
    if message_type.is_empty() {
        return vec![ValidationIssue::EmptyMessageType];
    }
    let parts: Vec<&str> = message_type.split('/').collect();
    let well_formed = parts.len() == 3
        && parts.iter().all(|p| !p.is_empty())
        && matches!(parts[1], "msg" | "srv" | "action");
    if well_formed {
        Vec::new()
    } else {
        vec![ValidationIssue::MalformedMessageType]
    }
}

/// Whether `type_str` is `package/<kind>/Type` for one specific `kind`.
fn is_typed_as(type_str: &str, kind: &str) -> bool {
    let parts: Vec<&str> = type_str.split('/').collect();
    parts.len() == 3 && parts.iter().all(|p| !p.is_empty()) && parts[1] == kind
}

/// Parameter-name lints — non-empty, `.`-separated segments, each
/// segment a non-empty run of `[A-Za-z0-9_]`.
fn param_name_issues(name: &str) -> Vec<ValidationIssue> {
    if name.is_empty() {
        return vec![ValidationIssue::EmptyParamName];
    }
    let malformed = name
        .split('.')
        .any(|seg| seg.is_empty() || !seg.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'));
    if malformed {
        vec![ValidationIssue::MalformedParamName]
    } else {
        Vec::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::param::ParamValue;
    use atomr_physical_core::{ActuatorId, SensorId};

    #[test]
    fn well_formed_endpoint_has_no_issues() {
        let ep = Ros2Endpoint::publish("/arm/joint_states", "sensor_msgs/msg/JointState");
        assert!(validate_endpoint(&ep).is_empty());
    }

    #[test]
    fn empty_topic_is_flagged() {
        let ep = Ros2Endpoint::publish("", "std_msgs/msg/Float64");
        assert_eq!(validate_endpoint(&ep), vec![ValidationIssue::EmptyTopic]);
    }

    #[test]
    fn unqualified_topic_is_flagged() {
        let ep = Ros2Endpoint::publish("arm/joint_states", "std_msgs/msg/Float64");
        assert!(validate_endpoint(&ep).contains(&ValidationIssue::UnqualifiedTopic));
    }

    #[test]
    fn illegal_topic_char_is_flagged() {
        let ep = Ros2Endpoint::publish("/arm/joint states", "std_msgs/msg/Float64");
        assert!(validate_endpoint(&ep).contains(&ValidationIssue::IllegalTopicChar(' ')));
    }

    #[test]
    fn empty_topic_segment_is_flagged() {
        let double = Ros2Endpoint::publish("/arm//joints", "std_msgs/msg/Float64");
        assert!(validate_endpoint(&double).contains(&ValidationIssue::EmptyTopicSegment));
        let trailing = Ros2Endpoint::publish("/arm/joints/", "std_msgs/msg/Float64");
        assert!(validate_endpoint(&trailing).contains(&ValidationIssue::EmptyTopicSegment));
    }

    #[test]
    fn empty_message_type_is_flagged() {
        let ep = Ros2Endpoint::publish("/t", "");
        assert_eq!(validate_endpoint(&ep), vec![ValidationIssue::EmptyMessageType]);
    }

    #[test]
    fn malformed_message_type_is_flagged() {
        for bad in ["Float64", "std_msgs/Float64", "std_msgs/bogus/Float64", "/msg/"] {
            let ep = Ros2Endpoint::publish("/t", bad);
            assert!(
                validate_endpoint(&ep).contains(&ValidationIssue::MalformedMessageType),
                "expected {bad:?} to be flagged"
            );
        }
    }

    #[test]
    fn srv_and_action_kinds_are_well_formed_messages() {
        for ok in [
            "std_srvs/srv/Trigger",
            "control_msgs/action/FollowJointTrajectory",
        ] {
            let ep = Ros2Endpoint::publish("/t", ok);
            assert!(
                message_type_issues(&ep.message_type).is_empty(),
                "expected {ok:?} to be well-formed"
            );
        }
    }

    #[test]
    fn direction_mismatch_is_flagged_per_binding() {
        let mut map = TopicMap::new();
        // A sensor bound to a *subscribe* endpoint — wrong way round.
        map.bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::subscribe("/arm/temp", "sensor_msgs/msg/Temperature"),
        );
        // An actuator bound to a *publish* endpoint — wrong way round.
        map.bind_actuator(
            ActuatorId::from("a1"),
            Ros2Endpoint::publish("/arm/cmd", "std_msgs/msg/Float64"),
        );
        let errors = validate_topic_map(&map);
        assert!(errors.iter().any(|e| matches!(
            e.issue,
            ValidationIssue::DirectionMismatch {
                expected: Ros2Direction::Publish,
                found: Ros2Direction::Subscribe,
            }
        )));
        assert!(errors.iter().any(|e| matches!(
            e.issue,
            ValidationIssue::DirectionMismatch {
                expected: Ros2Direction::Subscribe,
                found: Ros2Direction::Publish,
            }
        )));
    }

    #[test]
    fn well_formed_topic_map_validates_clean() {
        let mut map = TopicMap::new();
        map.bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::publish("/arm/temp", "sensor_msgs/msg/Temperature"),
        );
        map.bind_actuator(
            ActuatorId::from("a1"),
            Ros2Endpoint::subscribe("/arm/cmd", "std_msgs/msg/Float64"),
        );
        assert!(validate_topic_map(&map).is_empty());
    }

    #[test]
    fn service_type_must_be_srv_kind() {
        let bad = Ros2ServiceEndpoint::server("/arm/home", "std_srvs/msg/Trigger");
        assert!(validate_service_endpoint(&bad).contains(&ValidationIssue::MalformedServiceType));
        let good = Ros2ServiceEndpoint::server("/arm/home", "std_srvs/srv/Trigger");
        assert!(validate_service_endpoint(&good).is_empty());
    }

    #[test]
    fn action_type_must_be_action_kind() {
        let bad = Ros2ActionEndpoint::server("/arm/traj", "control_msgs/srv/FollowJointTrajectory");
        assert!(validate_action_endpoint(&bad).contains(&ValidationIssue::MalformedActionType));
        let good = Ros2ActionEndpoint::server("/arm/traj", "control_msgs/action/FollowJointTrajectory");
        assert!(validate_action_endpoint(&good).is_empty());
    }

    #[test]
    fn param_names_are_linted() {
        assert_eq!(
            validate_param_decl(&Ros2ParamDecl::new("", ParamValue::Int(0))),
            vec![ValidationIssue::EmptyParamName]
        );
        for bad in ["shoulder..period", ".leading", "trailing.", "has space"] {
            assert!(
                validate_param_decl(&Ros2ParamDecl::new(bad, ParamValue::Int(0)))
                    .contains(&ValidationIssue::MalformedParamName),
                "expected {bad:?} to be flagged"
            );
        }
        assert!(
            validate_param_decl(&Ros2ParamDecl::new("shoulder.period_ms", ParamValue::Int(100))).is_empty()
        );
    }

    #[test]
    fn validate_plan_collects_problems_from_every_kind() {
        let mut plan = Ros2Plan::new();
        plan.topics_mut().bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::subscribe("arm/temp", "sensor_msgs/msg/Temperature"),
        );
        plan.add_service(Ros2ServiceEndpoint::server("/arm/home", "std_srvs/Trigger"));
        plan.add_action(Ros2ActionEndpoint::server("/arm/traj", "bad_type"));
        plan.declare_param(Ros2ParamDecl::new("bad..name", ParamValue::Int(0)));
        let errors = validate_plan(&plan);
        assert!(errors
            .iter()
            .any(|e| e.issue == ValidationIssue::UnqualifiedTopic));
        assert!(errors
            .iter()
            .any(|e| e.issue == ValidationIssue::MalformedServiceType));
        assert!(errors
            .iter()
            .any(|e| e.issue == ValidationIssue::MalformedActionType));
        assert!(errors
            .iter()
            .any(|e| e.issue == ValidationIssue::MalformedParamName));
    }

    #[test]
    fn validate_plan_is_clean_for_a_well_formed_plan() {
        let mut plan = Ros2Plan::new();
        plan.topics_mut().bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::publish("/arm/temp", "sensor_msgs/msg/Temperature"),
        );
        plan.add_service(Ros2ServiceEndpoint::server("/arm/home", "std_srvs/srv/Trigger"));
        plan.add_action(Ros2ActionEndpoint::server(
            "/arm/traj",
            "control_msgs/action/FollowJointTrajectory",
        ));
        plan.declare_param(Ros2ParamDecl::new("shoulder.period_ms", ParamValue::Int(100)));
        assert!(validate_plan(&plan).is_empty());
    }
}
