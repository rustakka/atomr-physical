//! The ROS2 bridge's internal error type.
//!
//! `Ros2Error` carries structured detail for bridge-internal logic —
//! which endpoint, which message type, what failed. At the crate
//! boundary it collapses into [`PhysicalError::Ros2Bridge`] (via the
//! [`From`] impl) so callers across atomr-physical see a single ROS2
//! error variant rather than a second error taxonomy.

use atomr_physical_core::PhysicalError;
use thiserror::Error;

/// An error raised inside the ROS2 bridge.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Ros2Error {
    /// A plan failed validation before the bridge could be spun.
    #[error("invalid ros2 plan: {0}")]
    InvalidPlan(String),

    /// A message could not be encoded to its bound ROS2 type.
    #[error("encode failed for {message_type} on {endpoint}: {reason}")]
    Encode {
        /// The endpoint (topic / service / action name) being encoded for.
        endpoint: String,
        /// The ROS2 message type the encode targeted.
        message_type: String,
        /// What went wrong.
        reason: String,
    },

    /// A message could not be decoded from its bound ROS2 type.
    #[error("decode failed for {message_type} on {endpoint}: {reason}")]
    Decode {
        /// The endpoint (topic / service / action name) being decoded from.
        endpoint: String,
        /// The ROS2 message type the decode expected.
        message_type: String,
        /// What went wrong.
        reason: String,
    },

    /// No codec is registered for a message type the plan references.
    #[error("no codec registered for message type {0}")]
    UnknownMessageType(String),

    /// A codec was asked to perform an operation it does not support —
    /// e.g. a topic codec asked to encode a service payload.
    #[error("codec for {message_type} does not support {operation}")]
    UnsupportedOperation {
        /// The message type whose codec was asked.
        message_type: String,
        /// The operation that is not supported (`encode_reading`,
        /// `decode_command`, `encode_payload`, or `decode_payload`).
        operation: &'static str,
    },

    /// The `rclrs` feature is not enabled, so the live bridge cannot run.
    #[error("rclrs feature not enabled — {0}")]
    FeatureDisabled(String),

    /// The transport task or the ROS2 node session was lost.
    #[error("ros2 transport lost: {0}")]
    TransportLost(String),
}

impl From<Ros2Error> for PhysicalError {
    fn from(err: Ros2Error) -> Self {
        PhysicalError::Ros2Bridge(err.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn folds_into_physical_error_preserving_the_message() {
        let err = Ros2Error::UnknownMessageType("pkg/msg/Nope".into());
        let text = err.to_string();
        let physical: PhysicalError = err.into();
        match physical {
            PhysicalError::Ros2Bridge(msg) => assert_eq!(msg, text),
            other => panic!("expected Ros2Bridge, got {other:?}"),
        }
    }

    #[test]
    fn encode_error_names_the_endpoint_and_type() {
        let err = Ros2Error::Encode {
            endpoint: "/arm/joint_states".into(),
            message_type: "sensor_msgs/msg/JointState".into(),
            reason: "unit mismatch".into(),
        };
        let text = err.to_string();
        assert!(text.contains("/arm/joint_states"));
        assert!(text.contains("sensor_msgs/msg/JointState"));
        assert!(text.contains("unit mismatch"));
    }

    #[test]
    fn feature_disabled_error_round_trips_message() {
        let err = Ros2Error::FeatureDisabled("node planned but not spun".into());
        assert!(err.to_string().contains("rclrs feature not enabled"));
    }
}
