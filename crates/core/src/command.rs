//! Actuation commands — the output side of the physical layer.

use serde::{Deserialize, Serialize};

use crate::ids::ActuatorId;
use crate::units::Quantity;

/// How an actuator should interpret a [`Command`]'s setpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[non_exhaustive]
pub enum ControlMode {
    /// Drive to an absolute position / level and hold.
    Position,
    /// Track a velocity setpoint.
    Velocity,
    /// Apply a force / effort setpoint.
    Effort,
    /// Set a raw duty cycle (`0`–`100%`).
    Duty,
}

/// An instruction dispatched to an actuator.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Command {
    /// The actuator this command targets.
    pub actuator: ActuatorId,
    /// The control mode the setpoint is expressed in.
    pub mode: ControlMode,
    /// The commanded setpoint.
    pub setpoint: Quantity,
    /// Issue time, milliseconds since the Unix epoch.
    pub issued_ms: i64,
}

impl Command {
    /// Build a command, stamping it with the current wall-clock time.
    pub fn now(actuator: ActuatorId, mode: ControlMode, setpoint: Quantity) -> Self {
        Self {
            actuator,
            mode,
            setpoint,
            issued_ms: chrono::Utc::now().timestamp_millis(),
        }
    }
}

/// The acknowledgement an actuator returns once a [`Command`] has been
/// accepted (or rejected) at the driver boundary.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CommandAck {
    /// The actuator that produced the ack.
    pub actuator: ActuatorId,
    /// `true` if the command was accepted for execution.
    pub accepted: bool,
    /// A human-readable note — rejection reason, clamp applied, etc.
    pub detail: Option<String>,
    /// Ack time, milliseconds since the Unix epoch.
    pub acked_ms: i64,
}

impl CommandAck {
    /// An accepting ack.
    pub fn accepted(actuator: ActuatorId) -> Self {
        Self {
            actuator,
            accepted: true,
            detail: None,
            acked_ms: chrono::Utc::now().timestamp_millis(),
        }
    }

    /// A rejecting ack carrying a reason.
    pub fn rejected(actuator: ActuatorId, reason: impl Into<String>) -> Self {
        Self {
            actuator,
            accepted: false,
            detail: Some(reason.into()),
            acked_ms: chrono::Utc::now().timestamp_millis(),
        }
    }
}
