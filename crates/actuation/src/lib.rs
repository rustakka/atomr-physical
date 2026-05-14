//! Actuator-side actors for atomr-physical.
//!
//! A hardware driver implements [`atomr_physical_core::Actuator`] in
//! plain async Rust. This crate adapts that implementation into a
//! supervised atomr actor — [`ActuatorActor`] — that serialises
//! [`Command`]s and enforces a [`SafetyEnvelope`] before anything
//! reaches hardware.
//!
//! The atomr actor runtime is re-exported as [`actor`] so downstream
//! crates have a single import path for it.

use std::sync::Arc;

use atomr_physical_core::{Actuator, ActuatorId, Command, CommandAck, PhysicalError, Result};
use serde::{Deserialize, Serialize};

/// Re-export of the atomr actor runtime this crate builds on.
pub use atomr_core as actor;

/// A min / max clamp on an actuator setpoint.
///
/// Commands whose setpoint falls outside the envelope are either
/// clamped to the boundary or rejected outright, depending on
/// [`SafetyEnvelope::clamp`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SafetyEnvelope {
    /// Lowest setpoint value the actuator may be driven to.
    pub min: f64,
    /// Highest setpoint value the actuator may be driven to.
    pub max: f64,
    /// If `true`, out-of-range setpoints are clamped into `[min, max]`.
    /// If `false`, they are rejected with [`PhysicalError::OutOfRange`].
    pub clamp: bool,
}

impl SafetyEnvelope {
    /// An envelope that clamps setpoints into `[min, max]`.
    pub fn clamping(min: f64, max: f64) -> Self {
        Self {
            min,
            max,
            clamp: true,
        }
    }

    /// An envelope that rejects out-of-range setpoints.
    pub fn rejecting(min: f64, max: f64) -> Self {
        Self {
            min,
            max,
            clamp: false,
        }
    }

    /// Apply the envelope to a raw setpoint value.
    ///
    /// Returns the (possibly clamped) value, or
    /// [`PhysicalError::OutOfRange`] if the value is outside the
    /// envelope and clamping is disabled.
    pub fn enforce(&self, actuator: &ActuatorId, value: f64) -> Result<f64> {
        if value >= self.min && value <= self.max {
            return Ok(value);
        }
        if self.clamp {
            Ok(value.clamp(self.min, self.max))
        } else {
            Err(PhysicalError::OutOfRange {
                device: actuator.to_string(),
                value,
                min: self.min,
                max: self.max,
            })
        }
    }
}

/// Adapts an [`Actuator`] driver into a supervised atomr actor.
///
/// **Phase 2** wires `ActuatorActor` into [`actor`]'s `Actor` trait so
/// commands arrive over a mailbox and the queue drains under
/// supervision. The current form holds the driver and its safety
/// envelope and exposes a direct async [`dispatch`] path that runs the
/// envelope check before the driver sees the command.
///
/// [`dispatch`]: ActuatorActor::dispatch
pub struct ActuatorActor {
    actuator: Arc<dyn Actuator>,
    envelope: Option<SafetyEnvelope>,
}

impl ActuatorActor {
    /// Wrap an actuator driver. No safety envelope is enforced until one
    /// is attached with [`with_envelope`](ActuatorActor::with_envelope).
    pub fn new(actuator: Arc<dyn Actuator>) -> Self {
        Self {
            actuator,
            envelope: None,
        }
    }

    /// Builder-style: attach a safety envelope.
    pub fn with_envelope(mut self, envelope: SafetyEnvelope) -> Self {
        self.envelope = Some(envelope);
        self
    }

    /// The id of the wrapped actuator.
    pub fn id(&self) -> ActuatorId {
        ActuatorId::from(self.actuator.descriptor().id.as_str())
    }

    /// The safety envelope in force, if any.
    pub fn envelope(&self) -> Option<SafetyEnvelope> {
        self.envelope
    }

    /// Enforce the safety envelope (if any) and dispatch the command to
    /// the underlying driver.
    pub async fn dispatch(&self, mut command: Command) -> Result<CommandAck> {
        if let Some(envelope) = &self.envelope {
            let safe = envelope.enforce(&command.actuator, command.setpoint.value)?;
            command.setpoint.value = safe;
        }
        self.actuator.apply(command).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_physical_core::{ControlMode, Quantity, Unit};
    use atomr_physical_testkit::MockActuator;

    #[test]
    fn rejecting_envelope_errors_out_of_range() {
        let env = SafetyEnvelope::rejecting(-1.0, 1.0);
        let id = ActuatorId::from("a1");
        assert!(env.enforce(&id, 0.5).is_ok());
        assert!(env.enforce(&id, 2.0).is_err());
    }

    #[test]
    fn clamping_envelope_pins_to_boundary() {
        let env = SafetyEnvelope::clamping(-1.0, 1.0);
        let id = ActuatorId::from("a1");
        assert_eq!(env.enforce(&id, 5.0).unwrap(), 1.0);
        assert_eq!(env.enforce(&id, -5.0).unwrap(), -1.0);
    }

    #[tokio::test]
    async fn actuator_actor_clamps_before_dispatch() {
        let driver = Arc::new(MockActuator::new("a1"));
        let actor = ActuatorActor::new(driver.clone()).with_envelope(SafetyEnvelope::clamping(0.0, 1.0));
        let cmd = Command::now(
            ActuatorId::from("a1"),
            ControlMode::Duty,
            Quantity::new(3.0, Unit::Percent),
        );
        let ack = actor.dispatch(cmd).await.unwrap();
        assert!(ack.accepted);
        assert_eq!(driver.log()[0].setpoint.value, 1.0);
    }
}
