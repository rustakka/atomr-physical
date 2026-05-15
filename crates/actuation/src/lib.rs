//! Actuator-side actors for atomr-physical.
//!
//! A hardware driver implements [`atomr_physical_core::Actuator`] in
//! plain async Rust. This crate adapts that implementation into a
//! supervised atomr actor — [`ActuatorActor`] — that serialises
//! [`Command`]s through a mailbox and enforces a [`SafetyEnvelope`]
//! before anything reaches hardware.
//!
//! Two ways to use an [`ActuatorActor`]:
//!
//! 1. **Direct** — construct it and call [`ActuatorActor::dispatch`]. No
//!    runtime; useful in tests and one-shot commands.
//! 2. **Supervised** — call [`ActuatorActor::spawn`] to promote it into
//!    a live atomr actor under an [`atomr_core::actor::ActorSystem`].
//!    The returned [`ActuatorActorRef`] is a typed handle to the
//!    mailbox so callers see the envelope check + driver dispatch run
//!    under supervision.
//!
//! The atomr actor runtime is re-exported as [`actor`] so downstream
//! crates have a single import path for it.

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_physical_core::{Actuator, ActuatorId, Command, CommandAck, PhysicalError, Result};
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;

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
/// Construct with [`ActuatorActor::new`], attach a [`SafetyEnvelope`]
/// with [`with_envelope`](Self::with_envelope), and either call
/// [`dispatch`](Self::dispatch) directly for hardware-free tests or
/// [`spawn`](Self::spawn) (or [`spawn_under`](Self::spawn_under) for a
/// child under a parent actor) to promote it to a live, supervised
/// actor running on an [`ActorSystem`].
///
/// `Clone` is cheap — the only non-`Copy` field is the
/// `Arc<dyn Actuator>` — so the same configuration can both be
/// dispatched directly and spawned as an actor.
#[derive(Clone)]
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
    /// the underlying driver directly.
    ///
    /// This bypasses the mailbox. After [`spawn`](Self::spawn) the same
    /// effect is available through [`ActuatorActorRef::dispatch`].
    pub async fn dispatch(&self, mut command: Command) -> Result<CommandAck> {
        if let Some(envelope) = &self.envelope {
            let safe = envelope.enforce(&command.actuator, command.setpoint.value)?;
            command.setpoint.value = safe;
        }
        self.actuator.apply(command).await
    }

    /// Promote this actuator into a supervised atomr actor under
    /// `system`, registered at `name`. Returns an [`ActuatorActorRef`]
    /// — a typed handle to the mailbox.
    ///
    /// The actor drains a single command per `handle` call: it runs the
    /// envelope check, forwards to the driver, and replies with the
    /// driver's [`CommandAck`] (or the envelope's
    /// [`PhysicalError::OutOfRange`]).
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<ActuatorActorRef, ActorSystemError> {
        let (props, id, envelope) = self.into_runner_props();
        let actor_ref = system.actor_of(props, name)?;
        Ok(ActuatorActorRef {
            inner: actor_ref,
            id,
            envelope,
        })
    }

    /// Promote this actuator into a supervised atomr actor as a
    /// **child** of the parent actor `P` whose [`Context`] is `ctx`.
    /// Used by [`atomr_physical_robotics::RobotActor`] to build its
    /// supervised subtree.
    ///
    /// Returns [`PhysicalError::Fault`] if atomr refuses the spawn (e.g.
    /// duplicate child name). The underlying `SpawnError` type isn't
    /// reachable through atomr-core 0.9.2's public surface, so we
    /// stringify it at the boundary.
    pub fn spawn_under<P: Actor>(self, ctx: &mut Context<P>, name: &str) -> Result<ActuatorActorRef> {
        let (props, id, envelope) = self.into_runner_props();
        let actor_ref = ctx
            .spawn(props, name)
            .map_err(|e| PhysicalError::Fault(format!("actuator child spawn failed: {e}")))?;
        Ok(ActuatorActorRef {
            inner: actor_ref,
            id,
            envelope,
        })
    }

    fn into_runner_props(self) -> (Props<ActuatorRunner>, ActuatorId, Option<SafetyEnvelope>) {
        let id = self.id();
        let actuator = self.actuator;
        let envelope = self.envelope;
        let props = Props::create(move || ActuatorRunner {
            actuator: actuator.clone(),
            envelope,
        });
        (props, id, envelope)
    }
}

/// The mailbox protocol of a live [`ActuatorActor`].
///
/// Construct messages through [`ActuatorActorRef`] rather than reaching
/// for the variants directly; the helpers wrap the oneshot replies and
/// the ask timeout.
pub enum ActuatorMsg {
    /// Run the envelope check and forward to the driver, replying over
    /// `reply` with the driver's ack or the envelope's error.
    Dispatch {
        /// The command to dispatch.
        command: Command,
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<CommandAck>>,
    },
    /// Run the driver's health check and reply.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
}

/// A typed handle to a spawned [`ActuatorActor`].
///
/// Cheap to clone; `tell`/`ask` go over the actor's mailbox.
#[derive(Clone)]
pub struct ActuatorActorRef {
    inner: ActorRef<ActuatorMsg>,
    id: ActuatorId,
    envelope: Option<SafetyEnvelope>,
}

impl ActuatorActorRef {
    /// The id of the wrapped actuator.
    pub fn id(&self) -> &ActuatorId {
        &self.id
    }

    /// The safety envelope in force, if any.
    pub fn envelope(&self) -> Option<SafetyEnvelope> {
        self.envelope
    }

    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<ActuatorMsg> {
        &self.inner
    }

    /// Ask the actor to run the envelope check and dispatch a command,
    /// returning the driver's [`CommandAck`].
    pub async fn dispatch(&self, command: Command) -> Result<CommandAck> {
        self.inner
            .ask_with(|reply| ActuatorMsg::Dispatch { command, reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }

    /// Ask the actor to run the driver's health check.
    pub async fn health_check(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| ActuatorMsg::Health { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }
}

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("actuator actor ask failed: {e:?}"))
}

/// Internal Actor implementation backing a spawned [`ActuatorActor`].
struct ActuatorRunner {
    actuator: Arc<dyn Actuator>,
    envelope: Option<SafetyEnvelope>,
}

#[async_trait]
impl Actor for ActuatorRunner {
    type Msg = ActuatorMsg;

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: ActuatorMsg) {
        match msg {
            ActuatorMsg::Dispatch { mut command, reply } => {
                let result = if let Some(envelope) = &self.envelope {
                    match envelope.enforce(&command.actuator, command.setpoint.value) {
                        Ok(safe) => {
                            command.setpoint.value = safe;
                            self.actuator.apply(command).await
                        }
                        Err(e) => Err(e),
                    }
                } else {
                    self.actuator.apply(command).await
                };
                let _ = reply.send(result);
            }
            ActuatorMsg::Health { reply } => {
                let _ = reply.send(self.actuator.health_check().await);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_core::actor::ActorSystem;
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

    #[tokio::test]
    async fn spawned_actuator_clamps_before_dispatch() {
        let sys = ActorSystem::create("actuation-clamp", atomr_config::Config::reference())
            .await
            .unwrap();
        let driver = Arc::new(MockActuator::new("a1"));
        let actor_ref = ActuatorActor::new(driver.clone())
            .with_envelope(SafetyEnvelope::clamping(0.0, 1.0))
            .spawn(&sys, "joint-0")
            .unwrap();
        let cmd = Command::now(
            ActuatorId::from("a1"),
            ControlMode::Duty,
            Quantity::new(3.0, Unit::Percent),
        );
        let ack = actor_ref.dispatch(cmd).await.unwrap();
        assert!(ack.accepted);
        assert_eq!(driver.log()[0].setpoint.value, 1.0);
        sys.terminate().await;
    }

    #[tokio::test]
    async fn spawned_actuator_rejects_out_of_range() {
        let sys = ActorSystem::create("actuation-reject", atomr_config::Config::reference())
            .await
            .unwrap();
        let driver = Arc::new(MockActuator::new("a1"));
        let actor_ref = ActuatorActor::new(driver.clone())
            .with_envelope(SafetyEnvelope::rejecting(0.0, 1.0))
            .spawn(&sys, "joint-1")
            .unwrap();
        let cmd = Command::now(
            ActuatorId::from("a1"),
            ControlMode::Duty,
            Quantity::new(3.0, Unit::Percent),
        );
        let err = actor_ref.dispatch(cmd).await.unwrap_err();
        assert!(matches!(err, PhysicalError::OutOfRange { .. }));
        assert_eq!(driver.command_count(), 0);
        sys.terminate().await;
    }

    #[tokio::test]
    async fn spawned_actuator_health_check_succeeds() {
        let sys = ActorSystem::create("actuation-health", atomr_config::Config::reference())
            .await
            .unwrap();
        let driver = Arc::new(MockActuator::new("a1"));
        let actor_ref = ActuatorActor::new(driver).spawn(&sys, "h").unwrap();
        actor_ref.health_check().await.unwrap();
        sys.terminate().await;
    }
}
