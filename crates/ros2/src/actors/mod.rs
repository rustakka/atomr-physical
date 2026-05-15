//! The orchestration layer — atomr actors wiring atomr-physical devices
//! to the ROS2 graph (Model 2: one actor per endpoint, supervised by a
//! [`Ros2NodeActor`]).
//!
//! `SensorActor` / `ActuatorActor` are not yet live atomr `Actor`s
//! ("Phase 2 wires them"), so the orchestration is built against an
//! interim **device seam**: [`ReadingSource`] and [`CommandSink`]. Today,
//! [`SensorActorSource`] / [`ActuatorActorSink`] adapt the plain-struct
//! device actors; when the Phase-2 actor wiring lands, only those
//! adapters change — the orchestration actors are untouched.
//!
//! [`Ros2NodeActor`]: node::Ros2NodeActor

pub mod action;
pub mod node;
pub mod param;
pub mod publisher;
pub mod service;
pub mod subscriber;

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use atomr_physical_actuation::ActuatorActor;
use atomr_physical_core::{ActuatorId, Command, CommandAck, Reading, Result, SensorId};
use atomr_physical_sensing::SensorActor;

use crate::codec::CodecValue;
use crate::error::Ros2Error;
use crate::param::ParamValue;

pub use action::{Ros2ActionActor, Ros2ActionMsg};
pub use node::{spawn_inbound_pump, Ros2NodeActor, Ros2NodeMsg};
pub use param::{Ros2ParamActor, Ros2ParamMsg};
pub use publisher::{Ros2PublisherActor, Ros2PublisherMsg};
pub use service::{Ros2ServiceActor, Ros2ServiceMsg};
pub use subscriber::{Ros2SubscriberActor, Ros2SubscriberMsg};

#[cfg(any(test, feature = "mock"))]
pub mod testing;

/// A source of [`Reading`]s for one sensor — the device seam a
/// [`Ros2PublisherActor`] pulls from.
///
/// [`SensorActorSource`] is the standard implementation, adapting a
/// `SensorActor`. Tests and downstream code can implement this directly.
#[async_trait]
pub trait ReadingSource: Send + Sync + 'static {
    /// The sensor this source produces readings for.
    fn sensor_id(&self) -> SensorId;

    /// The sampling period, if the source is rate-based. `None` means
    /// the source is sampled only on explicit demand.
    fn sampling_period(&self) -> Option<Duration>;

    /// Take the next reading from the source.
    async fn next_reading(&self) -> Result<Reading>;
}

/// A sink for [`Command`]s for one actuator — the device seam a
/// [`Ros2SubscriberActor`] dispatches into.
///
/// [`ActuatorActorSink`] is the standard implementation, adapting an
/// `ActuatorActor`. Tests and downstream code can implement this
/// directly.
#[async_trait]
pub trait CommandSink: Send + Sync + 'static {
    /// The actuator this sink dispatches to.
    fn actuator_id(&self) -> ActuatorId;

    /// Dispatch a command to the actuator and return its acknowledgement.
    async fn deliver(&self, command: Command) -> Result<CommandAck>;
}

/// Handles requests for a ROS2 service this node serves — the seam a
/// [`Ros2ServiceActor`] in the server role `ask`s.
///
/// Request and response are generic [`CodecValue`]s: a ROS2 service
/// shape rarely maps onto a `Reading` or `Command`, so the handler owns
/// the interpretation.
#[async_trait]
pub trait ServiceHandler: Send + Sync + 'static {
    /// Produce a response for one service request.
    async fn handle(&self, request: CodecValue) -> std::result::Result<CodecValue, Ros2Error>;
}

/// Drives goals for a ROS2 action this node serves — the seam a
/// [`Ros2ActionActor`] in the server role delegates to.
///
/// `execute` runs one goal to completion and returns its result; the
/// `Ros2ActionActor` owns the goal lifecycle (accept, feedback fan-out,
/// result, cancel) around it.
#[async_trait]
pub trait ActionHandler: Send + Sync + 'static {
    /// Execute one action goal to completion.
    async fn execute(&self, goal: CodecValue) -> std::result::Result<CodecValue, Ros2Error>;
}

/// Mirrors atomr-physical configuration as ROS2 parameters — the seam a
/// [`Ros2ParamActor`] reads from and writes back to.
pub trait ParamStore: Send + Sync + 'static {
    /// The current parameter values to mirror onto the ROS2 node.
    fn snapshot(&self) -> Vec<(String, ParamValue)>;

    /// Apply a parameter value an external client changed. Returns an
    /// error if the parameter is unknown or the value is rejected.
    fn apply(&self, name: &str, value: ParamValue) -> std::result::Result<(), Ros2Error>;
}

/// Adapts a [`SensorActor`] into a [`ReadingSource`].
///
/// Holds the device actor behind an `Arc` so the bridge can keep its own
/// handle. When `SensorActor` becomes a live atomr `Actor`, this adapter
/// changes to talk to its mailbox — nothing else in the bridge moves.
pub struct SensorActorSource {
    inner: Arc<SensorActor>,
}

impl SensorActorSource {
    /// Adapt a `SensorActor` shared behind an `Arc`.
    pub fn new(sensor: Arc<SensorActor>) -> Self {
        Self { inner: sensor }
    }
}

#[async_trait]
impl ReadingSource for SensorActorSource {
    fn sensor_id(&self) -> SensorId {
        self.inner.id()
    }

    fn sampling_period(&self) -> Option<Duration> {
        self.inner.policy().period()
    }

    async fn next_reading(&self) -> Result<Reading> {
        self.inner.sample().await
    }
}

/// Adapts an [`ActuatorActor`] into a [`CommandSink`].
///
/// Holds the device actor behind an `Arc` so the bridge can keep its own
/// handle. When `ActuatorActor` becomes a live atomr `Actor`, this
/// adapter changes to talk to its mailbox — nothing else in the bridge
/// moves.
pub struct ActuatorActorSink {
    inner: Arc<ActuatorActor>,
}

impl ActuatorActorSink {
    /// Adapt an `ActuatorActor` shared behind an `Arc`.
    pub fn new(actuator: Arc<ActuatorActor>) -> Self {
        Self { inner: actuator }
    }
}

#[async_trait]
impl CommandSink for ActuatorActorSink {
    fn actuator_id(&self) -> ActuatorId {
        self.inner.id()
    }

    async fn deliver(&self, command: Command) -> Result<CommandAck> {
        self.inner.dispatch(command).await
    }
}

/// The device-seam wiring a [`Ros2NodeActor`] needs to spawn its
/// per-endpoint children: a [`ReadingSource`] per published sensor, a
/// [`CommandSink`] per subscribed actuator, a [`ServiceHandler`] per
/// served service, an [`ActionHandler`] per served action, and at most
/// one [`ParamStore`] for the node's parameters.
///
/// Built with the fluent `with_*` methods, then handed to
/// [`Ros2NodeActor::new`] alongside the [`Ros2Plan`](crate::Ros2Plan).
/// A plan endpoint with no matching entry here is skipped with a
/// warning when the node starts.
#[derive(Clone, Default)]
pub struct Ros2Wiring {
    /// Reading sources, keyed by sensor id.
    pub sources: HashMap<SensorId, Arc<dyn ReadingSource>>,
    /// Command sinks, keyed by actuator id.
    pub sinks: HashMap<ActuatorId, Arc<dyn CommandSink>>,
    /// Service handlers, keyed by service name.
    pub service_handlers: HashMap<String, Arc<dyn ServiceHandler>>,
    /// Action handlers, keyed by action name.
    pub action_handlers: HashMap<String, Arc<dyn ActionHandler>>,
    /// The node's parameter store, if any parameters are mirrored.
    pub param_store: Option<Arc<dyn ParamStore>>,
}

impl Ros2Wiring {
    /// Empty wiring.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a reading source, keyed by its [`sensor_id`](ReadingSource::sensor_id).
    pub fn with_source(mut self, source: Arc<dyn ReadingSource>) -> Self {
        self.sources.insert(source.sensor_id(), source);
        self
    }

    /// Add a command sink, keyed by its [`actuator_id`](CommandSink::actuator_id).
    pub fn with_sink(mut self, sink: Arc<dyn CommandSink>) -> Self {
        self.sinks.insert(sink.actuator_id(), sink);
        self
    }

    /// Add a service handler, keyed by the service name it serves.
    pub fn with_service_handler(
        mut self,
        service: impl Into<String>,
        handler: Arc<dyn ServiceHandler>,
    ) -> Self {
        self.service_handlers.insert(service.into(), handler);
        self
    }

    /// Add an action handler, keyed by the action name it serves.
    pub fn with_action_handler(mut self, action: impl Into<String>, handler: Arc<dyn ActionHandler>) -> Self {
        self.action_handlers.insert(action.into(), handler);
        self
    }

    /// Set the node's parameter store.
    pub fn with_param_store(mut self, store: Arc<dyn ParamStore>) -> Self {
        self.param_store = Some(store);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_physical_core::Unit;
    use atomr_physical_sensing::SamplingPolicy;
    use atomr_physical_testkit::{MockActuator, MockSensor};

    #[tokio::test]
    async fn sensor_actor_source_adapts_a_sensor_actor() {
        let driver = Arc::new(MockSensor::constant("s1", 21.0, Unit::Celsius));
        let actor = Arc::new(SensorActor::new(
            driver,
            SamplingPolicy::FixedRate { period_ms: 50 },
        ));
        let source = SensorActorSource::new(actor);
        assert_eq!(source.sensor_id(), SensorId::from("s1"));
        assert_eq!(source.sampling_period(), Some(Duration::from_millis(50)));
        let reading = source.next_reading().await.unwrap();
        assert_eq!(reading.quantity.value, 21.0);
    }

    #[tokio::test]
    async fn actuator_actor_sink_adapts_an_actuator_actor() {
        let driver = Arc::new(MockActuator::new("a1"));
        let actor = Arc::new(ActuatorActor::new(driver.clone()));
        let sink = ActuatorActorSink::new(actor);
        assert_eq!(sink.actuator_id(), ActuatorId::from("a1"));

        let command = Command::now(
            ActuatorId::from("a1"),
            atomr_physical_core::ControlMode::Position,
            atomr_physical_core::Quantity::new(1.0, Unit::Radian),
        );
        let ack = sink.deliver(command).await.unwrap();
        assert!(ack.accepted);
        assert_eq!(driver.command_count(), 1);
    }
}
