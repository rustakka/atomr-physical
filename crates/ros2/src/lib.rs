//! ROS2 bridge for atomr-physical.
//!
//! This crate maps atomr-physical's actor world onto ROS2's
//! topic / service / action graph: a `SensorActor`'s reading stream
//! becomes a published topic, an `ActuatorActor`'s command mailbox
//! becomes a subscription, and a `RobotActor` becomes a ROS2 node.
//!
//! The offline surface — [`Ros2Endpoint`], [`TopicMap`], [`Ros2Bridge`]
//! — is transport-agnostic and builds with **no ROS2 installation**.
//! Behind the `rclrs` feature, [`Ros2Bridge::spin`] is implemented
//! against the [`rclrs`](https://github.com/ros2-rust/ros2_rust) client
//! library: it stands up a real ROS2 node, attaches a
//! [`DynamicPublisher`](rclrs::DynamicPublisher) for each sensor
//! endpoint and a [`DynamicSubscription`](rclrs::DynamicSubscription)
//! for each actuator endpoint, and drives them on the atomr runtime.
//! See [`docs/ros2-bridge.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/ros2-bridge.md).

use std::collections::HashMap;

#[cfg(not(feature = "rclrs"))]
use atomr_physical_core::PhysicalError;
use atomr_physical_core::{ActuatorId, Result, RobotId, SensorId};
use serde::{Deserialize, Serialize};

/// Re-export of the atomr actor runtime this crate builds on.
pub use atomr_core as actor;

/// Re-export of the `rclrs` client library when the matching feature is
/// enabled, so downstream crates have a single import path for it.
#[cfg(feature = "rclrs")]
pub use rclrs;

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

    /// Iterate every bound sensor → endpoint pair.
    pub fn sensor_bindings(&self) -> impl Iterator<Item = (&SensorId, &Ros2Endpoint)> {
        self.sensor_topics.iter()
    }

    /// Iterate every bound actuator → endpoint pair.
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

/// Bridges a `RobotActor`'s actor graph onto a ROS2 node.
///
/// The offline form owns a node name, a robot id, and a [`TopicMap`] —
/// the plan can be built, serialised, and unit-tested without any ROS2
/// installation.
///
/// Behind the `rclrs` feature, [`spin`](Self::spin) creates a real
/// `rclrs` context + node + executor, attaches a dynamic publisher /
/// subscription per [`TopicMap`] endpoint, and drives the executor
/// until the returned [`Ros2BridgeHandle`] is dropped or
/// [`shutdown`](Ros2BridgeHandle::shutdown)ed.
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

    /// Stand up the bridge against a live ROS 2 graph.
    ///
    /// Without the `rclrs` feature this returns
    /// [`PhysicalError::Ros2Bridge`] so callers fail fast rather than
    /// silently no-op.
    ///
    /// With `rclrs` enabled this:
    /// 1. initialises an `rclrs::Context` from process env (ROS args
    ///    are honoured),
    /// 2. creates an `rclrs::Executor` with a node named
    ///    [`node_name`](Self::node_name),
    /// 3. registers a [`DynamicPublisher`](rclrs::DynamicPublisher) for
    ///    every sensor endpoint and a
    ///    [`DynamicSubscription`](rclrs::DynamicSubscription) for every
    ///    actuator endpoint,
    /// 4. drives the executor asynchronously and returns a
    ///    [`Ros2BridgeHandle`] the caller uses to publish readings and
    ///    halt the executor.
    #[cfg(not(feature = "rclrs"))]
    pub async fn spin(&self) -> Result<Ros2BridgeHandle> {
        Err(PhysicalError::Ros2Bridge(format!(
            "rclrs feature not enabled — node {:?} ({} endpoints) planned but not spun; \
             see docs/ros2-bridge.md",
            self.node_name,
            self.topics.len(),
        )))
    }

    #[cfg(feature = "rclrs")]
    pub async fn spin(&self) -> Result<Ros2BridgeHandle> {
        rclrs_spin::spin(&self.node_name, &self.robot, &self.topics).await
    }
}

/// Live handle to a spinning [`Ros2Bridge`].
///
/// Without the `rclrs` feature this is an empty type returned only in
/// the error path; with the feature it carries the executor commands
/// and the per-endpoint publisher map, and exposes
/// [`publish_reading`](Self::publish_reading) /
/// [`shutdown`](Self::shutdown).
pub struct Ros2BridgeHandle {
    #[cfg(feature = "rclrs")]
    inner: rclrs_spin::SpinHandle,
}

impl Ros2BridgeHandle {
    /// Stop the spinning executor and wait for the spin task to wind
    /// down.
    pub async fn shutdown(self) -> Result<()> {
        #[cfg(feature = "rclrs")]
        {
            self.inner.shutdown().await
        }
        #[cfg(not(feature = "rclrs"))]
        {
            let _ = self;
            Err(PhysicalError::Ros2Bridge(
                "rclrs feature not enabled — no live bridge to shut down".into(),
            ))
        }
    }

    /// Publish a [`Reading`] on the topic bound to `sensor`. The
    /// reading's numeric value is written to the first `f64`-shaped
    /// field of the bound message type (`std_msgs/msg/Float64::data`,
    /// `sensor_msgs/msg/Temperature::temperature`, …).
    ///
    /// [`Reading`]: atomr_physical_core::Reading
    #[cfg(feature = "rclrs")]
    pub fn publish_reading(&self, sensor: &SensorId, reading: &atomr_physical_core::Reading) -> Result<()> {
        self.inner.publish_reading(sensor, reading)
    }

    /// Number of subscriptions currently observing this bridge's
    /// publisher endpoints. Useful for tests / discovery probes.
    #[cfg(feature = "rclrs")]
    pub fn subscriber_count(&self, sensor: &SensorId) -> Result<usize> {
        self.inner.subscriber_count(sensor)
    }

    /// The set of sensor ids that have a live publisher attached.
    #[cfg(feature = "rclrs")]
    pub fn published_sensors(&self) -> Vec<SensorId> {
        self.inner.published_sensors()
    }
}

#[cfg(feature = "rclrs")]
mod rclrs_spin {
    //! `rclrs` integration backing [`Ros2Bridge::spin`].
    //!
    //! Lives in its own module so the offline / no-rclrs build never
    //! has to mention any rclrs symbol. The dynamic-message API
    //! (`DynamicPublisher` / `DynamicSubscription`) lets us drive the
    //! TopicMap's `message_type` strings without colcon-generated Rust
    //! crates — exactly the trade-off `dyn_msg` is designed for.

    use std::collections::HashMap;
    use std::sync::{Arc, Mutex};

    use atomr_physical_core::{ActuatorId, PhysicalError, Reading, Result, RobotId, SensorId};
    use rclrs::*;
    use tokio::task::JoinHandle;

    use super::{Ros2BridgeHandle, Ros2Direction, TopicMap};

    /// A publisher together with the metadata needed to mint fresh
    /// `DynamicMessage`s of its type on every `publish_reading` call.
    struct PublisherEntry {
        publisher: DynamicPublisher,
        metadata: DynamicMessageMetadata,
    }

    /// State carried by a spinning bridge.
    pub(super) struct SpinHandle {
        publishers: HashMap<SensorId, PublisherEntry>,
        // The futures-oneshot sender held alongside the executor's
        // `until_promise_resolved` receiver. Dropping it on shutdown
        // resolves (cancels) the promise, which the executor's wait
        // loop notices and exits.
        shutdown_promise: Option<futures::channel::oneshot::Sender<()>>,
        commands: Arc<ExecutorCommands>,
        // Subscription handles are dropped on Drop; we keep them alive
        // for the lifetime of the bridge.
        _subscriptions: Vec<DynamicSubscription>,
        received: Arc<Mutex<Vec<(ActuatorId, String)>>>,
        spin_task: Option<JoinHandle<()>>,
    }

    impl SpinHandle {
        pub(super) async fn shutdown(mut self) -> Result<()> {
            // Resolving the promise tells the executor to stop spinning.
            // We belt-and-braces with `halt_spinning()` in case the
            // executor's promise polling is gated on a wait-set event.
            drop(self.shutdown_promise.take());
            self.commands.halt_spinning();
            if let Some(task) = self.spin_task.take() {
                let _ = task.await;
            }
            Ok(())
        }

        pub(super) fn publish_reading(&self, sensor: &SensorId, reading: &Reading) -> Result<()> {
            let entry = self.publishers.get(sensor).ok_or_else(|| {
                PhysicalError::Ros2Bridge(format!("no publisher bound for sensor {sensor}"))
            })?;
            let mut message = entry
                .metadata
                .create()
                .map_err(|e| PhysicalError::Ros2Bridge(format!("metadata.create: {e}")))?;
            // Walk the structure once and write the reading's numeric
            // value into the first `f64`-shaped field. This covers the
            // common-case message types — `std_msgs/Float64::data`,
            // `sensor_msgs/Temperature::temperature`, etc. — without
            // requiring a per-type codec table.
            write_first_float_field(&mut message, reading.quantity.value);
            entry
                .publisher
                .publish(message)
                .map_err(|e| PhysicalError::Ros2Bridge(format!("publish: {e}")))?;
            Ok(())
        }

        pub(super) fn subscriber_count(&self, sensor: &SensorId) -> Result<usize> {
            let entry = self.publishers.get(sensor).ok_or_else(|| {
                PhysicalError::Ros2Bridge(format!("no publisher bound for sensor {sensor}"))
            })?;
            entry
                .publisher
                .get_subscription_count()
                .map_err(|e| PhysicalError::Ros2Bridge(format!("get_subscription_count: {e}")))
        }

        pub(super) fn published_sensors(&self) -> Vec<SensorId> {
            self.publishers.keys().cloned().collect()
        }

        /// Snapshot of (actuator, topic) callbacks the bridge has
        /// received so far. Useful for tests.
        #[allow(dead_code)]
        pub(super) fn received(&self) -> Vec<(ActuatorId, String)> {
            self.received.lock().map(|g| g.clone()).unwrap_or_default()
        }
    }

    pub(super) async fn spin(
        node_name: &str,
        _robot: &RobotId,
        topics: &TopicMap,
    ) -> Result<Ros2BridgeHandle> {
        let context =
            Context::default_from_env().map_err(|e| PhysicalError::Ros2Bridge(format!("rclrs init: {e}")))?;
        let executor = context.create_basic_executor();
        let node = executor
            .create_node(node_name)
            .map_err(|e| PhysicalError::Ros2Bridge(format!("create_node({node_name}): {e}")))?;

        let mut publishers: HashMap<SensorId, PublisherEntry> = HashMap::new();
        for (sensor_id, endpoint) in topics.sensor_bindings() {
            debug_assert_eq!(endpoint.direction, Ros2Direction::Publish);
            let topic_type: MessageTypeName = endpoint.message_type.as_str().try_into().map_err(|e| {
                PhysicalError::Ros2Bridge(format!("invalid message type {:?}: {e}", endpoint.message_type))
            })?;
            let metadata = DynamicMessageMetadata::new(topic_type.clone()).map_err(|e| {
                PhysicalError::Ros2Bridge(format!("metadata({}): {e}", endpoint.message_type))
            })?;
            let publisher = node
                .create_dynamic_publisher(topic_type, endpoint.topic.as_str())
                .map_err(|e| {
                    PhysicalError::Ros2Bridge(format!(
                        "create_dynamic_publisher({}, {}): {e}",
                        endpoint.message_type, endpoint.topic
                    ))
                })?;
            publishers.insert(sensor_id.clone(), PublisherEntry { publisher, metadata });
        }

        let received: Arc<Mutex<Vec<(ActuatorId, String)>>> = Arc::new(Mutex::new(Vec::new()));
        let mut subscriptions: Vec<DynamicSubscription> = Vec::new();
        for (actuator_id, endpoint) in topics.actuator_bindings() {
            debug_assert_eq!(endpoint.direction, Ros2Direction::Subscribe);
            let topic_type: MessageTypeName = endpoint.message_type.as_str().try_into().map_err(|e| {
                PhysicalError::Ros2Bridge(format!("invalid message type {:?}: {e}", endpoint.message_type))
            })?;
            let received_for_cb = received.clone();
            let actuator_id_cb = actuator_id.clone();
            let endpoint_label = endpoint.topic.clone();
            let subscription = node
                .create_dynamic_subscription(
                    topic_type,
                    endpoint.topic.as_str(),
                    move |_msg: DynamicMessage, _info: MessageInfo| {
                        if let Ok(mut log) = received_for_cb.lock() {
                            log.push((actuator_id_cb.clone(), endpoint_label.clone()));
                        }
                        tracing::trace!(
                            actuator = %actuator_id_cb,
                            topic = %endpoint_label,
                            "actuator command received from ROS2"
                        );
                    },
                )
                .map_err(|e| {
                    PhysicalError::Ros2Bridge(format!(
                        "create_dynamic_subscription({}, {}): {e}",
                        endpoint.message_type, endpoint.topic
                    ))
                })?;
            subscriptions.push(subscription);
        }

        let commands = executor.commands().clone();
        let (shutdown_tx, shutdown_rx) = futures::channel::oneshot::channel::<()>();
        // The executor's `spin_async` consumes the executor by value.
        // We move it into a tokio task so the bridge handle can keep
        // halting it from the caller's task — the
        // `until_promise_resolved` wires the futures-oneshot above into
        // the executor's shutdown path so dropping `shutdown_tx` halts
        // spin promptly.
        let spin_task = tokio::spawn(async move {
            let options = SpinOptions::default().until_promise_resolved(shutdown_rx);
            let (_executor, _errs) = executor.spin_async(options).await;
        });

        Ok(Ros2BridgeHandle {
            inner: SpinHandle {
                publishers,
                shutdown_promise: Some(shutdown_tx),
                commands,
                _subscriptions: subscriptions,
                received,
                spin_task: Some(spin_task),
            },
        })
    }

    /// Walk the dynamic message's fields and write `value` into the
    /// first floating-point primitive. Returns silently if no float
    /// field is found — the publish still goes out with defaults.
    fn write_first_float_field(message: &mut DynamicMessage, value: f64) {
        let field_names: Vec<String> = message.iter().map(|(name, _)| name.to_string()).collect();
        for name in field_names {
            if let Some(field) = message.get_mut(&name) {
                if let ValueMut::Simple(simple) = field {
                    match simple {
                        SimpleValueMut::Double(slot) => {
                            *slot = value;
                            return;
                        }
                        SimpleValueMut::Float(slot) => {
                            *slot = value as f32;
                            return;
                        }
                        _ => {}
                    }
                }
            }
        }
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
    #[cfg(not(feature = "rclrs"))]
    async fn spin_without_rclrs_feature_errors() {
        let bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("r1"));
        assert!(bridge.spin().await.is_err());
    }

    #[tokio::test]
    #[cfg(feature = "rclrs")]
    async fn spin_with_rclrs_feature_stands_up_a_node() {
        let mut bridge = Ros2Bridge::new("atomr_physical_test_node", RobotId::from("r1"));
        bridge.topics_mut().bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::publish("/atomr_physical/test/temp", "std_msgs/msg/Float64"),
        );
        bridge.topics_mut().bind_actuator(
            ActuatorId::from("a1"),
            Ros2Endpoint::subscribe("/atomr_physical/test/cmd", "std_msgs/msg/Float64"),
        );
        let handle = bridge.spin().await.expect("spin should succeed");
        assert_eq!(handle.published_sensors().len(), 1);
        // Publishing a reading should not error even if no subscriber
        // is listening yet — the discovery layer just drops it.
        let reading = atomr_physical_core::Reading::now(
            SensorId::from("s1"),
            atomr_physical_core::Quantity::new(21.5, atomr_physical_core::Unit::Celsius),
        );
        handle.publish_reading(&SensorId::from("s1"), &reading).unwrap();
        handle.shutdown().await.unwrap();
    }
}
