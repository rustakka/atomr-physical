//! [`Ros2NodeActor`] — the per-robot supervisor and event router.
//!
//! In `pre_start` the node reads the [`Ros2Plan`] and spawns one actor
//! per endpoint — a [`Ros2PublisherActor`] per bound sensor, a
//! [`Ros2SubscriberActor`] per bound actuator, a [`Ros2ServiceActor`]
//! per served service, a [`Ros2ActionActor`] per served action, and one
//! [`Ros2ParamActor`] if parameters are declared — building a routing
//! table from endpoint name to child `ActorRef`. It then routes each
//! inbound [`Ros2Event`] to the child that owns the endpoint. The actor
//! graph mirrors the ROS2 graph — that is Model 2.
//!
//! The node does **not** own the inbound event receiver (it would not
//! survive a restart): the receiver is drained by [`spawn_inbound_pump`],
//! a detached task that `tell`s the node, wired up once the node's
//! `ActorRef` exists.

use std::collections::HashMap;
use std::sync::Arc;

use atomr_physical_core::{ActuatorId, SensorId};
use tokio::sync::mpsc;

use crate::action::ActionRole;
use crate::actor::prelude::*;
use crate::actors::action::{Ros2ActionActor, Ros2ActionMsg};
use crate::actors::param::{Ros2ParamActor, Ros2ParamMsg};
use crate::actors::publisher::{Ros2PublisherActor, Ros2PublisherMsg};
use crate::actors::service::{Ros2ServiceActor, Ros2ServiceMsg};
use crate::actors::subscriber::{Ros2SubscriberActor, Ros2SubscriberMsg};
use crate::actors::Ros2Wiring;
use crate::plan::Ros2Plan;
use crate::service::ServiceRole;
use crate::transport::{Ros2Event, Ros2Link};

/// Messages a [`Ros2NodeActor`] handles.
pub enum Ros2NodeMsg {
    /// An event arrived from the transport — delivered by the inbound
    /// pump ([`spawn_inbound_pump`]).
    Event(Ros2Event),
    /// Trigger an immediate publish on a sensor's publisher. Drives
    /// on-demand sensors, and gives tests a deterministic publish point.
    TriggerPublish(SensorId),
}

/// The per-robot supervisor: spawns one actor per endpoint and routes
/// inbound events to the child that owns the endpoint.
pub struct Ros2NodeActor {
    node_name: String,
    plan: Ros2Plan,
    link: Ros2Link,
    wiring: Ros2Wiring,
    publishers: HashMap<SensorId, ActorRef<Ros2PublisherMsg>>,
    subscribers: HashMap<ActuatorId, ActorRef<Ros2SubscriberMsg>>,
    services: HashMap<String, ActorRef<Ros2ServiceMsg>>,
    actions: HashMap<String, ActorRef<Ros2ActionMsg>>,
    param_actor: Option<ActorRef<Ros2ParamMsg>>,
}

impl Ros2NodeActor {
    /// Construct a node actor for `node_name`.
    ///
    /// `plan` is the full ROS2 plan; `wiring` supplies the device seam
    /// for every endpoint it binds. A plan endpoint with no matching
    /// entry in `wiring` is skipped with a warning when the node starts.
    pub fn new(node_name: impl Into<String>, plan: Ros2Plan, link: Ros2Link, wiring: Ros2Wiring) -> Self {
        Self {
            node_name: node_name.into(),
            plan,
            link,
            wiring,
            publishers: HashMap::new(),
            subscribers: HashMap::new(),
            services: HashMap::new(),
            actions: HashMap::new(),
            param_actor: None,
        }
    }

    /// Route one inbound event to the child that owns its endpoint.
    fn route_event(&self, event: Ros2Event) {
        match event {
            Ros2Event::Inbound {
                actuator,
                topic,
                command,
            } => match self.subscribers.get(&actuator) {
                Some(subscriber) => subscriber.tell(Ros2SubscriberMsg::Deliver(command)),
                None => tracing::warn!(
                    node = %self.node_name,
                    actuator = %actuator,
                    topic = %topic,
                    "ros2 node: inbound command for an unbound actuator",
                ),
            },
            Ros2Event::ServiceRequest {
                service,
                request_id,
                payload,
            } => match self.services.get(&service) {
                Some(actor) => actor.tell(Ros2ServiceMsg::Request { request_id, payload }),
                None => tracing::warn!(
                    node = %self.node_name,
                    %service,
                    "ros2 node: request for a service this node does not serve",
                ),
            },
            Ros2Event::ActionGoal {
                action,
                goal_id,
                payload,
            } => match self.actions.get(&action) {
                Some(actor) => actor.tell(Ros2ActionMsg::Goal { goal_id, payload }),
                None => tracing::warn!(
                    node = %self.node_name,
                    %action,
                    "ros2 node: goal for an action this node does not serve",
                ),
            },
            Ros2Event::ActionCancel { action, goal_id } => match self.actions.get(&action) {
                Some(actor) => actor.tell(Ros2ActionMsg::Cancel { goal_id }),
                None => tracing::warn!(
                    node = %self.node_name,
                    %action,
                    "ros2 node: cancel for an action this node does not serve",
                ),
            },
            Ros2Event::ParamChanged { name, value } => match &self.param_actor {
                Some(actor) => actor.tell(Ros2ParamMsg::Changed { name, value }),
                None => tracing::warn!(
                    node = %self.node_name,
                    param = %name,
                    "ros2 node: param change but no parameter store is wired",
                ),
            },
            Ros2Event::NodeReady { node_name } => {
                tracing::info!(node = %node_name, "ros2 node: graph ready");
            }
            Ros2Event::DecodeError { endpoint, detail } => {
                tracing::warn!(node = %self.node_name, %endpoint, %detail, "ros2 node: decode error");
            }
            Ros2Event::EndpointFault { endpoint, detail } => {
                tracing::warn!(node = %self.node_name, %endpoint, %detail, "ros2 node: endpoint fault");
            }
            Ros2Event::Closed { reason } => {
                tracing::warn!(node = %self.node_name, ?reason, "ros2 node: transport closed");
            }
        }
    }

    /// Spawn the per-sensor publisher children.
    fn spawn_publishers(&mut self, ctx: &mut Context<Self>) {
        let bindings: Vec<(SensorId, _)> = self
            .plan
            .topics()
            .sensor_bindings()
            .map(|(id, endpoint)| (id.clone(), endpoint.clone()))
            .collect();
        for (sensor, endpoint) in bindings {
            let Some(source) = self.wiring.sources.get(&sensor).cloned() else {
                tracing::warn!(
                    node = %self.node_name,
                    sensor = %sensor,
                    "ros2 node: sensor bound in the plan has no ReadingSource — skipping",
                );
                continue;
            };
            let link = self.link.clone();
            let name = format!("pub-{}", sensor.as_str());
            match ctx.spawn(
                Props::create(move || {
                    Ros2PublisherActor::new(endpoint.clone(), Arc::clone(&source), link.clone())
                }),
                &name,
            ) {
                Ok(child) => {
                    self.publishers.insert(sensor, child);
                }
                Err(err) => tracing::warn!(
                    node = %self.node_name, sensor = %sensor, error = %err,
                    "ros2 node: failed to spawn publisher",
                ),
            }
        }
    }

    /// Spawn the per-actuator subscriber children.
    fn spawn_subscribers(&mut self, ctx: &mut Context<Self>) {
        let bindings: Vec<(ActuatorId, _)> = self
            .plan
            .topics()
            .actuator_bindings()
            .map(|(id, endpoint)| (id.clone(), endpoint.clone()))
            .collect();
        for (actuator, endpoint) in bindings {
            let Some(sink) = self.wiring.sinks.get(&actuator).cloned() else {
                tracing::warn!(
                    node = %self.node_name,
                    actuator = %actuator,
                    "ros2 node: actuator bound in the plan has no CommandSink — skipping",
                );
                continue;
            };
            let name = format!("sub-{}", actuator.as_str());
            match ctx.spawn(
                Props::create(move || Ros2SubscriberActor::new(endpoint.clone(), Arc::clone(&sink))),
                &name,
            ) {
                Ok(child) => {
                    self.subscribers.insert(actuator, child);
                }
                Err(err) => tracing::warn!(
                    node = %self.node_name, actuator = %actuator, error = %err,
                    "ros2 node: failed to spawn subscriber",
                ),
            }
        }
    }

    /// Spawn one service actor per **served** service endpoint. Client-
    /// role endpoints are wired with the live `rclrs` service client.
    fn spawn_services(&mut self, ctx: &mut Context<Self>) {
        let endpoints: Vec<_> = self.plan.services().to_vec();
        for endpoint in endpoints {
            if endpoint.role != ServiceRole::Server {
                tracing::debug!(
                    node = %self.node_name,
                    service = %endpoint.service,
                    "ros2 node: client-role service — wired with the rclrs service client",
                );
                continue;
            }
            let Some(handler) = self.wiring.service_handlers.get(&endpoint.service).cloned() else {
                tracing::warn!(
                    node = %self.node_name,
                    service = %endpoint.service,
                    "ros2 node: served service has no ServiceHandler — skipping",
                );
                continue;
            };
            let link = self.link.clone();
            let name = format!("svc-{}", sanitize(&endpoint.service));
            let service = endpoint.service.clone();
            match ctx.spawn(
                Props::create(move || {
                    Ros2ServiceActor::new(endpoint.clone(), Arc::clone(&handler), link.clone())
                }),
                &name,
            ) {
                Ok(child) => {
                    self.services.insert(service, child);
                }
                Err(err) => tracing::warn!(
                    node = %self.node_name, %service, error = %err,
                    "ros2 node: failed to spawn service actor",
                ),
            }
        }
    }

    /// Spawn one action actor per **served** action endpoint. Client-
    /// role endpoints are wired with the live `rclrs` action client.
    fn spawn_actions(&mut self, ctx: &mut Context<Self>) {
        let endpoints: Vec<_> = self.plan.actions().to_vec();
        for endpoint in endpoints {
            if endpoint.role != ActionRole::Server {
                tracing::debug!(
                    node = %self.node_name,
                    action = %endpoint.action,
                    "ros2 node: client-role action — wired with the rclrs action client",
                );
                continue;
            }
            let Some(handler) = self.wiring.action_handlers.get(&endpoint.action).cloned() else {
                tracing::warn!(
                    node = %self.node_name,
                    action = %endpoint.action,
                    "ros2 node: served action has no ActionHandler — skipping",
                );
                continue;
            };
            let link = self.link.clone();
            let name = format!("act-{}", sanitize(&endpoint.action));
            let action = endpoint.action.clone();
            match ctx.spawn(
                Props::create(move || {
                    Ros2ActionActor::new(endpoint.clone(), Arc::clone(&handler), link.clone())
                }),
                &name,
            ) {
                Ok(child) => {
                    self.actions.insert(action, child);
                }
                Err(err) => tracing::warn!(
                    node = %self.node_name, %action, error = %err,
                    "ros2 node: failed to spawn action actor",
                ),
            }
        }
    }

    /// Spawn the node's single parameter actor, if a store is wired and
    /// the plan declares parameters.
    fn spawn_param_actor(&mut self, ctx: &mut Context<Self>) {
        if self.plan.params().is_empty() {
            return;
        }
        let Some(store) = self.wiring.param_store.clone() else {
            tracing::warn!(
                node = %self.node_name,
                "ros2 node: plan declares parameters but no ParamStore is wired — skipping",
            );
            return;
        };
        let link = self.link.clone();
        match ctx.spawn(
            Props::create(move || Ros2ParamActor::new(Arc::clone(&store), link.clone())),
            "params",
        ) {
            Ok(child) => self.param_actor = Some(child),
            Err(err) => tracing::warn!(
                node = %self.node_name, error = %err,
                "ros2 node: failed to spawn parameter actor",
            ),
        }
    }
}

/// Replace ROS2 name separators so a name is usable as an actor-path
/// element.
fn sanitize(name: &str) -> String {
    name.trim_matches('/').replace('/', "_")
}

#[async_trait]
impl Actor for Ros2NodeActor {
    type Msg = Ros2NodeMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        self.spawn_publishers(ctx);
        self.spawn_subscribers(ctx);
        self.spawn_services(ctx);
        self.spawn_actions(ctx);
        self.spawn_param_actor(ctx);

        tracing::info!(
            node = %self.node_name,
            publishers = self.publishers.len(),
            subscribers = self.subscribers.len(),
            services = self.services.len(),
            actions = self.actions.len(),
            params = self.param_actor.is_some(),
            "ros2 node: endpoints spawned",
        );
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: Ros2NodeMsg) {
        match msg {
            Ros2NodeMsg::Event(event) => self.route_event(event),
            Ros2NodeMsg::TriggerPublish(sensor) => match self.publishers.get(&sensor) {
                Some(publisher) => publisher.tell(Ros2PublisherMsg::Tick),
                None => tracing::warn!(
                    node = %self.node_name,
                    sensor = %sensor,
                    "ros2 node: TriggerPublish for an unknown sensor",
                ),
            },
        }
    }
}

/// Drain the transport's inbound event stream into a node actor.
///
/// Spawns a detached task that forwards every [`Ros2Event`] to `node` as
/// a [`Ros2NodeMsg::Event`]. Call this once, after the node's `ActorRef`
/// exists — the node itself does not own the receiver so that it
/// survives a supervised restart.
pub fn spawn_inbound_pump(mut event_rx: mpsc::UnboundedReceiver<Ros2Event>, node: ActorRef<Ros2NodeMsg>) {
    tokio::spawn(async move {
        while let Some(event) = event_rx.recv().await {
            node.tell(Ros2NodeMsg::Event(event));
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action::Ros2ActionEndpoint;
    use crate::actors::testing::{MockCommandSink, MockReadingSource};
    use crate::actors::{ActionHandler, CommandSink, ReadingSource};
    use crate::codec::{CodecValue, Ros2Payload};
    use crate::endpoint::Ros2Endpoint;
    use crate::error::Ros2Error;
    use crate::service::Ros2ServiceEndpoint;
    use crate::transport::{MockRos2Transport, Ros2Command, Ros2Transport};
    use atomr_physical_core::{Command, ControlMode, Quantity, Reading, Unit};
    use std::sync::Mutex;
    use std::time::Duration;

    fn temperature(sensor: &str) -> Reading {
        Reading {
            sensor: SensorId::from(sensor),
            quantity: Quantity::new(22.0, Unit::Celsius),
            timestamp_ms: 0,
            frame: None,
        }
    }

    struct OkServiceHandler;

    #[async_trait]
    impl crate::actors::ServiceHandler for OkServiceHandler {
        async fn handle(&self, _request: CodecValue) -> Result<CodecValue, Ros2Error> {
            Ok(CodecValue::new(serde_json::json!({ "ok": true })))
        }
    }

    struct OkActionHandler;

    #[async_trait]
    impl ActionHandler for OkActionHandler {
        async fn execute(&self, _goal: CodecValue) -> Result<CodecValue, Ros2Error> {
            Ok(CodecValue::empty())
        }
    }

    /// Build a node wired to one sensor, one actuator, one served
    /// service, and one served action over a mock transport.
    async fn wired_node(
        sys: &ActorSystem,
    ) -> (
        ActorRef<Ros2NodeMsg>,
        crate::transport::MockRos2Handle,
        Arc<Mutex<Vec<Command>>>,
    ) {
        let (transport, handle) = MockRos2Transport::new();
        let (link, event_rx) = transport.start();

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

        let sink = MockCommandSink::new("a1");
        let log = sink.log_handle();
        let source: Arc<dyn ReadingSource> = Arc::new(MockReadingSource::new("s1", temperature("s1")));
        let sink_dyn: Arc<dyn CommandSink> = Arc::new(sink);
        let wiring = Ros2Wiring::new()
            .with_source(source)
            .with_sink(sink_dyn)
            .with_service_handler("/arm/home", Arc::new(OkServiceHandler))
            .with_action_handler("/arm/traj", Arc::new(OkActionHandler));

        let node = sys
            .actor_of(
                Props::create(move || {
                    Ros2NodeActor::new("arm_node", plan.clone(), link.clone(), wiring.clone())
                }),
                "node",
            )
            .unwrap();
        spawn_inbound_pump(event_rx, node.clone());
        (node, handle, log)
    }

    #[tokio::test]
    async fn node_routes_a_triggered_publish_out_to_the_transport() {
        let sys = ActorSystem::create("node-pub", Config::empty()).await.unwrap();
        let (node, mut handle, _log) = wired_node(&sys).await;

        node.tell(Ros2NodeMsg::TriggerPublish(SensorId::from("s1")));

        let command = tokio::time::timeout(Duration::from_secs(1), handle.next_command())
            .await
            .expect("no publish reached the transport")
            .expect("link closed");
        match command {
            Ros2Command::Publish { sensor, reading } => {
                assert_eq!(sensor, SensorId::from("s1"));
                assert_eq!(reading.quantity.value, 22.0);
            }
            other => panic!("expected Publish, got {other:?}"),
        }
        sys.terminate().await;
    }

    #[tokio::test]
    async fn node_routes_an_inbound_command_to_the_subscriber() {
        let sys = ActorSystem::create("node-sub", Config::empty()).await.unwrap();
        let (_node, handle, log) = wired_node(&sys).await;

        let command = Command::now(
            ActuatorId::from("a1"),
            ControlMode::Position,
            Quantity::new(1.25, Unit::Radian),
        );
        assert!(handle.inject(Ros2Event::Inbound {
            actuator: ActuatorId::from("a1"),
            topic: "/arm/cmd".into(),
            command,
        }));

        let mut delivered = false;
        for _ in 0..50 {
            if log.lock().unwrap().len() == 1 {
                delivered = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(delivered, "inbound command never reached the sink");
        sys.terminate().await;
    }

    #[tokio::test]
    async fn node_routes_a_service_request_to_the_service_actor() {
        let sys = ActorSystem::create("node-svc", Config::empty()).await.unwrap();
        let (_node, mut handle, _log) = wired_node(&sys).await;

        assert!(handle.inject(Ros2Event::ServiceRequest {
            service: "/arm/home".into(),
            request_id: 1,
            payload: Ros2Payload::empty(),
        }));

        let command = tokio::time::timeout(Duration::from_secs(1), handle.next_command())
            .await
            .expect("no service response reached the transport")
            .expect("link closed");
        match command {
            Ros2Command::ServiceResponse { request_id, .. } => assert_eq!(request_id, 1),
            other => panic!("expected ServiceResponse, got {other:?}"),
        }
        sys.terminate().await;
    }

    #[tokio::test]
    async fn node_routes_an_action_goal_to_the_action_actor() {
        let sys = ActorSystem::create("node-act", Config::empty()).await.unwrap();
        let (_node, mut handle, _log) = wired_node(&sys).await;

        assert!(handle.inject(Ros2Event::ActionGoal {
            action: "/arm/traj".into(),
            goal_id: crate::action::GoalId::from("g1"),
            payload: Ros2Payload::empty(),
        }));

        let command = tokio::time::timeout(Duration::from_secs(1), handle.next_command())
            .await
            .expect("no action result reached the transport")
            .expect("link closed");
        match command {
            Ros2Command::ActionResult { goal_id, .. } => {
                assert_eq!(goal_id, crate::action::GoalId::from("g1"));
            }
            other => panic!("expected ActionResult, got {other:?}"),
        }
        sys.terminate().await;
    }

    #[tokio::test]
    async fn node_drops_inbound_events_for_unbound_endpoints() {
        let sys = ActorSystem::create("node-unbound", Config::empty())
            .await
            .unwrap();
        let (_node, handle, log) = wired_node(&sys).await;

        // An actuator not in the plan — dropped, not panicked.
        let command = Command::now(
            ActuatorId::from("a9"),
            ControlMode::Position,
            Quantity::new(0.0, Unit::Radian),
        );
        assert!(handle.inject(Ros2Event::Inbound {
            actuator: ActuatorId::from("a9"),
            topic: "/arm/ghost".into(),
            command,
        }));

        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(log.lock().unwrap().is_empty());
        sys.terminate().await;
    }
}
