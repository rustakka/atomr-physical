//! [`Ros2Bridge`] — the entry point that owns a robot's ROS2 plan and,
//! under the `rclrs` feature, spins it against a live ROS2 graph.

use std::sync::Arc;

use atomr_physical_core::{Result, RobotId};

use crate::actor::prelude::{ActorRef, ActorSystem};
use crate::actors::{Ros2NodeMsg, Ros2Wiring};
use crate::codec::CodecRegistry;
use crate::error::Ros2Error;
use crate::plan::Ros2Plan;
use crate::topic_map::TopicMap;
use crate::transport::{Ros2Command, Ros2Link};
use crate::validate::{validate_plan, ValidationError};

#[cfg(feature = "rclrs")]
use crate::actor::prelude::Props;
#[cfg(feature = "rclrs")]
use crate::actors::{spawn_inbound_pump, Ros2NodeActor};
#[cfg(feature = "rclrs")]
use crate::transport::{RclrsTransport, Ros2Transport};

/// Bridges a `RobotActor`'s actor graph onto a ROS2 node.
///
/// The bridge owns the node name and the [`Ros2Plan`] so the ROS2 graph
/// can be declared and validated offline. Under the `rclrs` feature,
/// [`Ros2Bridge::run`] spins the plan against a live `rclrs` node and
/// the Model 2 orchestration actors.
pub struct Ros2Bridge {
    node_name: String,
    robot: RobotId,
    plan: Ros2Plan,
}

impl Ros2Bridge {
    /// Construct a bridge for a robot, naming the ROS2 node.
    pub fn new(node_name: impl Into<String>, robot: RobotId) -> Self {
        Self {
            node_name: node_name.into(),
            robot,
            plan: Ros2Plan::new(),
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

    /// The bridge's full ROS2 plan — topics, services, actions, params.
    pub fn plan(&self) -> &Ros2Plan {
        &self.plan
    }

    /// The bridge's full ROS2 plan — mutate it to add services, actions,
    /// or parameters.
    pub fn plan_mut(&mut self) -> &mut Ros2Plan {
        &mut self.plan
    }

    /// The bridge's topic plan — the pub/sub slice of the full plan.
    pub fn topics(&self) -> &TopicMap {
        self.plan.topics()
    }

    /// The bridge's topic plan — mutate it to bind devices to topics.
    pub fn topics_mut(&mut self) -> &mut TopicMap {
        self.plan.topics_mut()
    }

    /// Lint the bridge's plan, returning every problem found.
    ///
    /// An empty `Vec` means the plan is well-formed. Callers should run
    /// this before [`run`](Ros2Bridge::run) — a bad plan is far easier
    /// to diagnose as data than as an `rclrs` failure on a live graph.
    pub fn validate(&self) -> Vec<ValidationError> {
        validate_plan(&self.plan)
    }

    /// Fail-fast shim — the live entry point is [`run`](Ros2Bridge::run).
    ///
    /// `spin` predates the live bridge; the live path needs an
    /// [`ActorSystem`], a [`Ros2Wiring`], and a [`CodecRegistry`], so it
    /// moved to `run`. `spin` always returns
    /// [`PhysicalError::Ros2Bridge`] so older callers fail fast rather
    /// than silently no-op.
    ///
    /// [`PhysicalError::Ros2Bridge`]: atomr_physical_core::PhysicalError::Ros2Bridge
    pub async fn spin(&self) -> Result<()> {
        Err(Ros2Error::FeatureDisabled(format!(
            "node {:?} ({} endpoints) planned but not spun — call `Ros2Bridge::run`; \
             see docs/ros2-bridge.md",
            self.node_name,
            self.plan.len(),
        ))
        .into())
    }

    /// Spin the bridge against a live ROS2 graph.
    ///
    /// With the `rclrs` feature **on**, this builds the live transport
    /// core, spawns the [`Ros2NodeActor`](crate::actors::Ros2NodeActor)
    /// (and its per-endpoint children) onto `sys`, wires the inbound
    /// event pump, and returns a running [`Ros2BridgeHandle`]. With the
    /// feature **off** it returns
    /// [`PhysicalError::Ros2Bridge`] so callers fail fast.
    ///
    /// `wiring` supplies the device seam for every endpoint the plan
    /// binds; `codecs` encodes/decodes messages on the wire.
    ///
    /// [`PhysicalError::Ros2Bridge`]: atomr_physical_core::PhysicalError::Ros2Bridge
    pub async fn run(
        &self,
        sys: &ActorSystem,
        wiring: Ros2Wiring,
        codecs: Arc<CodecRegistry>,
    ) -> Result<Ros2BridgeHandle> {
        #[cfg(not(feature = "rclrs"))]
        {
            let _ = (sys, wiring, codecs);
            Err(Ros2Error::FeatureDisabled(format!(
                "node {:?} ({} endpoints) — `Ros2Bridge::run` needs the `rclrs` feature; \
                 see docs/ros2-bridge.md",
                self.node_name,
                self.plan.len(),
            ))
            .into())
        }

        #[cfg(feature = "rclrs")]
        {
            // L1 — the live transport core.
            let transport = RclrsTransport::new(self.node_name.clone(), self.plan.clone(), codecs);
            let (link, event_rx) = transport.start();

            // L4 — the Model 2 orchestration actors.
            let node_name = self.node_name.clone();
            let plan = self.plan.clone();
            let node_link = link.clone();
            let node = sys
                .actor_of(
                    Props::create(move || {
                        Ros2NodeActor::new(node_name.clone(), plan.clone(), node_link.clone(), wiring.clone())
                    }),
                    "ros2-node",
                )
                .map_err(|err| {
                    Ros2Error::TransportLost(format!("failed to spawn the ros2 node actor: {err}"))
                })?;
            spawn_inbound_pump(event_rx, node.clone());

            Ok(Ros2BridgeHandle { node, link })
        }
    }
}

/// A running ROS2 bridge — the handle [`Ros2Bridge::run`] returns.
///
/// Holds the [`Ros2NodeActor`](crate::actors::Ros2NodeActor) reference
/// (for `tell`-ing it directly, e.g.
/// [`Ros2NodeMsg::TriggerPublish`](crate::actors::Ros2NodeMsg)) and the
/// transport [`Ros2Link`] (for [`shutdown`](Ros2BridgeHandle::shutdown)).
pub struct Ros2BridgeHandle {
    node: ActorRef<Ros2NodeMsg>,
    link: Ros2Link,
}

impl Ros2BridgeHandle {
    /// The node actor — `tell` it a [`Ros2NodeMsg`].
    pub fn node(&self) -> &ActorRef<Ros2NodeMsg> {
        &self.node
    }

    /// Tear down the live ROS2 node: signal the transport task to stop.
    pub fn shutdown(&self) -> Result<()> {
        self.link.send(Ros2Command::Shutdown)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::endpoint::Ros2Endpoint;
    use crate::service::Ros2ServiceEndpoint;
    use atomr_physical_core::SensorId;

    #[tokio::test]
    async fn spin_is_a_fail_fast_shim() {
        let bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("r1"));
        assert!(bridge.spin().await.is_err());
    }

    #[tokio::test]
    async fn run_without_rclrs_feature_errors() {
        let sys = ActorSystem::create("run-test", crate::actor::prelude::Config::empty())
            .await
            .unwrap();
        let bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("r1"));
        let result = bridge
            .run(&sys, Ros2Wiring::new(), Arc::new(CodecRegistry::builtin()))
            .await;
        // With `rclrs` off this is the fail-fast path; with `rclrs` on it
        // builds a live bridge. Either way it must not panic.
        #[cfg(not(feature = "rclrs"))]
        assert!(result.is_err());
        #[cfg(feature = "rclrs")]
        let _ = result;
        sys.terminate().await;
    }

    #[test]
    fn topics_delegate_to_the_plan() {
        let mut bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("r1"));
        bridge.topics_mut().bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::publish("/arm/temp", "sensor_msgs/msg/Temperature"),
        );
        assert_eq!(bridge.topics().len(), 1);
        assert_eq!(bridge.plan().len(), 1);
    }

    #[test]
    fn plan_mut_takes_services_and_actions() {
        let mut bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("r1"));
        bridge
            .plan_mut()
            .add_service(Ros2ServiceEndpoint::server("/arm/home", "std_srvs/srv/Trigger"));
        assert_eq!(bridge.plan().services().len(), 1);
        assert_eq!(bridge.plan().len(), 1);
    }

    #[test]
    fn validate_surfaces_plan_problems() {
        let mut bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("r1"));
        bridge.topics_mut().bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::subscribe("arm/temp", "sensor_msgs/msg/Temperature"),
        );
        assert!(!bridge.validate().is_empty());
    }

    #[test]
    fn validate_is_clean_for_a_well_formed_plan() {
        let mut bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("r1"));
        bridge.topics_mut().bind_sensor(
            SensorId::from("s1"),
            Ros2Endpoint::publish("/arm/temp", "sensor_msgs/msg/Temperature"),
        );
        assert!(bridge.validate().is_empty());
    }
}
