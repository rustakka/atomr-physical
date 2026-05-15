//! Robot-level orchestration for atomr-physical.
//!
//! A [`RobotActor`] is the supervisor at the top of a physical system:
//! it owns a set of [`SensorActor`]s and [`ActuatorActor`]s, exposes the
//! robot's kinematic structure as a [`RobotModel`], and (since 0.2.0)
//! runs as a true atomr supervisor — each child sensor / actuator is
//! spawned as a supervised child, so a driver fault restarts only the
//! affected subtree.
//!
//! Two ways to use a [`RobotActor`]:
//!
//! 1. **Offline** — construct it, add sensors / actuators with
//!    [`add_sensor`](RobotActor::add_sensor) /
//!    [`add_actuator`](RobotActor::add_actuator), and inspect
//!    [`sensor`](RobotActor::sensor) /
//!    [`actuator`](RobotActor::actuator). No runtime.
//! 2. **Supervised** — call [`RobotActor::spawn`] to promote it to a
//!    live supervisor under an [`ActorSystem`]. The returned
//!    [`RobotActorRef`] exposes typed handles to every child.
//!
//! The atomr actor runtime is re-exported as [`actor`] so downstream
//! crates have a single import path for it.

use std::collections::HashMap;
use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_core::supervision::{Directive, OneForOneStrategy, SupervisorStrategy};
use atomr_physical_actuation::{ActuatorActor, ActuatorActorRef};
use atomr_physical_core::{ActuatorId, JointId, PhysicalError, Result, RobotId, SensorId};
use atomr_physical_sensing::{SensorActor, SensorActorRef};
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;

/// Re-export of the atomr actor runtime this crate builds on.
pub use atomr_core as actor;

/// One articulated joint in a robot — the pairing of an actuator that
/// drives it with the sensor that reports its state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    /// Stable joint identifier.
    pub id: JointId,
    /// Human-readable joint name, e.g. `"shoulder_pan"`.
    pub name: String,
    /// The actuator that drives this joint.
    pub actuator: ActuatorId,
    /// The sensor that reports this joint's state, if instrumented.
    pub feedback: Option<SensorId>,
}

impl Joint {
    /// Construct a joint with a driving actuator and no feedback sensor.
    pub fn new(id: JointId, name: impl Into<String>, actuator: ActuatorId) -> Self {
        Self {
            id,
            name: name.into(),
            actuator,
            feedback: None,
        }
    }

    /// Builder-style: attach a feedback sensor.
    pub fn with_feedback(mut self, sensor: SensorId) -> Self {
        self.feedback = Some(sensor);
        self
    }
}

/// The static kinematic description of a robot — its joints, plus the
/// sensors not bound to a joint (chassis IMU, battery monitor, …).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RobotModel {
    /// The robot's articulated joints, in declaration order.
    pub joints: Vec<Joint>,
    /// Sensors that report robot-level state rather than a joint's.
    pub auxiliary_sensors: Vec<SensorId>,
}

impl RobotModel {
    /// An empty model.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style: add a joint.
    pub fn with_joint(mut self, joint: Joint) -> Self {
        self.joints.push(joint);
        self
    }

    /// Builder-style: add an auxiliary (non-joint) sensor.
    pub fn with_auxiliary_sensor(mut self, sensor: SensorId) -> Self {
        self.auxiliary_sensors.push(sensor);
        self
    }

    /// Look a joint up by id.
    pub fn joint(&self, id: &JointId) -> Option<&Joint> {
        self.joints.iter().find(|j| &j.id == id)
    }
}

/// The supervisor at the top of a physical system.
///
/// Construct one with [`RobotActor::new`], register sensors and
/// actuators with [`add_sensor`](Self::add_sensor) /
/// [`add_actuator`](Self::add_actuator), and call [`spawn`](Self::spawn)
/// to promote it to a live atomr supervisor. The supervisor's
/// [`SupervisorStrategy`] defaults to one-for-one restart on failure —
/// a driver fault inside one child does not bring down its siblings.
#[derive(Clone)]
pub struct RobotActor {
    id: RobotId,
    model: RobotModel,
    sensors: HashMap<SensorId, SensorActor>,
    actuators: HashMap<ActuatorId, ActuatorActor>,
    supervisor_strategy: SupervisorStrategy,
}

impl RobotActor {
    /// Construct a robot supervisor from a kinematic model.
    pub fn new(id: RobotId, model: RobotModel) -> Self {
        Self {
            id,
            model,
            sensors: HashMap::new(),
            actuators: HashMap::new(),
            supervisor_strategy: OneForOneStrategy::default().into(),
        }
    }

    /// This robot's identifier.
    pub fn id(&self) -> &RobotId {
        &self.id
    }

    /// This robot's kinematic model.
    pub fn model(&self) -> &RobotModel {
        &self.model
    }

    /// Register a sensor actor as a supervised child.
    pub fn add_sensor(&mut self, sensor: SensorActor) {
        self.sensors.insert(sensor.id(), sensor);
    }

    /// Register an actuator actor as a supervised child.
    pub fn add_actuator(&mut self, actuator: ActuatorActor) {
        self.actuators.insert(actuator.id(), actuator);
    }

    /// Borrow a registered sensor actor by id (offline form — see
    /// [`RobotActorRef::sensor`] for the spawned form).
    pub fn sensor(&self, id: &SensorId) -> Option<&SensorActor> {
        self.sensors.get(id)
    }

    /// Borrow a registered actuator actor by id (offline form — see
    /// [`RobotActorRef::actuator`] for the spawned form).
    pub fn actuator(&self, id: &ActuatorId) -> Option<&ActuatorActor> {
        self.actuators.get(id)
    }

    /// Number of supervised child actors (sensors + actuators).
    pub fn child_count(&self) -> usize {
        self.sensors.len() + self.actuators.len()
    }

    /// Customise the supervisor strategy applied to children. Defaults
    /// to one-for-one restart with 10 retries / 60 s.
    pub fn with_supervisor_strategy(mut self, strategy: SupervisorStrategy) -> Self {
        self.supervisor_strategy = strategy;
        self
    }

    /// Promote this robot into a supervised atomr actor under `system`,
    /// registered at `name`. Returns a [`RobotActorRef`] — a typed
    /// handle that can look up each child's [`SensorActorRef`] /
    /// [`ActuatorActorRef`].
    ///
    /// On spawn, the supervisor's `pre_start` spawns every registered
    /// sensor / actuator as a true atomr child. Children are named
    /// `sensor-<id>` and `actuator-<id>` inside the supervisor's path.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<RobotActorRef, ActorSystemError> {
        let RobotActor {
            id,
            model,
            sensors,
            actuators,
            supervisor_strategy,
        } = self;
        let robot_id_for_factory = id.clone();
        let strategy_for_factory = supervisor_strategy.clone();
        let sensors_factory = sensors.clone();
        let actuators_factory = actuators.clone();
        let props = Props::create(move || RobotRunner {
            id: robot_id_for_factory.clone(),
            sensor_specs: sensors_factory.clone(),
            actuator_specs: actuators_factory.clone(),
            sensor_refs: HashMap::new(),
            actuator_refs: HashMap::new(),
            strategy: strategy_for_factory.clone(),
        })
        .with_supervisor_strategy(supervisor_strategy);
        let actor_ref = system.actor_of(props, name)?;
        Ok(RobotActorRef {
            inner: actor_ref,
            id,
            model,
        })
    }
}

/// The mailbox protocol of a live [`RobotActor`].
///
/// Construct messages through [`RobotActorRef`] rather than reaching
/// for the variants directly; the helpers wrap the oneshot replies and
/// the ask timeout.
pub enum RobotMsg {
    /// Resolve a sensor child's typed handle.
    LookupSensor {
        /// The sensor id to look up.
        id: SensorId,
        /// One-shot reply channel.
        reply: oneshot::Sender<Option<SensorActorRef>>,
    },
    /// Resolve an actuator child's typed handle.
    LookupActuator {
        /// The actuator id to look up.
        id: ActuatorId,
        /// One-shot reply channel.
        reply: oneshot::Sender<Option<ActuatorActorRef>>,
    },
    /// Snapshot of every supervised child id (sensors and actuators).
    ChildIds {
        /// One-shot reply channel: `(sensor_ids, actuator_ids)`.
        reply: oneshot::Sender<(Vec<SensorId>, Vec<ActuatorId>)>,
    },
}

/// A typed handle to a spawned [`RobotActor`].
///
/// Look up children with [`sensor`](Self::sensor) /
/// [`actuator`](Self::actuator); inspect the live child id set with
/// [`child_ids`](Self::child_ids).
#[derive(Clone)]
pub struct RobotActorRef {
    inner: ActorRef<RobotMsg>,
    id: RobotId,
    model: RobotModel,
}

impl RobotActorRef {
    /// This robot's identifier.
    pub fn id(&self) -> &RobotId {
        &self.id
    }

    /// This robot's kinematic model.
    pub fn model(&self) -> &RobotModel {
        &self.model
    }

    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<RobotMsg> {
        &self.inner
    }

    /// Look up a supervised sensor's typed handle.
    pub async fn sensor(&self, id: &SensorId) -> Result<Option<SensorActorRef>> {
        let id = id.clone();
        self.inner
            .ask_with(|reply| RobotMsg::LookupSensor { id, reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)
    }

    /// Look up a supervised actuator's typed handle.
    pub async fn actuator(&self, id: &ActuatorId) -> Result<Option<ActuatorActorRef>> {
        let id = id.clone();
        self.inner
            .ask_with(|reply| RobotMsg::LookupActuator { id, reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)
    }

    /// Snapshot of every supervised child id.
    pub async fn child_ids(&self) -> Result<(Vec<SensorId>, Vec<ActuatorId>)> {
        self.inner
            .ask_with(|reply| RobotMsg::ChildIds { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)
    }
}

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("robot actor ask failed: {e:?}"))
}

/// Internal supervisor implementation backing a spawned [`RobotActor`].
struct RobotRunner {
    id: RobotId,
    sensor_specs: HashMap<SensorId, SensorActor>,
    actuator_specs: HashMap<ActuatorId, ActuatorActor>,
    sensor_refs: HashMap<SensorId, SensorActorRef>,
    actuator_refs: HashMap<ActuatorId, ActuatorActorRef>,
    strategy: SupervisorStrategy,
}

#[async_trait]
impl Actor for RobotRunner {
    type Msg = RobotMsg;

    fn supervisor_strategy(&self) -> SupervisorStrategy {
        self.strategy.clone()
    }

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        // Sensors first, then actuators — order is stable across restarts
        // because the spec HashMaps are populated in the same order from
        // the original `add_sensor`/`add_actuator` calls.
        let specs: Vec<(SensorId, SensorActor)> = self
            .sensor_specs
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        for (id, spec) in specs {
            let name = format!("sensor-{}", id.as_str());
            match spec.spawn_under(ctx, &name) {
                Ok(sref) => {
                    self.sensor_refs.insert(id, sref);
                }
                Err(e) => {
                    tracing::error!(
                        robot = %self.id,
                        sensor = %id,
                        error = %e,
                        "failed to spawn sensor child"
                    );
                }
            }
        }
        let act_specs: Vec<(ActuatorId, ActuatorActor)> = self
            .actuator_specs
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        for (id, spec) in act_specs {
            let name = format!("actuator-{}", id.as_str());
            match spec.spawn_under(ctx, &name) {
                Ok(aref) => {
                    self.actuator_refs.insert(id, aref);
                }
                Err(e) => {
                    tracing::error!(
                        robot = %self.id,
                        actuator = %id,
                        error = %e,
                        "failed to spawn actuator child"
                    );
                }
            }
        }
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: RobotMsg) {
        match msg {
            RobotMsg::LookupSensor { id, reply } => {
                let _ = reply.send(self.sensor_refs.get(&id).cloned());
            }
            RobotMsg::LookupActuator { id, reply } => {
                let _ = reply.send(self.actuator_refs.get(&id).cloned());
            }
            RobotMsg::ChildIds { reply } => {
                let mut sensors: Vec<SensorId> = self.sensor_refs.keys().cloned().collect();
                let mut actuators: Vec<ActuatorId> = self.actuator_refs.keys().cloned().collect();
                sensors.sort();
                actuators.sort();
                let _ = reply.send((sensors, actuators));
            }
        }
    }
}

// Touch the `Directive` import path so the dependency is exercised in
// every build configuration — kept for completeness while the default
// `OneForOneStrategy` decider already returns `Directive::Restart`.
#[allow(dead_code)]
const _DEFAULT_DIRECTIVE: Directive = Directive::Restart;

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    use atomr_core::actor::ActorSystem;
    use atomr_physical_actuation::ActuatorActor;
    use atomr_physical_sensing::{SamplingPolicy, SensorActor};
    use atomr_physical_testkit::{MockActuator, MockSensor};

    #[test]
    fn robot_model_resolves_joints() {
        let joint = Joint::new(JointId::from("j1"), "shoulder_pan", ActuatorId::from("a1"));
        let model = RobotModel::new().with_joint(joint);
        assert!(model.joint(&JointId::from("j1")).is_some());
        assert!(model.joint(&JointId::from("absent")).is_none());
    }

    #[test]
    fn robot_actor_tracks_children() {
        let mut robot = RobotActor::new(RobotId::from("r1"), RobotModel::new());
        let sensor = SensorActor::new(
            Arc::new(MockSensor::constant("s1", 1.0, atomr_physical_core::Unit::Scalar)),
            SamplingPolicy::OnDemand,
        );
        let actuator = ActuatorActor::new(Arc::new(MockActuator::new("a1")));
        robot.add_sensor(sensor);
        robot.add_actuator(actuator);
        assert_eq!(robot.child_count(), 2);
        assert!(robot.sensor(&SensorId::from("s1")).is_some());
        assert!(robot.actuator(&ActuatorId::from("a1")).is_some());
    }

    #[tokio::test]
    async fn spawned_robot_supervises_children() {
        let sys = ActorSystem::create("robotics-spawn", atomr_config::Config::reference())
            .await
            .unwrap();
        let mut robot = RobotActor::new(RobotId::from("r1"), RobotModel::new());
        robot.add_sensor(SensorActor::new(
            Arc::new(MockSensor::constant("s1", 7.0, atomr_physical_core::Unit::Scalar)),
            SamplingPolicy::OnDemand,
        ));
        robot.add_actuator(ActuatorActor::new(Arc::new(MockActuator::new("a1"))));
        let robot_ref = robot.spawn(&sys, "r1").unwrap();

        // Children should be supervised under the robot — we can resolve
        // them via the typed mailbox and exercise them.
        let sensor_ref = robot_ref.sensor(&SensorId::from("s1")).await.unwrap().unwrap();
        let reading = sensor_ref.sample().await.unwrap();
        assert_eq!(reading.quantity.value, 7.0);

        let actuator_ref = robot_ref
            .actuator(&ActuatorId::from("a1"))
            .await
            .unwrap()
            .unwrap();
        let cmd = atomr_physical_core::Command::now(
            ActuatorId::from("a1"),
            atomr_physical_core::ControlMode::Duty,
            atomr_physical_core::Quantity::new(0.5, atomr_physical_core::Unit::Percent),
        );
        let ack = actuator_ref.dispatch(cmd).await.unwrap();
        assert!(ack.accepted);

        let (sensor_ids, actuator_ids) = robot_ref.child_ids().await.unwrap();
        assert_eq!(sensor_ids, vec![SensorId::from("s1")]);
        assert_eq!(actuator_ids, vec![ActuatorId::from("a1")]);

        sys.terminate().await;
    }

    #[tokio::test]
    async fn spawned_robot_resolves_missing_child_to_none() {
        let sys = ActorSystem::create("robotics-missing", atomr_config::Config::reference())
            .await
            .unwrap();
        let robot = RobotActor::new(RobotId::from("r2"), RobotModel::new());
        let robot_ref = robot.spawn(&sys, "r2").unwrap();
        assert!(robot_ref
            .sensor(&SensorId::from("ghost"))
            .await
            .unwrap()
            .is_none());
        assert!(robot_ref
            .actuator(&ActuatorId::from("ghost"))
            .await
            .unwrap()
            .is_none());
        sys.terminate().await;
    }
}
