//! Robot-level orchestration for atomr-physical.
//!
//! A [`RobotActor`] is the supervisor at the top of a physical system:
//! it owns a set of [`SensorActor`]s and [`ActuatorActor`]s, exposes the
//! robot's kinematic structure as a [`RobotModel`], and (Phase 2) runs
//! the sense → decide → actuate control loop under supervision.
//!
//! The atomr actor runtime is re-exported as [`actor`] so downstream
//! crates have a single import path for it.

use std::collections::HashMap;

use atomr_physical_actuation::ActuatorActor;
use atomr_physical_core::{ActuatorId, JointId, RobotId, SensorId};
use atomr_physical_sensing::SensorActor;
use serde::{Deserialize, Serialize};

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
/// **Phase 2** wires `RobotActor` into [`actor`]'s `Actor` and
/// supervision API so each child sensor / actuator runs as a supervised
/// actor and a driver fault restarts only the affected subtree. The
/// current form owns the child actors keyed by id and the robot's
/// kinematic model, giving downstream code a single handle to the
/// physical system ahead of the supervision wiring.
pub struct RobotActor {
    id: RobotId,
    model: RobotModel,
    sensors: HashMap<SensorId, SensorActor>,
    actuators: HashMap<ActuatorId, ActuatorActor>,
}

impl RobotActor {
    /// Construct a robot supervisor from a kinematic model.
    pub fn new(id: RobotId, model: RobotModel) -> Self {
        Self {
            id,
            model,
            sensors: HashMap::new(),
            actuators: HashMap::new(),
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

    /// Borrow a registered sensor actor by id.
    pub fn sensor(&self, id: &SensorId) -> Option<&SensorActor> {
        self.sensors.get(id)
    }

    /// Borrow a registered actuator actor by id.
    pub fn actuator(&self, id: &ActuatorId) -> Option<&ActuatorActor> {
        self.actuators.get(id)
    }

    /// Number of supervised child actors (sensors + actuators).
    pub fn child_count(&self) -> usize {
        self.sensors.len() + self.actuators.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

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
}
