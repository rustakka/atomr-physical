//! Robot-level bindings: `Joint` and `RobotModel`.

use atomr_physical_core::{ActuatorId, JointId, SensorId};
use atomr_physical_robotics::{Joint, RobotModel};
use pyo3::prelude::*;

/// One articulated joint — an actuator that drives it, optionally paired
/// with a feedback sensor.
#[pyclass(name = "Joint", module = "atomr_physical._native.robotics")]
#[derive(Clone)]
pub struct PyJoint {
    pub(crate) inner: Joint,
}

#[pymethods]
impl PyJoint {
    #[new]
    #[pyo3(signature = (id, name, actuator, feedback=None))]
    fn new(id: String, name: String, actuator: String, feedback: Option<String>) -> Self {
        let mut joint = Joint::new(JointId::from(id), name, ActuatorId::from(actuator));
        joint.feedback = feedback.map(SensorId::from);
        Self { inner: joint }
    }

    #[getter]
    fn id(&self) -> String {
        self.inner.id.to_string()
    }

    #[getter]
    fn name(&self) -> String {
        self.inner.name.clone()
    }

    #[getter]
    fn actuator(&self) -> String {
        self.inner.actuator.to_string()
    }

    #[getter]
    fn feedback(&self) -> Option<String> {
        self.inner.feedback.as_ref().map(|s| s.to_string())
    }

    fn __repr__(&self) -> String {
        format!(
            "Joint(id={:?}, name={:?}, actuator={:?}, feedback={:?})",
            self.inner.id.as_str(),
            self.inner.name,
            self.inner.actuator.as_str(),
            self.inner.feedback.as_ref().map(|s| s.as_str()),
        )
    }
}

/// The static kinematic description of a robot.
#[pyclass(name = "RobotModel", module = "atomr_physical._native.robotics")]
#[derive(Clone, Default)]
pub struct PyRobotModel {
    pub(crate) inner: RobotModel,
}

#[pymethods]
impl PyRobotModel {
    #[new]
    fn new() -> Self {
        Self::default()
    }

    /// Add a joint to the model.
    fn add_joint(&mut self, joint: PyJoint) {
        self.inner.joints.push(joint.inner);
    }

    /// Add an auxiliary (non-joint) sensor to the model.
    fn add_auxiliary_sensor(&mut self, sensor: String) {
        self.inner.auxiliary_sensors.push(SensorId::from(sensor));
    }

    /// The joint ids declared in this model.
    #[getter]
    fn joint_ids(&self) -> Vec<String> {
        self.inner.joints.iter().map(|j| j.id.to_string()).collect()
    }

    /// The auxiliary sensor ids declared in this model.
    #[getter]
    fn auxiliary_sensor_ids(&self) -> Vec<String> {
        self.inner
            .auxiliary_sensors
            .iter()
            .map(|s| s.to_string())
            .collect()
    }

    /// Number of joints in the model.
    #[getter]
    fn joint_count(&self) -> usize {
        self.inner.joints.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "RobotModel(joints={}, auxiliary_sensors={})",
            self.inner.joints.len(),
            self.inner.auxiliary_sensors.len(),
        )
    }
}

/// Register the `robotics` submodule.
pub fn register(py: Python<'_>, parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(py, "robotics")?;
    m.add_class::<PyJoint>()?;
    m.add_class::<PyRobotModel>()?;
    parent.add_submodule(&m)?;
    Ok(())
}
