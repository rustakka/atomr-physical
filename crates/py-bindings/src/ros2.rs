//! ROS2 bridge bindings: `Ros2Endpoint` and `TopicMap`.

use atomr_physical_core::{ActuatorId, SensorId};
use atomr_physical_ros2::{Ros2Direction, Ros2Endpoint, TopicMap};
use pyo3::prelude::*;

fn direction_name(d: Ros2Direction) -> &'static str {
    match d {
        Ros2Direction::Publish => "publish",
        Ros2Direction::Subscribe => "subscribe",
    }
}

/// A single ROS2 endpoint bound to an atomr-physical device.
#[pyclass(name = "Ros2Endpoint", module = "atomr_physical._native.ros2")]
#[derive(Clone)]
pub struct PyRos2Endpoint {
    pub(crate) inner: Ros2Endpoint,
}

#[pymethods]
impl PyRos2Endpoint {
    /// A publishing endpoint (sensor readings flow out to ROS2).
    #[staticmethod]
    fn publish(topic: String, message_type: String) -> Self {
        Self {
            inner: Ros2Endpoint::publish(topic, message_type),
        }
    }

    /// A subscribing endpoint (commands flow in from ROS2).
    #[staticmethod]
    fn subscribe(topic: String, message_type: String) -> Self {
        Self {
            inner: Ros2Endpoint::subscribe(topic, message_type),
        }
    }

    #[getter]
    fn topic(&self) -> String {
        self.inner.topic.clone()
    }

    #[getter]
    fn message_type(&self) -> String {
        self.inner.message_type.clone()
    }

    #[getter]
    fn direction(&self) -> &'static str {
        direction_name(self.inner.direction)
    }

    fn __repr__(&self) -> String {
        format!(
            "Ros2Endpoint(topic={:?}, message_type={:?}, direction={:?})",
            self.inner.topic,
            self.inner.message_type,
            direction_name(self.inner.direction),
        )
    }
}

/// The topic-graph plan for one robot.
#[pyclass(name = "TopicMap", module = "atomr_physical._native.ros2")]
#[derive(Clone, Default)]
pub struct PyTopicMap {
    pub(crate) inner: TopicMap,
}

#[pymethods]
impl PyTopicMap {
    #[new]
    fn new() -> Self {
        Self::default()
    }

    /// Bind a sensor's reading stream to a published topic.
    fn bind_sensor(&mut self, sensor: String, endpoint: PyRos2Endpoint) {
        self.inner.bind_sensor(SensorId::from(sensor), endpoint.inner);
    }

    /// Bind an actuator's command mailbox to a subscribed topic.
    fn bind_actuator(&mut self, actuator: String, endpoint: PyRos2Endpoint) {
        self.inner
            .bind_actuator(ActuatorId::from(actuator), endpoint.inner);
    }

    /// The endpoint a sensor publishes to, if bound.
    fn sensor_endpoint(&self, sensor: String) -> Option<PyRos2Endpoint> {
        self.inner
            .sensor_endpoint(&SensorId::from(sensor))
            .cloned()
            .map(|inner| PyRos2Endpoint { inner })
    }

    /// The endpoint an actuator subscribes from, if bound.
    fn actuator_endpoint(&self, actuator: String) -> Option<PyRos2Endpoint> {
        self.inner
            .actuator_endpoint(&ActuatorId::from(actuator))
            .cloned()
            .map(|inner| PyRos2Endpoint { inner })
    }

    /// Total number of bound endpoints.
    #[getter]
    fn len(&self) -> usize {
        self.inner.len()
    }

    fn __repr__(&self) -> String {
        format!("TopicMap(endpoints={})", self.inner.len())
    }
}

/// Register the `ros2` submodule.
pub fn register(py: Python<'_>, parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(py, "ros2")?;
    m.add_class::<PyRos2Endpoint>()?;
    m.add_class::<PyTopicMap>()?;
    parent.add_submodule(&m)?;
    Ok(())
}
