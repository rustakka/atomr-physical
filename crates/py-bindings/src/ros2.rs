//! ROS2 bridge bindings — the offline planning surface.
//!
//! The full offline plan (`Ros2Endpoint`, `TopicMap`, `Ros2Plan`, the
//! service / action / parameter endpoint types, `QosProfile`, and a
//! read-only view of the `CodecRegistry`) is exposed here. The live
//! transport — `Ros2Bridge::spin`, the codecs, the orchestration actors
//! — stays Rust-only.

use atomr_physical_core::{ActuatorId, SensorId};
use atomr_physical_ros2::{
    ActionRole, CodecRegistry, Durability, History, ParamValue, QosProfile, Reliability, Ros2ActionEndpoint,
    Ros2ClockSource, Ros2Direction, Ros2Endpoint, Ros2ParamDecl, Ros2Plan, Ros2ServiceEndpoint, ServiceRole,
    TopicMap,
};
use pyo3::prelude::*;

fn direction_name(d: Ros2Direction) -> &'static str {
    match d {
        Ros2Direction::Publish => "publish",
        Ros2Direction::Subscribe => "subscribe",
    }
}

fn reliability_name(r: Reliability) -> &'static str {
    match r {
        Reliability::Reliable => "reliable",
        Reliability::BestEffort => "best_effort",
    }
}

fn durability_name(d: Durability) -> &'static str {
    match d {
        Durability::Volatile => "volatile",
        Durability::TransientLocal => "transient_local",
    }
}

fn history_name(h: History) -> &'static str {
    match h {
        History::KeepLast => "keep_last",
        History::KeepAll => "keep_all",
    }
}

fn service_role_name(r: ServiceRole) -> &'static str {
    match r {
        ServiceRole::Server => "server",
        ServiceRole::Client => "client",
    }
}

fn action_role_name(r: ActionRole) -> &'static str {
    match r {
        ActionRole::Server => "server",
        ActionRole::Client => "client",
    }
}

fn clock_source_name(c: Ros2ClockSource) -> &'static str {
    match c {
        Ros2ClockSource::Wall => "wall",
        Ros2ClockSource::RosTime => "ros_time",
        Ros2ClockSource::SimTime => "sim_time",
    }
}

/// A ROS2 Quality-of-Service profile.
#[pyclass(name = "QosProfile", module = "atomr_physical._native.ros2")]
#[derive(Clone, Copy)]
pub struct PyQosProfile {
    pub(crate) inner: QosProfile,
}

#[pymethods]
impl PyQosProfile {
    /// The sensor-data profile: best-effort, volatile, keep-last(5).
    #[staticmethod]
    fn sensor_data() -> Self {
        Self {
            inner: QosProfile::sensor_data(),
        }
    }

    /// The command profile: reliable, volatile, keep-last(10).
    #[staticmethod]
    fn command() -> Self {
        Self {
            inner: QosProfile::command(),
        }
    }

    #[getter]
    fn reliability(&self) -> &'static str {
        reliability_name(self.inner.reliability)
    }

    #[getter]
    fn durability(&self) -> &'static str {
        durability_name(self.inner.durability)
    }

    #[getter]
    fn history(&self) -> &'static str {
        history_name(self.inner.history)
    }

    #[getter]
    fn depth(&self) -> u32 {
        self.inner.depth
    }

    fn __repr__(&self) -> String {
        format!(
            "QosProfile(reliability={:?}, durability={:?}, history={:?}, depth={})",
            self.reliability(),
            self.durability(),
            self.history(),
            self.inner.depth,
        )
    }
}

/// The clock source the bridge stamps outbound messages from.
#[pyclass(name = "Ros2ClockSource", module = "atomr_physical._native.ros2")]
#[derive(Clone, Copy)]
pub struct PyRos2ClockSource {
    pub(crate) inner: Ros2ClockSource,
}

#[pymethods]
impl PyRos2ClockSource {
    /// The host wall clock — the default.
    #[staticmethod]
    fn wall() -> Self {
        Self {
            inner: Ros2ClockSource::Wall,
        }
    }

    /// The ROS2 graph's time, published on `/clock`.
    #[staticmethod]
    fn ros_time() -> Self {
        Self {
            inner: Ros2ClockSource::RosTime,
        }
    }

    /// Simulation time, published on `/clock` by a simulator.
    #[staticmethod]
    fn sim_time() -> Self {
        Self {
            inner: Ros2ClockSource::SimTime,
        }
    }

    #[getter]
    fn name(&self) -> &'static str {
        clock_source_name(self.inner)
    }

    fn __repr__(&self) -> String {
        format!("Ros2ClockSource({:?})", self.name())
    }
}

/// A single ROS2 topic endpoint bound to an atomr-physical device.
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

    /// Return a copy of this endpoint with an explicit QoS profile.
    fn with_qos(&self, qos: PyQosProfile) -> Self {
        Self {
            inner: self.inner.clone().with_qos(qos.inner),
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

    /// The explicit QoS profile, or `None` if the endpoint takes the
    /// per-direction default.
    #[getter]
    fn qos(&self) -> Option<PyQosProfile> {
        self.inner.qos.map(|inner| PyQosProfile { inner })
    }

    /// The QoS profile this endpoint resolves to — its explicit profile,
    /// or the per-direction default.
    #[getter]
    fn effective_qos(&self) -> PyQosProfile {
        PyQosProfile {
            inner: self.inner.effective_qos(),
        }
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

/// A ROS2 service endpoint bound to an atomr-physical handler.
#[pyclass(name = "Ros2ServiceEndpoint", module = "atomr_physical._native.ros2")]
#[derive(Clone)]
pub struct PyRos2ServiceEndpoint {
    pub(crate) inner: Ros2ServiceEndpoint,
}

#[pymethods]
impl PyRos2ServiceEndpoint {
    /// A service atomr-physical serves — external clients call in.
    #[staticmethod]
    fn server(service: String, service_type: String) -> Self {
        Self {
            inner: Ros2ServiceEndpoint::server(service, service_type),
        }
    }

    /// A service atomr-physical calls — it is an external server.
    #[staticmethod]
    fn client(service: String, service_type: String) -> Self {
        Self {
            inner: Ros2ServiceEndpoint::client(service, service_type),
        }
    }

    #[getter]
    fn service(&self) -> String {
        self.inner.service.clone()
    }

    #[getter]
    fn service_type(&self) -> String {
        self.inner.service_type.clone()
    }

    #[getter]
    fn role(&self) -> &'static str {
        service_role_name(self.inner.role)
    }

    fn __repr__(&self) -> String {
        format!(
            "Ros2ServiceEndpoint(service={:?}, service_type={:?}, role={:?})",
            self.inner.service,
            self.inner.service_type,
            service_role_name(self.inner.role),
        )
    }
}

/// A ROS2 action endpoint bound to an atomr-physical handler.
#[pyclass(name = "Ros2ActionEndpoint", module = "atomr_physical._native.ros2")]
#[derive(Clone)]
pub struct PyRos2ActionEndpoint {
    pub(crate) inner: Ros2ActionEndpoint,
}

#[pymethods]
impl PyRos2ActionEndpoint {
    /// An action atomr-physical serves — external clients send goals.
    #[staticmethod]
    fn server(action: String, action_type: String) -> Self {
        Self {
            inner: Ros2ActionEndpoint::server(action, action_type),
        }
    }

    /// An action atomr-physical calls — it is an external server.
    #[staticmethod]
    fn client(action: String, action_type: String) -> Self {
        Self {
            inner: Ros2ActionEndpoint::client(action, action_type),
        }
    }

    #[getter]
    fn action(&self) -> String {
        self.inner.action.clone()
    }

    #[getter]
    fn action_type(&self) -> String {
        self.inner.action_type.clone()
    }

    #[getter]
    fn role(&self) -> &'static str {
        action_role_name(self.inner.role)
    }

    fn __repr__(&self) -> String {
        format!(
            "Ros2ActionEndpoint(action={:?}, action_type={:?}, role={:?})",
            self.inner.action,
            self.inner.action_type,
            action_role_name(self.inner.role),
        )
    }
}

/// A ROS2 parameter declaration the bridge mirrors.
#[pyclass(name = "Ros2ParamDecl", module = "atomr_physical._native.ros2")]
#[derive(Clone)]
pub struct PyRos2ParamDecl {
    pub(crate) inner: Ros2ParamDecl,
}

#[pymethods]
impl PyRos2ParamDecl {
    /// Declare a boolean parameter with a default value.
    #[staticmethod]
    fn bool_param(name: String, default: bool) -> Self {
        Self {
            inner: Ros2ParamDecl::new(name, ParamValue::Bool(default)),
        }
    }

    /// Declare an integer parameter with a default value.
    #[staticmethod]
    fn int_param(name: String, default: i64) -> Self {
        Self {
            inner: Ros2ParamDecl::new(name, ParamValue::Int(default)),
        }
    }

    /// Declare a double parameter with a default value.
    #[staticmethod]
    fn double_param(name: String, default: f64) -> Self {
        Self {
            inner: Ros2ParamDecl::new(name, ParamValue::Double(default)),
        }
    }

    /// Declare a string parameter with a default value.
    #[staticmethod]
    fn str_param(name: String, default: String) -> Self {
        Self {
            inner: Ros2ParamDecl::new(name, ParamValue::Str(default)),
        }
    }

    /// Return a copy of this declaration with a description attached.
    fn with_description(&self, description: String) -> Self {
        Self {
            inner: self.inner.clone().with_description(description),
        }
    }

    /// Return a copy of this declaration marked read-only.
    fn read_only(&self) -> Self {
        Self {
            inner: self.inner.clone().read_only(),
        }
    }

    #[getter]
    fn name(&self) -> String {
        self.inner.name.clone()
    }

    #[getter]
    fn description(&self) -> String {
        self.inner.description.clone()
    }

    #[getter]
    fn is_read_only(&self) -> bool {
        self.inner.read_only
    }

    #[getter]
    fn param_type(&self) -> String {
        format!("{:?}", self.inner.param_type())
    }

    fn __repr__(&self) -> String {
        format!(
            "Ros2ParamDecl(name={:?}, param_type={:?}, read_only={})",
            self.inner.name,
            self.param_type(),
            self.inner.read_only,
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

/// The complete ROS2 plan for one robot — topics, services, actions,
/// and parameters.
#[pyclass(name = "Ros2Plan", module = "atomr_physical._native.ros2")]
#[derive(Clone, Default)]
pub struct PyRos2Plan {
    pub(crate) inner: Ros2Plan,
}

#[pymethods]
impl PyRos2Plan {
    #[new]
    fn new() -> Self {
        Self::default()
    }

    /// Bind a sensor's reading stream to a published topic.
    fn bind_sensor(&mut self, sensor: String, endpoint: PyRos2Endpoint) {
        self.inner
            .topics_mut()
            .bind_sensor(SensorId::from(sensor), endpoint.inner);
    }

    /// Bind an actuator's command mailbox to a subscribed topic.
    fn bind_actuator(&mut self, actuator: String, endpoint: PyRos2Endpoint) {
        self.inner
            .topics_mut()
            .bind_actuator(ActuatorId::from(actuator), endpoint.inner);
    }

    /// Add a service endpoint to the plan.
    fn add_service(&mut self, endpoint: PyRos2ServiceEndpoint) {
        self.inner.add_service(endpoint.inner);
    }

    /// Add an action endpoint to the plan.
    fn add_action(&mut self, endpoint: PyRos2ActionEndpoint) {
        self.inner.add_action(endpoint.inner);
    }

    /// Declare a parameter the bridge mirrors.
    fn declare_param(&mut self, decl: PyRos2ParamDecl) {
        self.inner.declare_param(decl.inner);
    }

    /// Total number of bound endpoints across every kind.
    #[getter]
    fn len(&self) -> usize {
        self.inner.len()
    }

    /// Whether the plan binds nothing at all.
    #[getter]
    fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Lint the plan, returning a list of human-readable problems. An
    /// empty list means the plan is well-formed.
    fn validate(&self) -> Vec<String> {
        atomr_physical_ros2::validate_plan(&self.inner)
            .into_iter()
            .map(|err| format!("{}: {:?}", err.endpoint, err.issue))
            .collect()
    }

    fn __repr__(&self) -> String {
        format!("Ros2Plan(endpoints={})", self.inner.len())
    }
}

/// A read-only view of the message-codec registry.
#[pyclass(name = "CodecRegistry", module = "atomr_physical._native.ros2")]
pub struct PyCodecRegistry {
    pub(crate) inner: CodecRegistry,
}

#[pymethods]
impl PyCodecRegistry {
    /// The registry pre-populated with the curated builtin codecs —
    /// empty unless the native extension was built with the `rclrs`
    /// feature.
    #[staticmethod]
    fn builtin() -> Self {
        Self {
            inner: CodecRegistry::builtin(),
        }
    }

    /// The message types this registry can encode / decode.
    fn registered_types(&self) -> Vec<String> {
        self.inner.registered_types().map(str::to_string).collect()
    }

    /// Whether a codec is registered for `message_type`.
    fn has(&self, message_type: String) -> bool {
        self.inner.contains(&message_type)
    }

    #[getter]
    fn len(&self) -> usize {
        self.inner.len()
    }

    fn __repr__(&self) -> String {
        format!("CodecRegistry(codecs={})", self.inner.len())
    }
}

/// Register the `ros2` submodule.
pub fn register(py: Python<'_>, parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(py, "ros2")?;
    m.add_class::<PyQosProfile>()?;
    m.add_class::<PyRos2ClockSource>()?;
    m.add_class::<PyRos2Endpoint>()?;
    m.add_class::<PyRos2ServiceEndpoint>()?;
    m.add_class::<PyRos2ActionEndpoint>()?;
    m.add_class::<PyRos2ParamDecl>()?;
    m.add_class::<PyTopicMap>()?;
    m.add_class::<PyRos2Plan>()?;
    m.add_class::<PyCodecRegistry>()?;
    parent.add_submodule(&m)?;
    Ok(())
}
