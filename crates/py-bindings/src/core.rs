//! Core value types: `Quantity`, `Reading`, `Command`, `CommandAck`,
//! and `DeviceDescriptor`.
//!
//! Each `#[pyclass]` wraps its `atomr_physical_core` counterpart with
//! value semantics — construct from Python, read fields back through
//! getters. Units and control modes cross the FFI boundary as short
//! strings (`"rad"`, `"m/s"`, `"position"`, …).

use atomr_physical_core::{
    ActuatorId, Capability, Command, CommandAck, ControlMode, DeviceDescriptor, DeviceId, DeviceKind,
    Quantity, Reading, SensorId, Unit,
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

// ----- string <-> enum helpers ---------------------------------------------

/// Parse a unit string (symbol or name) into a [`Unit`].
pub(crate) fn unit_from_str(s: &str) -> PyResult<Unit> {
    Ok(match s {
        "scalar" | "" => Unit::Scalar,
        "m" | "metre" | "meter" => Unit::Metre,
        "m/s" => Unit::MetrePerSecond,
        "rad" | "radian" => Unit::Radian,
        "rad/s" => Unit::RadianPerSecond,
        "N" | "newton" => Unit::Newton,
        "N·m" | "Nm" | "newton_metre" => Unit::NewtonMetre,
        "C" | "celsius" => Unit::Celsius,
        "Pa" | "pascal" => Unit::Pascal,
        "V" | "volt" => Unit::Volt,
        "A" | "ampere" => Unit::Ampere,
        "%" | "percent" => Unit::Percent,
        other => return Err(PyValueError::new_err(format!("unknown unit: {other:?}"))),
    })
}

/// The short string name for a [`Unit`].
pub(crate) fn unit_name(u: Unit) -> &'static str {
    match u {
        Unit::Scalar => "scalar",
        Unit::Metre => "m",
        Unit::MetrePerSecond => "m/s",
        Unit::Radian => "rad",
        Unit::RadianPerSecond => "rad/s",
        Unit::Newton => "N",
        Unit::NewtonMetre => "N·m",
        Unit::Celsius => "C",
        Unit::Pascal => "Pa",
        Unit::Volt => "V",
        Unit::Ampere => "A",
        Unit::Percent => "%",
        // `Unit` is `#[non_exhaustive]`; new variants map here until the
        // binding is updated.
        _ => "scalar",
    }
}

fn mode_from_str(s: &str) -> PyResult<ControlMode> {
    Ok(match s {
        "position" => ControlMode::Position,
        "velocity" => ControlMode::Velocity,
        "effort" => ControlMode::Effort,
        "duty" => ControlMode::Duty,
        other => return Err(PyValueError::new_err(format!("unknown control mode: {other:?}"))),
    })
}

fn mode_name(m: ControlMode) -> &'static str {
    match m {
        ControlMode::Position => "position",
        ControlMode::Velocity => "velocity",
        ControlMode::Effort => "effort",
        ControlMode::Duty => "duty",
        _ => "position",
    }
}

fn kind_from_str(s: &str) -> PyResult<DeviceKind> {
    Ok(match s {
        "sensor" => DeviceKind::Sensor,
        "actuator" => DeviceKind::Actuator,
        "composite" => DeviceKind::Composite,
        other => return Err(PyValueError::new_err(format!("unknown device kind: {other:?}"))),
    })
}

fn kind_name(k: DeviceKind) -> &'static str {
    match k {
        DeviceKind::Sensor => "sensor",
        DeviceKind::Actuator => "actuator",
        DeviceKind::Composite => "composite",
        _ => "sensor",
    }
}

// ----- Quantity ------------------------------------------------------------

/// A scalar physical quantity: a value paired with its unit.
#[pyclass(name = "Quantity", module = "atomr_physical._native.core")]
#[derive(Clone)]
pub struct PyQuantity {
    pub(crate) inner: Quantity,
}

#[pymethods]
impl PyQuantity {
    #[new]
    #[pyo3(signature = (value, unit="scalar"))]
    fn new(value: f64, unit: &str) -> PyResult<Self> {
        Ok(Self {
            inner: Quantity::new(value, unit_from_str(unit)?),
        })
    }

    #[getter]
    fn value(&self) -> f64 {
        self.inner.value
    }

    #[getter]
    fn unit(&self) -> &'static str {
        unit_name(self.inner.unit)
    }

    fn __repr__(&self) -> String {
        format!(
            "Quantity(value={}, unit={:?})",
            self.inner.value,
            unit_name(self.inner.unit)
        )
    }
}

// ----- Reading -------------------------------------------------------------

/// A single timestamped sample emitted by a sensor.
#[pyclass(name = "Reading", module = "atomr_physical._native.core")]
#[derive(Clone)]
pub struct PyReading {
    pub(crate) inner: Reading,
}

#[pymethods]
impl PyReading {
    #[new]
    #[pyo3(signature = (sensor, value, unit="scalar", frame=None))]
    fn new(sensor: String, value: f64, unit: &str, frame: Option<String>) -> PyResult<Self> {
        let mut reading = Reading::now(SensorId::from(sensor), Quantity::new(value, unit_from_str(unit)?));
        reading.frame = frame;
        Ok(Self { inner: reading })
    }

    #[getter]
    fn sensor(&self) -> String {
        self.inner.sensor.to_string()
    }

    #[getter]
    fn quantity(&self) -> PyQuantity {
        PyQuantity {
            inner: self.inner.quantity,
        }
    }

    #[getter]
    fn value(&self) -> f64 {
        self.inner.quantity.value
    }

    #[getter]
    fn unit(&self) -> &'static str {
        unit_name(self.inner.quantity.unit)
    }

    #[getter]
    fn timestamp_ms(&self) -> i64 {
        self.inner.timestamp_ms
    }

    #[getter]
    fn frame(&self) -> Option<String> {
        self.inner.frame.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "Reading(sensor={:?}, value={}, unit={:?}, ts={})",
            self.inner.sensor.as_str(),
            self.inner.quantity.value,
            unit_name(self.inner.quantity.unit),
            self.inner.timestamp_ms,
        )
    }
}

// ----- Command -------------------------------------------------------------

/// An instruction dispatched to an actuator.
#[pyclass(name = "Command", module = "atomr_physical._native.core")]
#[derive(Clone)]
pub struct PyCommand {
    pub(crate) inner: Command,
}

#[pymethods]
impl PyCommand {
    #[new]
    #[pyo3(signature = (actuator, setpoint, mode="position", unit="scalar"))]
    fn new(actuator: String, setpoint: f64, mode: &str, unit: &str) -> PyResult<Self> {
        Ok(Self {
            inner: Command::now(
                ActuatorId::from(actuator),
                mode_from_str(mode)?,
                Quantity::new(setpoint, unit_from_str(unit)?),
            ),
        })
    }

    #[getter]
    fn actuator(&self) -> String {
        self.inner.actuator.to_string()
    }

    #[getter]
    fn mode(&self) -> &'static str {
        mode_name(self.inner.mode)
    }

    #[getter]
    fn setpoint(&self) -> f64 {
        self.inner.setpoint.value
    }

    #[getter]
    fn unit(&self) -> &'static str {
        unit_name(self.inner.setpoint.unit)
    }

    #[getter]
    fn issued_ms(&self) -> i64 {
        self.inner.issued_ms
    }

    fn __repr__(&self) -> String {
        format!(
            "Command(actuator={:?}, mode={:?}, setpoint={})",
            self.inner.actuator.as_str(),
            mode_name(self.inner.mode),
            self.inner.setpoint.value,
        )
    }
}

// ----- CommandAck ----------------------------------------------------------

/// The acknowledgement an actuator returns for a [`PyCommand`].
#[pyclass(name = "CommandAck", module = "atomr_physical._native.core")]
#[derive(Clone)]
pub struct PyCommandAck {
    pub(crate) inner: CommandAck,
}

#[pymethods]
impl PyCommandAck {
    #[staticmethod]
    fn accepted(actuator: String) -> Self {
        Self {
            inner: CommandAck::accepted(ActuatorId::from(actuator)),
        }
    }

    #[staticmethod]
    fn rejected(actuator: String, reason: String) -> Self {
        Self {
            inner: CommandAck::rejected(ActuatorId::from(actuator), reason),
        }
    }

    #[getter]
    fn actuator(&self) -> String {
        self.inner.actuator.to_string()
    }

    #[getter]
    fn accepted_flag(&self) -> bool {
        self.inner.accepted
    }

    #[getter]
    fn detail(&self) -> Option<String> {
        self.inner.detail.clone()
    }

    #[getter]
    fn acked_ms(&self) -> i64 {
        self.inner.acked_ms
    }

    fn __repr__(&self) -> String {
        format!(
            "CommandAck(actuator={:?}, accepted={}, detail={:?})",
            self.inner.actuator.as_str(),
            self.inner.accepted,
            self.inner.detail,
        )
    }
}

// ----- DeviceDescriptor ----------------------------------------------------

/// Static metadata describing a device.
#[pyclass(name = "DeviceDescriptor", module = "atomr_physical._native.core")]
#[derive(Clone)]
pub struct PyDeviceDescriptor {
    pub(crate) inner: DeviceDescriptor,
}

#[pymethods]
impl PyDeviceDescriptor {
    #[new]
    fn new(id: String, kind: &str, model: String) -> PyResult<Self> {
        Ok(Self {
            inner: DeviceDescriptor::new(DeviceId::from(id), kind_from_str(kind)?, model),
        })
    }

    /// Advertise a capability — a `(name, unit)` the device can measure
    /// or drive.
    fn add_capability(&mut self, name: String, unit: &str) -> PyResult<()> {
        self.inner
            .capabilities
            .push(Capability::new(name, unit_from_str(unit)?));
        Ok(())
    }

    #[getter]
    fn id(&self) -> String {
        self.inner.id.to_string()
    }

    #[getter]
    fn kind(&self) -> &'static str {
        kind_name(self.inner.kind)
    }

    #[getter]
    fn model(&self) -> String {
        self.inner.model.clone()
    }

    /// The advertised capabilities, as `(name, unit)` tuples.
    #[getter]
    fn capabilities(&self) -> Vec<(String, &'static str)> {
        self.inner
            .capabilities
            .iter()
            .map(|c| (c.name.clone(), unit_name(c.unit)))
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "DeviceDescriptor(id={:?}, kind={:?}, model={:?}, capabilities={})",
            self.inner.id.as_str(),
            kind_name(self.inner.kind),
            self.inner.model,
            self.inner.capabilities.len(),
        )
    }
}

// ----- module registration -------------------------------------------------

/// Register the `core` submodule.
pub fn register(py: Python<'_>, parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(py, "core")?;
    m.add_class::<PyQuantity>()?;
    m.add_class::<PyReading>()?;
    m.add_class::<PyCommand>()?;
    m.add_class::<PyCommandAck>()?;
    m.add_class::<PyDeviceDescriptor>()?;
    parent.add_submodule(&m)?;
    Ok(())
}
