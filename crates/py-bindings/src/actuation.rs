//! Actuator-side bindings: `SafetyEnvelope`.

use atomr_physical_actuation::SafetyEnvelope;
use atomr_physical_core::{ActuatorId, PhysicalError};
use pyo3::prelude::*;

/// A min / max clamp on an actuator setpoint.
#[pyclass(name = "SafetyEnvelope", module = "atomr_physical._native.actuation")]
#[derive(Clone, Copy)]
pub struct PySafetyEnvelope {
    pub(crate) inner: SafetyEnvelope,
}

#[pymethods]
impl PySafetyEnvelope {
    /// An envelope that clamps out-of-range setpoints into `[min, max]`.
    #[staticmethod]
    fn clamping(min: f64, max: f64) -> Self {
        Self {
            inner: SafetyEnvelope::clamping(min, max),
        }
    }

    /// An envelope that rejects out-of-range setpoints.
    #[staticmethod]
    fn rejecting(min: f64, max: f64) -> Self {
        Self {
            inner: SafetyEnvelope::rejecting(min, max),
        }
    }

    #[getter]
    fn min(&self) -> f64 {
        self.inner.min
    }

    #[getter]
    fn max(&self) -> f64 {
        self.inner.max
    }

    #[getter]
    fn clamp(&self) -> bool {
        self.inner.clamp
    }

    /// Apply the envelope to a raw setpoint. Returns the (possibly
    /// clamped) value, or raises `OutOfRange` if the value is outside
    /// the envelope and clamping is disabled.
    fn enforce(&self, actuator: String, value: f64) -> PyResult<f64> {
        self.inner
            .enforce(&ActuatorId::from(actuator), value)
            .map_err(|e| match e {
                PhysicalError::OutOfRange { .. } => PyErr::new::<crate::errors::OutOfRange, _>(e.to_string()),
                other => crate::errors::map(other),
            })
    }

    fn __repr__(&self) -> String {
        format!(
            "SafetyEnvelope(min={}, max={}, clamp={})",
            self.inner.min, self.inner.max, self.inner.clamp
        )
    }
}

/// Register the `actuation` submodule.
pub fn register(py: Python<'_>, parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(py, "actuation")?;
    m.add_class::<PySafetyEnvelope>()?;
    parent.add_submodule(&m)?;
    Ok(())
}
