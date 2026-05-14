//! Sensor-side bindings: `SamplingPolicy` and `Calibration`.

use atomr_physical_sensing::{Calibration, SamplingPolicy};
use pyo3::prelude::*;

/// How often a sensor actor should poll its underlying driver.
#[pyclass(name = "SamplingPolicy", module = "atomr_physical._native.sensing")]
#[derive(Clone, Copy)]
pub struct PySamplingPolicy {
    pub(crate) inner: SamplingPolicy,
}

#[pymethods]
impl PySamplingPolicy {
    /// A fixed-rate polling policy with the given period in milliseconds.
    #[staticmethod]
    fn fixed_rate(period_ms: u64) -> Self {
        Self {
            inner: SamplingPolicy::FixedRate { period_ms },
        }
    }

    /// A request / response polling policy — read only when asked.
    #[staticmethod]
    fn on_demand() -> Self {
        Self {
            inner: SamplingPolicy::OnDemand,
        }
    }

    /// The polling period in milliseconds, or `None` for on-demand.
    #[getter]
    fn period_ms(&self) -> Option<u64> {
        match self.inner {
            SamplingPolicy::FixedRate { period_ms } => Some(period_ms),
            SamplingPolicy::OnDemand => None,
        }
    }

    /// `True` if this is an on-demand (non-rate) policy.
    #[getter]
    fn is_on_demand(&self) -> bool {
        matches!(self.inner, SamplingPolicy::OnDemand)
    }

    fn __repr__(&self) -> String {
        match self.inner {
            SamplingPolicy::FixedRate { period_ms } => {
                format!("SamplingPolicy.fixed_rate({period_ms})")
            }
            SamplingPolicy::OnDemand => "SamplingPolicy.on_demand()".to_string(),
        }
    }
}

/// A linear sensor calibration: `corrected = raw * scale + offset`.
#[pyclass(name = "Calibration", module = "atomr_physical._native.sensing")]
#[derive(Clone, Copy)]
pub struct PyCalibration {
    pub(crate) inner: Calibration,
}

#[pymethods]
impl PyCalibration {
    #[new]
    #[pyo3(signature = (scale=1.0, offset=0.0))]
    fn new(scale: f64, offset: f64) -> Self {
        Self {
            inner: Calibration { scale, offset },
        }
    }

    #[getter]
    fn scale(&self) -> f64 {
        self.inner.scale
    }

    #[getter]
    fn offset(&self) -> f64 {
        self.inner.offset
    }

    /// Apply the calibration to a raw value.
    fn apply(&self, raw: f64) -> f64 {
        self.inner.apply(raw)
    }

    fn __repr__(&self) -> String {
        format!(
            "Calibration(scale={}, offset={})",
            self.inner.scale, self.inner.offset
        )
    }
}

/// Register the `sensing` submodule.
pub fn register(py: Python<'_>, parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(py, "sensing")?;
    m.add_class::<PySamplingPolicy>()?;
    m.add_class::<PyCalibration>()?;
    parent.add_submodule(&m)?;
    Ok(())
}
