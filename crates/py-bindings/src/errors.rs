//! Python exception hierarchy mirroring `atomr_physical_core::PhysicalError`.
//!
//! Hierarchy (Python side):
//!
//! ```text
//! PhysicalError
//!  ├─ OutOfRange
//!  └─ DeviceFault
//! ```
//!
//! Rust callers funnel `?` through [`map`] so the Python side sees
//! consistent exception types.

use pyo3::create_exception;
use pyo3::exceptions::PyException;
use pyo3::prelude::*;

create_exception!(
    atomr_physical,
    PhysicalError,
    PyException,
    "Base atomr-physical error."
);
create_exception!(
    atomr_physical,
    OutOfRange,
    PhysicalError,
    "An actuation setpoint fell outside the safety envelope."
);
create_exception!(
    atomr_physical,
    DeviceFault,
    PhysicalError,
    "A driver / transport fault, or an unreachable device."
);

/// Map any `Display` error onto the base `PhysicalError`.
pub fn map<E: std::fmt::Display>(e: E) -> PyErr {
    PyErr::new::<PhysicalError, _>(e.to_string())
}

/// Register the `errors` submodule.
pub fn register(py: Python<'_>, parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(py, "errors")?;
    m.add("PhysicalError", py.get_type_bound::<PhysicalError>())?;
    m.add("OutOfRange", py.get_type_bound::<OutOfRange>())?;
    m.add("DeviceFault", py.get_type_bound::<DeviceFault>())?;
    parent.add_submodule(&m)?;
    Ok(())
}
