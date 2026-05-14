//! # atomr-physical-py-bindings
//!
//! PyO3 bindings exposing atomr-physical's value types and device
//! contract to Python as the `atomr_physical._native` extension module.
//!
//! Module layout (mirrors the upstream `atomr` / `atomr-agents` Python
//! binding convention — one native submodule per Rust crate, with a
//! thin pure-Python facade over each in `python/atomr_physical/`):
//!
//! - `atomr_physical._native.errors`    — exception hierarchy
//! - `atomr_physical._native.core`      — `Quantity`, `Reading`,
//!                                        `Command`, `CommandAck`,
//!                                        `DeviceDescriptor`
//! - `atomr_physical._native.sensing`   — `SamplingPolicy`, `Calibration`
//! - `atomr_physical._native.actuation` — `SafetyEnvelope`
//! - `atomr_physical._native.robotics`  — `Joint`, `RobotModel`
//! - `atomr_physical._native.ros2`      — `Ros2Endpoint`, `TopicMap`
//!
//! Each submodule's `register` fn creates a Python submodule, registers
//! its `#[pyclass]`es, and attaches it to the parent — the same pattern
//! `atomr-agents-py-bindings` uses.

#![allow(non_local_definitions)] // pyo3 macros emit local impls in modules.

use pyo3::prelude::*;

mod actuation;
mod core;
mod errors;
mod robotics;
mod ros2;
mod sensing;

/// Module init. Exposed as `atomr_physical._native`.
///
/// The function name matches `[lib].name` in `Cargo.toml`, which is
/// what pyo3 uses to derive the `PyInit__native` symbol CPython looks
/// up on import.
#[pymodule]
fn _native(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    errors::register(py, m)?;
    core::register(py, m)?;
    sensing::register(py, m)?;
    actuation::register(py, m)?;
    robotics::register(py, m)?;
    ros2::register(py, m)?;
    Ok(())
}
