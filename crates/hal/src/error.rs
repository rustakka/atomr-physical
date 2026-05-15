//! Error taxonomy for the HAL layer.
//!
//! [`HalError`] unifies bus-transport errors (CAN/I2C/SPI) and device-
//! driver errors so each driver can return one consistent type. A
//! conversion into [`atomr_physical_core::PhysicalError`] is provided so
//! drivers slot into the [`atomr_physical_core::Sensor`] /
//! [`atomr_physical_core::Actuator`] contracts which return
//! `Result<_, PhysicalError>`.

use atomr_physical_core::PhysicalError;

/// Result alias used throughout the HAL crate.
pub type Result<T> = std::result::Result<T, HalError>;

/// Errors raised by HAL bus actors and device drivers.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum HalError {
    /// A bus-transport-level error (socket open / read / write, etc.).
    #[error("bus error: {0}")]
    Bus(String),
    /// A driver-level error tied to a specific device.
    #[error("driver error: {device}: {message}")]
    Driver {
        /// The device whose driver raised the error.
        device: String,
        /// Driver-level explanation.
        message: String,
    },
    /// A read / response did not arrive within the expected window.
    #[error("timeout waiting for {0}")]
    Timeout(String),
    /// A wire-protocol frame could not be parsed or constructed.
    #[error("invalid frame: {0}")]
    Frame(String),
    /// The device has not been initialised / configured.
    #[error("device not configured: {0}")]
    NotConfigured(String),
}

impl From<HalError> for PhysicalError {
    fn from(e: HalError) -> Self {
        PhysicalError::Fault(e.to_string())
    }
}
