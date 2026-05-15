//! Concrete device drivers built on the bus actors.
//!
//! Each driver lives behind its own feature flag. A driver implements
//! the atomr-physical [`Sensor`] / [`Actuator`] contract traits and
//! talks to its bus through the supervised bus actor handles in
//! [`crate::bus`].
//!
//! [`Sensor`]: atomr_physical_core::Sensor
//! [`Actuator`]: atomr_physical_core::Actuator

#[cfg(feature = "bno085")]
pub mod bno085;
#[cfg(feature = "odrive")]
pub mod odrive;
#[cfg(feature = "qdd-mit")]
pub mod qdd_mit;
#[cfg(feature = "as5048a")]
pub mod as5048a;
