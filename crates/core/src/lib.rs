//! Core types for the **atomr-physical** framework — the physical
//! sensing, output, and robotics layer of the
//! [atomr](https://github.com/rustakka/atomr) actor ecosystem.
//!
//! This crate is the pure-data foundation: identifiers, physical
//! quantities, sensor [`Reading`]s, actuation [`Command`]s, the error
//! taxonomy, and the [`Device`] / [`Sensor`] / [`Actuator`] contract
//! traits. It carries no actor-runtime, hardware-driver, or ROS2
//! dependency — the layers that do (`atomr-physical-sensing`,
//! `atomr-physical-actuation`, `atomr-physical-robotics`,
//! `atomr-physical-ros2`) build on top of it.
//!
//! The contract traits are the seam between atomr-physical and real
//! hardware: a driver implements [`Sensor`] or [`Actuator`] in plain
//! async Rust, and the upper crates adapt that implementation into a
//! supervised atomr actor.

mod command;
mod device;
mod error;
mod ids;
mod reading;
mod units;

pub use command::{Command, CommandAck, ControlMode};
pub use device::{Actuator, Capability, Device, DeviceDescriptor, DeviceKind, Sensor};
pub use error::{PhysicalError, Result};
pub use ids::{ActuatorId, DeviceId, JointId, RobotId, SensorId};
pub use reading::{Reading, ReadingBatch};
pub use units::{Quantity, Unit};

/// The recommended glob-import surface for downstream crates.
pub mod prelude {
    pub use crate::command::{Command, CommandAck, ControlMode};
    pub use crate::device::{Actuator, Capability, Device, DeviceDescriptor, DeviceKind, Sensor};
    pub use crate::error::{PhysicalError, Result};
    pub use crate::ids::{ActuatorId, DeviceId, JointId, RobotId, SensorId};
    pub use crate::reading::{Reading, ReadingBatch};
    pub use crate::units::{Quantity, Unit};
}
