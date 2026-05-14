//! The device contract: descriptors, capabilities, and the [`Sensor`] /
//! [`Actuator`] / [`Device`] traits every driver implements.
//!
//! These traits are the seam between atomr-physical and real hardware.
//! A driver implements [`Sensor`] or [`Actuator`] in plain async Rust;
//! the `atomr-physical-sensing` and `atomr-physical-actuation` crates
//! wrap that implementation in a supervised atomr actor so a device
//! becomes an addressable `ActorRef` in a robot's supervision tree.

use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use crate::command::{Command, CommandAck};
use crate::error::Result;
use crate::ids::DeviceId;
use crate::reading::Reading;
use crate::units::Unit;

/// The broad class a device belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[non_exhaustive]
pub enum DeviceKind {
    /// An input device that produces readings.
    Sensor,
    /// An output device that consumes commands.
    Actuator,
    /// A device that both senses and actuates — e.g. a servo with
    /// position feedback.
    Composite,
}

/// A capability a device advertises — what it can measure or drive, and
/// in what unit.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Capability {
    /// A stable capability name, e.g. `"joint_position"` or
    /// `"chassis_temperature"`.
    pub name: String,
    /// The unit readings / commands for this capability are expressed in.
    pub unit: Unit,
}

impl Capability {
    /// Construct a capability descriptor.
    pub fn new(name: impl Into<String>, unit: Unit) -> Self {
        Self {
            name: name.into(),
            unit,
        }
    }
}

/// Static metadata describing a device — surfaced to the registry, the
/// CLI, and the ROS2 bridge so a device can be discovered without
/// touching hardware.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DeviceDescriptor {
    /// The device's stable identifier.
    pub id: DeviceId,
    /// The device class.
    pub kind: DeviceKind,
    /// A human-readable model / driver name.
    pub model: String,
    /// Everything this device can measure or drive.
    pub capabilities: Vec<Capability>,
}

impl DeviceDescriptor {
    /// Construct a descriptor with no capabilities yet.
    pub fn new(id: DeviceId, kind: DeviceKind, model: impl Into<String>) -> Self {
        Self {
            id,
            kind,
            model: model.into(),
            capabilities: Vec::new(),
        }
    }

    /// Builder-style: advertise a capability.
    pub fn with_capability(mut self, capability: Capability) -> Self {
        self.capabilities.push(capability);
        self
    }
}

/// The surface shared by every device — the part that doesn't depend on
/// the direction of data flow.
#[async_trait]
pub trait Device: Send + Sync {
    /// The device's static descriptor.
    fn descriptor(&self) -> &DeviceDescriptor;

    /// Probe the device and return `Ok(())` if it is reachable and ready
    /// to serve reads / commands.
    async fn health_check(&self) -> Result<()>;
}

/// An input device. Drivers implement this in plain async Rust; the
/// sensing crate adapts it into a supervised actor that owns a sampling
/// loop.
#[async_trait]
pub trait Sensor: Device {
    /// Take a single reading from the device.
    async fn read(&self) -> Result<Reading>;
}

/// An output device. Drivers implement this in plain async Rust; the
/// actuation crate adapts it into a supervised actor that enforces the
/// safe-envelope and command-queue policies.
#[async_trait]
pub trait Actuator: Device {
    /// Apply a command to the device, returning the driver-level ack.
    async fn apply(&self, command: Command) -> Result<CommandAck>;
}
