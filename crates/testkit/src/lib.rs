//! Test doubles for atomr-physical.
//!
//! [`MockSensor`] and [`MockActuator`] implement the
//! [`atomr_physical_core::Sensor`] / [`Actuator`](atomr_physical_core::Actuator)
//! contract traits with in-memory behaviour, so sensing, actuation, and
//! robotics code can be exercised without hardware or a ROS2 graph.
//!
//! With the `sdr` feature enabled, [`MockSdrDriver`] is also available
//! — it implements [`atomr_physical_sdr::SdrBackend`] by generating
//! synthetic IQ samples, so SDR actor tests run hardware-free.

use std::sync::Mutex;

use async_trait::async_trait;
use atomr_physical_core::{
    Actuator, ActuatorId, Capability, Command, CommandAck, Device, DeviceDescriptor, DeviceId, DeviceKind,
    Quantity, Reading, Result, Sensor, SensorId, Unit,
};

/// A sensor that replays a script of canned [`Quantity`] values, cycling
/// back to the start once the script is exhausted.
pub struct MockSensor {
    descriptor: DeviceDescriptor,
    script: Vec<Quantity>,
    cursor: Mutex<usize>,
}

impl MockSensor {
    /// Build a mock sensor that replays `script` on each
    /// [`read`](Sensor::read). The script must be non-empty.
    pub fn new(id: impl Into<String>, script: Vec<Quantity>) -> Self {
        assert!(!script.is_empty(), "MockSensor script must be non-empty");
        let descriptor = DeviceDescriptor::new(DeviceId::from(id.into()), DeviceKind::Sensor, "mock-sensor")
            .with_capability(Capability::new("mock", Unit::Scalar));
        Self {
            descriptor,
            script,
            cursor: Mutex::new(0),
        }
    }

    /// A mock sensor that always reports the same constant value.
    pub fn constant(id: impl Into<String>, value: f64, unit: Unit) -> Self {
        Self::new(id, vec![Quantity::new(value, unit)])
    }
}

#[async_trait]
impl Device for MockSensor {
    fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    async fn health_check(&self) -> Result<()> {
        Ok(())
    }
}

#[async_trait]
impl Sensor for MockSensor {
    async fn read(&self) -> Result<Reading> {
        let quantity = {
            let mut cursor = self.cursor.lock().expect("mock sensor cursor poisoned");
            let q = self.script[*cursor % self.script.len()];
            *cursor = cursor.wrapping_add(1);
            q
        };
        Ok(Reading::now(
            SensorId::from(self.descriptor.id.as_str()),
            quantity,
        ))
    }
}

/// An actuator that records every [`Command`] it receives and always
/// acknowledges acceptance.
pub struct MockActuator {
    descriptor: DeviceDescriptor,
    log: Mutex<Vec<Command>>,
}

impl MockActuator {
    /// Build a mock actuator.
    pub fn new(id: impl Into<String>) -> Self {
        let descriptor =
            DeviceDescriptor::new(DeviceId::from(id.into()), DeviceKind::Actuator, "mock-actuator")
                .with_capability(Capability::new("mock", Unit::Scalar));
        Self {
            descriptor,
            log: Mutex::new(Vec::new()),
        }
    }

    /// Every command this actuator has accepted, in arrival order.
    pub fn log(&self) -> Vec<Command> {
        self.log.lock().expect("mock actuator log poisoned").clone()
    }

    /// Number of commands received.
    pub fn command_count(&self) -> usize {
        self.log.lock().expect("mock actuator log poisoned").len()
    }
}

#[async_trait]
impl Device for MockActuator {
    fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    async fn health_check(&self) -> Result<()> {
        Ok(())
    }
}

#[async_trait]
impl Actuator for MockActuator {
    async fn apply(&self, command: Command) -> Result<CommandAck> {
        let actuator = ActuatorId::from(self.descriptor.id.as_str());
        self.log.lock().expect("mock actuator log poisoned").push(command);
        Ok(CommandAck::accepted(actuator))
    }
}

#[cfg(feature = "sdr")]
mod sdr_mock;

#[cfg(feature = "sdr")]
pub use sdr_mock::{MockSdrDriver, MockWaveform};

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_physical_core::ControlMode;

    #[tokio::test]
    async fn mock_sensor_cycles_script() {
        let sensor = MockSensor::new("s1", vec![Quantity::scalar(1.0), Quantity::scalar(2.0)]);
        assert_eq!(sensor.read().await.unwrap().quantity.value, 1.0);
        assert_eq!(sensor.read().await.unwrap().quantity.value, 2.0);
        assert_eq!(sensor.read().await.unwrap().quantity.value, 1.0);
    }

    #[tokio::test]
    async fn mock_actuator_logs_commands() {
        let actuator = MockActuator::new("a1");
        let cmd = Command::now(
            ActuatorId::from("a1"),
            ControlMode::Position,
            Quantity::new(0.5, Unit::Radian),
        );
        let ack = actuator.apply(cmd).await.unwrap();
        assert!(ack.accepted);
        assert_eq!(actuator.command_count(), 1);
    }
}
