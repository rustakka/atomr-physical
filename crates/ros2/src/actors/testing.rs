//! Test doubles for the device seam.
//!
//! [`MockReadingSource`] and [`MockCommandSink`] implement
//! [`ReadingSource`] / [`CommandSink`] directly, so the orchestration
//! actors can be exercised without a `SensorActor` / `ActuatorActor` or
//! any hardware. Available in the crate's own tests and, for downstream
//! test suites, behind the `mock` feature.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use atomr_physical_core::{ActuatorId, Command, CommandAck, Reading, Result, SensorId};

use super::{CommandSink, ReadingSource};

/// A [`ReadingSource`] that yields a preset reading on every call.
pub struct MockReadingSource {
    sensor: SensorId,
    period: Option<Duration>,
    reading: Reading,
    calls: Arc<AtomicUsize>,
}

impl MockReadingSource {
    /// A source for `sensor` that always yields `reading`, sampled only
    /// on explicit demand (no period).
    pub fn new(sensor: impl Into<String>, reading: Reading) -> Self {
        Self {
            sensor: SensorId::from(sensor.into()),
            period: None,
            reading,
            calls: Arc::new(AtomicUsize::new(0)),
        }
    }

    /// Builder-style: give the source a sampling period so a publisher
    /// self-paces against it.
    pub fn with_period(mut self, period: Duration) -> Self {
        self.period = Some(period);
        self
    }

    /// A handle to this source's call counter — clone it out before the
    /// source is wrapped in `Arc<dyn ReadingSource>`.
    pub fn call_counter(&self) -> Arc<AtomicUsize> {
        Arc::clone(&self.calls)
    }
}

#[async_trait]
impl ReadingSource for MockReadingSource {
    fn sensor_id(&self) -> SensorId {
        self.sensor.clone()
    }

    fn sampling_period(&self) -> Option<Duration> {
        self.period
    }

    async fn next_reading(&self) -> Result<Reading> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        Ok(self.reading.clone())
    }
}

/// A [`CommandSink`] that records every command delivered to it — the
/// `CommandSink` analogue of `testkit::MockActuator`.
pub struct MockCommandSink {
    actuator: ActuatorId,
    log: Arc<Mutex<Vec<Command>>>,
}

impl MockCommandSink {
    /// A sink for `actuator` that accepts and records every command.
    pub fn new(actuator: impl Into<String>) -> Self {
        Self {
            actuator: ActuatorId::from(actuator.into()),
            log: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// A handle to this sink's command log — clone it out before the
    /// sink is wrapped in `Arc<dyn CommandSink>`, then read it to see
    /// what the orchestration delivered.
    pub fn log_handle(&self) -> Arc<Mutex<Vec<Command>>> {
        Arc::clone(&self.log)
    }
}

#[async_trait]
impl CommandSink for MockCommandSink {
    fn actuator_id(&self) -> ActuatorId {
        self.actuator.clone()
    }

    async fn deliver(&self, command: Command) -> Result<CommandAck> {
        let ack = CommandAck::accepted(command.actuator.clone());
        self.log.lock().expect("command log poisoned").push(command);
        Ok(ack)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_physical_core::{ControlMode, Quantity, Unit};

    fn reading() -> Reading {
        Reading {
            sensor: SensorId::from("s1"),
            quantity: Quantity::new(1.0, Unit::Celsius),
            timestamp_ms: 0,
            frame: None,
        }
    }

    #[tokio::test]
    async fn mock_reading_source_counts_calls() {
        let source = MockReadingSource::new("s1", reading());
        let counter = source.call_counter();
        let _ = source.next_reading().await.unwrap();
        let _ = source.next_reading().await.unwrap();
        assert_eq!(counter.load(Ordering::Relaxed), 2);
    }

    #[tokio::test]
    async fn mock_command_sink_records_commands() {
        let sink = MockCommandSink::new("a1");
        let log = sink.log_handle();
        let command = Command::now(
            ActuatorId::from("a1"),
            ControlMode::Position,
            Quantity::new(0.5, Unit::Radian),
        );
        let ack = sink.deliver(command).await.unwrap();
        assert!(ack.accepted);
        assert_eq!(log.lock().unwrap().len(), 1);
    }
}
