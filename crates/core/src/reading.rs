//! Sensor readings — the input side of the physical layer.

use serde::{Deserialize, Serialize};

use crate::ids::SensorId;
use crate::units::Quantity;

/// A single timestamped sample emitted by a sensor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Reading {
    /// The sensor that produced this sample.
    pub sensor: SensorId,
    /// The measured quantity.
    pub quantity: Quantity,
    /// Capture time, milliseconds since the Unix epoch.
    pub timestamp_ms: i64,
    /// Optional spatial reference frame (e.g. `"base_link"`), mirroring
    /// the `frame_id` convention used by ROS2 message headers.
    pub frame: Option<String>,
}

impl Reading {
    /// Build a reading, stamping it with the current wall-clock time.
    pub fn now(sensor: SensorId, quantity: Quantity) -> Self {
        Self {
            sensor,
            quantity,
            timestamp_ms: chrono::Utc::now().timestamp_millis(),
            frame: None,
        }
    }

    /// Builder-style: attach a reference frame to this reading.
    pub fn with_frame(mut self, frame: impl Into<String>) -> Self {
        self.frame = Some(frame.into());
        self
    }
}

/// A batch of readings captured together — e.g. one scan of a
/// multi-axis sensor — returned as a unit so downstream actors see a
/// consistent timestamp grouping.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ReadingBatch {
    /// The readings, in capture order.
    pub readings: Vec<Reading>,
}

impl ReadingBatch {
    /// An empty batch.
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a reading to the batch.
    pub fn push(&mut self, reading: Reading) {
        self.readings.push(reading);
    }

    /// Number of readings in the batch.
    pub fn len(&self) -> usize {
        self.readings.len()
    }

    /// Returns `true` if the batch has no readings.
    pub fn is_empty(&self) -> bool {
        self.readings.is_empty()
    }
}
