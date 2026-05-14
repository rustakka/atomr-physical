//! Sensor-side actors for atomr-physical.
//!
//! A hardware driver implements [`atomr_physical_core::Sensor`] in plain
//! async Rust. This crate adapts that implementation into a supervised
//! atomr actor — [`SensorActor`] — that owns a sampling loop, applies a
//! [`Calibration`], and (Phase 2) publishes [`Reading`]s onto its
//! mailbox so it slots into a robot's supervision tree as an
//! addressable `ActorRef`.
//!
//! The atomr actor runtime is re-exported as [`actor`] so downstream
//! crates have a single import path for it.

use std::sync::Arc;
use std::time::Duration;

use atomr_physical_core::{Reading, Result, Sensor, SensorId};
use serde::{Deserialize, Serialize};

/// Re-export of the atomr actor runtime this crate builds on.
pub use atomr_core as actor;

/// How often a [`SensorActor`] should poll its underlying [`Sensor`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SamplingPolicy {
    /// Poll on a fixed period.
    FixedRate {
        /// The polling period, in milliseconds.
        period_ms: u64,
    },
    /// Take a reading only when explicitly asked (request / response).
    OnDemand,
}

impl SamplingPolicy {
    /// A 10 Hz fixed-rate policy — a sane default for most chassis
    /// sensors.
    pub fn default_rate() -> Self {
        SamplingPolicy::FixedRate { period_ms: 100 }
    }

    /// The sampling period as a [`Duration`], if this policy is
    /// rate-based.
    pub fn period(&self) -> Option<Duration> {
        match self {
            SamplingPolicy::FixedRate { period_ms } => Some(Duration::from_millis(*period_ms)),
            SamplingPolicy::OnDemand => None,
        }
    }
}

/// A linear calibration applied to a raw sensor value:
/// `corrected = raw * scale + offset`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Calibration {
    /// Multiplicative scale factor.
    pub scale: f64,
    /// Additive offset, applied after scaling.
    pub offset: f64,
}

impl Calibration {
    /// The identity calibration — passes raw values through unchanged.
    pub fn identity() -> Self {
        Self {
            scale: 1.0,
            offset: 0.0,
        }
    }

    /// Apply the calibration to a raw value.
    pub fn apply(&self, raw: f64) -> f64 {
        raw * self.scale + self.offset
    }
}

impl Default for Calibration {
    fn default() -> Self {
        Self::identity()
    }
}

/// Adapts a [`Sensor`] driver into a supervised atomr actor.
///
/// **Phase 2** wires `SensorActor` into [`actor`]'s `Actor` trait so the
/// sampling loop runs under supervision and `Reading`s flow over the
/// actor's mailbox. The current form holds the driver, its sampling
/// policy, and calibration, and exposes a direct async [`sample`] path
/// so the type is usable ahead of the actor wiring.
///
/// [`sample`]: SensorActor::sample
pub struct SensorActor {
    sensor: Arc<dyn Sensor>,
    policy: SamplingPolicy,
    calibration: Calibration,
}

impl SensorActor {
    /// Wrap a sensor driver with a sampling policy.
    pub fn new(sensor: Arc<dyn Sensor>, policy: SamplingPolicy) -> Self {
        Self {
            sensor,
            policy,
            calibration: Calibration::identity(),
        }
    }

    /// Builder-style: attach a calibration applied to every reading.
    pub fn with_calibration(mut self, calibration: Calibration) -> Self {
        self.calibration = calibration;
        self
    }

    /// The id of the wrapped sensor.
    pub fn id(&self) -> SensorId {
        SensorId::from(self.sensor.descriptor().id.as_str())
    }

    /// This actor's sampling policy.
    pub fn policy(&self) -> SamplingPolicy {
        self.policy
    }

    /// Take one calibrated reading from the underlying driver.
    pub async fn sample(&self) -> Result<Reading> {
        let mut reading = self.sensor.read().await?;
        reading.quantity.value = self.calibration.apply(reading.quantity.value);
        Ok(reading)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_physical_core::Unit;
    use atomr_physical_testkit::MockSensor;

    #[test]
    fn calibration_is_linear() {
        let cal = Calibration {
            scale: 2.0,
            offset: 1.0,
        };
        assert_eq!(cal.apply(3.0), 7.0);
    }

    #[tokio::test]
    async fn sensor_actor_applies_calibration() {
        let driver = Arc::new(MockSensor::constant("s1", 10.0, Unit::Celsius));
        let actor = SensorActor::new(driver, SamplingPolicy::OnDemand).with_calibration(Calibration {
            scale: 1.0,
            offset: -5.0,
        });
        let reading = actor.sample().await.unwrap();
        assert_eq!(reading.quantity.value, 5.0);
    }
}
