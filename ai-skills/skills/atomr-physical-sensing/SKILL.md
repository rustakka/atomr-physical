---
name: atomr-physical-sensing
description: Use when implementing a sensor driver against the atomr-physical `Sensor` contract trait, wrapping it in a SensorActor, choosing a SamplingPolicy (FixedRate vs OnDemand), or applying a linear Calibration to raw readings. Triggers on `impl Sensor for`, `SensorActor::new`, `SamplingPolicy::`, or `Calibration {`.
---

# atomr-physical sensing

The input side of the physical layer: a `Sensor` driver, adapted into a
supervised `SensorActor` with a sampling loop and a calibration.

## The mental model

- **A driver implements one trait.** `Sensor: Device` —
  `async fn read(&self) -> Result<Reading>` plus the `Device`
  descriptor + health check. That is the entire hardware seam.
- **`SensorActor` owns the loop.** It wraps an `Arc<dyn Sensor>` with a
  `SamplingPolicy` and a `Calibration`. `SensorActor::sample` takes one
  calibrated reading.
- **A `Reading` is timestamped and framed.** `Reading { sensor,
  quantity, timestamp_ms, frame }` — `frame` mirrors ROS2's `frame_id`.

## Implementing a driver

```rust
use async_trait::async_trait;
use atomr_physical::prelude::*;

struct Bme280 { descriptor: DeviceDescriptor /* + bus handle */ }

#[async_trait]
impl Device for Bme280 {
    fn descriptor(&self) -> &DeviceDescriptor { &self.descriptor }
    async fn health_check(&self) -> Result<()> {
        // probe the bus; return PhysicalError::NotReady on failure
        Ok(())
    }
}

#[async_trait]
impl Sensor for Bme280 {
    async fn read(&self) -> Result<Reading> {
        let celsius = /* read the chip */ 21.4;
        Ok(Reading::now(
            SensorId::from(self.descriptor().id.as_str()),
            Quantity::new(celsius, Unit::Celsius),
        ).with_frame("base_link"))
    }
}
```

## Wrapping it

```rust
use std::sync::Arc;
use atomr_physical::sensing::{Calibration, SamplingPolicy, SensorActor};

let sensor = SensorActor::new(Arc::new(driver), SamplingPolicy::FixedRate { period_ms: 50 })
    // corrected = raw * scale + offset
    .with_calibration(Calibration { scale: 1.0, offset: -0.8 });

let reading = sensor.sample().await?;   // calibration already applied
```

`SamplingPolicy::default_rate()` is 10 Hz. `SamplingPolicy::OnDemand`
means "read only when asked" — `period()` returns `None`.

## Canonical references

- [`docs/architecture.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/architecture.md) — the device-actor model
- `crates/core/src/device.rs` — the `Sensor` / `Device` traits
- `crates/sensing/src/lib.rs` — `SensorActor`, `SamplingPolicy`, `Calibration`
- `crates/testkit/src/lib.rs` — `MockSensor` as a reference driver

## Common mistakes

- **Returning a `Reading` with the wrong `SensorId`.** Build it from
  `self.descriptor().id` so the id matches the registered device.
- **Putting calibration in the driver.** The driver returns *raw*
  values; the `SensorActor`'s `Calibration` corrects them. Keeping the
  driver raw means recalibration needs no driver change.
- **Choosing `FixedRate` for a slow bus.** If a `read()` takes longer
  than `period_ms`, the loop falls behind — size the period to the
  driver, or use `OnDemand`.
