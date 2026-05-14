---
name: atomr-physical-quickstart
description: Use when standing up the first atomr-physical project, picking feature flags for the `atomr-physical` umbrella, wrapping a hardware driver in a SensorActor or ActuatorActor, or running an end-to-end device interaction against MockSensor / MockActuator. Triggers on adding `atomr-physical = ...` to Cargo.toml, writing the first `SensorActor::new` / `ActuatorActor::new`, or asking "how do I get atomr-physical running".
---

# atomr-physical quickstart

The physical sensing, output, and ROS2-integrated robotics layer of the
[atomr](https://github.com/rustakka/atomr) actor ecosystem. A sensor is
an actor; an actuator is an actor; a robot is the supervisor at the top
of that tree.

## The 30-second mental model

- **A device is an actor.** A driver implements the `Sensor` or
  `Actuator` contract trait in plain async Rust; `SensorActor` /
  `ActuatorActor` adapt it into a supervised atomr actor with a
  sampling loop / command queue.
- **Quantities carry units.** `Quantity { value, unit }` — physical
  magnitudes never cross a public API as a bare `f64`.
- **Safety is at the boundary.** An `ActuatorActor` runs every setpoint
  through a `SafetyEnvelope` (clamp into range, or reject) *before* the
  driver sees it.
- **`core` is pure.** `atomr-physical-core` has no actor-runtime,
  hardware, or ROS2 dependency — it is data + the three contract
  traits. The layers above it add the actors.

## The minimal consumer Cargo.toml

```toml
[dependencies]
# Defaults: sensing + actuation + robotics
atomr-physical = "0.1"

# Add the ROS2 bridge and/or test doubles:
# atomr-physical = { version = "0.1", features = ["ros2", "testkit"] }

# Or pull subsystem crates directly:
# atomr-physical-core      = "0.1"
# atomr-physical-sensing   = "0.1"
# atomr-physical-actuation = "0.1"
```

See [`docs/feature-matrix.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/feature-matrix.md)
for every flag and the canonical shapes.

## Wrapping a driver

```rust
use std::sync::Arc;
use atomr_physical::prelude::*;
use atomr_physical::sensing::{SamplingPolicy, SensorActor};
use atomr_physical::actuation::{ActuatorActor, SafetyEnvelope};
use atomr_physical_testkit::{MockActuator, MockSensor};

// `MockSensor` / `MockActuator` implement the contract traits — swap
// them for real drivers and the same code runs unchanged.
let temp_driver = Arc::new(MockSensor::constant("imu-temp", 21.0, Unit::Celsius));
let sensor = SensorActor::new(temp_driver, SamplingPolicy::default_rate());

let servo_driver = Arc::new(MockActuator::new("joint-0"));
let servo = ActuatorActor::new(servo_driver)
    .with_envelope(SafetyEnvelope::clamping(-1.57, 1.57));

let reading = sensor.sample().await?;
let ack = servo.dispatch(Command::now(
    ActuatorId::from("joint-0"),
    ControlMode::Position,
    Quantity::new(3.0, Unit::Radian),   // clamped to 1.57 by the envelope
)).await?;
```

## When to reach beyond the quickstart

| You need… | Reach for… |
|---|---|
| To implement a real sensor driver | the `atomr-physical-sensing` skill |
| To implement a real actuator driver + safety bounds | the `atomr-physical-actuation` skill |
| To orchestrate a multi-joint robot | the `atomr-physical-robotics` skill |
| ROS2 topic / node interop | the `atomr-physical-ros2` skill |
| To drive devices from Python | the `atomr-physical-python` skill |

## Canonical references

- [`docs/index.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/index.md) — documentation hub
- [`docs/architecture.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/architecture.md) — crate stack + the device-actor model
- [`docs/feature-matrix.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/feature-matrix.md) — every feature flag

## Common mistakes

- **Passing bare `f64`s.** Use `Quantity::new(value, Unit::…)` — the
  unit is part of the value.
- **Skipping the `SafetyEnvelope`.** An `ActuatorActor::new` with no
  `.with_envelope(...)` dispatches setpoints unchecked. Attach one.
- **Expecting the actor loop to run.** At 0.1.0 `SensorActor` /
  `ActuatorActor` expose direct `sample` / `dispatch` paths; the
  supervised sampling loop is Phase 2 (see `docs/architecture.md`).
- **Using `MockSensor` / `MockActuator` in production.** They return
  canned values and log commands — swap in a real driver.
