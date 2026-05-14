---
name: atomr-physical-actuation
description: Use when implementing an actuator driver against the atomr-physical `Actuator` contract trait, wrapping it in an ActuatorActor, or configuring a SafetyEnvelope (clamping vs rejecting) to bound setpoints before they reach hardware. Triggers on `impl Actuator for`, `ActuatorActor::new`, `SafetyEnvelope::`, `ControlMode::`, or `Command::now`.
---

# atomr-physical actuation

The output side of the physical layer: an `Actuator` driver, adapted
into a supervised `ActuatorActor` that enforces a `SafetyEnvelope`
before anything reaches hardware.

## The mental model

- **A driver implements one trait.** `Actuator: Device` —
  `async fn apply(&self, command: Command) -> Result<CommandAck>` plus
  the `Device` descriptor + health check.
- **`ActuatorActor` owns the safety boundary.** It wraps an
  `Arc<dyn Actuator>` with an optional `SafetyEnvelope`.
  `ActuatorActor::dispatch` runs the envelope check, then calls the
  driver.
- **A `Command` carries a mode and a unit.** `Command { actuator, mode,
  setpoint, issued_ms }` — `ControlMode` is `Position` / `Velocity` /
  `Effort` / `Duty`; `setpoint` is a `Quantity`.
- **An envelope clamps or rejects.** `SafetyEnvelope::clamping(min,
  max)` pins out-of-range setpoints to the boundary;
  `SafetyEnvelope::rejecting(min, max)` returns
  `PhysicalError::OutOfRange`.

## Implementing a driver

```rust
use async_trait::async_trait;
use atomr_physical::prelude::*;

struct Dynamixel { descriptor: DeviceDescriptor /* + bus handle */ }

#[async_trait]
impl Device for Dynamixel {
    fn descriptor(&self) -> &DeviceDescriptor { &self.descriptor }
    async fn health_check(&self) -> Result<()> { Ok(()) }
}

#[async_trait]
impl Actuator for Dynamixel {
    async fn apply(&self, command: Command) -> Result<CommandAck> {
        let id = command.actuator.clone();
        // write command.setpoint.value to the servo over the bus …
        Ok(CommandAck::accepted(id))
        // …or CommandAck::rejected(id, "torque-disabled") on refusal
    }
}
```

## Wrapping it with a safety envelope

```rust
use std::sync::Arc;
use atomr_physical::actuation::{ActuatorActor, SafetyEnvelope};

let actuator = ActuatorActor::new(Arc::new(driver))
    .with_envelope(SafetyEnvelope::clamping(-1.57, 1.57));   // ± 90°

let ack = actuator.dispatch(Command::now(
    ActuatorId::from("shoulder"),
    ControlMode::Position,
    Quantity::new(2.5, Unit::Radian),       // clamped to 1.57
)).await?;
```

Use `rejecting(min, max)` when an out-of-range command should *fail
loud* rather than be silently clamped — e.g. when an upstream planner
should never have produced it.

## Canonical references

- [`docs/architecture.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/architecture.md) — the device-actor model
- `crates/core/src/device.rs` — the `Actuator` / `Device` traits
- `crates/core/src/command.rs` — `Command`, `CommandAck`, `ControlMode`
- `crates/actuation/src/lib.rs` — `ActuatorActor`, `SafetyEnvelope`

## Common mistakes

- **No envelope.** `ActuatorActor::new(driver)` with no
  `.with_envelope(...)` dispatches every setpoint unchecked. Always
  attach one for real hardware.
- **Confusing clamp and reject.** Clamping silently corrects;
  rejecting fails. A planner bug is better surfaced by `rejecting`.
- **Putting the envelope in the driver.** The driver should apply what
  it is given; the `ActuatorActor` owns the policy, so the envelope can
  be retuned without a driver change.
- **Choosing envelope bounds blindly.** The envelope enforces *your*
  declared bounds — picking bounds that are safe for the hardware is
  the integrator's job (see `SECURITY.md`).
