---
name: atomr-physical-troubleshooting
description: Use when debugging an atomr-physical error or unexpected behaviour — PhysicalError variants (OutOfRange, Ros2Bridge, UnitMismatch, NotReady, Timeout, Fault), a setpoint that was silently clamped, a ROS2 bridge that won't spin, or hitting a Phase-2 stub boundary. Triggers on a `PhysicalError` in a stack trace, "why was my setpoint clamped", or "ros2 bridge does nothing".
---

# atomr-physical troubleshooting

Given an error or a surprising result, where to look.

## `PhysicalError` variants

| Variant | Usual cause | Fix |
|---|---|---|
| `OutOfRange { device, value, min, max }` | A setpoint outside a `rejecting` `SafetyEnvelope`. | Clamp upstream, widen the envelope, or fix the planner that produced `value`. |
| `Ros2Bridge(_)` | `Ros2Bridge::spin` called without the `rclrs` feature. | Build with `--features rclrs` (+ a ROS2 toolchain), or stay offline and only use the `TopicMap` plan. |
| `UnitMismatch { from, to }` | A conversion between incompatible `Unit`s. | Check the `Quantity`'s `unit` at the call site — physical magnitudes carry their unit for exactly this reason. |
| `NotReady { device, reason }` | `health_check` failed, or a driver got a command before init. | Probe with `Device::health_check` before dispatch; surface `reason`. |
| `Timeout { device, millis }` | A driver didn't respond within budget. | Size the `SamplingPolicy` period to the bus; check the driver. |
| `SensorRead { .. }` / `ActuationRejected { .. }` / `Fault(_)` | Driver / transport fault. | A driver-level problem — inspect `reason`; the contract trait surfaced it faithfully. |

## "My setpoint was silently changed"

A `SafetyEnvelope::clamping(min, max)` **clamps** out-of-range
setpoints to the boundary and returns `Ok` — that is by design. If you
want an out-of-range setpoint to *fail loud* instead, use
`SafetyEnvelope::rejecting(min, max)`, which returns
`PhysicalError::OutOfRange`. See the `atomr-physical-actuation` skill.

## "The ROS2 bridge does nothing"

`Ros2Bridge::spin` returns `PhysicalError::Ros2Bridge` unless the crate
was built with the `rclrs` feature — the live bridge is Phase 2. The
offline `TopicMap` plan (`bind_sensor` / `bind_actuator` /
`sensor_endpoint`) works without `rclrs` and is the right surface for
tests. See `docs/ros2-bridge.md`.

## "The actor loop never runs"

At 0.1.0 `SensorActor` / `ActuatorActor` / `RobotActor` are **not yet
live atomr `Actor`s** — that wiring is Phase 2 (see
`docs/architecture.md`). Use the direct paths today:
`SensorActor::sample`, `ActuatorActor::dispatch`,
`RobotActor::sensor` / `::actuator`. The supervised sampling loop,
mailbox-driven command queue, and restart tree land in Phase 2.

## "`cargo build --all-features` fails"

`--all-features` enables `rclrs`, which needs a ROS2 toolchain on the
host. Use `--features full` for the everything-except-`rclrs` surface;
that is what CI's `feature-flags` job does.

## "`import atomr_physical` fails / has no attributes"

The Python facade imports the native extension `_native`. Build it
first: `maturin develop -m crates/py-bindings/Cargo.toml`. See the
`atomr-physical-python` skill.

## Canonical references

- [`docs/architecture.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/architecture.md) — the Phase-2 stub boundaries
- [`docs/ros2-bridge.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/ros2-bridge.md) — the `rclrs` feature
- `crates/core/src/error.rs` — the full `PhysicalError` taxonomy
- `SECURITY.md` — what `SafetyEnvelope` does and does not guarantee
