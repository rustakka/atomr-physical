# atomr-physical

The physical sensing, output, and ROS2-integrated robotics layer of the
[atomr](https://github.com/rustakka/atomr) actor ecosystem. atomr-physical
extends atomr off the screen and into hardware: a sensor is an actor
that publishes readings, an actuator is an actor that drains a command
queue behind a safety envelope, and a robot is the supervisor at the
top of that tree. Every type is a native Rust actor; the Python API is
a first-class overlay.

## Why this design

**A device is an actor.** atomr already gives us mailboxes, supervision
trees, dispatchers, and backpressure. A sensor that owns a sampling
loop, an actuator that serialises a command queue, a robot that
supervises a fleet of both — these are actors, and modelling them as
anything else throws away the substrate.

**Safety belongs at the type boundary.** A `Quantity` carries its
`Unit`. A setpoint passes through a `SafetyEnvelope` before any driver
sees it. The `Sensor` / `Actuator` contract traits keep the hardware
seam explicit and small — a driver is plain async Rust implementing one
trait; the sensing / actuation crates supply the actor, the loop, and
the policy.

**ROS2 is a bridge, not a foundation.** The actor world is
self-contained and builds with no ROS2 toolchain. `atomr-physical-ros2`
maps device actors onto the ROS2 topic graph as a `TopicMap` you can
plan and test offline, and (behind the `rclrs` feature) spin against a
live graph.

**Python is first-class.** The same Rust value types back the Python
overlay — `Quantity`, `Reading`, `Command`, `SafetyEnvelope`,
`RobotModel`, `TopicMap` are the *same* objects across the FFI
boundary, so there is no second implementation to drift.

## The crate stack

```
                    atomr-physical            (umbrella, feature-gated)
                          │
   ┌──────────────┬───────┴────────┬──────────────┐
   │              │                │              │
 ros2          robotics         cli            py-bindings
   │              │                                │
   └──────┬───────┴────────────────────────────────┘
          │
   ┌──────┴───────┐
 sensing      actuation        testkit
   │              │              │
   └──────┬───────┴──────────────┘
          │
        core              (pure data + Device/Sensor/Actuator traits)
          │
        atomr             (the actor runtime — a crates.io dependency)
```

`atomr-physical-core` is the pure-data foundation — no actor-runtime,
hardware, or ROS2 dependency. Everything above it builds on the `atomr`
actor runtime, consumed as a crates.io dependency.

## Getting started

### Rust

```bash
cargo build --workspace
cargo test  --workspace
cargo run   -p atomr-physical-cli -- devices
```

### Python

```bash
maturin develop -m crates/py-bindings/Cargo.toml
python -c "from atomr_physical import Quantity; print(Quantity(1.0, 'rad'))"
```

## Project status

atomr-physical is at **0.1.0**. The workspace structure, device-contract
traits, value types, safety / calibration policies, the offline ROS2
topic-graph plan, and the Python overlay are in place and tested. The
actor-runtime wiring and the `rclrs` live bridge are marked **Phase 2**
in the source — see [architecture.md](architecture.md).

## Documentation map

- [Architecture](architecture.md) — crate stack, the device-actor model, the Phase-2 roadmap.
- [ROS2 bridge](ros2-bridge.md) — the topic-graph mapping and the `rclrs` feature.
- [ROS2 bridge — progress](ros2-bridge-progress.md) — increment-by-increment build status: what's done, in flight, and next.
- [Python API](python-api.md) — the `atomr_physical.*` module map and the native-overlay pattern.
- [Feature matrix](feature-matrix.md) — every feature flag and what it pulls in.
- [Release pipeline](release-pipeline.md) / [Release process](release-process.md) — the release pipeline (currently manual-only).
- [`../README.md`](https://github.com/rustakka/atomr-physical) — repository overview.
- [`../ai-skills/`](https://github.com/rustakka/atomr-physical/tree/main/ai-skills) — skills for AI-assisted coding.
