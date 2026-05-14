# Architecture

atomr-physical is a layered Rust workspace: a pure-data foundation, two
device-direction crates, an orchestration crate, a ROS2 bridge, a
Python binding, and a CLI — all built on the
[atomr](https://github.com/rustakka/atomr) actor runtime.

## Layers

### `atomr-physical-core` — the contract

The foundation carries **no** actor-runtime, hardware, or ROS2
dependency. It is pure data plus three contract traits:

- **Identifiers** — `DeviceId`, `SensorId`, `ActuatorId`, `RobotId`,
  `JointId`. String newtypes: cheap to clone, stable across
  serialization, impossible to mix up at a call site.
- **Quantities** — `Unit` (an SI-aligned enum) + `Quantity`
  (value + unit). Physical magnitudes never cross a public API as a
  bare `f64`.
- **Messages** — `Reading` / `ReadingBatch` (sensor side), `Command` /
  `CommandAck` / `ControlMode` (actuator side).
- **Errors** — the `PhysicalError` taxonomy + `Result` alias.
- **The device contract** — `Device` (descriptor + health check),
  `Sensor` (`async fn read`), `Actuator` (`async fn apply`). A hardware
  driver implements one of these in plain async Rust. **This is the
  only seam a driver author touches.**

### `atomr-physical-sensing` / `atomr-physical-actuation` — the device actors

Each crate owns one direction of data flow and adapts a contract-trait
driver into a supervised actor:

- **`SensorActor`** wraps an `Arc<dyn Sensor>` with a `SamplingPolicy`
  (`FixedRate { period_ms }` or `OnDemand`) and a linear `Calibration`
  (`corrected = raw * scale + offset`). `SensorActor::sample` takes one
  calibrated reading.
- **`ActuatorActor`** wraps an `Arc<dyn Actuator>` with an optional
  `SafetyEnvelope`. `ActuatorActor::dispatch` runs the envelope check —
  clamp into `[min, max]` or reject with `PhysicalError::OutOfRange` —
  *before* the driver sees the command.

Both crates re-export the atomr runtime as `actor` so downstream code
has one import path for it.

### `atomr-physical-robotics` — the supervisor

`RobotActor` is the supervisor at the top of a physical system. It owns
a `RobotModel` (the kinematic structure — `Joint`s pairing an actuator
with an optional feedback sensor, plus auxiliary sensors) and a set of
child `SensorActor`s / `ActuatorActor`s keyed by id.

### `atomr-physical-ros2` — the bridge

Maps the actor graph onto the ROS2 topic graph. `TopicMap` binds each
device to a `Ros2Endpoint` (topic name + message type + direction);
`Ros2Bridge` owns the node name and topic plan. The crate is
transport-agnostic and builds with no ROS2 installation — see
[ros2-bridge.md](ros2-bridge.md).

### `atomr-physical-testkit` — the test doubles

`MockSensor` (replays a script of `Quantity` values) and `MockActuator`
(records every `Command`) implement the contract traits with in-memory
behaviour. `sensing`, `actuation`, and `robotics` carry it as a
dev-dependency.

### `atomr-physical-py-bindings` / `python/atomr_physical` — the Python overlay

A PyO3 `cdylib` (`atomr_physical._native`) with one submodule per Rust
crate, plus a thin pure-Python facade per submodule. See
[python-api.md](python-api.md).

### `atomr-physical-cli` — the operator surface

The `atomr-physical` binary: `devices`, `sense`, `actuate`, `ros2`.

## The device-actor model

```
   ┌────────────────────────────────────────────────────┐
   │                   RobotActor                       │   supervisor
   │  ┌──────────────┐            ┌──────────────────┐   │
   │  │ SensorActor  │  Reading   │  ActuatorActor   │   │
   │  │  ┌────────┐  │ ─────────▶ │  ┌────────────┐  │   │
   │  │  │Calib.  │  │            │  │SafetyEnvel.│  │   │
   │  │  └────────┘  │            │  └────────────┘  │   │
   │  │  ┌────────┐  │            │  ┌────────────┐  │   │
   │  │  │Sampling│  │ ◀───────── │  │ Command Q  │  │   │
   │  │  └────────┘  │  Command   │  └────────────┘  │   │
   │  └──────┬───────┘            └────────┬─────────┘   │
   └─────────┼─────────────────────────────┼─────────────┘
             │ impl Sensor                 │ impl Actuator
       ┌─────▼──────┐                ┌─────▼──────┐
       │  driver    │   (hardware)   │  driver    │
       └────────────┘                └────────────┘
```

A driver is plain async Rust. The device crates supply the actor, the
loop, the policy, and the supervision.

## Phase-2 roadmap

atomr-physical 0.1.0 ships the structure, the contract, the value
types, and the policies — all compiled and tested. The actor-runtime
wiring is scaffolded with explicit `Phase 2` markers in the source:

| Marker | Lands |
|---|---|
| `SensorActor` as a live atomr `Actor` | the sampling loop runs under supervision; `Reading`s flow over a mailbox / event channel |
| `ActuatorActor` as a live atomr `Actor` | commands arrive over a mailbox; the queue drains under supervision |
| `RobotActor` as a supervisor | each child sensor / actuator runs supervised; a driver fault restarts only its subtree |
| `Ros2Bridge::spin` behind the `rclrs` feature | the bridge creates an `rclrs` node and wires real publishers / subscriptions from the `TopicMap` |

The seam is deliberate: today's types are usable directly (a
`SensorActor` exposes `sample`, an `ActuatorActor` exposes `dispatch`),
so callers and the Python overlay can be built against the API ahead of
the supervision wiring.

## Dependency on atomr

`atomr-physical` consumes the `atomr` actor runtime (`atomr`,
`atomr-core`, `atomr-macros`, `atomr-telemetry`, `atomr-streams`) as
**crates.io version pins** — never path-deps. The build is
self-contained; CI needs no side-by-side checkout. For local
development against an unreleased `atomr` change, use a
`[patch.crates-io]` override (see `CONTRIBUTING.md`).
