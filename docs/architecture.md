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

### `atomr-physical-sdr` — the SDR subsystem

Opt-in (umbrella `sdr` feature) crate that extends the input surface
beyond the single-`Reading` shape into **streaming I/Q**. `SdrActor`
adapts a HackRF One (via [`rs-hackrf`](https://crates.io/crates/rs-hackrf))
into a supervised atomr actor: an `SdrActorRef::subscribe()` hands
out a `tokio::sync::broadcast::Receiver<IqChunk>` carrying interleaved
`ci8_le` samples behind an `Arc<[i8]>` so every subscriber (live
consumer, SigMF writer, future ROS2 bridge) shares the buffer without
copying. With the `sdr-sigmf` umbrella feature, `SigmfWriter` drains
the broadcast channel into a [SigMF](https://github.com/sigmf/SigMF)
pair on disk (`*.sigmf-data` raw + `*.sigmf-meta` JSON) that GNU Radio,
`inspectrum`, and `gqrx` read directly. See [sdr.md](sdr.md).

The SDR subsystem is opt-in by design — its USB / capture deps stay
off default builds. The same two-form contract every other device
actor uses applies: `SdrActor::snapshot(n_samples)` is the offline
form (drives the backend through one start → drain → stop cycle, no
runtime needed); `.spawn(system, name)` promotes it into a live
supervised actor. TX is on the surface but returns `Unsupported` —
`rs-hackrf` 0.4 is RX-only.

### `atomr-physical-projection` — the projection supervisor

Opt-in (umbrella `projection` feature) crate that extends the output
surface beyond `Command` dispatch into full video projection.
`ProjectionActor` is the supervisor at the top of a projection
subtree: it owns a `VkmsDisplayManager` for headless virtual
displays, a `PortAllocator` for stride-shifted Sunshine port windows,
a pool of supervised `SunshineInstanceActor` subprocess children, an
`MdnsBroadcaster` advertising each instance as
`_nvstream._tcp.local.`, and a `ClientProvisioner` driving Sunshine's
HTTPS pairing API. A sibling `atomr-physical-projection-client` crate
ships the receiver-side binary that browses mDNS, pairs, and execs
`moonlight-embedded`. See [projection.md](projection.md).

The projection subsystem is opt-in by design — its network deps
(`reqwest`, `mdns-sd`) stay off default builds. The same two-form
contract every other device actor uses applies:
`ProjectionActor::with_test_offline(true)` plus a `/bin/sleep` Sunshine
binary lets the whole supervised pipeline run hardware-free under CI.

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

The `atomr-physical` binary: `devices`, `sense`, `actuate`, `ros2`,
`project`, `sdr`.

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

## Phase 2 — landed in 0.2.x

The actor-runtime wiring and the `rclrs` live bridge landed alongside
the 0.2.x line. Every type below now has both an **offline** form
(directly callable, no runtime) and a **supervised** form behind a
`.spawn(...)` method that returns a typed `*Ref` handle:

| Surface | Offline (always available) | Supervised (`.spawn(system, …)`) |
|---|---|---|
| `SensorActor` | `sample().await` — direct calibrated read | `SensorActor::spawn` → `SensorActorRef` with `sample()`, `health_check()`, and a `broadcast::Receiver<Reading>` fed by the periodic sampling loop |
| `ActuatorActor` | `dispatch(cmd).await` — envelope + driver | `ActuatorActor::spawn` → `ActuatorActorRef` with `dispatch()` and `health_check()`; commands serialise through the mailbox |
| `RobotActor` | `add_sensor` / `add_actuator` + offline lookups | `RobotActor::spawn` → `RobotActorRef` with `sensor(id)` / `actuator(id)` / `child_ids()`; children spawned in `pre_start` under the supervisor's `OneForOneStrategy`, so a driver fault restarts only the affected subtree |
| `Ros2Bridge` | `topics_mut().bind_*` — offline TopicMap | `Ros2Bridge::spin().await` — behind the `rclrs` feature, stands up a real ROS 2 node with a `DynamicPublisher` per sensor / `DynamicSubscription` per actuator and returns a `Ros2BridgeHandle` for `publish_reading` + `shutdown` |
| `SdrActor` *(opt-in)* | `snapshot(n).await` — one-shot capture, drives the backend through start → drain → stop with no runtime | `SdrActor::spawn` → `SdrActorRef` with `subscribe()` (a `broadcast::Receiver<IqChunk>` of streaming I/Q), `tune()` mid-stream, `start_rx()` / `stop_rx()`, and `transmit()` (currently `Unsupported` on rs-hackrf 0.4) |

### Restart semantics

`RobotRunner::pre_start` spawns each sensor / actuator child via
`SensorActor::spawn_under(ctx, …)` / `ActuatorActor::spawn_under(ctx,
…)`. Children are named `sensor-<id>` / `actuator-<id>` so the
supervisor path stays predictable, and the supervisor's
`SupervisorStrategy` (one-for-one with 10 retries / 60 s by default —
configurable via `RobotActor::with_supervisor_strategy`) decides what
to do on a driver fault.

### The two-form contract

The split is deliberate: the offline form is fast to construct, easy to
test (no `tokio::test` overhead), and gives the Python overlay a
hardware-free surface. The supervised form is the production path —
backpressure, restart, and the broadcast fan-out come from atomr
unchanged. Both share the same configuration types; promoting offline
to supervised is a single `.spawn(system, name)` call.

### The `rclrs` feature shape

`Ros2Bridge` builds with **no ROS 2 toolchain**. Behind the `rclrs`
feature it depends on `rclrs = "0.7"` and uses the dynamic-message API
(no colcon-generated message crates required): each endpoint's
`message_type` string drives a `DynamicPublisher` or
`DynamicSubscription` at runtime. The bridge spins the executor in a
tokio task and halts it via a `futures::oneshot` promise wired into
`SpinOptions::until_promise_resolved` — drop the
[`Ros2BridgeHandle`](https://docs.rs/atomr-physical-ros2/latest/atomr_physical_ros2/struct.Ros2BridgeHandle.html)
or call `.shutdown().await` to take the node down cleanly.

The `dyn_msg` path means the build needs:

- `rcl`, `rmw`, an `rmw_implementation` (`rmw_cyclonedds_cpp` or
  `rmw_fastrtps_cpp`), `rosidl_runtime_c`,
  `rosidl_typesupport_introspection_c` from the ROS 2 install.
- The introspection `.so` for every message type the `TopicMap`
  references (e.g. `libstd_msgs__rosidl_typesupport_introspection_c.so`).
- A sourced setup script — `source $HOME/ros2_jazzy/install/setup.bash`
  for a from-source build, or `source /opt/ros/jazzy/setup.bash` for an
  apt install — so `AMENT_PREFIX_PATH` is populated.

## Dependency on atomr

`atomr-physical` consumes the `atomr` actor runtime (`atomr`,
`atomr-core`, `atomr-macros`, `atomr-telemetry`, `atomr-streams`) as
**crates.io version pins** — never path-deps. The build is
self-contained; CI needs no side-by-side checkout. For local
development against an unreleased `atomr` change, use a
`[patch.crates-io]` override (see `CONTRIBUTING.md`).
