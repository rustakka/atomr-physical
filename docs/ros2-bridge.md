# The ROS2 bridge

`atomr-physical-ros2` is the seam between atomr-physical's actor world
and the [ROS2](https://docs.ros.org/) graph. It maps device actors onto
ROS2 topics, services, actions, and parameters — a sensor's reading
stream becomes a published topic, an actuator's command mailbox becomes
a subscription, and a robot becomes a ROS2 node.

This document is the **specification** for the bridge. It describes the
layered design, the message-codec model, every ROS2 aspect the bridge
orchestrates, the actor topology, the testing strategy, and the staged
roadmap. Sections marked _(roadmap)_ describe work that lands across the
implementation increments in [§11](#11-phasing--roadmap); the offline
planning surface ([§2](#2-the-offline-topic-plan)) is available today.

## Design: a bridge, not a foundation

atomr-physical's actor world is **self-contained**. The `core`,
`sensing`, `actuation`, and `robotics` crates know nothing about ROS2,
and the `atomr-physical-ros2` crate itself builds with **no ROS2
installation** — the topic-graph plan is pure Rust data.

This is deliberate. ROS2 is one transport atomr-physical interoperates
with, not the substrate it is built on. The bridge **orchestrates
inputs and outputs** across the ROS2 graph — it does not reimplement
ROS2. You get atomr's supervision, backpressure, and mailbox semantics
*and* ROS2 interop, without one dictating the other.

### Hard invariants

The bridge holds to four invariants. Every change to this crate is
checked against them.

1. **Offline-buildable.** The workspace builds and unit-tests with **no
   ROS2 toolchain** when the `rclrs` feature is off. The offline
   planning layer, the codec trait + registry, and the orchestration
   actor seam are pure Rust, mock-testable without DDS.
2. **crates.io pins.** atomr deps stay published-crate pins
   (`atomr*` = `0.9.2`), never path deps.
3. **Opt-in CI.** CI never builds `--all-features` (that would pull
   `rclrs`); it builds `--features full`. The live bridge is checked
   separately on a ROS2-equipped runner.
4. **In-repo.** All bridge code lives in this repo; it consumes the
   atomr runtime as a published crate.

## 1. Layered architecture

The bridge is four layers. Everything except the transport-core
internals and the concrete `rosidl`-typed codecs compiles with the
`rclrs` feature **off**.

```
┌────────────────────────────────────────────────────────────────┐
│ L4 Orchestration — atomr actors (per-endpoint, Model 2)  SEAM   │
│    Ros2NodeActor supervises one actor per endpoint:             │
│    Publisher / Subscriber / Service / Action / Param actors     │
├────────────────────────────────────────────────────────────────┤
│ L3 Codec — MessageCodec trait + CodecRegistry          OFFLINE  │
│    encode Reading→wire, decode wire→Command, srv/action payloads│
│    (concrete rosidl-typed codecs are rclrs-GATED)               │
├────────────────────────────────────────────────────────────────┤
│ L2 Transport contract — Ros2Event / Ros2Command /      OFFLINE  │
│    Ros2Link / Ros2Transport trait (mpsc channel shapes)         │
├────────────────────────────────────────────────────────────────┤
│ L1 Transport core — owns the rclrs Context/Node/        GATED   │
│    executor in a dedicated tokio task (io::manager idiom)       │
└────────────────────────────────────────────────────────────────┘
```

| Tag | Meaning |
|---|---|
| **OFFLINE** | Compiles and unit-tests with no `rclrs`, no ROS2 toolchain. |
| **GATED** | Entire module is behind `#[cfg(feature = "rclrs")]`. |
| **SEAM** | The actor *types and trait objects* are offline (mock-testable); only the concrete `rclrs` binding is gated. |

**Why the split.** The channel enums, the codec trait, and the actor
logic are pure data and pure functions — they encode, route, and
round-trip without DDS. Only the code that *constructs `rclrs` messages
and spins the node* sits behind the feature. This keeps L4 testable
offline against a `MockRos2Transport`.

### L1 — transport core _(roadmap)_

The `rclrs` `Context`, `Node`, every publisher / subscription / service
/ action, and the executor live in a single `tokio::spawn`ed
`run_ros2(...)` task — never touched by an actor directly. It follows
atomr's own `io::manager` idiom (`TcpManager::spawn() -> (handle,
Receiver<IoEvent>)`): it is fed an `mpsc::UnboundedReceiver<Ros2Command>`
and emits an `mpsc::UnboundedSender<Ros2Event>`. The `rclrs` executor is
driven cooperatively (`spin_once` inside a `select!` loop) so it
co-exists with the tokio runtime atomr already uses.

When the `rclrs` feature is off, the L1 module is `#[cfg]`-ed out
entirely; L2, L3, and L4 still compile.

### L2 — transport contract

Two enums define the wire between the transport task and the
orchestration actors. They are pure data — no `rclrs` types leak across
this boundary.

```rust
/// Inbound: things that happened on the ROS2 graph, pushed toward actors.
pub enum Ros2Event {
    NodeReady      { node_name: String },
    Inbound        { actuator: ActuatorId, topic: String, command: Command },
    ServiceRequest { service: String, request_id: ReqId, payload: Ros2Payload },
    ActionGoal     { action: String, goal_id: GoalId, payload: Ros2Payload },
    ActionCancel   { action: String, goal_id: GoalId },
    ParamChanged   { name: String, value: ParamValue },
    DecodeError    { endpoint: String, detail: String },   // data error, non-fatal
    EndpointFault  { endpoint: String, detail: String },
    Closed         { reason: Option<String> },
}

/// Outbound: things actors want done on the ROS2 graph.
pub enum Ros2Command {
    Publish         { sensor: SensorId, reading: Reading },
    ServiceResponse { request_id: ReqId, payload: Ros2Payload },
    CallService     { service: String, request_id: ReqId, payload: Ros2Payload },
    ActionFeedback  { goal_id: GoalId, payload: Ros2Payload },
    ActionResult    { goal_id: GoalId, payload: Ros2Payload },
    SetParam        { name: String, value: ParamValue },
    Shutdown,
}
```

`Ros2Bridge::spin` hands back a `Ros2Link` (a `Ros2Command` sender plus
the transport task's `JoinHandle`) and the `Ros2Event` receiver. The
`Ros2Transport` trait is the seam both the `rclrs` transport and the
in-memory `MockRos2Transport` implement.

### L3 — codec layer

A `message_type` string like `"sensor_msgs/msg/JointState"` cannot
conjure a typed publisher: `rosidl_generator_rs` generates Rust message
structs **statically**, per interface package, at build time. The codec
layer bridges that gap with a registry.

```rust
/// Opaque wire payload. Offline: a tagged serde value capturing field
/// layout (round-trippable in tests). With `rclrs`: wraps a concrete
/// rosidl-generated type.
pub struct Ros2Payload { /* ... */ }

/// Downstream-extensible: external crates with their own rosidl-generated
/// message packages `impl MessageCodec` and `register()` them.
pub trait MessageCodec: Send + Sync {
    fn message_type(&self) -> &str;
    fn encode_reading(&self, e: &Ros2Endpoint, r: &Reading)     -> Result<Ros2Payload>;
    fn decode_command(&self, e: &Ros2Endpoint, p: &Ros2Payload) -> Result<Command>;
    fn encode_payload(&self, value: &CodecValue) -> Result<Ros2Payload>;  // srv/action
    fn decode_payload(&self, p: &Ros2Payload)    -> Result<CodecValue>;
}

pub struct CodecRegistry { /* HashMap<String, Arc<dyn MessageCodec>> */ }
impl CodecRegistry {
    pub fn builtin() -> Self;                                  // curated set, §3
    pub fn register(&mut self, codec: Arc<dyn MessageCodec>);   // public extension point
    pub fn get(&self, message_type: &str) -> Option<&Arc<dyn MessageCodec>>;
}
```

The `MessageCodec` trait and `CodecRegistry` are public from 0.1 — the
registry is **downstream-extensible**. The concrete `rosidl`-typed
codecs (the curated builtin set) are `#[cfg(feature = "rclrs")]`. Encode
and decode failures collapse into the crate-local `Ros2Error`, which is
surfaced as `PhysicalError::Ros2Bridge`.

### L4 — orchestration actors

The orchestration layer is built on atomr actors. `SensorActor` and
`ActuatorActor` are currently plain structs ("Phase 2 wires them into
atomr's `Actor` trait"); the bridge does **not** block on that work — it
ships against an interim seam:

```rust
#[async_trait] pub trait ReadingSource {
    fn id(&self) -> SensorId;
    async fn next_reading(&self) -> Result<Reading>;
}
#[async_trait] pub trait CommandSink {
    fn id(&self) -> ActuatorId;
    async fn deliver(&self, c: Command) -> Result<CommandAck>;
}
```

Today, thin adapters wrap `SensorActor::sample` / `ActuatorActor::
dispatch`. When the Phase-2 actor wiring lands, the adapters swap to
talk to `ActorRef` mailboxes — the bridge code is unchanged. This
decouples the two roadmaps.

## 2. The offline topic plan

A `TopicMap` binds each device to a `Ros2Endpoint`:

```rust
use atomr_physical::core::{ActuatorId, RobotId, SensorId};
use atomr_physical::ros2::{Ros2Bridge, Ros2Endpoint, TopicMap};

let mut bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("arm-1"));

bridge.topics_mut().bind_sensor(
    SensorId::from("shoulder-encoder"),
    Ros2Endpoint::publish("/arm/joint_states", "sensor_msgs/msg/JointState"),
);
bridge.topics_mut().bind_actuator(
    ActuatorId::from("shoulder-servo"),
    Ros2Endpoint::subscribe("/arm/joint_cmd", "std_msgs/msg/Float64"),
);

// The plan is inspectable and unit-testable with no ROS2 toolchain.
assert_eq!(bridge.topics().len(), 2);
```

A `Ros2Endpoint` carries a topic name, a ROS2 message type, a
direction, and an optional [`QosProfile`](#4-qos):

| Direction | Meaning |
|---|---|
| `Publish` | atomr-physical publishes to the topic — sensor readings flow **out** |
| `Subscribe` | atomr-physical subscribes to the topic — commands flow **in** |

Because the plan is plain data, you can build it, serialise it, diff it,
and assert on it in tests without ever touching a DDS layer. The full
plan — topics, services, actions, and parameters — is aggregated by
`Ros2Plan`; `TopicMap` is the topic slice of it, kept as a distinct
type for the common topic-only case.

## 3. Message mapping & codecs

The bridge maps atomr-physical's `Reading` / `Command` value types onto
ROS2 message types through the [codec layer](#l3--codec-layer).

| atomr-physical | ROS2 | Notes |
|---|---|---|
| `Reading { quantity, frame, timestamp_ms }` | a published message | `frame` → `header.frame_id`, `timestamp_ms` → `header.stamp` where the type has a header |
| `Command { setpoint, mode, issued_ms }` | a subscribed message decoded into a command | `mode` selects the target field for multi-field command types |
| `Quantity { value, unit }` | a scalar or vector field | the `Unit` is **checked**, not coerced — a mismatch is a `Ros2Error` |

### Unit compatibility

Each codec declares which `Unit`s it accepts. The `Unit` ↔ message-type
table (`codec/unit_map.rs`) is pure data and unit-tested offline. For
example `std_msgs/msg/Float64` accepts any scalar `Unit`;
`sensor_msgs/msg/Temperature` requires `Unit::Celsius`;
`geometry_msgs/msg/Twist` requires `MetrePerSecond` / `RadianPerSecond`
components.

### One reading per message vs many

Most codecs map **one `Reading` to one message**. Some ROS2 message
types aggregate multiple degrees of freedom —
`sensor_msgs/msg/JointState` carries a vector of joint positions. Its
codec aggregates several `Reading`s into one message, ordered against
`RobotModel.joints`. The `MessageCodec` trait supports both shapes; a
codec declares its arity.

### Curated builtin set (0.1)

Shipped behind the `rclrs` feature, pre-registered by
`CodecRegistry::builtin()`:

- **Topics:** `std_msgs/msg/Float64`, `sensor_msgs/msg/Temperature`,
  `sensor_msgs/msg/JointState`, `geometry_msgs/msg/Twist`,
  `std_msgs/msg/Float64MultiArray`, `sensor_msgs/msg/BatteryState`
- **Services:** `std_srvs/srv/Trigger`, `std_srvs/srv/SetBool`

Downstream crates that compile their own `rosidl`-generated message
packages add codecs through `CodecRegistry::register()`.

## 4. QoS

Every endpoint carries an optional `QosProfile` — pure data, mirroring
the ROS2 QoS settings the bridge actually uses:

```rust
pub struct QosProfile {
    pub reliability: Reliability,   // Reliable | BestEffort
    pub durability:  Durability,    // Volatile | TransientLocal
    pub history:     History,       // KeepLast | KeepAll
    pub depth:       u32,
}
```

Per-direction defaults follow ROS2 convention: sensor publishers default
to best-effort / keep-last (sensor data), command subscriptions default
to reliable / keep-last. Under [Model 2](#l4--orchestration-actors), QoS
is per-endpoint — it maps onto each endpoint actor's mailbox
configuration.

## 5. Services & actions _(roadmap)_

### Services

A ROS2 service is request/response — it maps onto atomr's `ask` pattern.

- **Server role:** a `Ros2ServiceActor` receives a `Ros2Event::
  ServiceRequest`, `ask`s a handler actor, and replies with a
  `Ros2Command::ServiceResponse` carrying the response payload.
- **Client role:** a `Ros2ServiceActor` `ask`-ed by an atomr actor emits
  a `Ros2Command::CallService` and routes the matching `ServiceResponse`
  back to the waiting `oneshot`.

`Ros2ServiceEndpoint` carries the service name, the service type, and a
`ServiceRole { Server, Client }`.

### Actions

A ROS2 action is goal / feedback / result — it maps onto `ask` plus a
feedback stream channel. atomr-physical's core has no "goal" concept, so
the action payload types (`ActionGoal`, `ActionFeedback`,
`ActionResult`) are **ros2-crate-local and generic**: they carry opaque
codec payloads and are delegated to a user-supplied handler actor. The
bridge orchestrates the goal lifecycle (accept, feedback, result,
cancel); it does not interpret the action semantics.

`Ros2ActionEndpoint` carries the action name, the action type, and a
role. `GoalId` identifies an in-flight goal across the feedback stream.

## 6. Parameters _(roadmap)_

A `Ros2ParamActor` mirrors atomr-physical configuration —
`SamplingPolicy`, `SafetyEnvelope`, `Calibration` — as ROS2 parameters,
and applies external parameter changes back onto the running actors
(read-write). `Ros2ParamDecl` declares a parameter's name, type, and
default; `ParamValue` is the pure-data value type that crosses the L2
boundary.

## 7. Time & clock

ROS2 message headers carry the authoritative timestamp on the wire. The
bridge is **not** a time source — it reads `Reading.timestamp_ms` and
writes it into `header.stamp`, or stamps from a configured clock. A
`Ros2ClockSource { Wall, RosTime, SimTime }` config enum selects the
clock; the default is `Wall` (matching ROS2's `use_sim_time = false`).

## 8. Orchestration topology

Two interaction models were considered for wiring atomr actors to the
ROS2 graph. The implementation uses **Model 2**.

### Model 2 — per-endpoint actors (used)

`Ros2NodeActor` is the supervisor, one per robot/node. It owns the
`Ros2Link` and the `Ros2Plan`; in `pre_start` it spawns one child actor
per endpoint and builds a topic→child routing table, refreshed via
`watch()` / `on_terminated`:

```
ActorSystem
└── Ros2NodeActor                       OneForOneStrategy + per-child decider
    ├── Ros2PublisherActor  (/arm/joint_states)   ← ReadingSource
    ├── Ros2SubscriberActor (/arm/joint_cmd)      → CommandSink
    ├── Ros2ServiceActor    (/arm/home)           ask ⇄ handler actor
    ├── Ros2ActionActor     (/arm/follow_traj)    goal/feedback/result
    └── Ros2ParamActor      (declared params)     config mirror
```

Outside the actor tree: a tiny `tokio::spawn` inbound pump drains the
`Ros2Event` receiver and `tell`s the `Ros2NodeActor`; the L1 transport
task owns the `rclrs` node.

- **Reading out:** a `ReadingSource` `tell`s the matching
  `Ros2PublisherActor` → `Ros2Command::Publish` on the link → the
  transport task encodes via the registry → `rclrs` publisher.
- **Command in:** an `rclrs` subscription callback → decode → `Ros2Event
  ::Inbound` → inbound pump → `Ros2NodeActor` routes by topic →
  `Ros2SubscriberActor` → `pipe_to(sink.deliver(cmd), self)` → the
  `CommandAck` returns to that actor only.

Model 2 makes the [mapping table](#10-mapping-conventions) *literally
the actor graph*. It gives per-endpoint supervision and failure
isolation — a flapping actuator restarts alone, and
`OneForOneStrategy::with_decider` can `Resume` on
`PhysicalError::OutOfRange` while it `Escalate`s on a lost link. Because
the L1 channels are unbounded, backpressure lives at the edges:
per-endpoint mailboxes isolate a slow consumer to its own topic.

### Model 1 — link/pump actors (considered, not used)

Two fixed actors — `Ros2InboundActor` (fed `Ros2Event`s by a pump
draining the receiver) and `Ros2OutboundActor` (holding the
`Ros2Command` sender) — fan messages to and from device actors via an
internal routing `HashMap`. This is the smallest possible actor count
and mirrors atomr's own `usb-link-probe` example. It was **not** chosen:
one slow actuator head-of-line-blocks all inbound traffic, supervision
is coarse (a restart kills every topic), and QoS would be global rather
than per-endpoint. Model 1 remains a reasonable fallback if the
per-endpoint actor count ever becomes a concern.

## 9. The `rclrs` feature

`Ros2Bridge::spin` is the live entry point.

```rust
// Without the `rclrs` feature — fail fast, do not silently no-op:
let err = bridge.spin(/* ... */).await.unwrap_err();
// PhysicalError::Ros2Bridge("rclrs feature not enabled — ...")
```

With the `rclrs` feature, `spin` builds the L1 transport task, spawns
the `Ros2NodeActor` (and its per-endpoint children) onto an
`ActorSystem`, wires them to the channels, and returns a running
`Ros2BridgeHandle`. The feature targets **ROS 2 Jazzy**.

```bash
# Requires a ROS2 installation + rosidl message generation on the host.
source /opt/ros/jazzy/setup.bash
cargo build -p atomr-physical-ros2 --features rclrs
# or, through the umbrella:
cargo build -p atomr-physical --features rclrs
```

The feature is off by default so the workspace — and CI — builds on any
host. The live bridge is exercised separately on a ROS2-equipped runner
(the `workflow_dispatch`-only `rclrs-bridge` CI job).

### `rclrs` dependency form

`rclrs` ships on crates.io — the pin is
`rclrs = { version = "0.7", optional = true }`, activated by
`rclrs = ["dep:rclrs"]`. That settles the form question (crates.io
release, not a git rev or a vendored checkout).

But the pin alone does **not** make `--features rclrs` build on an
arbitrary host. `rclrs` pulls `rosidl_runtime_rs`, whose build script
hard-fails without a sourced ROS 2 environment:

```
AMENT_PREFIX_PATH environment variable not set —
please source ROS 2 installation first.
```

So enabling the dependency is itself a ROS 2 Jazzy-host step: off-host,
`cargo build --features rclrs` cannot resolve the build, by `rclrs`'s
own construction. Three consequences shape the crate layout:

- **`crates/ros2/Cargo.toml` declares the optional deps.**
  `rclrs = ["dep:rclrs", …]` activates `rclrs` 0.7 plus the curated
  message crates (`std_msgs`, `sensor_msgs`, `geometry_msgs`,
  `std_srvs`, each with its `serde` feature). With the `rclrs` feature
  **off**, none of them enter the dependency graph, so invariant 1
  (offline-buildable) holds.
- **The `[patch]` table is a host-local, uncommitted change.**
  `.cargo/config.toml` is a tracked file that carries only `[alias]` in
  version control. The `[patch.crates-io]` block redirecting the
  `rclrs` / message-crate coordinates to the `/opt/ros/<distro>/share/…/rust`
  and `~/ros2_rust_ws/install/…/rust` paths is a **working-tree
  modification applied on the ROS 2 host and never committed** — the
  paths are host-specific. It is mirrored from the
  `colcon-ros-cargo`-generated `~/ros2_rust_ws/.cargo/config.toml`.
- **Typed message structs are not crates.io dependencies.**
  `std_msgs`, `sensor_msgs`, and friends are generated by
  `rosidl_generator_rs` at colcon build time, per interface package.
  The live transport and the concrete builtin codecs are therefore
  written *inside* a colcon workspace sourced against ROS 2 Jazzy —
  which is why the curated codec set ([§3](#3-message-mapping--codecs))
  ships as **structured-payload** codecs offline and the `rosidl`-typed
  materialisation is the host-side increment.

## 10. Mapping conventions

| atomr-physical | ROS2 |
|---|---|
| `SensorActor` reading stream | a publisher on the bound topic (`Ros2PublisherActor`) |
| `ActuatorActor` command mailbox | a subscription on the bound topic (`Ros2SubscriberActor`) |
| `RobotActor` | a ROS2 node (`Ros2NodeActor`, named by `Ros2Bridge::new`) |
| an `ask`-style request/response handler | a ROS2 service (`Ros2ServiceActor`) |
| a goal/feedback/result handler actor | a ROS2 action (`Ros2ActionActor`) |
| `SamplingPolicy` / `SafetyEnvelope` / `Calibration` | ROS2 parameters (`Ros2ParamActor`) |
| `Reading` / `Command` | encoded into / decoded from the bound `message_type` by a `MessageCodec` |
| `PhysicalError` from the bridge | surfaced as `PhysicalError::Ros2Bridge` |

## 11. Phasing & roadmap

The bridge is built in ten increments. **Increments 1–4 are offline** —
they need no ROS2 toolchain and are exercised by the standard CI jobs.
**Increments 5–10 touch the `rclrs` feature** and need a ROS 2 Jazzy
host.

| # | Increment | Feature gate |
|---|---|---|
| 1 | Module restructure + QoS + clock + validation + error | offline |
| 2 | Service / action / param endpoint types + `Ros2Plan` | offline |
| 3 | Codec layer — `MessageCodec` trait + extensible registry | offline |
| 4 | Model 2 orchestration actors + device seam + mock transport | offline |
| 5 | Transport core, topics live | `rclrs` |
| 6 | Concrete builtin codecs (topics) | `rclrs` |
| 7 | Services live | `rclrs` |
| 8 | Parameters live | `rclrs` |
| 9 | Actions live | `rclrs` |
| 10 | CLI live paths + Python bindings + end-to-end + docs | `rclrs` |

Increments 1–4 do **not** depend on the `SensorActor` / `ActuatorActor`
Phase-2 actor wiring — the interim device seam ([§1, L4](#l4--orchestration-actors))
absorbs that. The two roadmaps proceed in parallel.

The offline increments are complete: the planning surface, the codec
layer with its curated structured-payload codecs, the transport
contract, the `MockRos2Transport`, and the full Model 2 actor graph
(`Ros2NodeActor` plus the publisher / subscriber / service / action /
parameter actors) are all built and tested with no ROS2 toolchain.
Increments 5–9 are the live `rclrs` wiring — the `transport/rclrs.rs`
module structure and the `rclrs_integration.rs` scaffold are in place
and reviewed offline; completing them is a ROS 2 Jazzy-host step (see
[§9, `rclrs` dependency form](#rclrs-dependency-form)).

## 12. Testing & verification

Testing is layered to match the architecture.

- **Offline unit tests** (no toolchain) — the planning types, QoS and
  clock data, every validation lint, the codec registry and `Unit`
  table, service/action/param serde and `Ros2Plan` assembly. Run by
  `cargo test --workspace`.
- **Actor-level tests** (no toolchain) — the full Model 2 data flow
  against `MockRos2Transport`, built behind the `crates/ros2` `mock`
  feature. `MockRos2Transport.published()` records outbound,
  `MockRos2Transport.inject()` queues inbound — mirroring
  `MockActuator::log()` / `MockSensor`. Run by
  `cargo test -p atomr-physical-ros2 --features mock`.
- **Integration tests** behind `rclrs` (ROS 2 Jazzy host) — a gated
  `crates/ros2/tests/rclrs_integration.rs`: transport loopback, builtin
  codec field round-trips, live service / parameter / action
  round-trips, and `Ros2Bridge::spin` driving a `MockSensor`-backed
  bridge that a real `ros2` subscriber observes. Invoked via
  `cargo xtask ros2-it` and the `workflow_dispatch`-only `rclrs-bridge`
  CI job.

### Offline verification

```bash
cargo test -p atomr-physical-ros2
cargo test -p atomr-physical-ros2 --features mock
cargo test --workspace
cargo build -p atomr-physical-ros2          # must build with NO rclrs
cargo run -p atomr-physical-cli -- ros2 plan arm-1
cargo run -p atomr-physical-cli -- ros2 validate arm-1
cargo run -p atomr-physical-cli -- ros2 codecs
```

### `rclrs`-gated verification

```bash
source /opt/ros/jazzy/setup.bash
cargo build -p atomr-physical-ros2 --features rclrs
cargo xtask ros2-it
cargo run -p atomr-physical-cli --features rclrs -- ros2 spin arm-1 &
ros2 topic list && ros2 topic echo /arm/joint_states
```

## 13. From Python

The topic plan is fully available through the Python overlay:

```python
from atomr_physical import Ros2Endpoint, TopicMap

topics = TopicMap()
topics.bind_sensor("shoulder-encoder", Ros2Endpoint.publish(
    "/arm/joint_states", "sensor_msgs/msg/JointState"))
topics.bind_actuator("shoulder-servo", Ros2Endpoint.subscribe(
    "/arm/joint_cmd", "std_msgs/msg/Float64"))

assert topics.len == 2
assert topics.sensor_endpoint("shoulder-encoder").direction == "publish"
```

The offline planning surface — `QosProfile`, the service / action /
param endpoint types, `Ros2Plan`, and a read-only view of the
`CodecRegistry` — is exposed to Python as those increments land. The
live transport (`Ros2Bridge::spin`, the codecs, everything in the
transport layer) stays Rust-only; a future Python live-spin belongs with
the broader `pyo3-async-runtimes` work described in
[`python-api.md`](./python-api.md).
