# atomr-physical

A native Rust **physical-systems layer** — sensing, output (both
low-bandwidth `Command` dispatch and full Sunshine/Moonlight video
projection), and ROS2-integrated robotics — built as a supervised
actor topology on top of [atomr](https://github.com/rustakka/atomr).
atomr-physical extends the atomr actor ecosystem off the screen and
into hardware: a sensor is an actor that publishes readings, an
actuator is an actor that drains a command queue behind a safety
envelope, a projector is an actor that supervises Sunshine subprocesses
and pairs Moonlight clients, and a robot is the supervisor at the top
of that tree. Every type is a native Rust actor; the Python API is a
first-class overlay, not an afterthought.

```rust
use atomr_physical::prelude::*;
use atomr_physical::sensing::{Calibration, SamplingPolicy, SensorActor};
use atomr_physical::actuation::{ActuatorActor, SafetyEnvelope};

// A sensor driver implements the `Sensor` contract trait in plain
// async Rust; `SensorActor` adapts it into a supervised actor with a
// sampling loop and a linear calibration.
let temp = SensorActor::new(driver, SamplingPolicy::default_rate())
    .with_calibration(Calibration { scale: 1.0, offset: -0.5 });

// An actuator gets a safety envelope before anything reaches hardware.
let servo = ActuatorActor::new(servo_driver)
    .with_envelope(SafetyEnvelope::clamping(-1.57, 1.57));

// Direct form: no runtime, hardware-free tests.
let reading = temp.clone().sample().await?;
let ack = servo.clone().dispatch(Command::now(
    ActuatorId::from("joint-0"),
    ControlMode::Position,
    Quantity::new(0.8, Unit::Radian),
)).await?;

// Supervised form: promote to live atomr actors under a system.
let system = ActorSystem::create("robot", Config::reference()).await?;
let temp_ref = temp.spawn(&system, "imu-temp")?;
let servo_ref = servo.spawn(&system, "joint-0")?;
// Subscribe to the periodic sampling loop's broadcast fan-out.
let mut stream = temp_ref.subscribe();
let reading = temp_ref.sample().await?;
let ack = servo_ref.dispatch(Command::now(/* … */)).await?;
```

## Python parity

The Python facade ships the physical layer's value types and device
contract. The native extension `atomr_physical._native` is split into
per-domain submodules — `errors`, `core`, `sensing`, `actuation`,
`robotics`, `ros2` — each mirrored by a thin `.py` facade under
`atomr_physical/`, mirroring the binding convention used by
[atomr](https://github.com/rustakka/atomr) and
[atomr-agents](https://github.com/rustakka/atomr-agents). The package
ships a PEP 561 `py.typed` marker.

### Install

```bash
pip install atomr-physical
```

For an editable workflow against the local checkout:

```bash
pip install maturin
maturin develop -m crates/py-bindings/Cargo.toml
pip install -e ".[dev]"
```

### Plan a robot and its ROS2 graph from Python

```python
from atomr_physical import (
    Joint, RobotModel, SafetyEnvelope, Ros2Endpoint, TopicMap,
)

model = RobotModel()
model.add_joint(Joint("j1", "shoulder_pan", actuator="a1", feedback="s1"))
model.add_joint(Joint("j2", "shoulder_lift", actuator="a2", feedback="s2"))

# Bind each device to a ROS2 endpoint — the bridge plan is built and
# validated offline; `rclrs` drives it against a live graph.
topics = TopicMap()
topics.bind_sensor("s1", Ros2Endpoint.publish(
    "/robot/joint_states", "sensor_msgs/msg/JointState"))
topics.bind_actuator("a1", Ros2Endpoint.subscribe(
    "/robot/joint_cmd", "std_msgs/msg/Float64"))

# The safety envelope enforces the same bounds Rust does.
envelope = SafetyEnvelope.clamping(-1.57, 1.57)
assert envelope.enforce("a1", 3.0) == 1.57
```

The same `SafetyEnvelope`, `Calibration`, `Quantity`, and `Reading`
types back both languages — the Python objects wrap the Rust value
types directly, so there is no second implementation to drift.

## Why a physical layer, in Rust, on actors

Robotics middleware is where careful software goes to acquire
3 a.m. pages. A sensor driver wedges; a command races a feedback read;
an out-of-range setpoint reaches a joint; a ROS2 node restart loses the
device graph. These aren't model problems — they're substrate
problems, and the substrate is exactly where atomr is strong.

**A device is an actor.** A sensor is an actor that owns its sampling
loop and publishes `Reading`s; an actuator is an actor that drains a
command queue behind a `SafetyEnvelope`; a robot is the supervisor at
the top of that tree. A driver fault restarts one subtree, not the
process. The mailbox *is* the command queue — backpressure, ordering,
and supervision come from atomr unchanged.

**Safety belongs at the type boundary.** Quantities carry their `Unit`,
setpoints pass through a `SafetyEnvelope` before a driver sees them,
and the `Sensor` / `Actuator` contract traits keep the hardware seam
explicit. A driver is plain async Rust implementing a small trait; the
sensing / actuation crates supply the actor, the loop, and the policy.

**ROS2 is a bridge, not a foundation.** atomr-physical's actor world is
self-contained and builds with no ROS2 installation. The
`atomr-physical-ros2` crate maps sensor / actuator / robot actors onto
the ROS2 topic graph — a `TopicMap` you can plan and unit-test offline,
and (behind the `rclrs` feature) spin against a live graph. You get the
atomr supervision story *and* ROS2 interop, without one dictating the
other.

**Granular efficiency.** Rust gives deterministic resource use,
zero-cost abstractions, and ownership-as-concurrency-safety —
properties that matter when the actor is driving a motor on a real-time
budget. The whole workspace builds under `cargo check --workspace` in
seconds and ships zero runtime overhead beyond what the actor crate
already pays.

## What's in the box

| Crate | What it does |
|---|---|
| `atomr-physical` | Umbrella facade re-exporting the public surface, feature-flag-driven |
| `atomr-physical-core` | Pure-data foundation: device ids, physical `Quantity` / `Unit`, sensor `Reading`s, actuation `Command`s, the `PhysicalError` taxonomy, and the `Device` / `Sensor` / `Actuator` contract traits |
| `atomr-physical-sensing` | `SensorActor` — adapts a `Sensor` driver into a supervised actor with a `SamplingPolicy` and linear `Calibration` |
| `atomr-physical-actuation` | `ActuatorActor` — adapts an `Actuator` driver into a supervised actor that enforces a `SafetyEnvelope` (clamp or reject) before dispatch |
| `atomr-physical-robotics` | `RobotActor` — the supervisor at the top of a physical system; `Joint`, `RobotModel`, and the kinematic structure a robot exposes |
| `atomr-physical-ros2` | The ROS2 bridge: `Ros2Endpoint`, `TopicMap`, `Ros2Bridge` — maps device actors onto the ROS2 topic graph; `rclrs` feature drives a live graph |
| `atomr-physical-sdr` | **(opt-in)** Software-Defined Radio (HackRF One) as a supervised actor with streaming IQ broadcast and optional SigMF recording. `SdrActor` adapts [`rs-hackrf`](https://crates.io/crates/rs-hackrf) into the actor surface; `SdrActorRef::subscribe()` hands out a `broadcast::Receiver<IqChunk>` of interleaved `ci8_le` samples. The `sdr-sigmf` feature pairs the channel with a [SigMF](https://github.com/sigmf/SigMF) writer for on-disk capture |
| `atomr-physical-projection` | **(opt-in)** `ProjectionActor` — supervised Sunshine/Moonlight orchestration: vkms virtual displays, stride-shifted port windows, `SunshineInstanceActor` subprocess children, `_nvstream._tcp.local.` mDNS broadcast, HTTPS auto-pairing |
| `atomr-physical-projection-client` | **(opt-in)** receiver-side `atomr-projection-client` binary — runs on a Pi / Jetson, browses mDNS, pairs, and execs `moonlight-embedded` |
| `atomr-physical-testkit` | `MockSensor` / `MockActuator` implementing the device-contract traits with in-memory behaviour, for hardware-free tests |
| `atomr-physical-py-bindings` | `atomr_physical._native` PyO3 module — six submodules exposing the value types and device contract to Python |
| `atomr-physical-cli` | `atomr-physical` binary: `devices` / `sense` / `actuate` / `ros2` / `project` / `sdr` subcommands |

Plus a Python facade — `pip install atomr-physical` — that exposes the
same `Quantity` / `Reading` / `Command` / `SafetyEnvelope` /
`RobotModel` / `TopicMap` shapes from Python.

> **Project status.** atomr-physical's Phase 2 has landed: every
> device type has both an offline form (direct `sample` / `dispatch`)
> and a supervised form (`.spawn(system, name)` → typed `*Ref` over a
> mailbox, with `RobotActor` standing up its children under a
> one-for-one `SupervisorStrategy`). The `rclrs` feature now spins a
> real ROS 2 node with dynamic publishers and subscriptions from the
> `TopicMap`. A **projection** output subsystem has landed alongside
> Phase 2 — `atomr-physical-projection` orchestrates Sunshine/Moonlight
> as a supervised atomr actor tree (opt-in via the umbrella's
> `projection` feature so default builds stay free of `reqwest` /
> `mdns-sd`). See [`docs/architecture.md`](docs/architecture.md) and
> [`docs/projection.md`](docs/projection.md) for the lifecycle details.

## Quick start (Rust)

```toml
[dependencies]
# Defaults: sensing + actuation + robotics
atomr-physical = "0.1"

# Add the ROS2 topic-graph bridge and test doubles:
# atomr-physical = { version = "0.1", features = ["ros2", "testkit"] }

# Drive the bridge against a *live* ROS2 graph (requires a ROS2 install):
# atomr-physical = { version = "0.1", features = ["rclrs"] }

# Add Sunshine/Moonlight video projection (pulls reqwest + mdns-sd):
# atomr-physical = { version = "0.1", features = ["projection"] }

# Add the HackRF One SDR actor (streaming IQ broadcast, pulls rs-hackrf):
# atomr-physical = { version = "0.1", features = ["sdr"] }

# Same, with on-disk SigMF capture (adds the SigmfWriter):
# atomr-physical = { version = "0.1", features = ["sdr-sigmf"] }
```

Or pull subsystem crates directly — `atomr-physical-core`,
`atomr-physical-sensing`, `atomr-physical-actuation`,
`atomr-physical-robotics`, `atomr-physical-ros2`,
`atomr-physical-projection`, and `atomr-physical-sdr` are all separate
publishables.

```rust
use std::sync::Arc;
use atomr_physical::prelude::*;
use atomr_physical::sensing::{SamplingPolicy, SensorActor};
use atomr_physical::actor::actor::{ActorSystem, Config};
use atomr_physical_testkit::MockSensor;

# async fn run() -> atomr_physical::core::Result<()> {
// `MockSensor` implements the `Sensor` contract — swap it for a real
// driver and the same code runs unchanged.
let driver = Arc::new(MockSensor::constant("imu-temp", 21.0, Unit::Celsius));
let sensor = SensorActor::new(driver, SamplingPolicy::default_rate());

// Direct form (no runtime — handy in tests).
let reading = sensor.clone().sample().await?;
println!("{} = {}", reading.sensor, reading.quantity);

// Supervised form (live atomr actor under a system).
let system = ActorSystem::create("demo", Config::reference()).await.unwrap();
let sensor_ref = sensor.spawn(&system, "imu-temp").unwrap();
let mut stream = sensor_ref.subscribe();   // periodic readings on a broadcast channel
let reading = sensor_ref.sample().await?;  // or ask-style one-shot reads
# Ok(()) }
```

## Quick start (Python)

```bash
python -m venv .venv && source .venv/bin/activate
pip install atomr-physical
```

```python
from atomr_physical import Quantity, SafetyEnvelope

q = Quantity(0.8, "rad")
print(q.value, q.unit)            # 0.8 rad

envelope = SafetyEnvelope.clamping(-1.57, 1.57)
print(envelope.enforce("joint-0", 3.0))   # 1.57 — clamped to the envelope
```

## ROS2 integration

The `atomr-physical-ros2` crate is the seam onto the ROS2 graph. It is
transport-agnostic and builds with **no ROS2 installation** — you
declare a `TopicMap` binding each device to a `Ros2Endpoint`, and the
plan is inspectable and unit-testable offline. The `rclrs` feature
(Phase 2) links the [`rclrs`](https://github.com/ros2-rust/ros2_rust)
client library and spins the bridge against a live ROS2 graph. See
[`docs/ros2-bridge.md`](docs/ros2-bridge.md).

## Video projection (Sunshine / Moonlight)

The `atomr-physical-projection` crate extends the output surface from
low-bandwidth `Command` dispatch to full **video projection**. A
`ProjectionActor` is a supervised atomr actor tree that hands out
[`vkms`](https://docs.kernel.org/gpu/vkms.html) virtual displays,
spawns supervised `sunshine` subprocesses against stride-shifted port
windows, broadcasts each instance over `_nvstream._tcp.local.`, and
drives the Moonlight pairing handshake via Sunshine's HTTPS API. A
sibling `atomr-physical-projection-client` crate runs on a Pi / Jetson
receiver, browses mDNS for matching services, pairs, and execs
`moonlight-embedded`. The CLI exposes the pipeline as
`atomr-physical project demo` / `atomr-physical project pair`.

Gated behind the umbrella's opt-in `projection` feature so the network
deps (`reqwest`, `mdns-sd`) stay off default builds. See
[`docs/projection.md`](docs/projection.md).

## Documentation map

- [`docs/index.md`](docs/index.md) — documentation hub
- [`docs/architecture.md`](docs/architecture.md) — crate stack, the device-actor model, the Phase-2 roadmap
- [`docs/ros2-bridge.md`](docs/ros2-bridge.md) — the ROS2 topic-graph mapping and the `rclrs` feature
- [`docs/projection.md`](docs/projection.md) — the Sunshine/Moonlight projection subsystem
- [`docs/sdr.md`](docs/sdr.md) — the Software-Defined Radio subsystem (HackRF One)
- [`docs/python-api.md`](docs/python-api.md) — the `atomr_physical.*` module map and the native-overlay pattern
- [`docs/feature-matrix.md`](docs/feature-matrix.md) — every feature flag and what it pulls in
- [`docs/release-pipeline.md`](docs/release-pipeline.md) / [`docs/release-process.md`](docs/release-process.md) — the release pipeline (currently manual-only; see `RELEASING.md`)
- [`ai-skills/`](ai-skills/) — Claude Code / Agent SDK skills for AI-assisted coding against atomr-physical

## AI-assisted development

If you're using Claude Code, Cursor, or another AI coding assistant on
a project that depends on `atomr-physical`, install the
**[ai-skills bundle](ai-skills/)** — skills covering quickstart,
sensing, actuation, robotics, the ROS2 bridge, the Python overlay, and
troubleshooting.

```text
/plugin marketplace add rustakka/atomr-physical
/plugin install atomr-physical-ai-skills@atomr-physical
```

Each `SKILL.md` is a thin router into the canonical docs. Other
harnesses have install instructions in
[`ai-skills/README.md`](ai-skills/README.md).

Companion bundle for the runtime substrate:
[`atomr` ai-skills](https://github.com/rustakka/atomr/tree/main/ai-skills)
— actor design, supervision, persistence, clustering, Python bindings.

## License

Apache-2.0. See [`LICENSE`](LICENSE).
