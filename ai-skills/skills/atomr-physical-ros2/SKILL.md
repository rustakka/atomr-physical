---
name: atomr-physical-ros2
description: Use when bridging atomr-physical onto ROS2 — building a Ros2Plan or TopicMap, binding sensors/actuators/services/actions/parameters to endpoints, attaching QoS, validating a plan, registering message codecs, wiring the Model 2 orchestration actors, or enabling the `rclrs` feature to spin against a live ROS2 graph. Triggers on `Ros2Bridge::`, `Ros2Plan::`, `TopicMap::`, `Ros2Endpoint::`, `bind_sensor` / `bind_actuator`, `CodecRegistry`, `Ros2NodeActor`, `MockRos2Transport`, or the `rclrs` feature.
---

# atomr-physical ROS2 bridge

`atomr-physical-ros2` **orchestrates inputs and outputs across the ROS2
graph through idiomatic atomr actor patterns** — it does not reimplement
ROS2. It builds with **no ROS2 installation**; the `rclrs` feature links
the live transport on a ROS 2 Jazzy host.

## The mental model — four layers

1. **Offline plan** — `Ros2Plan` aggregates a `TopicMap` (pub/sub
   endpoints), service endpoints, action endpoints, and parameter
   declarations. Each `Ros2Endpoint` carries a topic, a message type, a
   direction, and an optional `QosProfile`. `Ros2Bridge` owns the plan;
   `validate_plan` / `Ros2Bridge::validate` lint it.
2. **Codec** — the `MessageCodec` trait and the downstream-extensible
   `CodecRegistry` map a `message_type` string to an encode/decode
   implementation. `CodecRegistry::builtin()` ships curated
   structured-payload codecs (pure Rust, available offline).
3. **Transport contract** — `Ros2Event` (inbound) / `Ros2Command`
   (outbound) cross `mpsc` channels; `Ros2Link` is the outbound handle.
   `Ros2Transport` is the seam — the live `RclrsTransport` or the
   in-memory `MockRos2Transport` for tests.
4. **Orchestration (Model 2)** — `Ros2NodeActor` supervises one actor
   per endpoint (`Ros2PublisherActor`, `Ros2SubscriberActor`,
   `Ros2ServiceActor`, `Ros2ActionActor`, `Ros2ParamActor`). The device
   + handler seam (`ReadingSource` / `CommandSink` / `ServiceHandler` /
   `ActionHandler` / `ParamStore`, assembled with `Ros2Wiring`)
   decouples the bridge from the still-scaffolded `SensorActor` /
   `ActuatorActor`.

## Planning a node graph (no ROS2 toolchain needed)

```rust
use atomr_physical::core::{RobotId, SensorId};
use atomr_physical::ros2::{Ros2Bridge, Ros2Endpoint, Ros2ServiceEndpoint, QosProfile};

let mut bridge = Ros2Bridge::new("atomr_physical_node", RobotId::from("arm-1"));

bridge.topics_mut().bind_sensor(
    SensorId::from("shoulder-encoder"),
    Ros2Endpoint::publish("/arm/joint_states", "sensor_msgs/msg/JointState")
        .with_qos(QosProfile::sensor_data()),
);
bridge.plan_mut().add_service(
    Ros2ServiceEndpoint::server("/arm/home", "std_srvs/srv/Trigger"),
);

assert!(bridge.validate().is_empty());   // lint the plan offline
```

## Testing the orchestration offline (`MockRos2Transport`)

The Model 2 actors run against `MockRos2Transport` with no ROS2
toolchain — `inject` an inbound `Ros2Event`, `drain_commands` to see the
`Ros2Command`s the actors emit. Build a `Ros2Wiring` with the device /
handler seam, hand it plus the `Ros2Plan` to `Ros2NodeActor::new`.

## Going live (the `rclrs` feature)

```bash
source /opt/ros/jazzy/setup.bash
cargo build -p atomr-physical --features rclrs
cargo xtask ros2-it          # the rclrs-gated integration tests
```

With `rclrs` enabled, `Ros2Bridge::run` builds the live `RclrsTransport`
and spawns the Model 2 actors on an `ActorSystem`, returning a
`Ros2BridgeHandle`. (`Ros2Bridge::spin` is the old fail-fast shim — use
`run`.) The feature is off by default so the workspace builds anywhere.

## Canonical references

- [`docs/ros2-bridge.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/ros2-bridge.md) — the full specification (layers, codecs, QoS, the ten-increment roadmap)
- `crates/ros2/src/` — `plan.rs`, `codec/`, `transport/`, `actors/`
- [`rclrs`](https://github.com/ros2-rust/ros2_rust) — the ROS2 Rust client library

## Common mistakes

- **Calling `spin` instead of `run`.** `spin` is a fail-fast shim; the
  live entry point is `Ros2Bridge::run(&sys, wiring, codecs)`.
- **Wrong endpoint direction.** A sensor publishes; an actuator
  subscribes. `bind_sensor` + `Ros2Endpoint::subscribe` is a plan error
  `validate` will flag.
- **A plan endpoint with no wiring.** Bind a sensor in the plan but
  forget a `ReadingSource` in the `Ros2Wiring` and the node skips it
  with a warning — wire every endpoint the plan declares.
- **`--all-features` in CI.** That enables `rclrs`, which needs a ROS2
  toolchain. Use `--features full`; the `rclrs` bridge is checked by the
  `workflow_dispatch`-only `rclrs-bridge` CI job.
- **Treating ROS2 as the foundation.** atomr-physical's actor world is
  self-contained — the bridge is one transport, not the substrate.
