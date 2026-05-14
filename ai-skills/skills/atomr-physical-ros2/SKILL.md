---
name: atomr-physical-ros2
description: Use when bridging atomr-physical onto ROS2 — building a TopicMap, binding sensors/actuators to Ros2Endpoints, constructing a Ros2Bridge, or enabling the `rclrs` feature to spin against a live ROS2 graph. Triggers on `Ros2Bridge::`, `TopicMap::`, `Ros2Endpoint::`, `bind_sensor` / `bind_actuator`, or the `rclrs` feature.
---

# atomr-physical ROS2 bridge

`atomr-physical-ros2` maps device actors onto the ROS2 topic graph. It
is **transport-agnostic and builds with no ROS2 installation** — the
topic plan is pure Rust data you assemble and test offline.

## The mental model

- **A bridge is a plan, then (Phase 2) a runtime.** `Ros2Bridge` owns a
  node name and a `TopicMap`. `Ros2Bridge::spin` is the live entry
  point — at 0.1.0 it returns `PhysicalError::Ros2Bridge` unless built
  with the `rclrs` feature.
- **A `TopicMap` binds devices to endpoints.** `bind_sensor` /
  `bind_actuator` associate a `SensorId` / `ActuatorId` with a
  `Ros2Endpoint`.
- **A `Ros2Endpoint` has a direction.** `publish` (sensor readings flow
  *out* to ROS2) or `subscribe` (commands flow *in* from ROS2), plus a
  topic name and a ROS2 message type.

## Planning a node graph (no ROS2 toolchain needed)

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

// Inspect / assert on the plan in a unit test.
assert_eq!(bridge.topics().len(), 2);
```

## Going live (the `rclrs` feature)

```bash
# Needs a ROS2 installation + rosidl message generation on the host.
cargo build -p atomr-physical --features rclrs
```

With `rclrs` enabled (Phase 2), `Ros2Bridge::spin` creates an `rclrs`
node, wires real publishers / subscriptions from the `TopicMap`, and
drives them on the atomr runtime. The feature is off by default so the
workspace builds anywhere.

## Canonical references

- [`docs/ros2-bridge.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/ros2-bridge.md) — the full mapping conventions + the `rclrs` feature
- `crates/ros2/src/lib.rs` — `Ros2Bridge`, `TopicMap`, `Ros2Endpoint`
- [`rclrs`](https://github.com/ros2-rust/ros2_rust) — the ROS2 Rust client library

## Common mistakes

- **Calling `spin` without `rclrs`.** It returns
  `PhysicalError::Ros2Bridge` by design — build with
  `--features rclrs` (and a ROS2 toolchain) for a live graph.
- **Wrong endpoint direction.** A sensor publishes; an actuator
  subscribes. `bind_sensor` + `Ros2Endpoint::subscribe` is almost
  always a mistake.
- **`--all-features` in CI.** That enables `rclrs`, which fails with no
  ROS2 toolchain. Use `--features full` for the no-ROS2 surface.
- **Treating ROS2 as the foundation.** atomr-physical's actor world is
  self-contained — the bridge is one transport, not the substrate.
