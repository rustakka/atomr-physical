# The ROS2 bridge

`atomr-physical-ros2` is the seam between atomr-physical's actor world
and the [ROS2](https://docs.ros.org/) graph. It maps device actors onto
ROS2 topics, services, and actions — a sensor's reading stream becomes
a published topic, an actuator's command mailbox becomes a
subscription, and a robot becomes a ROS2 node.

## Design: a bridge, not a foundation

atomr-physical's actor world is **self-contained**. The `core`,
`sensing`, `actuation`, and `robotics` crates know nothing about ROS2,
and the `atomr-physical-ros2` crate itself builds with **no ROS2
installation** — the topic-graph plan is pure Rust data.

This is deliberate. ROS2 is one transport atomr-physical interoperates
with, not the substrate it is built on. You get atomr's supervision,
backpressure, and mailbox semantics *and* ROS2 interop, without one
dictating the other.

## The offline topic plan

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

A `Ros2Endpoint` carries a topic name, a ROS2 message type, and a
direction:

| Direction | Meaning |
|---|---|
| `Publish` | atomr-physical publishes to the topic — sensor readings flow **out** |
| `Subscribe` | atomr-physical subscribes to the topic — commands flow **in** |

Because the plan is plain data, you can build it, serialise it, diff
it, and assert on it in tests without ever touching a DDS layer.

## The `rclrs` feature (Phase 2)

`Ros2Bridge::spin` is the live entry point. At 0.1.0 it returns
`PhysicalError::Ros2Bridge` so callers fail fast rather than silently
no-op:

```rust
// Without the `rclrs` feature:
let err = bridge.spin().await.unwrap_err();
// PhysicalError::Ros2Bridge("rclrs feature not enabled — ...")
```

The `rclrs` feature (Phase 2) links the
[`rclrs`](https://github.com/ros2-rust/ros2_rust) client library and
implements `spin` against a live graph: it creates an `rclrs` node,
wires real publishers / subscriptions from the `TopicMap`, and drives
them on the atomr runtime.

```bash
# Requires a ROS2 installation + rosidl message generation on the host.
cargo build -p atomr-physical-ros2 --features rclrs
# or, through the umbrella:
cargo build -p atomr-physical --features rclrs
```

The feature is off by default so the workspace — and CI — builds on any
host. The live bridge is exercised separately on a ROS2-equipped
runner.

## From Python

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

## Mapping conventions

| atomr-physical | ROS2 |
|---|---|
| `SensorActor` reading stream | a publisher on the bound topic |
| `ActuatorActor` command mailbox | a subscription on the bound topic |
| `RobotActor` | a ROS2 node (named by `Ros2Bridge::new`) |
| `Reading` / `Command` | encoded into / decoded from the bound `message_type` |
| `PhysicalError` from the bridge | surfaced as `PhysicalError::Ros2Bridge` |

Message-type encoding (the `Reading` ↔ `sensor_msgs/...` and `Command`
↔ `std_msgs/...` codecs) lands with the `rclrs` feature in Phase 2.
