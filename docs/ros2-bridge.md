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

## The `rclrs` feature (live bridge)

`Ros2Bridge::spin` is the live entry point. Without the `rclrs`
feature it returns `PhysicalError::Ros2Bridge` so callers fail fast
rather than silently no-op:

```rust
// Without the `rclrs` feature:
let err = bridge.spin().await.unwrap_err();
// PhysicalError::Ros2Bridge("rclrs feature not enabled — ...")
```

With the `rclrs` feature enabled (`atomr-physical = { version =
"0.2", features = ["rclrs"] }` — or `--features rclrs` on the
`atomr-physical-ros2` crate directly) the bridge links the
[`rclrs`](https://github.com/ros2-rust/ros2_rust) client library and
implements `spin` against a live graph:

```rust
let handle = bridge.spin().await?;
// publish a Reading on the topic bound to a sensor
handle.publish_reading(&SensorId::from("imu-temp"), &reading)?;
// inspect the live subscriber count for a publisher
let n = handle.subscriber_count(&SensorId::from("imu-temp"))?;
// take the node down cleanly
handle.shutdown().await?;
```

Under the hood `spin` creates an `rclrs::Context` from process env,
builds an `Executor` and a `Node` named by `Ros2Bridge::new`, and
attaches a `DynamicPublisher` for every sensor endpoint and a
`DynamicSubscription` for every actuator endpoint. The executor is
spun on a tokio task and halts via a `futures::oneshot` wired into
`SpinOptions::until_promise_resolved`, so `shutdown()` is responsive.

### Build prerequisites

Because the bridge uses rclrs's **dynamic-message** API, you don't need
colcon-generated Rust message crates — the `message_type` strings in
the `TopicMap` drive publishers / subscriptions at runtime. You **do**
need a sourced ROS 2 environment so the linker finds `librcl` /
`librmw` and so `AMENT_PREFIX_PATH` is populated for runtime
introspection lookups:

```bash
# Build the workspace's rcl + msg subset from source on a non-LTS host:
mkdir -p ~/ros2_jazzy/src && cd ~/ros2_jazzy
vcs import --workers 8 src \
  < <(curl -sSL https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos)
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
colcon build --symlink-install \
  --packages-up-to rcl_action rcl_lifecycle rmw_cyclonedds_cpp \
    rmw_fastrtps_cpp std_msgs sensor_msgs geometry_msgs builtin_interfaces \
    rosgraph_msgs action_msgs rcl_interfaces \
  --cmake-args -DBUILD_TESTING=OFF

# Then build atomr-physical against it:
source ~/ros2_jazzy/install/setup.bash
cargo build -p atomr-physical-ros2 --features rclrs
```

On an LTS host with a binary ROS 2 install, `source
/opt/ros/jazzy/setup.bash` is enough.

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

### Reading → ROS 2 message codec

`Ros2BridgeHandle::publish_reading(sensor_id, reading)` walks the bound
message type at runtime and writes `reading.quantity.value` into the
first floating-point field of the message. This covers the common
shapes — `std_msgs/msg/Float64::data`,
`sensor_msgs/msg/Temperature::temperature`, etc. — without a per-type
codec table.

For multi-field messages (`sensor_msgs/msg/JointState` etc.) the
single-field codec is a placeholder; callers that need richer encoding
fetch the publisher's `DynamicMessageMetadata` via
`rclrs::DynamicMessageMetadata::new` and populate fields by name. This
upgrade slots in alongside the existing `publish_reading` API.
