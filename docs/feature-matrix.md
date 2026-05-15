# Feature matrix

Every feature flag on the `atomr-physical` umbrella crate, what it
pulls in, and when to enable it.

## Umbrella features (`atomr-physical`)

| Feature | Default | Pulls in | Enable when… |
|---|:---:|---|---|
| `sensing` | ✓ | `atomr-physical-sensing` | you read from sensors (`SensorActor`, `Calibration`, `SamplingPolicy`) |
| `actuation` | ✓ | `atomr-physical-actuation` | you drive actuators (`ActuatorActor`, `SafetyEnvelope`) |
| `robotics` | ✓ | `atomr-physical-robotics` (+ `sensing`, `actuation`) | you orchestrate a robot (`RobotActor`, `RobotModel`, `Joint`) |
| `ros2` | — | `atomr-physical-ros2` (+ `robotics`) | you bridge onto the ROS2 topic graph (`TopicMap`, `Ros2Bridge`) |
| `testkit` | — | `atomr-physical-testkit` | you want `MockSensor` / `MockActuator` in tests |
| `rclrs` | — | `ros2` + `atomr-physical-ros2/rclrs` | you spin the bridge against a **live** ROS2 graph (needs a ROS2 toolchain) |
| `full` | — | `sensing` + `actuation` + `robotics` + `ros2` + `testkit` | you want everything except the `rclrs` live bridge |

The default feature set (`sensing` + `actuation` + `robotics`) is the
"control a robot offline" shape — everything you need to model a
physical system and exercise it with mock or real drivers, with no ROS2
toolchain required.

## Per-crate features

### `atomr-physical-ros2`

| Feature | Default | Effect |
|---|:---:|---|
| `rclrs` | — | Links the [`rclrs`](https://github.com/ros2-rust/ros2_rust) ROS 2 client library at version 0.7 plus the `futures` runtime, and implements `Ros2Bridge::spin` against a live graph using rclrs's **dynamic-message** API — every endpoint's `message_type` string drives a runtime publisher / subscription, so colcon-generated Rust message crates are **not** required. Needs a sourced ROS 2 environment (`AMENT_PREFIX_PATH` populated) so the linker finds `librcl` / `librmw` and the runtime can dlopen the introspection `.so`s. Off by default so the workspace builds anywhere. |

### `atomr-physical-cli`

| Feature | Default | Effect |
|---|:---:|---|
| `rclrs` | — | Forwards to `atomr-physical-ros2/rclrs` so `atomr-physical ros2 spin` drives a live ROS 2 graph. |

No other crate carries optional features today.

## Canonical shapes

```toml
# Offline robot control — the default. No ROS2 toolchain needed.
atomr-physical = "0.1"

# Add the ROS2 topic-graph bridge (still builds with no ROS2 install —
# the TopicMap is planned and tested offline).
atomr-physical = { version = "0.1", features = ["ros2"] }

# Spin the bridge against a live ROS2 graph (needs a ROS2 toolchain).
atomr-physical = { version = "0.1", features = ["rclrs"] }

# Everything except the rclrs live bridge — good for CI.
atomr-physical = { version = "0.1", features = ["full"] }

# Test scaffolding.
atomr-physical = { version = "0.1", features = ["testkit"] }
```

## CI note

`cargo build -p atomr-physical --all-features` would enable `rclrs`,
which fails on a host with no ROS 2 toolchain. The CI `feature-flags`
job exercises `--features full` instead; the `rclrs` bridge is checked
separately on a ROS 2-equipped runner. The runner needs:

- `librcl` / `librmw` / an `rmw_implementation` reachable via
  `AMENT_PREFIX_PATH`,
- the type-introspection `.so` for every `message_type` the bridge's
  tests reference (`std_msgs`, `sensor_msgs`, `geometry_msgs`,
  `builtin_interfaces`, `rosgraph_msgs`, `action_msgs`,
  `rcl_interfaces` cover everything in the repo today),
- and the build environment sourced via `setup.bash` before
  `cargo test -p atomr-physical-ros2 --features rclrs`.
