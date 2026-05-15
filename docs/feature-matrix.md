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
| `rclrs` | — | Links the [`rclrs`](https://github.com/ros2-rust/ros2_rust) ROS2 client library and the live transport core, concrete `rosidl`-typed codecs, and service / parameter / action paths. **Requires a ROS 2 Jazzy installation** (and `rosidl` message generation) on the build host. Off by default so the workspace builds anywhere. |
| `mock` | — | Exposes the in-memory `MockRos2Transport` and the `MockReadingSource` / `MockCommandSink` device-seam doubles for **downstream** test suites. The crate's own tests get them via `cfg(test)` without this. Pure Rust — no ROS2 toolchain. |

No other crate carries optional features at 0.1.0.

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
which fails on a host with no ROS2 toolchain. The CI `feature-flags`
job exercises `--features full` instead; the `rclrs` bridge is checked
separately on a ROS 2 Jazzy runner by the `workflow_dispatch`-only
`rclrs-bridge` job (see `.github/workflows/ci.yml`), invoked through
`cargo xtask ros2-it`.
