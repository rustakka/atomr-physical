# Changelog

All notable changes to atomr-physical are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Live device actors.** `SensorActor`, `ActuatorActor`, and
  `RobotActor` each gained a `spawn(system, name)` method that promotes
  the offline configuration into a supervised atomr actor; the new
  `SensorActorRef` / `ActuatorActorRef` / `RobotActorRef` handles
  expose `sample` / `dispatch` / `child lookup` over the mailbox plus
  `health_check`. `SensorActorRef::subscribe` returns a
  `tokio::sync::broadcast::Receiver<Reading>` fanned out by the
  periodic sampling loop.
- **RobotActor as a real supervisor.** `RobotRunner::pre_start` spawns
  each registered sensor / actuator as a true atomr child via
  `Sensor/ActuatorActor::spawn_under(ctx, …)`, named `sensor-<id>` /
  `actuator-<id>`. A `OneForOneStrategy` (configurable via
  `with_supervisor_strategy`) restarts only the affected subtree on a
  driver fault.
- **rclrs-backed Ros2Bridge::spin.** Behind the `rclrs` feature the
  bridge stands up a real `rclrs::Context` + `Node` + `Executor`,
  attaches a `DynamicPublisher` for each sensor endpoint and a
  `DynamicSubscription` for each actuator endpoint (using the
  dynamic-message API — no colcon-generated message crates required),
  and returns a `Ros2BridgeHandle` with `publish_reading`,
  `subscriber_count`, `published_sensors`, and `shutdown`. Spinning is
  halted by a `futures::oneshot` wired into
  `SpinOptions::until_promise_resolved`.
- **CLI fully wired.** `atomr-physical devices` / `sense` / `actuate`
  / `ros2 plan` / `ros2 spin` now exercise an in-process device
  registry seeded with the testkit mocks instead of printing stubs;
  the binary is the smallest hardware-free demo of the full pipeline.
  The `rclrs` feature flag forwards through to the bridge so a single
  `cargo install atomr-physical-cli --features rclrs` ships a CLI that
  drives a live ROS 2 graph.
- **xtask audit + bump.** `cargo xtask audit` runs `cargo fmt --check`
  + `cargo clippy --workspace --all-targets -- -D warnings` + an
  optional `cargo deny check` (skipped if `cargo-deny` is missing or
  via `--no-deny`). `cargo xtask bump <kind|--exact X.Y.Z>` delegates
  to `cargo set-version --workspace` and keeps `pyproject.toml` in
  lockstep.
- Initial repository scaffold for **atomr-physical** — the physical
  sensing, output, and ROS2-integrated robotics layer of the atomr
  actor ecosystem.
- **`atomr-physical-core`** — pure-data foundation: `DeviceId` /
  `SensorId` / `ActuatorId` / `RobotId` / `JointId` newtypes, the
  `Unit` enum + `Quantity` type, `Reading` / `ReadingBatch`, `Command`
  / `CommandAck` / `ControlMode`, the `PhysicalError` taxonomy, and the
  `Device` / `Sensor` / `Actuator` contract traits.
- **`atomr-physical-sensing`** — `SensorActor`, `SamplingPolicy`,
  linear `Calibration`.
- **`atomr-physical-actuation`** — `ActuatorActor`, `SafetyEnvelope`
  (clamp / reject).
- **`atomr-physical-robotics`** — `RobotActor`, `Joint`, `RobotModel`.
- **`atomr-physical-ros2`** — `Ros2Endpoint`, `TopicMap`, `Ros2Bridge`
  — the offline ROS2 topic-graph plan; an `rclrs` feature reserved for
  the Phase-2 live bridge.
- **`atomr-physical-testkit`** — `MockSensor` / `MockActuator` test
  doubles.
- **`atomr-physical-cli`** — the `atomr-physical` binary with
  `devices` / `sense` / `actuate` / `ros2` subcommands.
- **`atomr-physical-py-bindings`** — the `atomr_physical._native` PyO3
  extension module (`errors`, `core`, `sensing`, `actuation`,
  `robotics`, `ros2` submodules).
- **`atomr-physical`** — feature-flagged umbrella crate.
- Python overlay package `atomr_physical` with per-domain facade
  modules, a PEP 561 `py.typed` marker, and a smoke-test suite.
- The atomr release pipeline (`version-bump.yml`, `release.yml`,
  `ci.yml`, `docs.yml`) — wired and documented, but gated to manual
  `workflow_dispatch` only; there is no automated CI execution yet.
- `ai-skills/` plugin bundle and `.claude-plugin/marketplace.json` for
  AI-assisted development against atomr-physical.

### Notes

- The actor-runtime wiring and the `rclrs` live ROS 2 bridge that the
  initial scaffold called Phase 2 are now in place — every device
  type has both an offline form and a supervised form behind
  `.spawn(...)`, and `Ros2Bridge::spin` runs a real ROS 2 node.
