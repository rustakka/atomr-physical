# Changelog

All notable changes to atomr-physical are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

- The actor-runtime wiring (`SensorActor` / `ActuatorActor` /
  `RobotActor` as live atomr `Actor`s) and the `rclrs` live ROS2 bridge
  are scaffolded with **Phase 2** markers in the source; the contract
  traits, value types, safety / calibration policies, and the offline
  ROS2 topic plan are complete and tested.
