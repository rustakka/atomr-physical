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
- **ROS2 bridge — offline buildout (Increment 1).** `atomr-physical-ros2`
  restructured into focused modules; `QosProfile` (with `Reliability` /
  `Durability` / `History` and per-direction defaults) attachable to
  every `Ros2Endpoint`; `Ros2ClockSource` for timestamp planning; offline
  plan validation (`validate_endpoint` / `validate_topic_map`,
  `ValidationIssue`, and `Ros2Bridge::validate`); a crate-local
  `Ros2Error` that folds into `PhysicalError::Ros2Bridge`; and
  `TopicMap::sensor_bindings` / `actuator_bindings` iterators. All offline
  — no ROS2 toolchain required. See `docs/ros2-bridge.md` for the full
  bridge specification.
- **ROS2 bridge — service / action / parameter plan (Increment 2).**
  `Ros2ServiceEndpoint` (`ServiceRole`), `Ros2ActionEndpoint`
  (`ActionRole`, `GoalId`), and `Ros2ParamDecl` (`ParamType` /
  `ParamValue`) offline endpoint types; `Ros2Plan` aggregates topics,
  services, actions, and parameters into one plan; `Ros2Bridge` now owns
  a `Ros2Plan` (with `plan` / `plan_mut` accessors, `topics` /
  `topics_mut` still delegate); `validate_plan` and the
  service/action/param lints extend offline validation. All offline.
- **ROS2 bridge — codec layer (Increment 3).** The `MessageCodec` trait
  and a downstream-extensible `CodecRegistry` (public `register`) map a
  `message_type` string to an encode/decode implementation; `Ros2Payload`
  is the opaque wire payload (a tagged structured value offline);
  `CodecValue` is the generic atomr-side value for service / action
  payloads; a `Unit` ↔ message-type compatibility table (`UnitConstraint`,
  `unit_constraint`, `check_unit`) catches unit mismatches before
  encode. `CodecRegistry::builtin` ships the curated set as
  **structured-payload** codecs — pure Rust, available with `rclrs`
  off. All offline.
- **ROS2 bridge — orchestration actors (Increment 4).** The transport
  contract — `Ros2Event` / `Ros2Command` / `Ros2Link` and the
  `Ros2Transport` seam — plus an in-memory `MockRos2Transport` (behind a
  new `mock` feature) for testing the orchestration with no ROS2
  toolchain. The device seam (`ReadingSource` / `CommandSink` traits with
  `SensorActorSource` / `ActuatorActorSink` adapters) decouples the
  bridge from the still-scaffolded `SensorActor` / `ActuatorActor`. The
  Model 2 graph is live: `Ros2NodeActor` supervises one actor per
  endpoint — `Ros2PublisherActor`, `Ros2SubscriberActor`,
  `Ros2ServiceActor`, `Ros2ActionActor`, and a node-level
  `Ros2ParamActor` — spawned from the plan and routed by endpoint name.
  The device + handler seam (`ReadingSource` / `CommandSink` /
  `ServiceHandler` / `ActionHandler` / `ParamStore`, assembled by
  `Ros2Wiring`) decouples the bridge from the still-scaffolded
  `SensorActor` / `ActuatorActor`; `spawn_inbound_pump` drains the
  event stream into the node. All offline, all tested against
  `MockRos2Transport`.
- **ROS2 bridge — live transport & wiring (Increments 5–9).** The
  `rclrs`-gated transport core (`RclrsTransport`, the `run_ros2` task
  following the `io::manager` idiom, the cooperative `spin_once` loop)
  and `Ros2Bridge::run` — which builds the live transport + the Model 2
  actors on an `ActorSystem` and returns a `Ros2BridgeHandle`. The
  curated builtin codecs (`std_msgs/Float64`, `Float64MultiArray`,
  `sensor_msgs/Temperature`, `geometry_msgs/Twist`) and a gated
  `tests/rclrs_integration.rs` scaffold. The crate compiles with the
  `rclrs` feature both off and on; **wiring the `rclrs` crate
  dependency and the live publisher/subscription/service/action
  registration is the remaining step, done on a ROS 2 Jazzy host** —
  see `docs/ros2-bridge.md` §11–12.
- **ROS2 bridge — Python bindings, CLI & tooling (Increment 10).** The
  offline ROS2 plan surface is in the Python overlay: `QosProfile`,
  `Ros2ClockSource`, `Ros2ServiceEndpoint`, `Ros2ActionEndpoint`,
  `Ros2ParamDecl`, `Ros2Plan` (with `validate`), a read-only
  `CodecRegistry` view, and `Ros2Endpoint` QoS — re-exported at the
  package top level and smoke-tested. The CLI gains a live `ros2
  codecs` subcommand and an `rclrs` feature. `cargo xtask ros2-it`
  runs the `rclrs`-gated integration tests on a ROS 2 Jazzy host; a
  `workflow_dispatch`-only `rclrs-bridge` CI job wraps it.
  `RELEASING.md` / `xtask` pre-flight switched from `--all-features`
  (which pulls `rclrs`) to `--features full`.
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

- The device-actor wiring (`SensorActor` / `ActuatorActor` /
  `RobotActor` as live atomr `Actor`s) is still scaffolded with
  **Phase 2** markers in the source; the contract traits, value types,
  and safety / calibration policies are complete and tested.
- The **ROS2 bridge** is built through every offline layer — the plan,
  QoS, validation, the codec layer with its curated structured-payload
  codecs, the transport contract, the `MockRos2Transport`, and the full
  Model 2 orchestration (`Ros2NodeActor` plus the publisher /
  subscriber / service / action / parameter actors) — all compiled and
  tested with no ROS2 toolchain. The `rclrs`-gated transport core
  (`RclrsTransport`, `Ros2Bridge::run`) compiles with the feature on
  and off; **wiring the `rclrs` crate dependency and the live
  publisher / subscription / service / action registration is the
  remaining step, performed on a ROS 2 Jazzy host.**
