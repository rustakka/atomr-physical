# Changelog

All notable changes to atomr-physical are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **`atomr-physical-projection`** — projection output subsystem
  (opt-in via the umbrella's `projection` feature). `ProjectionActor`
  is the supervisor at the top of a Sunshine/Moonlight subtree,
  owning a `VkmsDisplayManager` for headless virtual displays, a
  `PortAllocator` that hands out stride-shifted Sunshine port windows,
  a pool of supervised `SunshineInstanceActor` subprocess children, an
  `MdnsBroadcaster` advertising each instance as
  `_nvstream._tcp.local.`, and a `ClientProvisioner` driving the
  Moonlight pairing handshake (announce + PIN) over Sunshine's local
  HTTPS API. Follows the same offline / supervised two-form contract
  every other device actor uses —
  `ProjectionActor::with_test_offline(true)` plus a `/bin/sleep`
  Sunshine binary lets the whole pipeline run hardware-free under CI.
  Graceful supervisor restarts on `BandwidthTier` boundaries adjust
  bitrate as additional clients mirror the stream.
- **`atomr-physical-projection-client`** — receiver-side
  `atomr-projection-client` binary intended for an ARM device on the
  same LAN (Raspberry Pi, Jetson). `discover` browses
  `_nvstream._tcp.local.` and prints matches; `run` pairs against the
  first match and execs `moonlight-embedded`. Stateless by design
  (every run is a fresh pair-and-stream cycle), ships a bundled
  systemd unit and a documented `aarch64-unknown-linux-gnu`
  cross-compile recipe.
- **CLI `project` subcommand** — `project demo` boots a
  `ProjectionActor` and spins up N stub projections; `project pair`
  runs the full pair-and-tear-down flow. Defaults to `/bin/sleep` +
  `--offline` so it requires no privileges; point `--sunshine-binary`
  at `/usr/bin/sunshine` for a real Sunshine install.
- **Umbrella `projection` feature** — opt-in so default builds stay
  free of the network deps the projection crate pulls in (`reqwest`,
  `mdns-sd`, `nix`, `tempfile`). The `full` feature now includes
  `projection`.
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
