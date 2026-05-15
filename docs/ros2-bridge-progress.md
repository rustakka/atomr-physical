# ROS2 bridge — implementation progress

Status tracker for the `atomr-physical-ros2` buildout. The design and
the increment definitions live in [`ros2-bridge.md`](./ros2-bridge.md);
this file records **what is done, what is in flight, and what is next**.

Last updated: 2026-05-14.

## Plan at a glance

The bridge is built in ten increments (see
[`ros2-bridge.md` §11](./ros2-bridge.md#11-phasing--roadmap)). Increments
1–4 are **offline** (no ROS 2 toolchain); 5–10 touch the `rclrs` feature
and need a ROS 2 Jazzy host.

| # | Increment | Status |
|---|---|---|
| 0 | Spec — `docs/ros2-bridge.md` (13 sections) | ✅ done, reviewed |
| 1 | Module restructure + QoS + clock + validation + error | ✅ done, verified offline |
| 2 | Service / action / param endpoint types + `Ros2Plan` | ✅ done, verified offline |
| 3 | Codec layer — `MessageCodec` trait + extensible `CodecRegistry` | ✅ done, verified offline |
| 4 | Model 2 orchestration actors + device seam + `MockRos2Transport` | ✅ done, verified offline |
| 5 | Transport core, topics live | 🔄 **in progress** |
| 6 | Concrete builtin codecs (topics) | ⏳ pending |
| 7 | Services live | ⏳ pending |
| 8 | Parameters live | ⏳ pending |
| 9 | Actions live | ⏳ pending |
| 10 | CLI live paths + Python bindings + end-to-end + docs | ◑ offline parts done; live parts pending |

## Done & verified (offline — no ROS 2 toolchain)

**Phase 0 + Increments 1–4** are complete and green:

- The offline plan surface — `Ros2Endpoint`, `TopicMap`, `Ros2Plan`,
  `QosProfile`, `Ros2ClockSource`, `validate_plan`, the crate-local
  `Ros2Error`, and the service / action / parameter endpoint types.
- The codec layer — the `MessageCodec` trait, the downstream-extensible
  `CodecRegistry`, `Ros2Payload` (structured `serde_json::Value` form),
  `CodecValue`, the `Unit` ↔ message-type table, and four curated
  **structured-payload** codecs (`Float64`, `Float64MultiArray`,
  `Temperature`, `Twist`) — pure Rust, unit-tested.
- The transport contract — `Ros2Event` / `Ros2Command` / `Ros2Link` /
  `Ros2Transport`, plus the in-memory `MockRos2Transport` behind the
  `mock` feature.
- The **full Model 2 actor graph** — `Ros2NodeActor` supervising
  `Ros2PublisherActor` / `Ros2SubscriberActor` / `Ros2ServiceActor` /
  `Ros2ActionActor` / `Ros2ParamActor`, the
  `ReadingSource` / `CommandSink` / `ServiceHandler` / `ActionHandler` /
  `ParamStore` seam, and `Ros2Wiring` — all tested against
  `MockRos2Transport`.

**Increment 10, offline portions** — the CLI `ros2 codecs` subcommand,
the Python bindings (`PyQosProfile`, the endpoint / plan types, the
read-only `PyCodecRegistry`), `cargo xtask ros2-it`, the
`workflow_dispatch`-only `rclrs-bridge` CI job, and the docs
(`ros2-bridge.md`, `architecture.md`, `feature-matrix.md`,
`python-api.md`, `README.md`, `RELEASING.md`, `CHANGELOG.md`, `SKILL.md`).

**Verification gate (all green):** `cargo fmt` clean;
`cargo clippy --workspace --all-features` clean; 106 workspace tests +
95 `--features mock` tests pass; build matrix OK; `cargo doc --workspace`
clean under the CI rustdoc flags.

## Environment setup — done this session (on the ROS 2 Jazzy host)

The live `rclrs` work was blocked on toolchain availability. That is now
resolved on this host:

- **ROS 2 Jazzy** installed at `/opt/ros/jazzy` (Ubuntu 24.04 Noble).
- **Rust↔colcon plumbing** — `cargo-ament-build`, `colcon-cargo`,
  `colcon-ros-cargo`.
- **`ros2_rust` workspace** at `~/ros2_rust_ws` — cloned, `vcs import`ed,
  and `colcon build`ed (41 packages, exit 0). This produced **`rclrs`
  0.7.0** and the `rosidl`-generated message crates: `std_msgs`,
  `sensor_msgs`, `geometry_msgs`, `std_srvs` (5.3.7), `builtin_interfaces`
  (2.0.4), under `/opt/ros/jazzy/share/<pkg>/rust` and
  `~/ros2_rust_ws/install/<pkg>/share/<pkg>/rust`.
- **Patch mechanism** — `colcon-ros-cargo` generated
  `~/ros2_rust_ws/.cargo/config.toml`, a `[patch.crates-io]` table
  redirecting the `rclrs` / message-crate coordinates to those paths.

### `rclrs` 0.7 API reference (learned this session)

- `Context::default_from_env()?` → `context.create_basic_executor()` →
  `executor.create_node("name")?`.
- `node.create_publisher::<T>(topic)?` → `publisher.publish(&msg)?`.
- `node.create_subscription::<T, _>(topic, move |msg: T| { … })?`.
- `node.create_service::<T, _>(…)`, `node.create_client::<T>(…)`.
- Cooperative spin: `executor.spin(SpinOptions::spin_once().timeout(d))`.
- Message structs (`std_msgs::msg::Float64 { data: f64 }`, …) carry an
  optional **`serde` feature** — with it on, `serde_json::to_value` /
  `from_value` does the structured↔native materialisation generically,
  so no hand-written per-type field mapping is needed where the codec's
  JSON shape already matches the `rosidl` layout (`Float64`, `Twist` do;
  `Float64MultiArray` and `Temperature` need the codec JSON aligned —
  Increment 6).

## Increment 5 — in progress

**Done:**

- `crates/ros2/Cargo.toml` — `rclrs = []` → `rclrs = ["dep:rclrs", …]`;
  added `rclrs` 0.7 + `std_msgs` / `sensor_msgs` / `geometry_msgs` /
  `std_srvs` as **optional** deps (the message crates with their `serde`
  feature).
- `.cargo/config.toml` — added the host-local `[patch.crates-io]` table
  (mirrored from the `ros2_rust` workspace) as an **uncommitted
  working-tree change**; the committed file carries only `[alias]`. The
  paths are host-specific and must never be committed.
- **Offline build re-verified** — `cargo build -p atomr-physical-ros2`
  with the feature off still succeeds; the patches are unused and inert,
  so invariant 1 (offline-buildable) holds.

**Next:**

- Implement `crates/ros2/src/transport/rclrs.rs` for real — replace the
  documented-comment skeleton with: `Context` / `Executor` / `Node`
  construction; typed publishers per sensor binding and typed
  subscriptions per actuator binding (dispatched on `message_type`);
  the structured↔native `serde_json` materialisation; the cooperative
  `spin_once` loop draining `cmd_rx` and applying `Ros2Command::Publish`.
  The transport task runs on a dedicated OS thread (rclrs spin is
  blocking) and communicates over the existing `mpsc` channels.
- Get `cargo build -p atomr-physical-ros2 --features rclrs` to compile.
- Run the gated loopback test in `tests/rclrs_integration.rs`
  (`transport_announces_node_ready`, then a topic round-trip).

## Remaining after Increment 5

- **Increment 6** — align the `Float64MultiArray` / `Temperature` codec
  JSON to the exact `rosidl` field layout (the others already match), so
  the generic serde materialisation round-trips the full curated set.
- **Increment 7** — live `rclrs` service server + client; wire
  `Ros2ServiceActor` to `std_srvs/srv/Trigger` + `SetBool`.
- **Increment 8** — live `rclrs` parameter declaration + the read-write
  config mirror in `Ros2ParamActor`.
- **Increment 9** — live `rclrs` action server + client for
  `Ros2ActionActor`.
- **Increment 10 (live)** — the CLI `ros2 spin` / `ros2 echo` paths and
  the documented end-to-end demo cross-checked with `ros2
  topic/service/action`.

## How to build & test the live path

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_rust_ws/install/setup.bash
cd ~/source/atomr-physical
cargo build -p atomr-physical-ros2 --features rclrs
cargo xtask ros2-it          # the rclrs-gated integration tests
```

The offline path is unchanged and must always pass:

```bash
cargo test --workspace                       # rclrs OFF
cargo test -p atomr-physical-ros2 --features mock
cargo build -p atomr-physical --features full
```
