//! `rclrs`-gated integration tests for the live ROS2 bridge.
//!
//! The whole file is behind `#[cfg(feature = "rclrs")]`: with the
//! feature off it is an empty test binary, so `cargo test` on a host
//! with no ROS2 toolchain stays green. With the feature on — on a ROS 2
//! Jazzy host — it exercises the live transport.
//!
//! Run it through the xtask helper, which preflights the ROS 2
//! environment:
//!
//! ```bash
//! source /opt/ros/jazzy/setup.bash
//! cargo xtask ros2-it
//! ```
//!
//! In CI this is the `workflow_dispatch`-only `rclrs-bridge` job.

#![cfg(feature = "rclrs")]

use std::sync::Arc;
use std::time::Duration;

use atomr_physical_ros2::transport::{RclrsTransport, Ros2Event, Ros2Transport};
use atomr_physical_ros2::{CodecRegistry, Ros2Plan};

/// The transport task comes up and announces the node.
///
/// This holds against the transport skeleton as-is — `run_ros2` emits
/// [`Ros2Event::NodeReady`] once the node is created — so it is the
/// first thing to pass once the `rclrs` dependency is wired.
#[tokio::test]
async fn transport_announces_node_ready() {
    let transport = RclrsTransport::new(
        "atomr_physical_it",
        Ros2Plan::new(),
        Arc::new(CodecRegistry::builtin()),
    );
    let (_link, mut event_rx) = transport.start();

    let event = tokio::time::timeout(Duration::from_secs(5), event_rx.recv())
        .await
        .expect("transport did not announce NodeReady in time")
        .expect("event channel closed");
    assert!(matches!(event, Ros2Event::NodeReady { .. }));
}

// The round-trip integration tests below need the live `rclrs` wiring in
// `transport/rclrs.rs` (the publisher / subscription / service / action
// registration and the `spin_once` slice) completed against a ROS 2
// Jazzy host. They are the acceptance criteria for Increments 5–9:
//
// - `topic_publish_round_trip` — a `Ros2Command::Publish` reaches a
//   real `ros2 topic echo` subscriber, encoded by the builtin codec.
// - `topic_subscribe_round_trip` — a message a real `ros2 topic pub`
//   sends arrives as a `Ros2Event::Inbound` carrying the decoded
//   `Command`.
// - `service_round_trip` — a real `ros2 service call` is answered by a
//   `Ros2ServiceActor`.
// - `param_round_trip` — `ros2 param get` / `set` reflects the
//   `Ros2ParamActor`'s store.
// - `action_round_trip` — a real `ros2 action send_goal` is driven by a
//   `Ros2ActionActor` to a result.
//
// Each is a few lines once the transport wiring is in; they are omitted
// here rather than stubbed so the suite never reports a hollow pass.
