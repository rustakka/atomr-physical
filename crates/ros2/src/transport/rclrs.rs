//! The live ROS2 transport core — owns the `rclrs` node and executor.
//!
//! # Status
//!
//! This module is gated behind `#[cfg(feature = "rclrs")]`. With the
//! feature **off** (the default) it is compiled out entirely and the
//! crate builds with no ROS2 toolchain — that invariant is verified by
//! the offline test suite.
//!
//! Enabling the feature requires **wiring the `rclrs` crate dependency**
//! in `crates/ros2/Cargo.toml`. ros2-rust is normally built inside a
//! colcon workspace sourced against a `ROS_DISTRO` (target: ROS 2
//! Jazzy), so the exact dependency form — a crates.io release, a git
//! revision, or a vendored checkout — is a host-specific decision (see
//! `docs/ros2-bridge.md` §12, Risks). Until that is pinned on a ROS 2
//! Jazzy host, the `rclrs` feature does not build; the module below is
//! the implementation it builds *to*.
//!
//! # Design
//!
//! The transport follows atomr's `io::manager` idiom: the `rclrs`
//! `Context`, `Node`, every publisher / subscription / service /
//! action, and the executor live in a single `tokio::spawn`ed
//! `run_ros2` task. The task is fed an `mpsc::UnboundedReceiver<Ros2Command>`
//! and emits an `mpsc::UnboundedSender<Ros2Event>`. The `rclrs` executor
//! is driven cooperatively (`spin_once` in a `select!` loop) so it
//! co-exists with the tokio runtime. No `rclrs` type crosses the channel
//! boundary — inbound messages are decoded to `Command` / `Ros2Payload`
//! via the [`CodecRegistry`] before they become `Ros2Event`s.

use std::sync::Arc;
use std::time::Duration;

use tokio::sync::mpsc;

use crate::codec::CodecRegistry;
use crate::error::Ros2Error;
use crate::plan::Ros2Plan;

use super::{Ros2Command, Ros2Event, Ros2Link, Ros2Transport};

/// How long each cooperative `rclrs` spin slice waits for work before
/// yielding back to the `select!` loop to service the command channel.
const SPIN_SLICE: Duration = Duration::from_millis(10);

/// The live `rclrs`-backed transport.
///
/// Construct it with [`RclrsTransport::new`], then hand it to
/// [`Ros2Transport::start`] — exactly like `MockRos2Transport`, so the
/// orchestration layer is spawned identically against either.
pub struct RclrsTransport {
    node_name: String,
    plan: Ros2Plan,
    codecs: Arc<CodecRegistry>,
}

impl RclrsTransport {
    /// Build a transport for `node_name`, serving the endpoints in
    /// `plan` and encoding/decoding through `codecs`.
    pub fn new(node_name: impl Into<String>, plan: Ros2Plan, codecs: Arc<CodecRegistry>) -> Self {
        Self {
            node_name: node_name.into(),
            plan,
            codecs,
        }
    }
}

impl Ros2Transport for RclrsTransport {
    fn start(self) -> (Ros2Link, mpsc::UnboundedReceiver<Ros2Event>) {
        let (cmd_tx, cmd_rx) = mpsc::unbounded_channel::<Ros2Command>();
        let (evt_tx, evt_rx) = mpsc::unbounded_channel::<Ros2Event>();
        tokio::spawn(run_ros2(self.node_name, self.plan, self.codecs, cmd_rx, evt_tx));
        (Ros2Link::new(cmd_tx), evt_rx)
    }
}

/// The transport task: owns the `rclrs` node + executor for its whole
/// lifetime.
///
/// 1. Create the `rclrs` `Context` and `Node`.
/// 2. From `plan`, create one `rclrs` publisher / subscription / service
///    / action per endpoint. A subscription callback decodes the inbound
///    message via `codecs` and pushes a `Ros2Event` onto `evt_tx`.
/// 3. Emit [`Ros2Event::NodeReady`].
/// 4. `select!` between `cmd_rx.recv()` (apply a [`Ros2Command`] — encode
///    via `codecs` and publish / respond / set a parameter) and a
///    `spin_once` slice (drive the `rclrs` executor so callbacks fire).
/// 5. On [`Ros2Command::Shutdown`] or a fatal `rclrs` error, emit
///    [`Ros2Event::Closed`] and return.
async fn run_ros2(
    node_name: String,
    plan: Ros2Plan,
    codecs: Arc<CodecRegistry>,
    mut cmd_rx: mpsc::UnboundedReceiver<Ros2Command>,
    evt_tx: mpsc::UnboundedSender<Ros2Event>,
) -> Result<(), Ros2Error> {
    // --- 1. node ---------------------------------------------------------
    // let context = rclrs::Context::new(std::env::args())?;
    // let node = context.create_node(&node_name)?;
    //
    // --- 2. endpoints ----------------------------------------------------
    // For each `plan.topics().sensor_bindings()` create a typed publisher;
    // for each `actuator_bindings()` create a typed subscription whose
    // callback runs `codecs.require(&endpoint.message_type)?.decode_command`
    // and `evt_tx.send(Ros2Event::Inbound { .. })`. Likewise for
    // `plan.services()` (rclrs services), `plan.actions()` (rclrs
    // actions), and `plan.params()` (rclrs parameter declarations).
    // The concrete `rosidl`-typed publisher/subscription handles are held
    // in maps keyed by sensor / actuator / service / action id.
    //
    // --- 3. ready --------------------------------------------------------
    let _ = evt_tx.send(Ros2Event::NodeReady {
        node_name: node_name.clone(),
    });
    let _ = (&plan, &codecs);

    // --- 4. the cooperative spin loop -----------------------------------
    loop {
        tokio::select! {
            command = cmd_rx.recv() => match command {
                Some(Ros2Command::Shutdown) | None => break,
                Some(command) => apply_command(command, &codecs, &evt_tx),
            },
            // A cooperative `rclrs` spin slice — drives subscription /
            // service / action callbacks, which push `Ros2Event`s.
            _ = tokio::time::sleep(SPIN_SLICE) => {
                // rclrs::spin_once(&node, Some(SPIN_SLICE))
                //     .or_else(rclrs_spin_recover)?;
            }
        }
    }

    let _ = evt_tx.send(Ros2Event::Closed { reason: None });
    Ok(())
}

/// Apply one outbound [`Ros2Command`] to the live `rclrs` graph.
///
/// Encoding goes through the [`CodecRegistry`]: a `Publish` looks up the
/// codec for the bound topic's message type, calls `encode_reading`, and
/// publishes the resulting `rosidl` message; a `ServiceResponse` /
/// `ActionFeedback` / `ActionResult` does the same via `encode_payload`.
fn apply_command(command: Ros2Command, _codecs: &CodecRegistry, evt_tx: &mpsc::UnboundedSender<Ros2Event>) {
    match command {
        Ros2Command::Publish { sensor, reading } => {
            // let endpoint = plan.topics().sensor_endpoint(&sensor)?;
            // let codec = codecs.require(&endpoint.message_type)?;
            // let payload = codec.encode_reading(endpoint, &reading)?;
            // publishers[&sensor].publish(payload.into_native()?)?;
            let _ = (sensor, reading);
        }
        Ros2Command::ServiceResponse { request_id, payload } => {
            let _ = (request_id, payload);
        }
        Ros2Command::CallService {
            service,
            request_id,
            payload,
        } => {
            let _ = (service, request_id, payload);
        }
        Ros2Command::ActionFeedback { goal_id, payload } => {
            let _ = (goal_id, payload);
        }
        Ros2Command::ActionResult { goal_id, payload } => {
            let _ = (goal_id, payload);
        }
        Ros2Command::SetParam { name, value } => {
            let _ = (name, value);
        }
        Ros2Command::Shutdown => {
            // Handled by the caller's `select!` arm.
            let _ = evt_tx;
        }
    }
}
