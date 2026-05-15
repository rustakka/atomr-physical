//! The transport contract — the channel shapes between the orchestration
//! actors and whatever drives the ROS2 graph underneath them.
//!
//! [`Ros2Event`] is the inbound stream (things that happened on the ROS2
//! graph, pushed toward the actors); [`Ros2Command`] is the outbound
//! stream (things the actors want done on the graph). [`Ros2Link`] is the
//! outbound handle; the inbound stream is a plain
//! [`mpsc::UnboundedReceiver`]. A [`Ros2Transport`] produces the pair.
//!
//! These types are pure data — no `rclrs` types cross this boundary.
//! Both the live `rclrs` transport (Increment 5) and the in-memory
//! `MockRos2Transport` produce the same channel pair, so the
//! orchestration layer runs identically against either.

#[cfg(any(test, feature = "mock"))]
pub mod mock;

#[cfg(feature = "rclrs")]
pub mod rclrs;

use atomr_physical_core::{ActuatorId, Command, Reading, SensorId};
use tokio::sync::mpsc;

use crate::action::GoalId;
use crate::codec::Ros2Payload;
use crate::error::Ros2Error;
use crate::param::ParamValue;

#[cfg(any(test, feature = "mock"))]
pub use mock::{MockRos2Handle, MockRos2Transport};

#[cfg(feature = "rclrs")]
pub use rclrs::RclrsTransport;

/// Correlates a service request with its response across the transport
/// boundary.
pub type ReqId = u64;

/// Something that happened on the ROS2 graph, pushed toward the
/// orchestration actors.
#[derive(Debug, Clone)]
pub enum Ros2Event {
    /// The ROS2 node was created and every endpoint bound.
    NodeReady {
        /// The node name that came up.
        node_name: String,
    },
    /// A message arrived on a subscribed topic and decoded into a
    /// command. The transport decodes via the codec registry, so the
    /// orchestration layer never sees a raw `rclrs` message.
    Inbound {
        /// The actuator the subscribed topic is bound to.
        actuator: ActuatorId,
        /// The topic the message arrived on.
        topic: String,
        /// The decoded command.
        command: Command,
    },
    /// A request arrived on a service this node serves.
    ServiceRequest {
        /// The service name.
        service: String,
        /// Correlates the eventual [`Ros2Command::ServiceResponse`].
        request_id: ReqId,
        /// The decoded request payload.
        payload: Ros2Payload,
    },
    /// A goal arrived on an action this node serves.
    ActionGoal {
        /// The action name.
        action: String,
        /// Identifies this goal across its feedback stream.
        goal_id: GoalId,
        /// The decoded goal payload.
        payload: Ros2Payload,
    },
    /// A cancellation arrived for an in-flight goal.
    ActionCancel {
        /// The action name.
        action: String,
        /// The goal being cancelled.
        goal_id: GoalId,
    },
    /// A parameter the node declared was changed by an external client.
    ParamChanged {
        /// The parameter name.
        name: String,
        /// The new value.
        value: ParamValue,
    },
    /// An inbound message failed to decode — a data error, not a fault;
    /// the endpoint stays up.
    DecodeError {
        /// The endpoint (topic / service / action name) that produced it.
        endpoint: String,
        /// What went wrong.
        detail: String,
    },
    /// An endpoint dropped or faulted on the ROS2 side.
    EndpointFault {
        /// The endpoint that faulted.
        endpoint: String,
        /// What went wrong.
        detail: String,
    },
    /// The transport task is shutting down — gracefully or after a fatal
    /// error.
    Closed {
        /// The reason, if the shutdown was not a clean request.
        reason: Option<String>,
    },
}

/// Something the orchestration actors want done on the ROS2 graph.
#[derive(Debug, Clone)]
pub enum Ros2Command {
    /// Publish a reading on the topic bound to `sensor`. The transport
    /// encodes via the codec registry.
    Publish {
        /// The sensor whose bound topic to publish on.
        sensor: SensorId,
        /// The reading to publish.
        reading: Reading,
    },
    /// Respond to a service request the node is serving.
    ServiceResponse {
        /// The request being answered.
        request_id: ReqId,
        /// The response payload.
        payload: Ros2Payload,
    },
    /// Call a service hosted on an external node.
    CallService {
        /// The service name.
        service: String,
        /// Correlates the eventual response.
        request_id: ReqId,
        /// The request payload.
        payload: Ros2Payload,
    },
    /// Publish feedback for an in-flight action goal.
    ActionFeedback {
        /// The goal the feedback belongs to.
        goal_id: GoalId,
        /// The feedback payload.
        payload: Ros2Payload,
    },
    /// Publish the final result for an action goal.
    ActionResult {
        /// The goal the result belongs to.
        goal_id: GoalId,
        /// The result payload.
        payload: Ros2Payload,
    },
    /// Set a parameter's value on the node.
    SetParam {
        /// The parameter name.
        name: String,
        /// The value to set.
        value: ParamValue,
    },
    /// Tear the node down and stop the transport task.
    Shutdown,
}

/// The outbound handle the orchestration actors send [`Ros2Command`]s
/// through.
///
/// Cheap to clone — each per-endpoint actor holds its own clone. The
/// channel is unbounded (the `io::manager` idiom): backpressure lives at
/// the actor edges, not in this channel.
#[derive(Debug, Clone)]
pub struct Ros2Link {
    tx: mpsc::UnboundedSender<Ros2Command>,
}

impl Ros2Link {
    /// Wrap a command sender as a link.
    pub fn new(tx: mpsc::UnboundedSender<Ros2Command>) -> Self {
        Self { tx }
    }

    /// Send a command to the transport.
    ///
    /// Returns [`Ros2Error::TransportLost`] if the transport task has
    /// already exited and the channel is closed.
    pub fn send(&self, command: Ros2Command) -> Result<(), Ros2Error> {
        self.tx
            .send(command)
            .map_err(|_| Ros2Error::TransportLost("command channel closed".into()))
    }

    /// A clone of the underlying command sender.
    pub fn sender(&self) -> mpsc::UnboundedSender<Ros2Command> {
        self.tx.clone()
    }
}

/// Produces the channel pair the orchestration layer runs against.
///
/// Both the live `rclrs` transport and the in-memory
/// `MockRos2Transport` implement this — the orchestration actors are
/// spawned the same way regardless of which is underneath.
pub trait Ros2Transport {
    /// Start the transport, returning the outbound command link and the
    /// inbound event stream.
    fn start(self) -> (Ros2Link, mpsc::UnboundedReceiver<Ros2Event>);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn link_send_errors_once_the_receiver_is_dropped() {
        let (tx, rx) = mpsc::unbounded_channel();
        let link = Ros2Link::new(tx);
        drop(rx);
        let err = link.send(Ros2Command::Shutdown).unwrap_err();
        assert!(matches!(err, Ros2Error::TransportLost(_)));
    }

    #[test]
    fn link_send_reaches_the_receiver() {
        let (tx, mut rx) = mpsc::unbounded_channel();
        let link = Ros2Link::new(tx);
        link.send(Ros2Command::Shutdown).unwrap();
        assert!(matches!(rx.try_recv(), Ok(Ros2Command::Shutdown)));
    }
}
