//! An in-memory [`Ros2Transport`] for testing the orchestration layer
//! with no ROS2 toolchain.
//!
//! [`MockRos2Transport`] is the transport handed to the orchestration
//! actors; [`MockRos2Handle`] is the test-side control surface —
//! `inject` an inbound [`Ros2Event`] as if it arrived from the ROS2
//! graph, and `drain_commands` / `next_command` to see the
//! [`Ros2Command`]s the actors sent out.
//!
//! Available in the crate's own tests and, for downstream test suites,
//! behind the `mock` feature.

use tokio::sync::mpsc;

use super::{Ros2Command, Ros2Event, Ros2Link, Ros2Transport};

/// An in-memory transport: a loopback over a pair of `mpsc` channels.
///
/// Construct it with [`MockRos2Transport::new`], which also hands back a
/// [`MockRos2Handle`] for the test to drive and inspect. Pass the
/// transport to the bridge / [`Ros2Transport::start`]; keep the handle.
pub struct MockRos2Transport {
    link: Ros2Link,
    event_rx: mpsc::UnboundedReceiver<Ros2Event>,
}

/// The test-side control surface for a [`MockRos2Transport`].
pub struct MockRos2Handle {
    /// Inbound events the test injects toward the orchestration.
    event_tx: mpsc::UnboundedSender<Ros2Event>,
    /// Outbound commands the orchestration sent — drained by the test.
    command_rx: mpsc::UnboundedReceiver<Ros2Command>,
}

impl MockRos2Transport {
    /// Construct a mock transport and its test-side handle.
    pub fn new() -> (Self, MockRos2Handle) {
        let (command_tx, command_rx) = mpsc::unbounded_channel();
        let (event_tx, event_rx) = mpsc::unbounded_channel();
        let transport = MockRos2Transport {
            link: Ros2Link::new(command_tx),
            event_rx,
        };
        let handle = MockRos2Handle { event_tx, command_rx };
        (transport, handle)
    }
}

impl Ros2Transport for MockRos2Transport {
    fn start(self) -> (Ros2Link, mpsc::UnboundedReceiver<Ros2Event>) {
        (self.link, self.event_rx)
    }
}

impl MockRos2Handle {
    /// Inject an inbound event, as if it arrived from the ROS2 graph.
    ///
    /// Returns `false` if the orchestration's event receiver has already
    /// been dropped.
    pub fn inject(&self, event: Ros2Event) -> bool {
        self.event_tx.send(event).is_ok()
    }

    /// Receive the next [`Ros2Command`] the orchestration sent out,
    /// awaiting one if none is queued. Returns `None` once every
    /// [`Ros2Link`] has been dropped.
    pub async fn next_command(&mut self) -> Option<Ros2Command> {
        self.command_rx.recv().await
    }

    /// Drain every [`Ros2Command`] queued so far without awaiting.
    pub fn drain_commands(&mut self) -> Vec<Ros2Command> {
        let mut drained = Vec::new();
        while let Ok(command) = self.command_rx.try_recv() {
            drained.push(command);
        }
        drained
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn start_yields_a_working_link() {
        let (transport, mut handle) = MockRos2Transport::new();
        let (link, _event_rx) = transport.start();
        link.send(Ros2Command::Shutdown).unwrap();
        assert!(matches!(handle.next_command().await, Some(Ros2Command::Shutdown)));
    }

    #[tokio::test]
    async fn injected_events_reach_the_event_stream() {
        let (transport, handle) = MockRos2Transport::new();
        let (_link, mut event_rx) = transport.start();
        assert!(handle.inject(Ros2Event::NodeReady {
            node_name: "n".into()
        }));
        assert!(matches!(event_rx.recv().await, Some(Ros2Event::NodeReady { .. })));
    }

    #[test]
    fn drain_commands_collects_without_blocking() {
        let (transport, mut handle) = MockRos2Transport::new();
        let (link, _event_rx) = transport.start();
        link.send(Ros2Command::Shutdown).unwrap();
        link.send(Ros2Command::Shutdown).unwrap();
        assert_eq!(handle.drain_commands().len(), 2);
        assert!(handle.drain_commands().is_empty());
    }
}
