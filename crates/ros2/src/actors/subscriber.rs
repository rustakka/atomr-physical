//! [`Ros2SubscriberActor`] — one actor per subscribing topic endpoint.
//!
//! The subscriber receives a [`Command`] the transport decoded from an
//! inbound message and dispatches it into a [`CommandSink`]. The
//! dispatch runs off the actor's thread via [`pipe_to`] so a slow
//! actuator backs up only this endpoint's mailbox, not the whole node.

use std::sync::Arc;

use atomr_physical_core::{Command, CommandAck, Result};

use crate::actor::prelude::*;
use crate::actors::CommandSink;
use crate::endpoint::Ros2Endpoint;

/// Messages a [`Ros2SubscriberActor`] handles.
pub enum Ros2SubscriberMsg {
    /// An inbound command, decoded by the transport from this endpoint's
    /// bound topic.
    Deliver(Command),
    /// Internal: the sink's acknowledgement, piped back after `deliver`.
    Delivered(Result<CommandAck>),
}

/// One actor per subscribing topic endpoint: dispatches inbound
/// [`Command`]s into a [`CommandSink`].
pub struct Ros2SubscriberActor {
    endpoint: Ros2Endpoint,
    sink: Arc<dyn CommandSink>,
}

impl Ros2SubscriberActor {
    /// Construct a subscriber for `endpoint`, dispatching into `sink`.
    pub fn new(endpoint: Ros2Endpoint, sink: Arc<dyn CommandSink>) -> Self {
        Self { endpoint, sink }
    }
}

#[async_trait]
impl Actor for Ros2SubscriberActor {
    type Msg = Ros2SubscriberMsg;

    async fn handle(&mut self, ctx: &mut Context<Self>, msg: Ros2SubscriberMsg) {
        match msg {
            Ros2SubscriberMsg::Deliver(command) => {
                // Dispatch off-thread: a slow actuator must not stall
                // this actor's mailbox or block sibling endpoints.
                let sink = Arc::clone(&self.sink);
                pipe_to(
                    async move { Ros2SubscriberMsg::Delivered(sink.deliver(command).await) },
                    ctx.self_ref().clone(),
                );
            }
            Ros2SubscriberMsg::Delivered(Ok(ack)) => {
                tracing::debug!(
                    topic = %self.endpoint.topic,
                    accepted = ack.accepted,
                    "ros2 subscriber: command dispatched",
                );
            }
            Ros2SubscriberMsg::Delivered(Err(err)) => {
                tracing::warn!(
                    topic = %self.endpoint.topic,
                    error = %err,
                    "ros2 subscriber: command dispatch failed",
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actors::testing::MockCommandSink;
    use atomr_physical_core::{ActuatorId, ControlMode, Quantity, Unit};
    use std::time::Duration;

    #[tokio::test]
    async fn subscriber_dispatches_inbound_commands_into_the_sink() {
        let sys = ActorSystem::create("sub-test", Config::empty()).await.unwrap();
        let sink = MockCommandSink::new("a1");
        let log = sink.log_handle();
        let endpoint = Ros2Endpoint::subscribe("/arm/cmd", "std_msgs/msg/Float64");
        let sink: Arc<dyn CommandSink> = Arc::new(sink);
        let subscriber = sys
            .actor_of(
                Props::create(move || Ros2SubscriberActor::new(endpoint.clone(), Arc::clone(&sink))),
                "sub",
            )
            .unwrap();

        let command = Command::now(
            ActuatorId::from("a1"),
            ControlMode::Position,
            Quantity::new(0.75, Unit::Radian),
        );
        subscriber.tell(Ros2SubscriberMsg::Deliver(command));

        // The dispatch is async — poll the log until it lands.
        let mut delivered = false;
        for _ in 0..50 {
            if log.lock().unwrap().len() == 1 {
                delivered = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(delivered, "subscriber did not dispatch the command");
        assert_eq!(log.lock().unwrap()[0].setpoint.value, 0.75);
        sys.terminate().await;
    }
}
