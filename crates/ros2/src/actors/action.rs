//! [`Ros2ActionActor`] — one actor per served ROS2 action endpoint.
//!
//! A ROS2 action is goal / feedback / result. In the **server** role,
//! this actor accepts a `Ros2Event::ActionGoal`, runs an
//! [`ActionHandler`] to completion off-thread via [`pipe_to`], and emits
//! a `Ros2Command::ActionResult` — unless the goal was cancelled first.
//! It tracks in-flight goals so a `Ros2Event::ActionCancel` drops the
//! result rather than publishing it.
//!
//! The **client** role — an atomr actor sending goals to an external
//! action server — lands with the live `rclrs` action client. Streamed
//! feedback fan-out is wired with the live action server.

use std::collections::HashSet;
use std::sync::Arc;

use crate::action::{GoalId, Ros2ActionEndpoint};
use crate::actor::prelude::*;
use crate::actors::ActionHandler;
use crate::codec::{CodecValue, Ros2Payload};
use crate::error::Ros2Error;
use crate::transport::{Ros2Command, Ros2Link};

/// Messages a [`Ros2ActionActor`] handles.
pub enum Ros2ActionMsg {
    /// An inbound goal from the transport for the served action.
    Goal {
        /// Identifies the goal across its lifecycle.
        goal_id: GoalId,
        /// The goal payload.
        payload: Ros2Payload,
    },
    /// An inbound cancellation for an in-flight goal.
    Cancel {
        /// The goal being cancelled.
        goal_id: GoalId,
    },
    /// Internal: a goal's handler ran to completion, piped back.
    Completed {
        /// The goal that completed.
        goal_id: GoalId,
        /// The handler's result.
        result: Result<CodecValue, Ros2Error>,
    },
}

/// One actor per served ROS2 action endpoint: drives goals through an
/// [`ActionHandler`] and reports results through the transport.
pub struct Ros2ActionActor {
    endpoint: Ros2ActionEndpoint,
    handler: Arc<dyn ActionHandler>,
    link: Ros2Link,
    in_flight: HashSet<GoalId>,
}

impl Ros2ActionActor {
    /// Construct an action actor for `endpoint`, driving goals with
    /// `handler` and reporting through `link`.
    pub fn new(endpoint: Ros2ActionEndpoint, handler: Arc<dyn ActionHandler>, link: Ros2Link) -> Self {
        Self {
            endpoint,
            handler,
            link,
            in_flight: HashSet::new(),
        }
    }
}

#[async_trait]
impl Actor for Ros2ActionActor {
    type Msg = Ros2ActionMsg;

    async fn handle(&mut self, ctx: &mut Context<Self>, msg: Ros2ActionMsg) {
        match msg {
            Ros2ActionMsg::Goal { goal_id, payload } => {
                let Some(goal) = payload.as_codec_value() else {
                    tracing::warn!(
                        action = %self.endpoint.action,
                        goal = %goal_id,
                        "ros2 action: goal payload was not structured",
                    );
                    return;
                };
                self.in_flight.insert(goal_id.clone());
                let handler = Arc::clone(&self.handler);
                pipe_to(
                    async move {
                        Ros2ActionMsg::Completed {
                            goal_id,
                            result: handler.execute(goal).await,
                        }
                    },
                    ctx.self_ref().clone(),
                );
            }
            Ros2ActionMsg::Cancel { goal_id } => {
                if self.in_flight.remove(&goal_id) {
                    tracing::info!(
                        action = %self.endpoint.action,
                        goal = %goal_id,
                        "ros2 action: goal cancelled — result will be dropped",
                    );
                } else {
                    tracing::debug!(
                        action = %self.endpoint.action,
                        goal = %goal_id,
                        "ros2 action: cancel for an unknown or finished goal",
                    );
                }
            }
            Ros2ActionMsg::Completed { goal_id, result } => {
                // A goal cancelled while its handler was running is no
                // longer in-flight — drop its result.
                if !self.in_flight.remove(&goal_id) {
                    tracing::debug!(
                        action = %self.endpoint.action,
                        goal = %goal_id,
                        "ros2 action: completed goal was already cancelled — dropping result",
                    );
                    return;
                }
                let payload = match result {
                    Ok(value) => value.into_payload(),
                    Err(err) => {
                        tracing::warn!(
                            action = %self.endpoint.action,
                            goal = %goal_id,
                            error = %err,
                            "ros2 action: handler failed",
                        );
                        Ros2Payload::structured(serde_json::json!({ "error": err.to_string() }))
                    }
                };
                if let Err(err) = self.link.send(Ros2Command::ActionResult { goal_id, payload }) {
                    tracing::warn!(
                        action = %self.endpoint.action,
                        error = %err,
                        "ros2 action: transport link send failed",
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{MockRos2Transport, Ros2Transport};
    use async_trait::async_trait;
    use serde_json::json;
    use std::time::Duration;

    /// A handler that completes immediately, echoing the goal.
    struct InstantHandler;

    #[async_trait]
    impl ActionHandler for InstantHandler {
        async fn execute(&self, goal: CodecValue) -> Result<CodecValue, Ros2Error> {
            Ok(CodecValue::new(
                json!({ "done": true, "goal": goal.as_value().clone() }),
            ))
        }
    }

    /// A handler that waits before completing — long enough for a test
    /// to cancel the goal first.
    struct SlowHandler;

    #[async_trait]
    impl ActionHandler for SlowHandler {
        async fn execute(&self, _goal: CodecValue) -> Result<CodecValue, Ros2Error> {
            tokio::time::sleep(Duration::from_millis(80)).await;
            Ok(CodecValue::empty())
        }
    }

    fn action_actor(
        sys: &ActorSystem,
        handler: Arc<dyn ActionHandler>,
    ) -> (ActorRef<Ros2ActionMsg>, crate::transport::MockRos2Handle) {
        let (transport, control) = MockRos2Transport::new();
        let (link, _event_rx) = transport.start();
        let endpoint = Ros2ActionEndpoint::server("/arm/traj", "control_msgs/action/FollowJointTrajectory");
        let actor = sys
            .actor_of(
                Props::create(move || {
                    Ros2ActionActor::new(endpoint.clone(), Arc::clone(&handler), link.clone())
                }),
                "action",
            )
            .unwrap();
        (actor, control)
    }

    #[tokio::test]
    async fn action_actor_runs_a_goal_to_a_result() {
        let sys = ActorSystem::create("act-test", Config::empty()).await.unwrap();
        let (actor, mut control) = action_actor(&sys, Arc::new(InstantHandler));

        actor.tell(Ros2ActionMsg::Goal {
            goal_id: GoalId::from("g1"),
            payload: Ros2Payload::structured(json!({ "target": 1.0 })),
        });
        let command = tokio::time::timeout(Duration::from_secs(1), control.next_command())
            .await
            .expect("no result reached the transport")
            .expect("link closed");
        match command {
            Ros2Command::ActionResult { goal_id, payload } => {
                assert_eq!(goal_id, GoalId::from("g1"));
                assert_eq!(payload.as_structured().unwrap()["done"], json!(true));
            }
            other => panic!("expected ActionResult, got {other:?}"),
        }
        sys.terminate().await;
    }

    #[tokio::test]
    async fn action_actor_drops_the_result_of_a_cancelled_goal() {
        let sys = ActorSystem::create("act-cancel-test", Config::empty())
            .await
            .unwrap();
        let (actor, mut control) = action_actor(&sys, Arc::new(SlowHandler));

        actor.tell(Ros2ActionMsg::Goal {
            goal_id: GoalId::from("g1"),
            payload: Ros2Payload::empty(),
        });
        // Cancel before the slow handler finishes.
        tokio::time::sleep(Duration::from_millis(20)).await;
        actor.tell(Ros2ActionMsg::Cancel {
            goal_id: GoalId::from("g1"),
        });

        // No ActionResult should arrive — the goal was cancelled.
        let outcome = tokio::time::timeout(Duration::from_millis(250), control.next_command()).await;
        assert!(outcome.is_err(), "a cancelled goal must not publish a result");
        sys.terminate().await;
    }
}
