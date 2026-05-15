//! [`Ros2ServiceActor`] — one actor per served ROS2 service endpoint.
//!
//! A ROS2 service is request/response; it maps onto atomr's `ask`. In
//! the **server** role, this actor receives a `Ros2Event::ServiceRequest`
//! from the transport, `ask`s a [`ServiceHandler`] off-thread via
//! [`pipe_to`], and replies with a `Ros2Command::ServiceResponse`. The
//! **client** role — an atomr actor `ask`ing an external service —
//! lands with the live `rclrs` service client (it needs request
//! correlation the offline transport contract does not yet model).

use std::sync::Arc;

use crate::actor::prelude::*;
use crate::actors::ServiceHandler;
use crate::codec::{CodecValue, Ros2Payload};
use crate::error::Ros2Error;
use crate::service::Ros2ServiceEndpoint;
use crate::transport::{ReqId, Ros2Command, Ros2Link};

/// Messages a [`Ros2ServiceActor`] handles.
pub enum Ros2ServiceMsg {
    /// An inbound request from the transport for the served service.
    Request {
        /// Correlates the response.
        request_id: ReqId,
        /// The request payload.
        payload: Ros2Payload,
    },
    /// Internal: the handler's response, piped back after `handle`.
    Responded {
        /// The request being answered.
        request_id: ReqId,
        /// The handler's result.
        result: Result<CodecValue, Ros2Error>,
    },
}

/// One actor per served ROS2 service endpoint: routes requests to a
/// [`ServiceHandler`] and replies through the transport.
pub struct Ros2ServiceActor {
    endpoint: Ros2ServiceEndpoint,
    handler: Arc<dyn ServiceHandler>,
    link: Ros2Link,
}

impl Ros2ServiceActor {
    /// Construct a service actor for `endpoint`, serving requests with
    /// `handler` and replying through `link`.
    pub fn new(endpoint: Ros2ServiceEndpoint, handler: Arc<dyn ServiceHandler>, link: Ros2Link) -> Self {
        Self {
            endpoint,
            handler,
            link,
        }
    }

    /// Send a response payload back through the transport.
    fn respond(&self, request_id: ReqId, payload: Ros2Payload) {
        if let Err(err) = self
            .link
            .send(Ros2Command::ServiceResponse { request_id, payload })
        {
            tracing::warn!(
                service = %self.endpoint.service,
                error = %err,
                "ros2 service: transport link send failed",
            );
        }
    }
}

/// A structured error payload — a ROS2 service always owes a response,
/// so a handler failure is reported as `{ "error": "…" }` rather than
/// left hanging.
fn error_payload(message: &str) -> Ros2Payload {
    Ros2Payload::structured(serde_json::json!({ "error": message }))
}

#[async_trait]
impl Actor for Ros2ServiceActor {
    type Msg = Ros2ServiceMsg;

    async fn handle(&mut self, ctx: &mut Context<Self>, msg: Ros2ServiceMsg) {
        match msg {
            Ros2ServiceMsg::Request { request_id, payload } => {
                let Some(request) = payload.as_codec_value() else {
                    tracing::warn!(
                        service = %self.endpoint.service,
                        "ros2 service: request payload was not structured",
                    );
                    self.respond(request_id, error_payload("request payload was not structured"));
                    return;
                };
                let handler = Arc::clone(&self.handler);
                pipe_to(
                    async move {
                        Ros2ServiceMsg::Responded {
                            request_id,
                            result: handler.handle(request).await,
                        }
                    },
                    ctx.self_ref().clone(),
                );
            }
            Ros2ServiceMsg::Responded { request_id, result } => {
                let payload = match result {
                    Ok(value) => value.into_payload(),
                    Err(err) => {
                        tracing::warn!(
                            service = %self.endpoint.service,
                            error = %err,
                            "ros2 service: handler failed",
                        );
                        error_payload(&err.to_string())
                    }
                };
                self.respond(request_id, payload);
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

    /// A handler that echoes the request back, tagged `handled: true`.
    struct EchoHandler;

    #[async_trait]
    impl ServiceHandler for EchoHandler {
        async fn handle(&self, request: CodecValue) -> Result<CodecValue, Ros2Error> {
            Ok(CodecValue::new(json!({
                "handled": true,
                "echo": request.as_value().clone(),
            })))
        }
    }

    /// A handler that always fails.
    struct FailingHandler;

    #[async_trait]
    impl ServiceHandler for FailingHandler {
        async fn handle(&self, _request: CodecValue) -> Result<CodecValue, Ros2Error> {
            Err(Ros2Error::InvalidPlan("handler refused".into()))
        }
    }

    async fn serve(handler: Arc<dyn ServiceHandler>, request: Ros2Payload) -> (Ros2Payload, ActorSystem) {
        let sys = ActorSystem::create("svc-test", Config::empty()).await.unwrap();
        let (transport, mut control) = MockRos2Transport::new();
        let (link, _event_rx) = transport.start();
        let endpoint = Ros2ServiceEndpoint::server("/arm/home", "std_srvs/srv/Trigger");
        let actor = sys
            .actor_of(
                Props::create(move || {
                    Ros2ServiceActor::new(endpoint.clone(), Arc::clone(&handler), link.clone())
                }),
                "svc",
            )
            .unwrap();

        actor.tell(Ros2ServiceMsg::Request {
            request_id: 7,
            payload: request,
        });
        let command = tokio::time::timeout(Duration::from_secs(1), control.next_command())
            .await
            .expect("no response reached the transport")
            .expect("link closed");
        match command {
            Ros2Command::ServiceResponse { request_id, payload } => {
                assert_eq!(request_id, 7);
                (payload, sys)
            }
            other => panic!("expected ServiceResponse, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn service_actor_routes_a_request_through_the_handler() {
        let (payload, sys) = serve(
            Arc::new(EchoHandler),
            Ros2Payload::structured(json!({ "ping": 1 })),
        )
        .await;
        let value = payload.as_structured().unwrap();
        assert_eq!(value["handled"], json!(true));
        assert_eq!(value["echo"], json!({ "ping": 1 }));
        sys.terminate().await;
    }

    #[tokio::test]
    async fn service_actor_reports_a_handler_failure_as_an_error_payload() {
        let (payload, sys) = serve(Arc::new(FailingHandler), Ros2Payload::empty()).await;
        let value = payload.as_structured().unwrap();
        assert!(value.get("error").is_some(), "expected an error payload");
        sys.terminate().await;
    }
}
