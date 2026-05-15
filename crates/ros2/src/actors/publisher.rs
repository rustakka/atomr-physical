//! [`Ros2PublisherActor`] — one actor per publishing topic endpoint.
//!
//! The publisher owns a [`ReadingSource`] and self-paces against it: if
//! the source is rate-based it samples on its period; otherwise it
//! samples on an explicit [`Ros2PublisherMsg::Tick`]. Each reading is
//! handed to the transport as a [`Ros2Command::Publish`].

use std::sync::Arc;

use atomr_physical_core::{Reading, Result};

use crate::actor::prelude::*;
use crate::actors::ReadingSource;
use crate::endpoint::Ros2Endpoint;
use crate::transport::{Ros2Command, Ros2Link};

/// Messages a [`Ros2PublisherActor`] handles.
pub enum Ros2PublisherMsg {
    /// Sample the source once and publish the reading. A rate-based
    /// source re-arms this itself; an on-demand source is driven by an
    /// external `Tick` (e.g. from [`Ros2NodeMsg::TriggerPublish`]).
    ///
    /// [`Ros2NodeMsg::TriggerPublish`]: crate::actors::Ros2NodeMsg::TriggerPublish
    Tick,
    /// Internal: a sample completed and is ready to publish.
    Sampled(Result<Reading>),
}

/// One actor per publishing topic endpoint: pulls [`Reading`]s from a
/// [`ReadingSource`] and pushes them onto the transport.
pub struct Ros2PublisherActor {
    endpoint: Ros2Endpoint,
    source: Arc<dyn ReadingSource>,
    link: Ros2Link,
}

impl Ros2PublisherActor {
    /// Construct a publisher for `endpoint`, pulling from `source` and
    /// publishing through `link`.
    pub fn new(endpoint: Ros2Endpoint, source: Arc<dyn ReadingSource>, link: Ros2Link) -> Self {
        Self {
            endpoint,
            source,
            link,
        }
    }

    /// Spawn a task that samples the source and pipes the result back as
    /// [`Ros2PublisherMsg::Sampled`].
    fn spawn_sample(&self, ctx: &mut Context<Self>) {
        let source = Arc::clone(&self.source);
        pipe_to(
            async move { Ros2PublisherMsg::Sampled(source.next_reading().await) },
            ctx.self_ref().clone(),
        );
    }

    /// If the source is rate-based, schedule the next [`Tick`] after the
    /// sampling period.
    ///
    /// [`Tick`]: Ros2PublisherMsg::Tick
    fn rearm(&self, ctx: &mut Context<Self>) {
        if let Some(period) = self.source.sampling_period() {
            let self_ref = ctx.self_ref().clone();
            pipe_to(
                async move {
                    tokio::time::sleep(period).await;
                    Ros2PublisherMsg::Tick
                },
                self_ref,
            );
        }
    }
}

#[async_trait]
impl Actor for Ros2PublisherActor {
    type Msg = Ros2PublisherMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        // A rate-based source drives itself; an on-demand source waits
        // for an external `Tick`.
        if self.source.sampling_period().is_some() {
            ctx.self_ref().tell(Ros2PublisherMsg::Tick);
        }
    }

    async fn handle(&mut self, ctx: &mut Context<Self>, msg: Ros2PublisherMsg) {
        match msg {
            Ros2PublisherMsg::Tick => {
                self.spawn_sample(ctx);
            }
            Ros2PublisherMsg::Sampled(Ok(reading)) => {
                let command = Ros2Command::Publish {
                    sensor: reading.sensor.clone(),
                    reading,
                };
                if let Err(err) = self.link.send(command) {
                    tracing::warn!(
                        topic = %self.endpoint.topic,
                        error = %err,
                        "ros2 publisher: transport link send failed",
                    );
                }
                self.rearm(ctx);
            }
            Ros2PublisherMsg::Sampled(Err(err)) => {
                // A bad sample is a data error — log it and keep going;
                // the source may recover on the next tick.
                tracing::warn!(
                    topic = %self.endpoint.topic,
                    error = %err,
                    "ros2 publisher: sample failed",
                );
                self.rearm(ctx);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actors::testing::MockReadingSource;
    use crate::transport::MockRos2Transport;
    use atomr_physical_core::{Quantity, SensorId, Unit};
    use std::time::Duration;

    fn reading(sensor: &str) -> Reading {
        Reading {
            sensor: SensorId::from(sensor),
            quantity: Quantity::new(19.5, Unit::Celsius),
            timestamp_ms: 0,
            frame: None,
        }
    }

    #[tokio::test]
    async fn on_demand_publisher_publishes_on_tick() {
        let sys = ActorSystem::create("pub-test", Config::empty()).await.unwrap();
        let (transport, mut handle) = MockRos2Transport::new();
        let (link, _event_rx) = {
            use crate::transport::Ros2Transport;
            transport.start()
        };
        let source: Arc<dyn ReadingSource> = Arc::new(MockReadingSource::new("s1", reading("s1")));
        let endpoint = Ros2Endpoint::publish("/arm/temp", "sensor_msgs/msg/Temperature");
        let publisher = sys
            .actor_of(
                Props::create(move || {
                    Ros2PublisherActor::new(endpoint.clone(), Arc::clone(&source), link.clone())
                }),
                "pub",
            )
            .unwrap();

        publisher.tell(Ros2PublisherMsg::Tick);
        let command = tokio::time::timeout(Duration::from_secs(1), handle.next_command())
            .await
            .expect("publisher did not publish in time")
            .expect("link closed");
        match command {
            Ros2Command::Publish { sensor, reading } => {
                assert_eq!(sensor, SensorId::from("s1"));
                assert_eq!(reading.quantity.value, 19.5);
            }
            other => panic!("expected Publish, got {other:?}"),
        }
        sys.terminate().await;
    }

    #[tokio::test]
    async fn rate_based_publisher_self_paces() {
        let sys = ActorSystem::create("pub-rate-test", Config::empty())
            .await
            .unwrap();
        let (transport, mut handle) = MockRos2Transport::new();
        let (link, _event_rx) = {
            use crate::transport::Ros2Transport;
            transport.start()
        };
        let source: Arc<dyn ReadingSource> =
            Arc::new(MockReadingSource::new("s1", reading("s1")).with_period(Duration::from_millis(20)));
        let endpoint = Ros2Endpoint::publish("/arm/temp", "sensor_msgs/msg/Temperature");
        let _publisher = sys
            .actor_of(
                Props::create(move || {
                    Ros2PublisherActor::new(endpoint.clone(), Arc::clone(&source), link.clone())
                }),
                "pub",
            )
            .unwrap();

        // Two ticks should arrive without anyone driving the actor.
        for _ in 0..2 {
            let command = tokio::time::timeout(Duration::from_secs(1), handle.next_command())
                .await
                .expect("publisher did not self-pace")
                .expect("link closed");
            assert!(matches!(command, Ros2Command::Publish { .. }));
        }
        sys.terminate().await;
    }
}
