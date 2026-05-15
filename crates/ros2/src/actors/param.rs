//! [`Ros2ParamActor`] — the node's parameter mirror.
//!
//! ROS2 parameters are node-global, so there is one `Ros2ParamActor` per
//! node, not one per endpoint. On start it pushes the [`ParamStore`]'s
//! snapshot out to the transport as `Ros2Command::SetParam`s; when an
//! external client changes a parameter, the transport delivers a
//! `Ros2Event::ParamChanged` and the actor applies it back onto the
//! store. The mirror is read-write.

use std::sync::Arc;

use crate::actor::prelude::*;
use crate::actors::ParamStore;
use crate::param::ParamValue;
use crate::transport::{Ros2Command, Ros2Link};

/// Messages a [`Ros2ParamActor`] handles.
pub enum Ros2ParamMsg {
    /// Push the store's full snapshot out to the ROS2 node.
    SyncSnapshot,
    /// An external client changed a parameter — apply it to the store.
    Changed {
        /// The parameter name.
        name: String,
        /// The new value.
        value: ParamValue,
    },
}

/// The node's parameter mirror: declares the [`ParamStore`]'s values on
/// the ROS2 node and applies external changes back to the store.
pub struct Ros2ParamActor {
    store: Arc<dyn ParamStore>,
    link: Ros2Link,
}

impl Ros2ParamActor {
    /// Construct a parameter actor mirroring `store` through `link`.
    pub fn new(store: Arc<dyn ParamStore>, link: Ros2Link) -> Self {
        Self { store, link }
    }
}

#[async_trait]
impl Actor for Ros2ParamActor {
    type Msg = Ros2ParamMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        // Mirror the initial snapshot as soon as the actor is up.
        ctx.self_ref().tell(Ros2ParamMsg::SyncSnapshot);
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: Ros2ParamMsg) {
        match msg {
            Ros2ParamMsg::SyncSnapshot => {
                for (name, value) in self.store.snapshot() {
                    if let Err(err) = self.link.send(Ros2Command::SetParam { name, value }) {
                        tracing::warn!(
                            error = %err,
                            "ros2 param: transport link send failed during snapshot sync",
                        );
                    }
                }
            }
            Ros2ParamMsg::Changed { name, value } => match self.store.apply(&name, value) {
                Ok(()) => tracing::debug!(param = %name, "ros2 param: external change applied"),
                Err(err) => tracing::warn!(
                    param = %name,
                    error = %err,
                    "ros2 param: external change rejected",
                ),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::Ros2Error;
    use crate::transport::{MockRos2Transport, Ros2Transport};
    use std::collections::HashMap;
    use std::sync::Mutex;
    use std::time::Duration;

    /// An in-memory parameter store seeded with two values.
    struct MapStore {
        values: Mutex<HashMap<String, ParamValue>>,
    }

    impl MapStore {
        fn seeded() -> Arc<Self> {
            let mut values = HashMap::new();
            values.insert("shoulder.period_ms".into(), ParamValue::Int(100));
            values.insert("envelope.clamp".into(), ParamValue::Bool(true));
            Arc::new(Self {
                values: Mutex::new(values),
            })
        }
    }

    impl ParamStore for MapStore {
        fn snapshot(&self) -> Vec<(String, ParamValue)> {
            self.values
                .lock()
                .unwrap()
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect()
        }

        fn apply(&self, name: &str, value: ParamValue) -> Result<(), Ros2Error> {
            let mut values = self.values.lock().unwrap();
            if values.contains_key(name) {
                values.insert(name.to_string(), value);
                Ok(())
            } else {
                Err(Ros2Error::InvalidPlan(format!("unknown parameter {name}")))
            }
        }
    }

    #[tokio::test]
    async fn param_actor_mirrors_the_snapshot_on_start() {
        let sys = ActorSystem::create("param-test", Config::empty()).await.unwrap();
        let (transport, mut control) = MockRos2Transport::new();
        let (link, _event_rx) = transport.start();
        let store = MapStore::seeded();
        let store_dyn: Arc<dyn ParamStore> = store.clone();
        let _actor = sys
            .actor_of(
                Props::create(move || Ros2ParamActor::new(Arc::clone(&store_dyn), link.clone())),
                "param",
            )
            .unwrap();

        // Two seeded values → two SetParam commands.
        let mut seen = 0;
        for _ in 0..2 {
            let command = tokio::time::timeout(Duration::from_secs(1), control.next_command())
                .await
                .expect("param snapshot did not reach the transport")
                .expect("link closed");
            assert!(matches!(command, Ros2Command::SetParam { .. }));
            seen += 1;
        }
        assert_eq!(seen, 2);
        sys.terminate().await;
    }

    #[tokio::test]
    async fn param_actor_applies_an_external_change_to_the_store() {
        let sys = ActorSystem::create("param-change-test", Config::empty())
            .await
            .unwrap();
        let (transport, _control) = MockRos2Transport::new();
        let (link, _event_rx) = transport.start();
        let store = MapStore::seeded();
        let store_dyn: Arc<dyn ParamStore> = store.clone();
        let actor = sys
            .actor_of(
                Props::create(move || Ros2ParamActor::new(Arc::clone(&store_dyn), link.clone())),
                "param",
            )
            .unwrap();

        actor.tell(Ros2ParamMsg::Changed {
            name: "shoulder.period_ms".into(),
            value: ParamValue::Int(20),
        });

        let mut applied = false;
        for _ in 0..50 {
            if matches!(
                store.values.lock().unwrap().get("shoulder.period_ms"),
                Some(ParamValue::Int(20))
            ) {
                applied = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(applied, "external parameter change was not applied");
        sys.terminate().await;
    }
}
