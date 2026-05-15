//! Generic state-machine actor.
//!
//! [`Fsm`] is the user-facing trait — implement it to describe the
//! transition function. [`FsmActor`] wraps that implementation into a
//! supervised atomr actor with a broadcast of state-transition events.
//! Every successful transition (a `step` that returns `Some(_)`) is
//! broadcast to all subscribers; transitions that return `None` (i.e.
//! events the FSM ignores) are silent.

use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_physical_core::{PhysicalError, Result};
use tokio::sync::{broadcast, oneshot};

/// A user-defined finite state machine.
///
/// `State` is the value broadcast on every transition; `Event` is the
/// input the actor receives over its mailbox.
///
/// The trait requires `Clone + Send + Sync` so the supervised
/// [`FsmActor`] can rebuild the FSM on actor restart — atomr's
/// `Props` factory must be re-callable.
pub trait Fsm: Clone + Send + Sync + 'static {
    /// The FSM's state type.
    type State: Clone + Send + Sync + 'static;
    /// The FSM's event type.
    type Event: Send + 'static;
    /// The initial state to start in.
    fn initial(&self) -> Self::State;
    /// Compute the next state from `state` and `event`. Return `None`
    /// to indicate the event should be ignored — no broadcast is
    /// emitted in that case.
    fn step(&mut self, state: &Self::State, event: Self::Event) -> Option<Self::State>;
}

const BROADCAST_CAPACITY: usize = 64;

/// Mailbox protocol of an [`FsmActor`].
pub enum FsmMsg<F: Fsm> {
    /// Inject an event into the FSM.
    Event(F::Event),
    /// Ask for the current state.
    Snapshot {
        /// One-shot reply channel.
        reply: oneshot::Sender<F::State>,
    },
    /// Subscribe to the transition broadcast.
    Subscribe {
        /// One-shot reply channel carrying the receiver back.
        reply: oneshot::Sender<broadcast::Receiver<F::State>>,
    },
}

/// A supervised FSM.
pub struct FsmActor<F: Fsm> {
    fsm: F,
}

impl<F: Fsm> FsmActor<F> {
    /// Wrap an [`Fsm`] in the actor adapter.
    pub fn new(fsm: F) -> Self {
        Self { fsm }
    }

    /// Promote this FSM into a supervised atomr actor.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<FsmActorRef<F>, ActorSystemError> {
        let (broadcast_tx, _) = broadcast::channel::<F::State>(BROADCAST_CAPACITY);
        let tx_for_factory = broadcast_tx.clone();
        let fsm = self.fsm;
        let props = Props::create(move || {
            let fsm = fsm.clone();
            let state = fsm.initial();
            FsmRunner {
                fsm,
                state,
                broadcast_tx: tx_for_factory.clone(),
            }
        });
        let inner = system.actor_of(props, name)?;
        Ok(FsmActorRef {
            inner,
            broadcast_tx,
        })
    }
}

/// A typed handle to a spawned [`FsmActor`].
pub struct FsmActorRef<F: Fsm> {
    inner: ActorRef<FsmMsg<F>>,
    broadcast_tx: broadcast::Sender<F::State>,
}

impl<F: Fsm> Clone for FsmActorRef<F> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
            broadcast_tx: self.broadcast_tx.clone(),
        }
    }
}

impl<F: Fsm> FsmActorRef<F> {
    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<FsmMsg<F>> {
        &self.inner
    }

    /// Inject an event into the FSM (fire-and-forget).
    pub fn send(&self, event: F::Event) {
        self.inner.tell(FsmMsg::Event(event));
    }

    /// Ask for the current state.
    pub async fn snapshot(&self) -> Result<F::State> {
        self.inner
            .ask_with(|reply| FsmMsg::Snapshot { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)
    }

    /// Subscribe to the transition broadcast. The returned receiver
    /// observes every successful state transition — events that
    /// returned `None` from `Fsm::step` are not emitted.
    pub fn subscribe(&self) -> broadcast::Receiver<F::State> {
        self.broadcast_tx.subscribe()
    }
}

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("fsm actor ask failed: {e:?}"))
}

/// Internal `Actor` backing an [`FsmActor`].
struct FsmRunner<F: Fsm> {
    fsm: F,
    state: F::State,
    broadcast_tx: broadcast::Sender<F::State>,
}

#[async_trait]
impl<F: Fsm> Actor for FsmRunner<F> {
    type Msg = FsmMsg<F>;

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: FsmMsg<F>) {
        match msg {
            FsmMsg::Event(event) => {
                if let Some(next) = self.fsm.step(&self.state, event) {
                    self.state = next.clone();
                    // Errors mean no subscribers — not a failure.
                    let _ = self.broadcast_tx.send(next);
                }
            }
            FsmMsg::Snapshot { reply } => {
                let _ = reply.send(self.state.clone());
            }
            FsmMsg::Subscribe { reply } => {
                let _ = reply.send(self.broadcast_tx.subscribe());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_core::actor::ActorSystem;

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum DemoState {
        Off,
        Idle,
        Running,
    }

    enum DemoEvent {
        PowerOn,
        Start,
        Stop,
    }

    #[derive(Clone)]
    struct DemoFsm;

    impl Fsm for DemoFsm {
        type State = DemoState;
        type Event = DemoEvent;
        fn initial(&self) -> DemoState {
            DemoState::Off
        }
        fn step(&mut self, state: &DemoState, event: DemoEvent) -> Option<DemoState> {
            match (state, event) {
                (DemoState::Off, DemoEvent::PowerOn) => Some(DemoState::Idle),
                (DemoState::Idle, DemoEvent::Start) => Some(DemoState::Running),
                (DemoState::Running, DemoEvent::Stop) => Some(DemoState::Idle),
                _ => None,
            }
        }
    }

    #[tokio::test]
    async fn fsm_actor_broadcasts_transitions() {
        let sys = ActorSystem::create("fsm-test", atomr_config::Config::reference())
            .await
            .unwrap();
        let fsm_ref = FsmActor::new(DemoFsm).spawn(&sys, "demo").unwrap();
        let mut rx = fsm_ref.subscribe();
        fsm_ref.send(DemoEvent::PowerOn);
        fsm_ref.send(DemoEvent::Start);
        fsm_ref.send(DemoEvent::Stop);

        let s1 = tokio::time::timeout(Duration::from_millis(500), rx.recv())
            .await
            .unwrap()
            .unwrap();
        let s2 = tokio::time::timeout(Duration::from_millis(500), rx.recv())
            .await
            .unwrap()
            .unwrap();
        let s3 = tokio::time::timeout(Duration::from_millis(500), rx.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(s1, DemoState::Idle);
        assert_eq!(s2, DemoState::Running);
        assert_eq!(s3, DemoState::Idle);

        let snap = fsm_ref.snapshot().await.unwrap();
        assert_eq!(snap, DemoState::Idle);
        sys.terminate().await;
    }

    #[tokio::test]
    async fn fsm_actor_ignores_unhandled_events() {
        let sys = ActorSystem::create("fsm-noop", atomr_config::Config::reference())
            .await
            .unwrap();
        let fsm_ref = FsmActor::new(DemoFsm).spawn(&sys, "demo2").unwrap();
        let mut rx = fsm_ref.subscribe();
        // Trying to Start from Off should be ignored.
        fsm_ref.send(DemoEvent::Start);
        let recv_result =
            tokio::time::timeout(Duration::from_millis(100), rx.recv()).await;
        assert!(
            recv_result.is_err(),
            "expected no broadcast for unhandled event"
        );
        let snap = fsm_ref.snapshot().await.unwrap();
        assert_eq!(snap, DemoState::Off);
        sys.terminate().await;
    }
}
