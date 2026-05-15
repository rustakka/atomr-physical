//! Supervised CAN bus owner over `socketcan`.
//!
//! One [`CanBusActor`] owns a SocketCAN socket. Child drivers see
//! received frames through a [`tokio::sync::broadcast`] fan-out and send
//! frames through the actor's mailbox so writes are serialised at the
//! socket boundary.
//!
//! The actor's [`pre_start`] hook opens the socket and spawns a
//! `spawn_blocking` reader task that pushes inbound frames onto the
//! broadcast channel. Writes are dispatched through the actor's
//! mailbox; the actual `socket.write_frame` call runs under
//! [`tokio::task::block_in_place`] because the SocketCAN write call is
//! blocking.
//!
//! [`pre_start`]: atomr_core::actor::Actor::pre_start

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use socketcan::{CanFrame, CanSocket, Frame, Socket};
use tokio::sync::{broadcast, oneshot};

use crate::error::{HalError, Result};

/// Default capacity of the per-bus broadcast channel. Sized to a couple
/// of seconds of headroom at 100 Hz bus traffic.
const DEFAULT_BROADCAST_CAPACITY: usize = 256;

/// Ask timeout for mailbox-driven calls on this actor.
const ASK_TIMEOUT: Duration = Duration::from_secs(5);

/// A supervised CAN bus actor.
///
/// Construct with [`CanBusActor::new`] and promote with
/// [`spawn`](Self::spawn) (or [`spawn_under`](Self::spawn_under) for a
/// child under a parent actor). The returned [`CanBusActorRef`] is the
/// only way to send frames or subscribe to the inbound stream.
#[derive(Clone)]
pub struct CanBusActor {
    interface: String,
    rx_capacity: usize,
}

impl CanBusActor {
    /// Build a CAN bus actor that will open the named SocketCAN
    /// interface (e.g. `"can0"`, `"vcan0"`) on `pre_start`.
    pub fn new(interface: impl Into<String>) -> Self {
        Self {
            interface: interface.into(),
            rx_capacity: DEFAULT_BROADCAST_CAPACITY,
        }
    }

    /// Builder-style: tune the receive broadcast channel's capacity.
    pub fn with_rx_capacity(mut self, cap: usize) -> Self {
        self.rx_capacity = cap.max(1);
        self
    }

    /// Promote into a supervised atomr actor under `system`, registered
    /// at `name`.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<CanBusActorRef, ActorSystemError> {
        let (props, broadcast_tx) = self.into_runner_props();
        let inner = system.actor_of(props, name)?;
        Ok(CanBusActorRef { inner, broadcast_tx })
    }

    /// Promote into a supervised atomr actor as a child of the parent
    /// actor `P` whose [`Context`] is `ctx`.
    pub fn spawn_under<P: Actor>(self, ctx: &mut Context<P>, name: &str) -> Result<CanBusActorRef> {
        let (props, broadcast_tx) = self.into_runner_props();
        let inner = ctx
            .spawn(props, name)
            .map_err(|e| HalError::Bus(format!("can bus child spawn failed: {e}")))?;
        Ok(CanBusActorRef { inner, broadcast_tx })
    }

    fn into_runner_props(self) -> (Props<CanBusRunner>, broadcast::Sender<CanFrame>) {
        let (broadcast_tx, _) = broadcast::channel::<CanFrame>(self.rx_capacity);
        let bx = broadcast_tx.clone();
        let interface = self.interface;
        let props = Props::create(move || CanBusRunner {
            interface: interface.clone(),
            broadcast_tx: bx.clone(),
            socket: None,
            reader_alive: None,
        });
        (props, broadcast_tx)
    }
}

/// Typed handle to a spawned [`CanBusActor`].
///
/// Cheap to clone; `send_frame` goes over the actor's mailbox, while
/// `subscribe` taps the broadcast fan-out directly (no mailbox hop).
#[derive(Clone)]
pub struct CanBusActorRef {
    inner: ActorRef<CanBusMsg>,
    broadcast_tx: broadcast::Sender<CanFrame>,
}

impl CanBusActorRef {
    /// Dispatch a frame onto the bus. The reply unblocks once the
    /// socket has accepted the frame.
    pub async fn send_frame(&self, frame: CanFrame) -> Result<()> {
        self.inner
            .ask_with(|reply| CanBusMsg::Send { frame, reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_hal)?
    }

    /// Subscribe to the full inbound frame stream.
    pub fn subscribe(&self) -> broadcast::Receiver<CanFrame> {
        self.broadcast_tx.subscribe()
    }

    /// Subscribe with a server-side ID filter — only frames whose raw
    /// ID satisfies `(id & id_mask) == id_match` reach the receiver.
    pub fn subscribe_filter(&self, id_mask: u32, id_match: u32) -> FilteredCanReceiver {
        FilteredCanReceiver {
            inner: self.broadcast_tx.subscribe(),
            id_mask,
            id_match,
        }
    }

    /// Probe the bus actor for liveness.
    pub async fn health_check(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| CanBusMsg::Health { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_hal)?
    }

    /// Construct a [`CanBusActorRef`] directly from a fabricated
    /// mailbox + broadcast pair. Used by the loopback test double so
    /// drivers can be exercised without opening a real socket.
    #[cfg(test)]
    pub(crate) fn from_channels(
        inner: ActorRef<CanBusMsg>,
        broadcast_tx: broadcast::Sender<CanFrame>,
    ) -> Self {
        Self { inner, broadcast_tx }
    }
}

/// A [`broadcast::Receiver`] wrapper that drops frames not matching a
/// CAN id filter. Used by drivers that only care about replies from
/// their own node.
pub struct FilteredCanReceiver {
    inner: broadcast::Receiver<CanFrame>,
    id_mask: u32,
    id_match: u32,
}

impl FilteredCanReceiver {
    /// Receive the next matching frame.
    pub async fn recv(&mut self) -> Result<CanFrame> {
        loop {
            let frame = self
                .inner
                .recv()
                .await
                .map_err(|e| HalError::Bus(format!("can recv: {e}")))?;
            if (frame.raw_id() & self.id_mask) == self.id_match {
                return Ok(frame);
            }
        }
    }
}

/// The mailbox protocol of a live [`CanBusActor`].
pub enum CanBusMsg {
    /// Send a frame on the bus, replying with the write result.
    Send {
        /// Frame to dispatch.
        frame: CanFrame,
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Probe for liveness — replies `Ok(())` if the reader task is
    /// still alive and the socket is open.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
}

fn ask_to_hal(e: atomr_core::actor::AskError) -> HalError {
    HalError::Bus(format!("can bus actor ask failed: {e:?}"))
}

/// Internal Actor implementation backing a spawned [`CanBusActor`].
struct CanBusRunner {
    interface: String,
    broadcast_tx: broadcast::Sender<CanFrame>,
    socket: Option<Arc<CanSocket>>,
    /// Sentinel handle on the reader task; checked from `Health` to
    /// detect a dead reader.
    reader_alive: Option<tokio::task::JoinHandle<()>>,
}

#[async_trait]
impl Actor for CanBusRunner {
    type Msg = CanBusMsg;

    async fn pre_start(&mut self, _ctx: &mut Context<Self>) {
        match CanSocket::open(&self.interface) {
            Ok(socket) => {
                let socket = Arc::new(socket);
                self.socket = Some(socket.clone());
                let bx = self.broadcast_tx.clone();
                let iface = self.interface.clone();
                // Blocking reader loop. We spawn it on the blocking
                // pool because SocketCAN read is blocking. If no
                // subscribers exist, `send` errors silently — that is
                // not a failure condition.
                let handle = tokio::task::spawn_blocking(move || {
                    loop {
                        match socket.read_frame() {
                            Ok(frame) => {
                                if bx.send(frame).is_err() {
                                    // No subscribers — keep reading.
                                }
                            }
                            Err(e) => {
                                // Hard read error: log and exit the
                                // loop. atomr-physical's supervisor
                                // will see the resulting `Health`
                                // failure on the next probe.
                                tracing::warn!(
                                    iface = %iface,
                                    error = %e,
                                    "can reader exiting on error"
                                );
                                break;
                            }
                        }
                    }
                });
                self.reader_alive = Some(handle);
            }
            Err(e) => {
                tracing::error!(
                    iface = %self.interface,
                    error = %e,
                    "failed to open SocketCAN interface"
                );
            }
        }
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: CanBusMsg) {
        match msg {
            CanBusMsg::Send { frame, reply } => {
                let result = match &self.socket {
                    Some(sock) => {
                        let sock = sock.clone();
                        // `write_frame` is blocking; run it under
                        // `block_in_place` so we don't park other tasks
                        // on the multi-threaded runtime.
                        tokio::task::block_in_place(|| sock.write_frame(&frame))
                            .map_err(|e| HalError::Bus(format!("can write: {e}")))
                    }
                    None => Err(HalError::NotConfigured("can socket not open".into())),
                };
                let _ = reply.send(result);
            }
            CanBusMsg::Health { reply } => {
                let alive = self.socket.is_some()
                    && self
                        .reader_alive
                        .as_ref()
                        .map(|h| !h.is_finished())
                        .unwrap_or(false);
                let result = if alive {
                    Ok(())
                } else {
                    Err(HalError::Bus("can reader not alive".into()))
                };
                let _ = reply.send(result);
            }
        }
    }
}
