//! [`SdrActor`] — the configuration wrapper that promotes an
//! [`SdrBackend`] into a supervised atomr actor, and [`SdrActorRef`]
//! — the typed handle returned by `.spawn`.
//!
//! This mirrors the two-form pattern used by
//! [`atomr_physical_sensing::SensorActor`] and the projection actors:
//! you can call [`SdrActor::snapshot`] hardware-free in tests, or
//! [`SdrActor::spawn`] to put a live, supervised SDR into your atomr
//! actor system.

use std::sync::Arc;
use std::time::Duration;

use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_physical_core::{DeviceId, PhysicalError, Result};
use tokio::sync::{broadcast, mpsc, oneshot};

use crate::config::SdrParams;
use crate::driver::SdrBackend;
use crate::iq::IqChunk;
use crate::messages::SdrMsg;
use crate::runner::SdrRunner;

/// Default broadcast capacity — at 4 MS/s with 256 KiB chunks
/// (~32 ms of audio bandwidth), 256 chunks buffer roughly 8 s of
/// stream for slow subscribers.
pub const DEFAULT_BROADCAST_CAPACITY: usize = 256;

/// Default ask timeout for non-RX control operations (tune, health,
/// params, stop).
pub const DEFAULT_ASK_TIMEOUT: Duration = Duration::from_secs(5);

/// Wrap an [`SdrBackend`] into a supervised atomr actor.
///
/// Two ways to use the result:
///
/// 1. **Direct** — call [`SdrActor::snapshot`] to take a hardware-free
///    capture of N samples. Useful for one-shot checks and the
///    in-process testkit. There is no `sample` method (SDR is
///    inherently streaming).
/// 2. **Supervised** — call [`SdrActor::spawn`] (or
///    [`SdrActor::spawn_under`]). The returned [`SdrActorRef`]
///    exposes the mailbox and a broadcast channel of [`IqChunk`]s.
#[derive(Clone)]
pub struct SdrActor {
    driver: Arc<dyn SdrBackend>,
    initial_params: SdrParams,
    broadcast_capacity: usize,
    auto_start_rx: bool,
}

impl SdrActor {
    /// Wrap a backend with the default RX params and broadcast depth.
    pub fn new(driver: Arc<dyn SdrBackend>) -> Self {
        Self {
            driver,
            initial_params: SdrParams::default_rx(),
            broadcast_capacity: DEFAULT_BROADCAST_CAPACITY,
            auto_start_rx: false,
        }
    }

    /// Override the initial parameter set the actor applies in
    /// `pre_start`.
    pub fn with_params(mut self, params: SdrParams) -> Self {
        self.initial_params = params;
        self
    }

    /// Override the per-actor broadcast channel depth.
    pub fn with_broadcast_capacity(mut self, capacity: usize) -> Self {
        self.broadcast_capacity = capacity.max(1);
        self
    }

    /// Start the RX loop automatically in `pre_start`. Off by default
    /// — you usually want to subscribe before the firehose opens.
    pub fn auto_start_rx(mut self, on: bool) -> Self {
        self.auto_start_rx = on;
        self
    }

    /// The id of the wrapped device.
    pub fn id(&self) -> DeviceId {
        self.driver.descriptor().id.clone()
    }

    /// The current parameter set this actor will start with.
    pub fn params(&self) -> &SdrParams {
        &self.initial_params
    }

    /// Take a hardware-free capture of approximately `n_samples`
    /// sample-pairs and return the first chunk. Drives the backend
    /// directly — no actor system needed.
    ///
    /// On the rs-hackrf backend the chunk size is fixed at 256 KiB
    /// (128 K sample pairs), so `n_samples` is a *minimum*: the
    /// snapshot returns once it has collected at least that many.
    pub async fn snapshot(&self, n_samples: usize) -> Result<IqChunk> {
        self.driver.apply(&self.initial_params).await?;
        let (tx, mut rx) = mpsc::channel::<IqChunk>(8);
        self.driver.start_rx(tx).await?;
        let mut accumulated: Vec<i8> = Vec::with_capacity(n_samples * 2);
        let mut meta: Option<IqChunk> = None;
        while accumulated.len() / 2 < n_samples {
            match tokio::time::timeout(Duration::from_secs(5), rx.recv()).await {
                Ok(Some(chunk)) => {
                    accumulated.extend_from_slice(&chunk.samples);
                    if meta.is_none() {
                        meta = Some(chunk);
                    }
                }
                _ => break,
            }
        }
        self.driver.stop_rx().await?;
        let template = meta.ok_or_else(|| {
            PhysicalError::Fault("sdr snapshot produced no chunks".into())
        })?;
        Ok(IqChunk {
            sequence: 0,
            captured_at: template.captured_at,
            centre_hz: template.centre_hz,
            sample_rate_hz: template.sample_rate_hz,
            samples: Arc::from(accumulated),
        })
    }

    /// Promote this actor into a live supervised atomr actor under
    /// `system`, registered at `name`.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<SdrActorRef, ActorSystemError> {
        let (props, broadcast_tx, id) = self.into_runner_props();
        let actor_ref = system.actor_of(props, name)?;
        Ok(SdrActorRef {
            inner: actor_ref,
            broadcast_tx,
            id,
        })
    }

    /// Promote this actor into a supervised atomr actor as a **child**
    /// of the parent actor `P`.
    pub fn spawn_under<P: Actor>(self, ctx: &mut Context<P>, name: &str) -> Result<SdrActorRef> {
        let (props, broadcast_tx, id) = self.into_runner_props();
        let actor_ref = ctx
            .spawn(props, name)
            .map_err(|e| PhysicalError::Fault(format!("sdr child spawn failed: {e}")))?;
        Ok(SdrActorRef {
            inner: actor_ref,
            broadcast_tx,
            id,
        })
    }

    fn into_runner_props(self) -> (Props<SdrRunner>, broadcast::Sender<IqChunk>, DeviceId) {
        let id = self.id();
        let (broadcast_tx, _) = broadcast::channel(self.broadcast_capacity);
        let bx = broadcast_tx.clone();
        let driver = self.driver;
        let params = self.initial_params;
        let auto = self.auto_start_rx;
        let props = Props::create(move || SdrRunner::new(driver.clone(), params.clone(), bx.clone(), auto));
        (props, broadcast_tx, id)
    }
}

/// Typed handle to a spawned [`SdrActor`]. Cheap to clone.
#[derive(Clone)]
pub struct SdrActorRef {
    inner: ActorRef<SdrMsg>,
    broadcast_tx: broadcast::Sender<IqChunk>,
    id: DeviceId,
}

impl SdrActorRef {
    /// The id of the wrapped device.
    pub fn id(&self) -> &DeviceId {
        &self.id
    }

    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<SdrMsg> {
        &self.inner
    }

    /// Subscribe to the IQ stream. Each chunk pulled off the device
    /// is fanned out to every live subscriber.
    pub fn subscribe(&self) -> broadcast::Receiver<IqChunk> {
        self.broadcast_tx.subscribe()
    }

    /// Send a new parameter set. While streaming, only the
    /// live-tunable subset is pushed to hardware; the rest of the
    /// fields are stashed and applied on the next stop/start cycle.
    pub async fn tune(&self, params: SdrParams) -> Result<()> {
        self.inner
            .ask_with(
                |reply| SdrMsg::Tune { params, reply },
                DEFAULT_ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_physical)?
    }

    /// Ask the runner to begin RX streaming.
    pub async fn start_rx(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| SdrMsg::StartRx { reply }, DEFAULT_ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }

    /// Ask the runner to stop RX streaming.
    pub async fn stop_rx(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| SdrMsg::StopRx { reply }, DEFAULT_ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }

    /// Submit a transmit burst. Returns `Unsupported` on the current
    /// backend.
    pub async fn transmit(&self, samples: Arc<[i8]>) -> Result<()> {
        self.inner
            .ask_with(
                |reply| SdrMsg::Transmit { samples, reply },
                DEFAULT_ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_physical)?
    }

    /// Fetch the actor's current parameter snapshot.
    pub async fn params(&self) -> Result<SdrParams> {
        self.inner
            .ask_with(|reply| SdrMsg::Params { reply }, DEFAULT_ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)
    }

    /// Run the driver's health check via the actor.
    pub async fn health(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| SdrMsg::Health { reply }, DEFAULT_ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }
}

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("sdr actor ask failed: {e:?}"))
}

/// One-shot wrapper a runner uses to reply to an ask.
pub(crate) fn reply<T>(tx: oneshot::Sender<T>, value: T) {
    let _ = tx.send(value);
}
