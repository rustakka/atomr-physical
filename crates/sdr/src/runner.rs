//! [`SdrRunner`] — the private [`Actor`] implementation behind a
//! spawned [`crate::SdrActor`].
//!
//! The runner serialises every operation through the mailbox so the
//! driver's state machine (idle / streaming / closed) doesn't race
//! against concurrent control messages. The streaming IQ chunks ride
//! through the same mailbox, posted by a small forwarder task the
//! runner spawns inside `StartRx`: ordering is therefore deterministic
//! and a supervisor restart drops everything cleanly via `post_stop`.

use std::sync::Arc;

use async_trait::async_trait;
use atomr_core::actor::{Actor, Context};
use atomr_physical_core::{PhysicalError, Result};
use tokio::sync::{broadcast, mpsc};

use crate::actor::reply;
use crate::config::SdrParams;
use crate::driver::SdrBackend;
use crate::iq::IqChunk;
use crate::messages::SdrMsg;

/// Bound on the internal mpsc that the driver pushes chunks into. A
/// chunk is ~256 KiB on the rs-hackrf backend, so 8 in flight =
/// ~2 MiB of buffered headroom before back-pressure hits the USB
/// thread.
const DRIVER_CHANNEL_CAPACITY: usize = 8;

/// The runtime backing a spawned [`crate::SdrActor`].
pub(crate) struct SdrRunner {
    driver: Arc<dyn SdrBackend>,
    params: SdrParams,
    broadcast_tx: broadcast::Sender<IqChunk>,
    auto_start_rx: bool,
    /// Set while RX is active so the forwarder task can be torn down
    /// on `StopRx` / `post_stop`.
    forwarder: Option<tokio::task::JoinHandle<()>>,
    /// Sequence counter applied to chunks before fan-out — replaces
    /// the driver's per-stream sequence so subscribers see a
    /// monotonic id across stop/restart cycles.
    sequence: u64,
}

impl SdrRunner {
    pub(crate) fn new(
        driver: Arc<dyn SdrBackend>,
        params: SdrParams,
        broadcast_tx: broadcast::Sender<IqChunk>,
        auto_start_rx: bool,
    ) -> Self {
        Self {
            driver,
            params,
            broadcast_tx,
            auto_start_rx,
            forwarder: None,
            sequence: 0,
        }
    }

    /// Start RX and spawn a forwarder that routes each chunk back
    /// into the actor's mailbox as `SdrMsg::RxChunk`.
    async fn handle_start_rx(&mut self, ctx: &mut Context<Self>) -> Result<()> {
        if self.forwarder.is_some() {
            return Ok(());
        }
        let (tx, mut rx) = mpsc::channel::<IqChunk>(DRIVER_CHANNEL_CAPACITY);
        self.driver
            .start_rx(tx)
            .await
            .map_err(PhysicalError::from)?;
        let me = ctx.self_ref().clone();
        let forwarder = tokio::spawn(async move {
            while let Some(chunk) = rx.recv().await {
                if me.is_terminated() {
                    break;
                }
                me.tell(SdrMsg::RxChunk(chunk));
            }
        });
        self.forwarder = Some(forwarder);
        Ok(())
    }

    /// Stop RX. Idempotent.
    async fn handle_stop_rx(&mut self) -> Result<()> {
        if let Some(handle) = self.forwarder.take() {
            handle.abort();
        }
        self.driver.stop_rx().await.map_err(PhysicalError::from)?;
        Ok(())
    }

    /// Tune. Streams live-tunable subset if streaming; otherwise
    /// pushes the full config to the device.
    async fn handle_tune(&mut self, params: SdrParams) -> Result<()> {
        params
            .validate()
            .map_err(|e| PhysicalError::Fault(format!("{e}")))?;
        if self.forwarder.is_some() {
            self.driver
                .tune_live(&params)
                .await
                .map_err(PhysicalError::from)?;
        } else {
            self.driver
                .apply(&params)
                .await
                .map_err(PhysicalError::from)?;
        }
        self.params = params;
        Ok(())
    }
}

#[async_trait]
impl Actor for SdrRunner {
    type Msg = SdrMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        if let Err(e) = self.driver.apply(&self.params).await {
            tracing::warn!(error = ?e, "sdr pre_start: initial apply failed");
        }
        if self.auto_start_rx {
            if let Err(e) = self.handle_start_rx(ctx).await {
                tracing::warn!(error = ?e, "sdr pre_start: auto_start_rx failed");
            }
        }
    }

    async fn post_stop(&mut self, _ctx: &mut Context<Self>) {
        if let Some(handle) = self.forwarder.take() {
            handle.abort();
        }
        if let Err(e) = self.driver.stop_rx().await {
            tracing::warn!(error = ?e, "sdr post_stop: stop_rx failed");
        }
    }

    async fn handle(&mut self, ctx: &mut Context<Self>, msg: SdrMsg) {
        match msg {
            SdrMsg::Tune { params, reply: rx } => {
                let result = self.handle_tune(params).await;
                reply(rx, result);
            }
            SdrMsg::StartRx { reply: rx } => {
                let result = self.handle_start_rx(ctx).await;
                reply(rx, result);
            }
            SdrMsg::StopRx { reply: rx } => {
                let result = self.handle_stop_rx().await;
                reply(rx, result);
            }
            SdrMsg::Transmit { samples, reply: rx } => {
                let result = self
                    .driver
                    .transmit(&samples)
                    .await
                    .map_err(PhysicalError::from);
                reply(rx, result);
            }
            SdrMsg::Params { reply: rx } => {
                reply(rx, self.params.clone());
            }
            SdrMsg::Health { reply: rx } => {
                let result = self.driver.health_check().await;
                reply(rx, result);
            }
            SdrMsg::RxChunk(mut chunk) => {
                // Re-stamp with the runner's monotonic sequence so
                // subscribers see a single sequence even across
                // stop/start cycles.
                chunk.sequence = self.sequence;
                self.sequence = self.sequence.wrapping_add(1);
                // `broadcast::send` errors when there are no live
                // subscribers — not a failure condition.
                let _ = self.broadcast_tx.send(chunk);
            }
        }
    }
}
