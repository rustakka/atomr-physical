//! The mailbox protocol of a live [`crate::SdrActor`].
//!
//! Each public variant carries a [`tokio::sync::oneshot::Sender`] for
//! its reply — the runner mails the response back through that channel
//! and the [`crate::SdrActorRef`] wrapper awaits it with a timeout.
//! `RxChunk` is an *internal* variant: the streaming forwarder posts
//! it from the device's USB thread into the runner so chunks are
//! ordered through the same mailbox as control messages.

use std::sync::Arc;

use atomr_physical_core::Result;
use tokio::sync::oneshot;

use crate::config::SdrParams;
use crate::iq::IqChunk;

/// Messages a [`crate::SdrActor`] understands.
pub enum SdrMsg {
    /// Apply a new parameter set. While streaming, only `centre_hz`,
    /// `lna_gain_db`, `vga_gain_db` and `amp_enable` are taken live;
    /// the rest require a stop/restart cycle and the runner answers
    /// with `Err(PhysicalError::Fault(...))` if it can't honour them.
    Tune {
        /// The new parameter set. Validated by the runner before being
        /// pushed to hardware.
        params: SdrParams,
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Open the device endpoint and start the RX streaming loop.
    StartRx {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Stop the RX streaming loop and release the endpoint.
    StopRx {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Submit a TX burst. Always returns `Err(Unsupported)` on the
    /// current rs-hackrf backend — kept on the surface so callers can
    /// integrate against it today and we don't break them when TX
    /// lands.
    Transmit {
        /// Interleaved I/Q samples to push.
        samples: Arc<[i8]>,
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Return the currently-active parameter set.
    Params {
        /// One-shot reply channel.
        reply: oneshot::Sender<SdrParams>,
    },
    /// Run the driver's health check.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Internal — the streaming forwarder hands the runner a chunk it
    /// just pulled off the USB thread. Not part of the public ask
    /// surface.
    RxChunk(IqChunk),
}
