//! **atomr-physical-sdr** — Software-Defined Radio as a supervised
//! atomr actor.
//!
//! Currently backed by [`rs-hackrf`](https://crates.io/crates/rs-hackrf)
//! against the HackRF One. The crate name is intentionally generic so
//! additional backends (e.g. an RTL-SDR adapter) can plug into the
//! same actor surface in a follow-up.
//!
//! ## Two-form pattern
//!
//! 1. **Direct** — open a [`HackRfDriver`] and call
//!    [`SdrActor::snapshot`] for a one-shot capture. Useful in tests
//!    and one-line scripts; no actor runtime required.
//! 2. **Supervised** — call [`SdrActor::spawn`] to promote the actor
//!    into an atomr [`ActorSystem`](atomr_core::actor::ActorSystem).
//!    The returned [`SdrActorRef`] is a typed handle: subscribe to
//!    [`IqChunk`]s on a [`tokio::sync::broadcast`] channel, tune
//!    mid-stream, request transmit bursts.
//!
//! ## Streaming
//!
//! IQ flows over a `tokio::sync::broadcast::Sender<IqChunk>`. Each
//! chunk carries a monotonic sequence number, a host-side capture
//! timestamp, the centre frequency and sample rate in effect at
//! capture time, and the raw `Arc<[i8]>` interleaved IQ buffer. The
//! `Arc<[i8]>` shape lets every subscriber (live consumer, SigMF
//! writer, future ROS2 bridge) share the buffer without copying.
//!
//! ## Persistence
//!
//! With the `sigmf` cargo feature enabled, [`SigmfWriter`] consumes
//! the broadcast channel and lays down a SigMF-compatible pair on
//! disk: `<base>.sigmf-data` (raw `ci8_le` samples) and
//! `<base>.sigmf-meta` (JSON header with a per-tune `captures[]`
//! entry). The format is the one GNU Radio, `inspectrum`, and `gqrx`
//! read directly.
//!
//! ## What's *not* here
//!
//! * **TX** — `rs-hackrf` 0.4 is RX-only. [`SdrActorRef::transmit`]
//!   returns `Unsupported`. The signature is on the surface so callers
//!   can integrate today and won't break when TX lands upstream.
//! * **Sweep mode** — HackRF's hardware FFT sweep needs its own state
//!   machine; a future `SdrSweepActor` will sit alongside this one.
//! * **ROS2 bridging** — IQ has no native `sensor_msgs` shape; a
//!   sister crate `atomr-physical-ros2` will provide that when the
//!   schema lands.
//!
//! The atomr actor runtime is re-exported as [`actor_runtime`] so
//! downstream crates have a single import path for it.

mod actor;
mod config;
mod driver;
mod error;
mod iq;
mod messages;
mod runner;

#[cfg(feature = "sigmf")]
mod persist;

pub use actor::{
    SdrActor, SdrActorRef, DEFAULT_ASK_TIMEOUT, DEFAULT_BROADCAST_CAPACITY,
};
pub use config::{
    SdrParams, MAX_CENTRE_HZ, MAX_LNA_GAIN_DB, MAX_SAMPLE_RATE_HZ, MAX_VGA_GAIN_DB,
    MIN_CENTRE_HZ, MIN_SAMPLE_RATE_HZ,
};
pub use driver::{HackRfDriver, HackRfInfo, SdrBackend, DEFAULT_MODEL};
pub use error::{SdrError, SdrResult};
pub use iq::IqChunk;
pub use messages::SdrMsg;

#[cfg(feature = "sigmf")]
pub use persist::{
    persist_until_eos, PersistConfig, SigmfCapture, SigmfMeta, SigmfWriter,
};

/// Re-export of the atomr actor runtime this crate builds on.
pub use atomr_core as actor_runtime;
