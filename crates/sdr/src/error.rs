//! Errors raised by the SDR subsystem.
//!
//! `SdrError` is the crate's internal taxonomy. At the public actor
//! boundary every `SdrError` is folded into
//! [`atomr_physical_core::PhysicalError`] (most often
//! `PhysicalError::Fault`) — the same convention every other adapter
//! crate follows, so a caller holding a [`crate::SdrActorRef`] only ever
//! needs to handle one error type.

use atomr_physical_core::PhysicalError;
use thiserror::Error;

/// Result alias used throughout the SDR crate.
pub type SdrResult<T> = std::result::Result<T, SdrError>;

/// The SDR-specific error taxonomy.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum SdrError {
    /// A configuration value violated a HackRF constraint
    /// (frequency / rate / gain step).
    #[error("invalid SDR parameter: {0}")]
    InvalidParameter(String),

    /// The underlying USB transport reported a failure.
    #[error("hackrf transport: {0}")]
    Transport(String),

    /// An operation was requested in a state that doesn't allow it
    /// — e.g. `start_rx` while already streaming.
    #[error("sdr not in expected state: {0}")]
    BadState(&'static str),

    /// The feature isn't supported by the active backend.
    ///
    /// Returned today for `transmit` (rs-hackrf 0.4 is RX-only).
    #[error("not supported by this SDR backend: {0}")]
    Unsupported(&'static str),

    /// A SigMF persistence operation failed at the filesystem layer.
    #[error("sigmf io: {0}")]
    SigmfIo(String),
}

impl From<SdrError> for PhysicalError {
    fn from(err: SdrError) -> Self {
        PhysicalError::Fault(format!("sdr: {err}"))
    }
}

impl From<rs_hackrf::Error> for SdrError {
    fn from(err: rs_hackrf::Error) -> Self {
        SdrError::Transport(format!("{err}"))
    }
}
