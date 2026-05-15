//! Bus actors and shared bus types.
//!
//! Each transport family lives in its own submodule and is gated behind
//! the relevant feature flag. The shared bus types (kinds, simple stats)
//! are always available — they carry no transport-specific dependency.

#[cfg(feature = "can")]
pub mod can;
#[cfg(feature = "i2c")]
pub mod i2c;
#[cfg(feature = "spi")]
pub mod spi;

pub use crate::error::{HalError, Result};

/// The transport family a bus actor speaks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusKind {
    /// CAN bus (SocketCAN).
    Can,
    /// I2C bus (Linux i2cdev).
    I2c,
    /// SPI bus (Linux spidev).
    Spi,
}

/// Coarse rolling stats on a bus actor's traffic. Intended for the
/// per-bus `Stats` mailbox reply once that variant is wired up; kept
/// public so downstream code can pattern-match on it.
#[derive(Debug, Clone, Default)]
pub struct BusTransactionStats {
    /// Total frames received.
    pub rx_frames: u64,
    /// Total frames transmitted.
    pub tx_frames: u64,
    /// Total error / drop events observed.
    pub errors: u64,
    /// Timestamp of the last received frame, milliseconds since the
    /// Unix epoch.
    pub last_rx_ms: i64,
}
