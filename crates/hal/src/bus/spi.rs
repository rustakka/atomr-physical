//! Direct SPI device handle.
//!
//! SPI on Linux is per-chip-select; the kernel arbitrates the bus, so
//! this is **not** an actor — just a thin handle around
//! `linux_embedded_hal::SpidevDevice`. The four operations the AS5048A
//! driver needs are exposed (`transfer`, `write`, `read`); concurrent
//! callers are kept honest by a [`parking_lot::Mutex`] guarding the
//! inner device.

use std::path::Path;
use std::sync::Arc;

use linux_embedded_hal::spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use parking_lot::Mutex;

use crate::error::{HalError, Result};

/// SPI clock-phase / clock-polarity mode.
#[derive(Debug, Clone, Copy)]
pub enum SpiMode {
    /// CPOL = 0, CPHA = 0.
    Mode0,
    /// CPOL = 0, CPHA = 1.
    Mode1,
    /// CPOL = 1, CPHA = 0.
    Mode2,
    /// CPOL = 1, CPHA = 1.
    Mode3,
}

impl SpiMode {
    fn flags(self) -> SpiModeFlags {
        match self {
            SpiMode::Mode0 => SpiModeFlags::SPI_MODE_0,
            SpiMode::Mode1 => SpiModeFlags::SPI_MODE_1,
            SpiMode::Mode2 => SpiModeFlags::SPI_MODE_2,
            SpiMode::Mode3 => SpiModeFlags::SPI_MODE_3,
        }
    }
}

/// A SPI device handle. Cheap to clone — both clones share the same
/// underlying Linux spidev character device, mutexed.
#[derive(Clone)]
pub struct SpiDevice {
    inner: Arc<Mutex<SpiInner>>,
}

enum SpiInner {
    /// A real Linux spidev device.
    Hw(Spidev),
    /// A loopback test double (see `crate::loopback`).
    #[cfg(test)]
    Loopback(LoopbackInner),
}

#[cfg(test)]
pub(crate) struct LoopbackInner {
    /// Pre-populated read responses, one Vec per `transfer`/`read`.
    pub responses: std::collections::VecDeque<Vec<u8>>,
    /// Every `write`/`transfer` tx buffer recorded for assertion.
    pub written: Vec<Vec<u8>>,
}

impl SpiDevice {
    /// Open a Linux spidev character device with the requested speed
    /// and mode.
    pub fn open<P: AsRef<Path>>(path: P, max_speed_hz: u32, mode: SpiMode) -> Result<Self> {
        let mut dev = Spidev::open(path).map_err(|e| HalError::Bus(format!("spi open: {e}")))?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(max_speed_hz)
            .mode(mode.flags())
            .build();
        dev.configure(&options)
            .map_err(|e| HalError::Bus(format!("spi configure: {e}")))?;
        Ok(Self {
            inner: Arc::new(Mutex::new(SpiInner::Hw(dev))),
        })
    }

    /// Full-duplex transfer: `tx.len()` bytes out, the response landing
    /// in `rx`. The two slices must be the same length.
    pub fn transfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<()> {
        if tx.len() != rx.len() {
            return Err(HalError::Bus(format!(
                "spi transfer length mismatch: tx={} rx={}",
                tx.len(),
                rx.len()
            )));
        }
        let mut guard = self.inner.lock();
        match &mut *guard {
            SpiInner::Hw(dev) => {
                let mut t = SpidevTransfer::read_write(tx, rx);
                dev.transfer(&mut t)
                    .map_err(|e| HalError::Bus(format!("spi transfer: {e}")))
            }
            #[cfg(test)]
            SpiInner::Loopback(lb) => {
                lb.written.push(tx.to_vec());
                let resp = lb
                    .responses
                    .pop_front()
                    .ok_or_else(|| HalError::Bus("loopback spi: no canned response".into()))?;
                let n = rx.len().min(resp.len());
                rx[..n].copy_from_slice(&resp[..n]);
                if resp.len() < rx.len() {
                    for b in &mut rx[n..] {
                        *b = 0;
                    }
                }
                Ok(())
            }
        }
    }

    /// Write `bytes` to the device.
    pub fn write(&self, bytes: &[u8]) -> Result<()> {
        let mut guard = self.inner.lock();
        match &mut *guard {
            SpiInner::Hw(dev) => {
                let mut t = SpidevTransfer::write(bytes);
                dev.transfer(&mut t)
                    .map_err(|e| HalError::Bus(format!("spi write: {e}")))
            }
            #[cfg(test)]
            SpiInner::Loopback(lb) => {
                lb.written.push(bytes.to_vec());
                Ok(())
            }
        }
    }

    /// Read `len` bytes from the device.
    pub fn read(&self, len: usize) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; len];
        let mut guard = self.inner.lock();
        match &mut *guard {
            SpiInner::Hw(dev) => {
                let mut t = SpidevTransfer::read(&mut buf);
                dev.transfer(&mut t)
                    .map_err(|e| HalError::Bus(format!("spi read: {e}")))?;
                Ok(buf)
            }
            #[cfg(test)]
            SpiInner::Loopback(lb) => {
                let resp = lb
                    .responses
                    .pop_front()
                    .ok_or_else(|| HalError::Bus("loopback spi: no canned response".into()))?;
                let n = len.min(resp.len());
                buf[..n].copy_from_slice(&resp[..n]);
                Ok(buf)
            }
        }
    }

    /// Construct a [`SpiDevice`] backed by an in-memory queue of canned
    /// transfer responses. Test-only.
    #[cfg(test)]
    pub(crate) fn from_loopback(inner: LoopbackInner) -> Self {
        Self {
            inner: Arc::new(Mutex::new(SpiInner::Loopback(inner))),
        }
    }

    /// Snapshot of every byte buffer written/transferred so far.
    /// Test-only.
    #[cfg(test)]
    pub(crate) fn loopback_written(&self) -> Vec<Vec<u8>> {
        match &*self.inner.lock() {
            SpiInner::Hw(_) => Vec::new(),
            SpiInner::Loopback(lb) => lb.written.clone(),
        }
    }

    /// Push a canned response onto a loopback device's queue.
    /// Test-only; no-op on a hardware device.
    #[cfg(test)]
    pub(crate) fn loopback_push_response(&self, bytes: Vec<u8>) {
        let mut guard = self.inner.lock();
        if let SpiInner::Loopback(lb) = &mut *guard {
            lb.responses.push_back(bytes);
        }
    }
}
