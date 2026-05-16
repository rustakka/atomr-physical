//! [`MockSdrDriver`] — hardware-free implementation of
//! [`atomr_physical_sdr::SdrBackend`] for SDR actor tests.
//!
//! Generates a deterministic synthetic IQ stream (tone, noise, or
//! constant) at the configured sample rate, with chunk pacing tuned
//! for fast tests rather than real-time fidelity.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use atomr_physical_core::{
    Capability, Device, DeviceDescriptor, DeviceId, DeviceKind, Result, Unit,
};
use atomr_physical_sdr::{IqChunk, SdrBackend, SdrError, SdrParams, SdrResult};
use chrono::Utc;
use tokio::sync::mpsc;

/// Which synthetic waveform the mock emits.
#[derive(Debug, Clone, Copy)]
pub enum MockWaveform {
    /// Sequential ramp `[0, 1, 2, ...]` — useful for asserting
    /// ordering / sequence numbers.
    Ramp,
    /// Zeros — useful for stress-testing buffer paths without
    /// numerical surprises.
    Zero,
    /// Constant `[i, q]` repeated — useful for asserting tune events
    /// (the mock encodes the centre frequency byte into `i`).
    Constant {
        /// In-phase component.
        i: i8,
        /// Quadrature component.
        q: i8,
    },
}

/// A `SdrBackend` that lives entirely in process.
pub struct MockSdrDriver {
    descriptor: DeviceDescriptor,
    /// Shared with the streaming task so `tune_live` updates are
    /// visible in the next-emitted chunk.
    params: Arc<Mutex<SdrParams>>,
    streaming: Arc<AtomicBool>,
    sequence: Arc<AtomicU64>,
    waveform: MockWaveform,
    /// How many sample-pairs to emit per chunk.
    pub chunk_samples: usize,
    /// How long to sleep between chunks. Set to `Duration::ZERO` to
    /// run as fast as the receiver can drain.
    pub chunk_interval: Duration,
    /// Whether to accept transmits. Default true (the real backend
    /// returns `Unsupported`; tests of the unsupported path can set
    /// this false to simulate).
    pub accept_transmit: bool,
    /// Every TX payload is appended here for assertion.
    pub tx_log: Arc<Mutex<Vec<Vec<i8>>>>,
}

impl MockSdrDriver {
    /// Build a mock SDR with the default RX params and a 2048-pair
    /// chunk size at 1 ms cadence.
    pub fn new(id: impl Into<String>, waveform: MockWaveform) -> Self {
        let descriptor =
            DeviceDescriptor::new(DeviceId::from(id.into()), DeviceKind::Composite, "mock-sdr")
                .with_capability(Capability::new("iq_stream", Unit::Iq))
                .with_capability(Capability::new("rf_tx", Unit::Iq));
        Self {
            descriptor,
            params: Arc::new(Mutex::new(SdrParams::default_rx())),
            streaming: Arc::new(AtomicBool::new(false)),
            sequence: Arc::new(AtomicU64::new(0)),
            waveform,
            chunk_samples: 2048,
            chunk_interval: Duration::from_millis(1),
            accept_transmit: true,
            tx_log: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Override the per-chunk sample count.
    pub fn with_chunk_samples(mut self, n: usize) -> Self {
        self.chunk_samples = n.max(1);
        self
    }

    /// Override the inter-chunk sleep.
    pub fn with_chunk_interval(mut self, d: Duration) -> Self {
        self.chunk_interval = d;
        self
    }

    /// Snapshot the transmit log.
    pub fn transmit_log(&self) -> Vec<Vec<i8>> {
        self.tx_log.lock().expect("tx log poisoned").clone()
    }
}

#[async_trait]
impl Device for MockSdrDriver {
    fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    async fn health_check(&self) -> Result<()> {
        Ok(())
    }
}

#[async_trait]
impl SdrBackend for MockSdrDriver {
    async fn apply(&self, params: &SdrParams) -> SdrResult<()> {
        params.validate()?;
        if self.streaming.load(Ordering::SeqCst) {
            return Err(SdrError::BadState("apply: mock is streaming"));
        }
        *self.params.lock().expect("mock params poisoned") = params.clone();
        Ok(())
    }

    async fn tune_live(&self, params: &SdrParams) -> SdrResult<()> {
        params.validate()?;
        if !self.streaming.load(Ordering::SeqCst) {
            return Err(SdrError::BadState("tune_live: mock not streaming"));
        }
        *self.params.lock().expect("mock params poisoned") = params.clone();
        Ok(())
    }

    async fn start_rx(&self, sink: mpsc::Sender<IqChunk>) -> SdrResult<()> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(SdrError::BadState("start_rx: mock already streaming"));
        }
        let params = Arc::clone(&self.params);
        let streaming = Arc::clone(&self.streaming);
        let sequence = Arc::clone(&self.sequence);
        let waveform = self.waveform;
        let chunk_samples = self.chunk_samples;
        let chunk_interval = self.chunk_interval;
        tokio::spawn(async move {
            while streaming.load(Ordering::SeqCst) {
                let p = params.lock().expect("mock params poisoned").clone();
                let samples = synth_samples(waveform, chunk_samples, &p);
                let chunk = IqChunk {
                    sequence: sequence.fetch_add(1, Ordering::SeqCst),
                    captured_at: Utc::now(),
                    centre_hz: p.centre_hz,
                    sample_rate_hz: p.sample_rate_hz,
                    samples: Arc::from(samples),
                };
                if sink.send(chunk).await.is_err() {
                    break;
                }
                if !chunk_interval.is_zero() {
                    tokio::time::sleep(chunk_interval).await;
                }
            }
        });
        Ok(())
    }

    async fn stop_rx(&self) -> SdrResult<()> {
        self.streaming.store(false, Ordering::SeqCst);
        // Yield once so the driver loop notices the flag and exits
        // before the caller assumes the stream is fully torn down.
        tokio::time::sleep(Duration::from_millis(5)).await;
        Ok(())
    }

    async fn transmit(&self, samples: &[i8]) -> SdrResult<()> {
        if !self.accept_transmit {
            return Err(SdrError::Unsupported("mock transmit disabled for this test"));
        }
        self.tx_log
            .lock()
            .expect("tx log poisoned")
            .push(samples.to_vec());
        Ok(())
    }

    fn params(&self) -> SdrParams {
        self.params.lock().expect("mock params poisoned").clone()
    }
}

/// Render a synthetic interleaved-IQ buffer of `n_pairs` pairs.
fn synth_samples(waveform: MockWaveform, n_pairs: usize, params: &SdrParams) -> Vec<i8> {
    let mut out = Vec::with_capacity(n_pairs * 2);
    match waveform {
        MockWaveform::Ramp => {
            for i in 0..n_pairs {
                out.push((i % 128) as i8);
                out.push(((i + 64) % 128) as i8);
            }
        }
        MockWaveform::Zero => {
            out.resize(n_pairs * 2, 0);
        }
        MockWaveform::Constant { i, q } => {
            // Encode the low byte of centre_hz into `i` so tests can
            // detect tune events by inspecting samples directly.
            let i_with_tag = i.wrapping_add((params.centre_hz & 0x7F) as i8);
            for _ in 0..n_pairs {
                out.push(i_with_tag);
                out.push(q);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn apply_records_params() {
        let mock = MockSdrDriver::new("m1", MockWaveform::Zero);
        let mut p = SdrParams::default_rx();
        p.centre_hz = 150_000_000;
        mock.apply(&p).await.unwrap();
        assert_eq!(mock.params().centre_hz, 150_000_000);
    }

    #[tokio::test]
    async fn start_emits_chunks_in_sequence() {
        let mock = MockSdrDriver::new("m1", MockWaveform::Ramp)
            .with_chunk_samples(64)
            .with_chunk_interval(Duration::from_millis(0));
        let (tx, mut rx) = mpsc::channel::<IqChunk>(8);
        mock.start_rx(tx).await.unwrap();
        let a = rx.recv().await.unwrap();
        let b = rx.recv().await.unwrap();
        mock.stop_rx().await.unwrap();
        assert_eq!(a.sequence + 1, b.sequence);
        assert_eq!(a.len_samples(), 64);
    }

    #[tokio::test]
    async fn tune_live_reflects_in_next_chunk() {
        let mock = MockSdrDriver::new("m1", MockWaveform::Zero)
            .with_chunk_samples(32)
            .with_chunk_interval(Duration::from_millis(0));
        let (tx, mut rx) = mpsc::channel::<IqChunk>(8);
        mock.start_rx(tx).await.unwrap();
        let first = rx.recv().await.unwrap();
        let mut p = mock.params();
        p.centre_hz = 200_000_000;
        mock.tune_live(&p).await.unwrap();
        // Drain a few chunks until centre changes.
        let mut found = false;
        for _ in 0..16 {
            let c = rx.recv().await.unwrap();
            if c.centre_hz == 200_000_000 {
                found = true;
                break;
            }
        }
        mock.stop_rx().await.unwrap();
        assert!(found, "tune_live did not propagate to the streaming task");
        assert_eq!(first.centre_hz, 100_000_000);
    }

    #[tokio::test]
    async fn transmit_logs_payload() {
        let mock = MockSdrDriver::new("m1", MockWaveform::Zero);
        mock.transmit(&[1, 2, 3, 4]).await.unwrap();
        assert_eq!(mock.transmit_log(), vec![vec![1i8, 2, 3, 4]]);
    }
}
