//! The streaming unit: [`IqChunk`].
//!
//! HackRF natively delivers interleaved 8-bit signed I/Q samples
//! (`[I0, Q0, I1, Q1, ...]`). We keep that layout end-to-end — no DSP
//! conversion on the wire — so a broadcast subscriber can hand the
//! buffer to GNU Radio, write it straight into a `.sigmf-data` file,
//! or pre-allocate its own complex buffer with the same length.
//!
//! Samples live behind an `Arc<[i8]>` so every subscriber (live
//! consumer, SigMF writer, future ROS2 bridge) shares the same
//! backing buffer without copying.

use std::sync::Arc;
use std::time::Duration;

use chrono::{DateTime, Utc};

/// One contiguous span of IQ samples produced by an SDR.
#[derive(Debug, Clone)]
pub struct IqChunk {
    /// Monotonic per-stream sequence number — increments by 1 for each
    /// successive chunk a runner emits. Resets when the actor stops
    /// and restarts (i.e. after a supervisor restart).
    pub sequence: u64,
    /// Wall-clock time the chunk was received from the driver, on the
    /// host. Not hardware-clocked — the chunk's *first* sample landed
    /// some milliseconds earlier; treat this as an "<= this instant"
    /// upper bound.
    pub captured_at: DateTime<Utc>,
    /// Centre frequency in effect when this chunk was captured.
    pub centre_hz: u64,
    /// Sample rate, in Hz, in effect when this chunk was captured.
    pub sample_rate_hz: u32,
    /// Interleaved I/Q samples (`[I, Q, I, Q, ...]`). Length is always
    /// even — divide by 2 to get the number of sample pairs.
    pub samples: Arc<[i8]>,
}

impl IqChunk {
    /// Number of sample *pairs* in this chunk.
    pub fn len_samples(&self) -> usize {
        self.samples.len() / 2
    }

    /// The byte length of the underlying buffer (== `samples.len()`).
    /// Convenience for sizing SigMF writes.
    pub fn len_bytes(&self) -> usize {
        self.samples.len()
    }

    /// Time-domain duration this chunk represents (`samples / rate`).
    /// Returns `Duration::ZERO` if `sample_rate_hz` is somehow 0.
    pub fn duration(&self) -> Duration {
        if self.sample_rate_hz == 0 {
            return Duration::ZERO;
        }
        let secs = self.len_samples() as f64 / self.sample_rate_hz as f64;
        Duration::from_secs_f64(secs.max(0.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chunk(samples: Vec<i8>, rate: u32) -> IqChunk {
        IqChunk {
            sequence: 0,
            captured_at: Utc::now(),
            centre_hz: 100_000_000,
            sample_rate_hz: rate,
            samples: Arc::from(samples),
        }
    }

    #[test]
    fn len_samples_pairs() {
        let c = chunk(vec![0; 16], 1_000_000);
        assert_eq!(c.len_samples(), 8);
        assert_eq!(c.len_bytes(), 16);
    }

    #[test]
    fn duration_matches_rate() {
        let c = chunk(vec![0; 2_000_000], 1_000_000);
        // 1_000_000 sample pairs @ 1 MS/s = 1 second.
        assert!((c.duration().as_secs_f64() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn zero_rate_is_zero_duration() {
        let c = chunk(vec![0; 4], 0);
        assert_eq!(c.duration(), Duration::ZERO);
    }
}
