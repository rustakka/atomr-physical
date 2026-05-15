//! Port-window allocation for stacked Sunshine instances.
//!
//! Each Sunshine server binds a fixed set of TCP and UDP ports. To run
//! multiple instances side-by-side on one host, every port is shifted by
//! `stride * N` where `N` is the instance index. The Sunshine binary
//! itself only takes a single `port = ...` knob (the HTTPS API port);
//! it derives the rest as fixed offsets from that one, so the allocator
//! only needs to ensure that an entire six-port window is free.
//!
//! This module is intentionally pure: the production constructor probes
//! real ports through `std::net::TcpListener::bind`, but
//! [`PortAllocator::with_probe`] takes any closure, so tests can drive
//! the allocator without touching the kernel.

use atomr_physical_core::{PhysicalError, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::sync::Arc;
use tracing::{debug, warn};

/// Base TCP ports a single Sunshine instance binds at offset 0.
///
/// Index 1 is Sunshine's HTTPS API; the other two are RTSP and pairing.
const BASE_TCP: [u16; 3] = [47984, 47989, 48010];

/// Base UDP ports a single Sunshine instance binds at offset 0.
const BASE_UDP: [u16; 3] = [47998, 48000, 48002];

/// Default stride between successive instances, in port units.
const DEFAULT_STRIDE: u16 = 100;

/// Default maximum number of co-resident Sunshine instances.
const DEFAULT_MAX_INSTANCES: u16 = 8;

/// A reserved window of Sunshine ports — three TCP and three UDP slots,
/// stride-shifted by `offset` (in multiples of 100) from the Moonlight
/// base. Instance 0 uses offset 0, instance 1 uses offset 100, etc.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PortWindow {
    /// The stride offset (in port units) applied to every base port.
    pub offset: u16,
    /// TCP ports: base `[47984, 47989, 48010]` + `offset`.
    pub tcp: [u16; 3],
    /// UDP ports: base `[47998, 48000, 48002]` + `offset`.
    pub udp: [u16; 3],
}

impl PortWindow {
    /// The canonical Sunshine port window — instance 0, offset 0.
    pub fn base() -> Self {
        Self::at_offset(0)
    }

    /// The port window for a stride-shifted Sunshine instance.
    pub fn at_offset(offset: u16) -> Self {
        Self {
            offset,
            tcp: [
                BASE_TCP[0] + offset,
                BASE_TCP[1] + offset,
                BASE_TCP[2] + offset,
            ],
            udp: [
                BASE_UDP[0] + offset,
                BASE_UDP[1] + offset,
                BASE_UDP[2] + offset,
            ],
        }
    }

    /// The HTTPS API port Sunshine reads from its config (`port = ...`).
    /// Sunshine derives every other port as a fixed offset from this.
    pub fn http_port(&self) -> u16 {
        self.tcp[1]
    }

    /// Every port in the window, TCP first then UDP, in declaration
    /// order. Useful for probing and for log output.
    pub fn all_ports(&self) -> Vec<u16> {
        let mut out = Vec::with_capacity(6);
        out.extend_from_slice(&self.tcp);
        out.extend_from_slice(&self.udp);
        out
    }
}

/// A function that decides whether a single port is available. Returns
/// `true` if the port can be claimed.
type ProbeFn = Arc<dyn Fn(u16) -> bool + Send + Sync>;

/// Reserves Sunshine port windows for stacked instances. Uses an
/// injectable bind-probe so unit tests can run without touching real
/// sockets.
pub struct PortAllocator {
    stride: u16,
    max_instances: u16,
    taken_offsets: HashSet<u16>,
    probe: ProbeFn,
}

impl PortAllocator {
    /// Production constructor: probes by attempting a synchronous
    /// `std::net::TcpListener::bind` on each TCP port and treating bind
    /// success as "available". The bind is released immediately; this
    /// is a TOCTOU race against any other process, but it is good
    /// enough for the once-per-spawn cadence of the projection
    /// supervisor.
    pub fn new() -> Self {
        let probe: ProbeFn = Arc::new(|port: u16| {
            match std::net::TcpListener::bind(("127.0.0.1", port)) {
                Ok(_) => true,
                Err(e) => {
                    debug!(port, error = %e, "port probe: bind failed, treating as taken");
                    false
                }
            }
        });
        Self {
            stride: DEFAULT_STRIDE,
            max_instances: DEFAULT_MAX_INSTANCES,
            taken_offsets: HashSet::new(),
            probe,
        }
    }

    /// Construct with a custom probe — used in tests to simulate
    /// occupied ports without binding.
    pub fn with_probe<F>(probe: F) -> Self
    where
        F: Fn(u16) -> bool + Send + Sync + 'static,
    {
        Self {
            stride: DEFAULT_STRIDE,
            max_instances: DEFAULT_MAX_INSTANCES,
            taken_offsets: HashSet::new(),
            probe: Arc::new(probe),
        }
    }

    /// Reserve the next available window. Returns
    /// `PortExhausted { needed: 6 }` when no offset in
    /// `[0, stride*max_instances]` has all six ports free.
    pub fn reserve(&mut self) -> Result<PortWindow> {
        for n in 0..self.max_instances {
            let offset = self.stride.saturating_mul(n);
            if self.taken_offsets.contains(&offset) {
                continue;
            }
            let candidate = PortWindow::at_offset(offset);
            if candidate.all_ports().iter().all(|p| (self.probe)(*p)) {
                self.taken_offsets.insert(offset);
                debug!(offset, http = candidate.http_port(), "reserved port window");
                return Ok(candidate);
            }
            debug!(offset, "port window unavailable, advancing");
        }
        warn!(
            max_instances = self.max_instances,
            "port allocator exhausted; all windows occupied"
        );
        Err(PhysicalError::PortExhausted { needed: 6 })
    }

    /// Release a reserved window so its offset can be reused.
    pub fn release(&mut self, window: &PortWindow) {
        if self.taken_offsets.remove(&window.offset) {
            debug!(offset = window.offset, "released port window");
        } else {
            warn!(
                offset = window.offset,
                "release: offset was not held by this allocator"
            );
        }
    }

    /// How many windows are currently held.
    pub fn active_count(&self) -> usize {
        self.taken_offsets.len()
    }

    /// The configured upper bound on co-resident instances.
    pub fn max_instances(&self) -> u16 {
        self.max_instances
    }
}

impl Default for PortAllocator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    #[test]
    fn port_window_all_ports_returns_six() {
        let w = PortWindow::base();
        let all = w.all_ports();
        assert_eq!(all.len(), 6);
        assert_eq!(&all[..3], &BASE_TCP);
        assert_eq!(&all[3..], &BASE_UDP);
        assert_eq!(w.http_port(), 47989);
    }

    #[test]
    fn reserve_allocates_distinct_offsets() {
        let mut alloc = PortAllocator::with_probe(|_| true);
        let w0 = alloc.reserve().unwrap();
        let w1 = alloc.reserve().unwrap();
        let w2 = alloc.reserve().unwrap();
        assert_eq!(w0.offset, 0);
        assert_eq!(w1.offset, 100);
        assert_eq!(w2.offset, 200);
        assert_eq!(alloc.active_count(), 3);
    }

    #[test]
    fn reserve_skips_taken_offsets() {
        let busy: HashSet<u16> = PortWindow::at_offset(0).all_ports().into_iter().collect();
        let mut alloc = PortAllocator::with_probe(move |p| !busy.contains(&p));
        let w = alloc.reserve().unwrap();
        assert_eq!(w.offset, 100);
    }

    #[test]
    fn reserve_exhausted_returns_port_exhausted() {
        let mut alloc = PortAllocator::with_probe(|_| false);
        let err = alloc.reserve().unwrap_err();
        assert!(matches!(err, PhysicalError::PortExhausted { needed: 6 }));
    }

    #[test]
    fn release_recycles_offset() {
        let mut alloc = PortAllocator::with_probe(|_| true);
        let w = alloc.reserve().unwrap();
        assert_eq!(w.offset, 0);
        alloc.release(&w);
        assert_eq!(alloc.active_count(), 0);
        let again = alloc.reserve().unwrap();
        assert_eq!(again.offset, 0);
    }

    #[test]
    fn reserve_records_offset_only_after_success() {
        // Probe rejects offset 0 once, then accepts everything.
        let seen: Arc<Mutex<HashSet<u16>>> = Arc::new(Mutex::new(HashSet::new()));
        let seen_clone = seen.clone();
        let mut alloc = PortAllocator::with_probe(move |p| {
            let mut s = seen_clone.lock().unwrap();
            // Reject the very first probed port; accept all subsequent.
            if s.is_empty() {
                s.insert(p);
                false
            } else {
                true
            }
        });
        let w = alloc.reserve().unwrap();
        assert_ne!(w.offset, 0, "offset 0 should have been skipped");
        assert_eq!(alloc.active_count(), 1);
        drop(seen.lock().unwrap());
    }
}
