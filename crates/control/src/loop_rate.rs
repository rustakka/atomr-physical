//! Tick scheduling for control loops.
//!
//! [`LoopRate`] is the control-side counterpart of
//! [`atomr_physical_sensing::SamplingPolicy`] — sensing decides how
//! often a driver is polled, control decides how often a control law
//! is evaluated. The two are independent on purpose: a 100 Hz balance
//! controller is happy to pull from a 200 Hz IMU.

use std::time::Duration;

use tokio::time::{interval, Interval, MissedTickBehavior};

/// A control-loop tick schedule.
///
/// Wraps `tokio::time::Interval` configuration so the same `LoopRate`
/// instance can be stored cheaply (it's `Copy`) and turned into a fresh
/// `Interval` on demand inside an actor's `pre_start`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoopRate {
    /// The tick period.
    pub period: Duration,
    /// What happens if the tokio task is late and one or more ticks
    /// were missed. Mirrors [`tokio::time::MissedTickBehavior`].
    pub missed: MissedTickBehavior,
}

impl LoopRate {
    /// A rate that ticks every `period`. Late ticks are *skipped*
    /// (`MissedTickBehavior::Skip`) — the controller does not try to
    /// catch up by emitting a burst of stale commands.
    pub fn new(period: Duration) -> Self {
        Self {
            period,
            missed: MissedTickBehavior::Skip,
        }
    }

    /// A rate that ticks every `period`. Late ticks are caught up in a
    /// *burst* (`MissedTickBehavior::Burst`) — useful when the loop is
    /// integrating something time-dependent and the integrator needs
    /// every tick.
    pub fn burst(period: Duration) -> Self {
        Self {
            period,
            missed: MissedTickBehavior::Burst,
        }
    }

    /// Construct a fresh `tokio::time::Interval` configured with the
    /// rate's period and missed-tick policy.
    pub fn interval(&self) -> Interval {
        let mut i = interval(self.period);
        i.set_missed_tick_behavior(self.missed);
        i
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_uses_skip_behaviour() {
        let rate = LoopRate::new(Duration::from_millis(10));
        assert_eq!(rate.period, Duration::from_millis(10));
        assert_eq!(rate.missed, MissedTickBehavior::Skip);
    }

    #[test]
    fn burst_uses_burst_behaviour() {
        let rate = LoopRate::burst(Duration::from_millis(5));
        assert_eq!(rate.missed, MissedTickBehavior::Burst);
    }

    #[tokio::test]
    async fn interval_period_matches_construction() {
        let rate = LoopRate::new(Duration::from_millis(10));
        let interval = rate.interval();
        assert_eq!(interval.period(), Duration::from_millis(10));
    }
}
