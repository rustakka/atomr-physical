//! Robot-agnostic control primitives for atomr-physical.
//!
//! This crate sits one level above sensing and actuation: it consumes
//! sensor readings, runs control law arithmetic, and dispatches
//! actuation commands. Nothing in here knows about any particular
//! robot — that specialisation lives in downstream crates that
//! parameterise the controllers exported from here.
//!
//! Every supervised type follows the same **two-form contract** the
//! rest of the workspace uses:
//!
//! 1. **Offline form** — construct the controller, call its methods
//!    directly, no actor runtime required. Useful in tests and for
//!    composing controllers inside other controllers.
//! 2. **Supervised form** — call `.spawn(system, name)` to promote it
//!    into a live atomr actor. The returned typed `*Ref` handle wraps
//!    a mailbox and (where relevant) a broadcast fan-out.
//!
//! Modules:
//!
//! - [`pid`] — single-axis PID controller, with a supervised
//!   [`pid::PidActor`] that pulls from a sensor broadcast and pushes
//!   commands to an actuator on a fixed loop rate.
//! - [`lqr`] — discrete-time infinite-horizon LQR solver and the
//!   [`lqr::BalanceEngine`] trait.
//! - [`pendulum`] — parametric two-wheel inverted-pendulum controller
//!   plus a [`pendulum::BalanceEngineActor`] wrapper.
//! - [`fsm`] — generic state-machine actor with a broadcast of
//!   state-transition events.
//! - [`imu_reading`] — typed [`imu_reading::ImuReading`] plus an
//!   aggregator that fans ten scalar sensor broadcasts into one IMU
//!   broadcast.
//! - [`joint_feedback`] — typed [`joint_feedback::JointState`] plus
//!   an aggregator that publishes joint snapshots from a robot's
//!   per-axis sensor children.
//! - [`loop_rate`] — [`loop_rate::LoopRate`] tick scheduling helper.

pub mod fsm;
pub mod imu_reading;
pub mod joint_feedback;
pub mod loop_rate;
pub mod lqr;
pub mod pendulum;
pub mod pid;

/// Re-export of the atomr actor runtime this crate builds on. Matches
/// the convention used by `atomr-physical-sensing` so downstream
/// consumers have a single import path.
pub use atomr_core as actor;

/// Recommended glob-import surface for downstream crates.
pub mod prelude {
    pub use crate::fsm::{Fsm, FsmActor, FsmActorRef, FsmMsg};
    pub use crate::imu_reading::{ImuAggregator, ImuAggregatorRef, ImuReading};
    pub use crate::joint_feedback::{JointFeedbackAggregator, JointFeedbackRef, JointState};
    pub use crate::loop_rate::LoopRate;
    pub use crate::lqr::{BalanceEngine, Lqr, LqrError};
    pub use crate::pendulum::{
        BalanceEngineActor, BalanceEngineActorRef, BalanceSnapshot, PendulumParams, TwoWheelPendulumController,
    };
    pub use crate::pid::{Pid, PidActor, PidActorRef, PidSnapshot, PidState};
}
