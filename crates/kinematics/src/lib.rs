//! Robot-agnostic kinematics for **atomr-physical**.
//!
//! This crate is pure math: it has no actor-runtime dependency, no
//! tokio, no ROS2. It builds on `atomr-physical-core` only for the
//! shared identifier ([`atomr_physical_core::JointId`]) and physical
//! [`Quantity`](atomr_physical_core::Quantity) types, and on
//! [nalgebra](https://docs.rs/nalgebra) for linear algebra and rigid
//! transforms.
//!
//! ## Conventions
//!
//! - Poses live in **SE(3)** and are represented as a translation in
//!   R³ plus a unit-quaternion rotation. See [`pose::Pose`].
//! - Composition is `parent.compose(&child)` — i.e. `T_pc = T_pb *
//!   T_bc`. Apply the right-hand transform first, then the left.
//! - Joint axes are unit vectors in the **parent link's** frame.
//! - Active joint positions are passed as a slice of
//!   [`Quantity`](atomr_physical_core::Quantity) in the order
//!   returned by [`chain::KinematicChain::active_joints`] —
//!   non-fixed joints in topological order from root → leaf.
//!
//! ## Inverse kinematics
//!
//! [`KinematicChain::inverse`] uses **damped least squares (DLS)**:
//! `dq = J^T (J J^T + λ² I)^{-1} e`. DLS is chosen over a plain
//! pseudo-inverse Newton step because it stays well-behaved near
//! singular configurations — at the cost of a slower convergence
//! rate than the un-damped version when far from singularities. See
//! [`inverse::IkOptions`] for the tunables.
//!
//! ## Module layout
//!
//! - [`pose`] — SE(3) wrapper over `nalgebra::Isometry3`.
//! - [`chain`] — [`Link`](chain::Link) / [`JointSpec`](chain::JointSpec)
//!   / [`KinematicChain`](chain::KinematicChain) topology.
//! - [`forward`] — forward kinematics (FK) over a chain.
//! - [`jacobian`] — geometric Jacobian assembly.
//! - [`inverse`] — DLS inverse kinematics.
//! - [`error`] — the crate's error taxonomy.

pub mod chain;
pub mod error;
pub mod forward;
pub mod inverse;
pub mod jacobian;
pub mod pose;

#[cfg(feature = "ros2")]
pub mod ros2;

pub use chain::{JointKind, JointSpec, KinematicChain, Link, LinkId};
pub use error::{KinematicsError, Result};
pub use inverse::IkOptions;
pub use pose::Pose;

/// The recommended glob-import surface for downstream crates.
pub mod prelude {
    pub use crate::chain::{JointKind, JointSpec, KinematicChain, Link, LinkId};
    pub use crate::error::{KinematicsError, Result};
    pub use crate::inverse::IkOptions;
    pub use crate::pose::Pose;
}
