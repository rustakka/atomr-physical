//! Error taxonomy for the kinematics crate.
//!
//! Every public function in this crate returns [`Result`] (the alias
//! over [`KinematicsError`]) so callers see a single, well-typed error
//! surface independent of `atomr-physical-core`'s broader
//! [`PhysicalError`](atomr_physical_core::PhysicalError). Callers that
//! need to bubble these into the physical-layer taxonomy can wrap them
//! at the boundary.

/// Errors raised by forward kinematics, Jacobian assembly, and inverse
/// kinematics iteration.
#[derive(Debug, thiserror::Error)]
pub enum KinematicsError {
    /// A joint command fell outside the joint's configured `[min, max]`
    /// limits. Reported with the offending joint id and the bounds.
    #[error("joint {id} position {value} outside limits [{min}, {max}]")]
    JointOutOfBounds {
        /// The joint id whose bounds were violated.
        id: String,
        /// The offending joint value.
        value: f64,
        /// Lower bound of the joint's configured range.
        min: f64,
        /// Upper bound of the joint's configured range.
        max: f64,
    },

    /// The number of joint positions handed to a kinematics call did
    /// not match the chain's active-joint count.
    #[error("joint count {given} does not match chain DOF {expected}")]
    DofMismatch {
        /// The number of values provided.
        given: usize,
        /// The number of values expected — the chain's DOF.
        expected: usize,
    },

    /// A link id was referenced that is not present in the chain.
    #[error("link {0:?} not found in chain")]
    LinkNotFound(String),

    /// Inverse kinematics did not satisfy both tolerances within
    /// `max_iter` steps. The remaining residual is reported.
    #[error("IK did not converge within {iters} iterations (residual {residual:.6e})")]
    IkDidNotConverge {
        /// How many iterations were spent before giving up.
        iters: usize,
        /// `||err||_2` at the final iteration.
        residual: f64,
    },

    /// The Jacobian was rank-deficient at the current configuration —
    /// IK cannot make progress without a different seed.
    #[error("Jacobian rank-deficient at this configuration")]
    SingularJacobian,
}

/// Result alias used across `atomr-physical-kinematics`.
pub type Result<T> = std::result::Result<T, KinematicsError>;
