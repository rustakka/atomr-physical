//! Discrete-time infinite-horizon LQR.
//!
//! [`Lqr::solve`] iterates the discrete algebraic Riccati equation
//! (DARE) to a fixed point and then derives the optimal feedback gain
//! `K`. The result is a static gain matrix usable as `u = -K x`. This
//! is the small, hardware-free building block the
//! [`crate::pendulum::TwoWheelPendulumController`] is built on top of.
//!
//! The [`BalanceEngine`] trait lets different inverted-pendulum models
//! share a common supervised wrapper without committing to a particular
//! state representation.

use nalgebra::SMatrix;
use nalgebra::SVector;
use thiserror::Error;

/// A discrete-time, infinite-horizon LQR.
///
/// Generic over the number of states `N` and the number of control
/// inputs `M`. Storing `A`, `B`, `Q`, `R`, and the precomputed gain `K`
/// keeps the controller introspectable — tests can verify the gain
/// stabilises the closed-loop system and downstream code can re-run
/// `solve` if the model changes.
#[derive(Debug, Clone, Copy)]
pub struct Lqr<const N: usize, const M: usize> {
    /// State transition matrix.
    pub a: SMatrix<f64, N, N>,
    /// Control input matrix.
    pub b: SMatrix<f64, N, M>,
    /// State weighting matrix.
    pub q: SMatrix<f64, N, N>,
    /// Control weighting matrix.
    pub r: SMatrix<f64, M, M>,
    /// Optimal feedback gain, `u = -K x`.
    pub k: SMatrix<f64, M, N>,
}

impl<const N: usize, const M: usize> Lqr<N, M> {
    /// Compute the infinite-horizon LQR gain.
    ///
    /// Iterates the discrete algebraic Riccati equation
    /// `P_{n+1} = AᵀP_n A − AᵀP_n B (R + BᵀP_n B)⁻¹ BᵀP_n A + Q`
    /// until `‖P_{n+1} − P_n‖` falls below `TOL`, capped at
    /// `MAX_ITER`. Converges for stabilisable `(A, B)` and
    /// detectable `(A, Q^{1/2})`.
    pub fn solve(
        a: SMatrix<f64, N, N>,
        b: SMatrix<f64, N, M>,
        q: SMatrix<f64, N, N>,
        r: SMatrix<f64, M, M>,
    ) -> Result<Self, LqrError> {
        const MAX_ITER: usize = 500;
        const TOL: f64 = 1e-9;

        let mut p = q;
        for iter in 0..MAX_ITER {
            // S = R + Bᵀ P B
            let s = r + b.transpose() * p * b;
            let s_inv = s.try_inverse().ok_or(LqrError::SingularR)?;
            // P_next = Aᵀ P A − Aᵀ P B S⁻¹ Bᵀ P A + Q
            let a_t_p = a.transpose() * p;
            let p_next = a_t_p * a - a_t_p * b * s_inv * b.transpose() * p * a + q;

            let diff = (p_next - p).norm();
            p = p_next;
            if diff < TOL {
                tracing::debug!(iter, diff, "LQR DARE converged");
                let s = r + b.transpose() * p * b;
                let s_inv = s.try_inverse().ok_or(LqrError::SingularR)?;
                let k = s_inv * b.transpose() * p * a;
                return Ok(Self { a, b, q, r, k });
            }
        }
        tracing::debug!("LQR DARE failed to converge after {} iterations", MAX_ITER);
        Err(LqrError::DidNotConverge)
    }

    /// Compute the control input `u = -K x`.
    pub fn control(&self, state: &SVector<f64, N>) -> SVector<f64, M> {
        -self.k * state
    }
}

/// Errors raised when solving the discrete algebraic Riccati equation.
#[derive(Debug, Error)]
pub enum LqrError {
    /// Iteration of the Riccati recursion did not converge inside the
    /// allotted number of steps.
    #[error("Riccati iteration did not converge")]
    DidNotConverge,
    /// The matrix `R + Bᵀ P B` was singular and could not be
    /// inverted — usually a sign the user-supplied `R` is not
    /// positive-definite, or the model is malformed.
    #[error("R matrix is not invertible")]
    SingularR,
}

/// A balance controller that maps an estimated robot state into a set
/// of control outputs.
///
/// Implementors live in [`crate::pendulum`] (and downstream crates).
/// The trait is intentionally small so the supervised
/// [`crate::pendulum::BalanceEngineActor`] wrapper can be shared across
/// concrete pendulum models without re-implementing the actor plumbing.
pub trait BalanceEngine: Send + Sync + 'static {
    /// The shape of the sensor state this engine consumes. Typically a
    /// tuple of `(pitch, pitch_rate, position, velocity)` or a more
    /// elaborate struct.
    type State: Send + Sync;
    /// Compute the controller's output(s) from `state`. The returned
    /// vector has one entry per wheel / actuator the engine drives.
    fn step(&self, state: &Self::State) -> Vec<f64>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix1, Matrix2, Vector2};

    #[test]
    fn lqr_stabilises_simple_2x2() {
        // Discrete-time LQR: A=[[0,1],[1,0]], B=[[0],[1]] has open-loop
        // eigenvalues ±1 (marginally unstable). LQR with Q=I, R=1 should
        // produce a gain such that the closed-loop A − B K has all
        // eigenvalues strictly inside the unit circle.
        let a = Matrix2::new(0.0, 1.0, 1.0, 0.0);
        let b = nalgebra::Matrix2x1::new(0.0, 1.0);
        let q = Matrix2::identity();
        let r = Matrix1::new(1.0);
        let lqr = Lqr::solve(a, b, q, r).expect("LQR solve");
        let closed = a - b * lqr.k;
        let eigs = closed.complex_eigenvalues();
        for ev in eigs.iter() {
            let magnitude = (ev.re * ev.re + ev.im * ev.im).sqrt();
            assert!(
                magnitude < 1.0,
                "expected discrete-stable |eigenvalue| < 1, got {ev:?} (|λ|={magnitude})"
            );
        }
    }

    #[test]
    fn lqr_control_is_negative_k_times_state() {
        let a = Matrix2::new(0.0, 1.0, 1.0, 0.0);
        let b = nalgebra::Matrix2x1::new(0.0, 1.0);
        let q = Matrix2::identity();
        let r = Matrix1::new(1.0);
        let lqr = Lqr::solve(a, b, q, r).unwrap();
        let x = Vector2::new(1.0, 2.0);
        let u = lqr.control(&x);
        let expected = -lqr.k * x;
        assert!((u - expected).norm() < 1e-12);
    }
}
