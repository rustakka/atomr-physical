//! Inverse kinematics by damped least squares (DLS).
//!
//! Given a target end-effector pose and a seed configuration, DLS
//! iterates
//!
//! ```text
//! e   = [Δp; Δω]                        // 6-vector position+rotation error
//! dq  = J^T (J J^T + λ² I)^{-1} e        // damped pseudo-inverse step
//! q  ← q + α dq                         // clamped to joint limits
//! ```
//!
//! until `||Δp|| < position_tol_m` **and** `||Δω|| < rotation_tol_rad`,
//! or until `max_iter` is exhausted.
//!
//! DLS is preferred over plain pseudo-inverse Newton because it stays
//! well-behaved near singularities; the damping parameter `λ` trades
//! convergence speed for stability and is exposed via [`IkOptions`].

use atomr_physical_core::{Quantity, Unit};
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

use crate::chain::{JointKind, KinematicChain};
use crate::error::{KinematicsError, Result};
use crate::pose::Pose;

/// Tunables for damped-least-squares IK.
#[derive(Debug, Clone, Copy)]
pub struct IkOptions {
    /// Maximum DLS iterations before giving up.
    pub max_iter: usize,
    /// Convergence tolerance on the linear error, in metres.
    pub position_tol_m: f64,
    /// Convergence tolerance on the angular error, in radians.
    pub rotation_tol_rad: f64,
    /// DLS damping parameter `λ`. Larger = more stable, slower.
    pub damping: f64,
    /// Step scale `α ∈ (0, 1]` applied to each DLS update.
    pub step_scale: f64,
}

impl Default for IkOptions {
    fn default() -> Self {
        Self {
            max_iter: 200,
            position_tol_m: 1e-4,
            rotation_tol_rad: 1e-3,
            damping: 1e-2,
            step_scale: 1.0,
        }
    }
}

impl KinematicChain {
    /// Solve `forward_end_effector(q) = target` for `q`, starting from
    /// `seed`.
    ///
    /// On convergence returns the joint positions tagged with the
    /// unit dictated by each joint's [`JointKind`]: radians for
    /// [`Revolute`](JointKind::Revolute), metres for
    /// [`Prismatic`](JointKind::Prismatic).
    ///
    /// On non-convergence returns [`KinematicsError::IkDidNotConverge`]
    /// with the residual norm at the final iterate.
    pub fn inverse(
        &self,
        target: Pose,
        seed: &[Quantity],
        opts: &IkOptions,
    ) -> Result<Vec<Quantity>> {
        let actives: Vec<crate::chain::JointSpec> = self
            .active_joints()
            .into_iter()
            .cloned()
            .collect();
        if seed.len() != actives.len() {
            return Err(KinematicsError::DofMismatch {
                given: seed.len(),
                expected: actives.len(),
            });
        }

        // Working joint vector — plain f64 values, snapped into limits
        // so the first FK call cannot trip JointOutOfBounds on a
        // borderline seed.
        let mut q_vec: Vec<f64> = seed
            .iter()
            .zip(actives.iter())
            .map(|(q, spec)| clamp(spec.limits, q.value))
            .collect();

        let dof = actives.len();
        for iter in 0..opts.max_iter {
            // Wrap q into Quantity vector for the FK / Jacobian call.
            let q_quants: Vec<Quantity> = q_vec
                .iter()
                .zip(actives.iter())
                .map(|(v, spec)| Quantity::new(*v, unit_for(spec.kind)))
                .collect();
            let current = self.forward_end_effector(&q_quants)?;

            let err_pos = target.translation - current.translation;
            let err_rot = orientation_error(target.rotation, current.rotation);

            if err_pos.norm() < opts.position_tol_m && err_rot.norm() < opts.rotation_tol_rad {
                return Ok(q_quants);
            }

            let jac = self.geometric_jacobian_end_effector(&q_quants)?;

            // err = [Δp; Δω] (6-vector)
            let mut err6 = DVector::<f64>::zeros(6);
            err6[0] = err_pos.x;
            err6[1] = err_pos.y;
            err6[2] = err_pos.z;
            err6[3] = err_rot.x;
            err6[4] = err_rot.y;
            err6[5] = err_rot.z;

            // dq = J^T (J J^T + λ² I)^{-1} err
            let jjt = &jac * jac.transpose();
            let damping2 = opts.damping * opts.damping;
            let n = jjt.nrows();
            let m = jjt.ncols();
            let damped = jjt + DMatrix::<f64>::identity(n, m) * damping2;
            let inv = damped.try_inverse().ok_or(KinematicsError::SingularJacobian)?;
            let dq = jac.transpose() * inv * err6;

            // Apply step, then clamp into each joint's limits.
            for i in 0..dof {
                let proposed = q_vec[i] + opts.step_scale * dq[i];
                q_vec[i] = clamp(actives[i].limits, proposed);
            }

            // Last-iteration logging hook: nothing to do — fall through
            // and let the next iteration's tolerance check decide.
            let _ = iter;
        }

        // Final residual at the abandoned iterate.
        let q_quants: Vec<Quantity> = q_vec
            .iter()
            .zip(actives.iter())
            .map(|(v, spec)| Quantity::new(*v, unit_for(spec.kind)))
            .collect();
        let current = self.forward_end_effector(&q_quants)?;
        let err_pos = target.translation - current.translation;
        let err_rot = orientation_error(target.rotation, current.rotation);
        let residual = (err_pos.norm().powi(2) + err_rot.norm().powi(2)).sqrt();
        Err(KinematicsError::IkDidNotConverge {
            iters: opts.max_iter,
            residual,
        })
    }
}

/// 3-vector orientation error = axis-angle of `target ∘ current^{-1}`,
/// scaled by the rotation magnitude — the standard log-map error
/// suitable for use as the angular part of a twist.
fn orientation_error(
    target: nalgebra::UnitQuaternion<f64>,
    current: nalgebra::UnitQuaternion<f64>,
) -> Vector3<f64> {
    let delta = target * current.inverse();
    // scaled_axis returns axis * angle (the log map of SO(3)).
    delta.scaled_axis()
}

fn unit_for(kind: JointKind) -> Unit {
    match kind {
        JointKind::Revolute => Unit::Radian,
        JointKind::Prismatic => Unit::Metre,
        JointKind::Fixed => Unit::Scalar, // unreachable for actives
    }
}

fn clamp((min, max): (f64, f64), value: f64) -> f64 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

// Pull in `Matrix3` so the doc-link above to the SO(3) error has a
// concrete reference; the constant is otherwise unused.
#[allow(dead_code)]
const _SO3_REF: Option<Matrix3<f64>> = None;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chain::{JointSpec, Link, LinkId};
    use atomr_physical_core::{JointId, Unit as PhysUnit};
    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_4;

    fn q_rad(v: f64) -> Quantity {
        Quantity::new(v, PhysUnit::Radian)
    }

    /// Two-revolute planar arm with two 1 m links, joints about z.
    fn planar_2dof() -> KinematicChain {
        KinematicChain::new()
            .with_link(Link::root("base"))
            .with_link(Link::child(
                "shoulder",
                "base",
                Pose::identity(),
                JointSpec::revolute(JointId::from("j1"), Vector3::z(), (-3.2, 3.2)),
            ))
            .with_link(Link::child(
                "elbow",
                "shoulder",
                Pose::from_translation(Vector3::new(1.0, 0.0, 0.0)),
                JointSpec::revolute(JointId::from("j2"), Vector3::z(), (-3.2, 3.2)),
            ))
            .with_link(Link::child(
                "ee",
                "elbow",
                Pose::from_translation(Vector3::new(1.0, 0.0, 0.0)),
                JointSpec::fixed(JointId::from("fix")),
            ))
    }

    #[test]
    fn ik_recovers_reachable_target_on_2dof_planar_arm() {
        let chain = planar_2dof();
        // Ground truth: at (q1, q2) = (π/6, π/4) the planar FK puts the
        // end-effector at a specific (x, y, 0). Solve from a seed near
        // (π/4, π/4) and check the FK at the returned config hits the
        // same point within tolerance.
        let q_true = [q_rad(std::f64::consts::FRAC_PI_6), q_rad(FRAC_PI_4)];
        let target = chain.forward_end_effector(&q_true).unwrap();
        let seed = [q_rad(FRAC_PI_4), q_rad(FRAC_PI_4)];

        let opts = IkOptions::default();
        let q = chain.inverse(target, &seed, &opts).unwrap();
        let recovered = chain.forward_end_effector(&q).unwrap();
        assert!(
            (recovered.translation - target.translation).norm() < opts.position_tol_m,
            "position residual {} exceeded tol {}",
            (recovered.translation - target.translation).norm(),
            opts.position_tol_m
        );
    }

    #[test]
    fn ik_unreachable_target_returns_did_not_converge() {
        let chain = planar_2dof();
        // Reach is at most 2 m; ask for 5 m along +x.
        let target = Pose::new(Vector3::new(5.0, 0.0, 0.0), UnitQuaternion::identity());
        let seed = [q_rad(0.1), q_rad(0.1)];
        let opts = IkOptions {
            max_iter: 50,
            ..IkOptions::default()
        };
        let err = chain.inverse(target, &seed, &opts).unwrap_err();
        assert!(matches!(err, KinematicsError::IkDidNotConverge { .. }));
    }

    #[test]
    fn ik_dof_mismatch_is_reported() {
        let chain = planar_2dof();
        let target = Pose::identity();
        let err = chain.inverse(target, &[], &IkOptions::default()).unwrap_err();
        assert!(matches!(
            err,
            KinematicsError::DofMismatch { given: 0, expected: 2 }
        ));
    }

    // Sanity check that LinkId is wired up — keeps the import live.
    #[test]
    fn end_effector_id_is_leaf() {
        let chain = planar_2dof();
        assert_eq!(chain.end_effector_id().unwrap(), LinkId::from("ee"));
    }
}
