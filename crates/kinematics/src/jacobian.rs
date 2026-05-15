//! Geometric Jacobian assembly.
//!
//! Given joint positions `q`, the geometric Jacobian `J(q) ∈ R^{6 × DOF}`
//! maps joint velocities `q̇` to the end-effector twist
//!
//! ```text
//! [v; ω] = J(q) q̇
//! ```
//!
//! expressed in the **base (world) frame**, with the linear part
//! stacked on top of the angular part.
//!
//! Per active joint `i`:
//!
//! - **Revolute**: column = `[ω_i × (p_target − p_i); ω_i]`
//! - **Prismatic**: column = `[ω_i; 0]`
//!
//! where `ω_i` is the joint axis in world frame and `p_i` is the
//! position of the joint's pivot in world frame.

use atomr_physical_core::Quantity;
use nalgebra::{DMatrix, Vector3};

use crate::chain::{JointKind, KinematicChain, LinkId};
use crate::error::{KinematicsError, Result};
use crate::forward::axis_in_parent;
use crate::pose::Pose;

impl KinematicChain {
    /// Geometric Jacobian of `wrt`'s pose with respect to the active
    /// joint positions, expressed in the base frame.
    ///
    /// Returns a `6 × dof()` matrix; rows `0..3` are linear, rows
    /// `3..6` are angular.
    pub fn geometric_jacobian(
        &self,
        joint_positions: &[Quantity],
        wrt: &LinkId,
    ) -> Result<DMatrix<f64>> {
        let actives = self.active_joints();
        if joint_positions.len() != actives.len() {
            return Err(KinematicsError::DofMismatch {
                given: joint_positions.len(),
                expected: actives.len(),
            });
        }

        // World poses for every link — limit-validation also happens
        // here as a side effect.
        let poses = self.forward_all(joint_positions)?;
        let p_target = poses
            .get(wrt)
            .ok_or_else(|| KinematicsError::LinkNotFound(wrt.0.clone()))?
            .translation;

        let dof = actives.len();
        let mut jac = DMatrix::<f64>::zeros(6, dof);

        // Walk the chain in topological order, keeping a column index
        // that advances exactly when we cross an active joint.
        let order = self.topo_order();
        let mut col = 0usize;
        for id in &order {
            let link = self
                .find_link(id)
                .ok_or_else(|| KinematicsError::LinkNotFound(id.0.clone()))?;
            let Some(spec) = &link.joint else { continue };
            if spec.kind == JointKind::Fixed {
                continue;
            }

            // Parent world pose (root special-case yields identity, but
            // a root link has no joint so this branch never hits with
            // None — every joint-bearing link has a parent).
            let parent_world = match &link.parent {
                Some(pid) => *poses
                    .get(pid)
                    .ok_or_else(|| KinematicsError::LinkNotFound(pid.0.clone()))?,
                None => Pose::identity(),
            };

            // Joint pivot in the world frame: the parent's world pose
            // composed with the fixed offset that takes us up to the
            // joint axis (the same `transform_from_parent` FK uses).
            let pivot_world = parent_world.compose(&link.transform_from_parent);
            let p_i = pivot_world.translation;
            // Axis in world frame: rotate the joint's parent-frame axis
            // by the pivot's rotation.
            let axis_local = axis_in_parent(spec);
            let omega_i: Vector3<f64> = pivot_world.rotation * axis_local;

            match spec.kind {
                JointKind::Revolute => {
                    let lin = omega_i.cross(&(p_target - p_i));
                    jac[(0, col)] = lin.x;
                    jac[(1, col)] = lin.y;
                    jac[(2, col)] = lin.z;
                    jac[(3, col)] = omega_i.x;
                    jac[(4, col)] = omega_i.y;
                    jac[(5, col)] = omega_i.z;
                }
                JointKind::Prismatic => {
                    jac[(0, col)] = omega_i.x;
                    jac[(1, col)] = omega_i.y;
                    jac[(2, col)] = omega_i.z;
                    // angular block stays zero
                }
                JointKind::Fixed => unreachable!("filtered above"),
            }
            col += 1;
        }

        debug_assert_eq!(col, dof, "column counter and DOF must match");
        Ok(jac)
    }

    /// Convenience: geometric Jacobian at the chain's end-effector.
    pub fn geometric_jacobian_end_effector(
        &self,
        joint_positions: &[Quantity],
    ) -> Result<DMatrix<f64>> {
        let leaf = self.end_effector_id()?;
        self.geometric_jacobian(joint_positions, &leaf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chain::{JointSpec, Link};
    use atomr_physical_core::{JointId, Unit as PhysUnit};
    use std::f64::consts::FRAC_PI_2;

    fn q_rad(v: f64) -> Quantity {
        Quantity::new(v, PhysUnit::Radian)
    }

    fn one_dof_z_revolute_with_1m_arm() -> KinematicChain {
        KinematicChain::new()
            .with_link(Link::root("base"))
            .with_link(Link::child(
                "shoulder",
                "base",
                Pose::identity(),
                JointSpec::revolute(JointId::from("j1"), Vector3::z(), (-3.2, 3.2)),
            ))
            .with_link(Link::child(
                "ee",
                "shoulder",
                Pose::from_translation(Vector3::new(1.0, 0.0, 0.0)),
                JointSpec::fixed(JointId::from("fix")),
            ))
    }

    #[test]
    fn jacobian_one_dof_revolute_z_at_zero() {
        // At q=0, end-effector is at (1, 0, 0); joint pivot at origin;
        // omega = z; lin = z × (1,0,0) = (0, 1, 0); ang = (0, 0, 1).
        let chain = one_dof_z_revolute_with_1m_arm();
        let j = chain.geometric_jacobian_end_effector(&[q_rad(0.0)]).unwrap();
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), 1);
        let expected = nalgebra::DMatrix::from_row_slice(6, 1, &[0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
        for r in 0..6 {
            assert!(
                (j[(r, 0)] - expected[(r, 0)]).abs() < 1e-12,
                "row {r}: got {}, expected {}",
                j[(r, 0)],
                expected[(r, 0)]
            );
        }
    }

    #[test]
    fn jacobian_one_dof_revolute_z_at_pi_over_two() {
        // q=π/2: end-effector at (0, 1, 0); pivot at origin; omega = z;
        // lin = z × (0,1,0) = (-1, 0, 0); ang = (0, 0, 1).
        let chain = one_dof_z_revolute_with_1m_arm();
        let j = chain
            .geometric_jacobian_end_effector(&[q_rad(FRAC_PI_2)])
            .unwrap();
        let expected = [-1.0f64, 0.0, 0.0, 0.0, 0.0, 1.0];
        for r in 0..6 {
            assert!(
                (j[(r, 0)] - expected[r]).abs() < 1e-10,
                "row {r}: got {}, expected {}",
                j[(r, 0)],
                expected[r]
            );
        }
    }

    #[test]
    fn jacobian_dof_mismatch_is_reported() {
        let chain = one_dof_z_revolute_with_1m_arm();
        let err = chain.geometric_jacobian_end_effector(&[]).unwrap_err();
        assert!(matches!(
            err,
            KinematicsError::DofMismatch { given: 0, expected: 1 }
        ));
    }
}
