//! Forward kinematics over a [`KinematicChain`].
//!
//! Forward kinematics walks the chain in topological order; each
//! link's world pose is
//!
//! ```text
//! T_world(link) = T_world(parent) * link.transform_from_parent * joint_transform(q)
//! ```
//!
//! where `joint_transform(q)` is identity for [`Fixed`](JointKind::Fixed)
//! joints, an axis-angle rotation for [`Revolute`](JointKind::Revolute)
//! joints, and an axis-displacement translation for
//! [`Prismatic`](JointKind::Prismatic) joints. Fixed joints still
//! participate in the walk — they advance the parent-child chain but
//! contribute identity to the joint transform.

use std::collections::HashMap;

use atomr_physical_core::Quantity;
use nalgebra::{Unit, UnitQuaternion, Vector3};

use crate::chain::{JointKind, JointSpec, KinematicChain, LinkId};
use crate::error::{KinematicsError, Result};
use crate::pose::Pose;

impl KinematicChain {
    /// Compute the world pose of every link in the chain.
    ///
    /// `joint_positions` is consumed in the order returned by
    /// [`KinematicChain::active_joints`] — one entry per active
    /// (non-fixed) joint. Each entry's [`Quantity::value`] is the
    /// joint position (radians or metres, per [`JointSpec::kind`]).
    pub fn forward_all(
        &self,
        joint_positions: &[Quantity],
    ) -> Result<HashMap<LinkId, Pose>> {
        let actives = self.active_joints();
        if joint_positions.len() != actives.len() {
            return Err(KinematicsError::DofMismatch {
                given: joint_positions.len(),
                expected: actives.len(),
            });
        }
        // Validate joint limits up front so callers see the error before
        // any FK math runs.
        for (q, spec) in joint_positions.iter().zip(actives.iter()) {
            check_limits(spec, q.value)?;
        }

        let order = self.topo_order();
        let mut poses: HashMap<LinkId, Pose> = HashMap::with_capacity(order.len());
        let mut active_idx = 0usize;
        for id in &order {
            let link = self
                .find_link(id)
                .ok_or_else(|| KinematicsError::LinkNotFound(id.0.clone()))?;

            // Parent's world pose, defaulting to identity for the root.
            let parent_world = match &link.parent {
                Some(pid) => *poses
                    .get(pid)
                    .ok_or_else(|| KinematicsError::LinkNotFound(pid.0.clone()))?,
                None => Pose::identity(),
            };

            // The transform on top of the parent: fixed offset, then
            // (for non-fixed joints) the joint motion.
            let joint_xform = match &link.joint {
                None => Pose::identity(),
                Some(spec) => match spec.kind {
                    JointKind::Fixed => Pose::identity(),
                    JointKind::Revolute => {
                        // Consume one active joint value.
                        let q = joint_positions[active_idx].value;
                        active_idx += 1;
                        joint_transform_revolute(spec, q)
                    }
                    JointKind::Prismatic => {
                        let q = joint_positions[active_idx].value;
                        active_idx += 1;
                        joint_transform_prismatic(spec, q)
                    }
                },
            };

            let local = link.transform_from_parent.compose(&joint_xform);
            let world = parent_world.compose(&local);
            poses.insert(link.id.clone(), world);
        }

        Ok(poses)
    }

    /// World pose of one specific link.
    pub fn forward(&self, joint_positions: &[Quantity], link: &LinkId) -> Result<Pose> {
        let poses = self.forward_all(joint_positions)?;
        poses
            .get(link)
            .copied()
            .ok_or_else(|| KinematicsError::LinkNotFound(link.0.clone()))
    }

    /// World pose of the chain's leaf link — the one with no children.
    ///
    /// Returns [`KinematicsError::LinkNotFound`] for empty chains or
    /// chains with multiple leaves (a current limitation; multi-leaf
    /// chains aren't a target of this crate today).
    pub fn forward_end_effector(&self, joint_positions: &[Quantity]) -> Result<Pose> {
        let leaf = self.end_effector_id()?;
        self.forward(joint_positions, &leaf)
    }

    /// The chain's leaf link id — the link no other link names as parent.
    pub(crate) fn end_effector_id(&self) -> Result<LinkId> {
        let mut leaves: Vec<&LinkId> = Vec::new();
        for link in &self.links {
            let has_child = self
                .links
                .iter()
                .any(|other| other.parent.as_ref() == Some(&link.id));
            if !has_child {
                leaves.push(&link.id);
            }
        }
        match leaves.as_slice() {
            [only] => Ok((*only).clone()),
            [] => Err(KinematicsError::LinkNotFound("<empty chain>".to_string())),
            _ => Err(KinematicsError::LinkNotFound(
                "<multiple leaves not supported>".to_string(),
            )),
        }
    }
}

fn check_limits(spec: &JointSpec, value: f64) -> Result<()> {
    let (min, max) = spec.limits;
    if value < min || value > max {
        Err(KinematicsError::JointOutOfBounds {
            id: spec.id.as_str().to_string(),
            value,
            min,
            max,
        })
    } else {
        Ok(())
    }
}

fn joint_transform_revolute(spec: &JointSpec, angle: f64) -> Pose {
    let axis = Unit::new_normalize(spec.axis);
    Pose::from_rotation(UnitQuaternion::from_axis_angle(&axis, angle))
}

fn joint_transform_prismatic(spec: &JointSpec, distance: f64) -> Pose {
    let axis = spec.axis.normalize();
    Pose::from_translation(axis * distance)
}

// Re-export for sibling modules.
pub(crate) fn axis_in_parent(spec: &JointSpec) -> Vector3<f64> {
    spec.axis.normalize()
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

    #[test]
    fn fk_dof_mismatch_is_reported() {
        let chain = KinematicChain::new()
            .with_link(Link::root("base"))
            .with_link(Link::child(
                "arm",
                "base",
                Pose::identity(),
                JointSpec::revolute(JointId::from("j1"), Vector3::z(), (-3.2, 3.2)),
            ));
        let err = chain.forward_all(&[]).unwrap_err();
        assert!(matches!(
            err,
            KinematicsError::DofMismatch { given: 0, expected: 1 }
        ));
    }

    #[test]
    fn fk_one_dof_revolute_about_z_at_zero_is_identity_rotation() {
        let chain = KinematicChain::new()
            .with_link(Link::root("base"))
            .with_link(Link::child(
                "arm",
                "base",
                Pose::from_translation(Vector3::new(1.0, 0.0, 0.0)),
                JointSpec::revolute(JointId::from("j1"), Vector3::z(), (-3.2, 3.2)),
            ));
        let ee = chain.forward_end_effector(&[q_rad(0.0)]).unwrap();
        assert!((ee.translation - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((ee.rotation.coords - UnitQuaternion::identity().coords).norm() < 1e-12);
    }

    #[test]
    fn fk_one_dof_revolute_about_z_at_pi_over_two() {
        // Base joint rotates the whole arm 90° about z; the arm link
        // has its fixed offset on the **distal** side of the joint, so
        // we put the joint at the parent and the offset on the next
        // link.
        let chain = KinematicChain::new()
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
            ));
        let ee = chain.forward_end_effector(&[q_rad(FRAC_PI_2)]).unwrap();
        // Rotating (1, 0, 0) by 90° about z gives (0, 1, 0).
        assert!((ee.translation - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-10);
    }

    #[test]
    fn fk_two_dof_planar_arm_at_zero_pi_over_two() {
        // Two revolute joints about z, each followed by a 1 m link
        // offset along +x.
        let chain = KinematicChain::new()
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
            ));
        // (q1=0, q2=π/2): the first link points along +x (so elbow at
        // (1,0,0)). At the elbow we rotate +90° about z, so the second
        // link points along +y in the world frame → ee at (1, 1, 0).
        let ee = chain
            .forward_end_effector(&[q_rad(0.0), q_rad(FRAC_PI_2)])
            .unwrap();
        assert!((ee.translation - Vector3::new(1.0, 1.0, 0.0)).norm() < 1e-10);
    }

    #[test]
    fn fk_rejects_out_of_bounds_joint() {
        let chain = KinematicChain::new()
            .with_link(Link::root("base"))
            .with_link(Link::child(
                "arm",
                "base",
                Pose::identity(),
                JointSpec::revolute(JointId::from("j1"), Vector3::z(), (-0.5, 0.5)),
            ));
        let err = chain.forward_end_effector(&[q_rad(1.0)]).unwrap_err();
        assert!(matches!(err, KinematicsError::JointOutOfBounds { .. }));
    }
}
