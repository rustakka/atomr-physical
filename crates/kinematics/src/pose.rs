//! SE(3) rigid-body poses.
//!
//! A [`Pose`] is the standard split of a rigid transform into a
//! translation in R³ and a unit-quaternion rotation. It is a thin
//! wrapper over [`nalgebra::Isometry3`] — the wrapper exists so the
//! crate's API stays stable even if we later swap the underlying
//! representation (homogeneous matrices, dual quaternions, …).
//!
//! Composition convention: `a.compose(&b)` returns the pose that
//! applies `b` first, then `a`. This matches the usual matrix-product
//! reading `T_ac = T_ab * T_bc`, where the right-hand transform is
//! expressed in the left-hand frame.

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

/// A rigid-body pose in SE(3) — translation plus rotation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// Translation component, in metres.
    pub translation: Vector3<f64>,
    /// Rotation component as a unit quaternion.
    pub rotation: UnitQuaternion<f64>,
}

impl Pose {
    /// The identity pose — zero translation, identity rotation.
    pub fn identity() -> Self {
        Self {
            translation: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
        }
    }

    /// Construct a pose from explicit translation and rotation parts.
    pub fn new(translation: Vector3<f64>, rotation: UnitQuaternion<f64>) -> Self {
        Self { translation, rotation }
    }

    /// Construct a pose with the given translation and identity rotation.
    pub fn from_translation(t: Vector3<f64>) -> Self {
        Self::new(t, UnitQuaternion::identity())
    }

    /// Construct a pose with zero translation and the given rotation.
    pub fn from_rotation(r: UnitQuaternion<f64>) -> Self {
        Self::new(Vector3::zeros(), r)
    }

    /// SE(3) composition: apply `other` first, then `self`. Matches the
    /// standard matrix-product reading `self * other`.
    pub fn compose(&self, other: &Pose) -> Pose {
        let iso = self.as_isometry() * other.as_isometry();
        Self::from_isometry(iso)
    }

    /// The inverse rigid transform: `p.compose(&p.inverse()) ≈ identity`.
    pub fn inverse(&self) -> Pose {
        Self::from_isometry(self.as_isometry().inverse())
    }

    /// Convert into a nalgebra [`Isometry3`] — useful for interop with
    /// nalgebra's geometry helpers.
    pub fn as_isometry(&self) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::from(self.translation), self.rotation)
    }

    /// Construct a [`Pose`] from a nalgebra [`Isometry3`].
    pub fn from_isometry(iso: Isometry3<f64>) -> Self {
        Self {
            translation: iso.translation.vector,
            rotation: iso.rotation,
        }
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::identity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn approx_pose_eq(a: &Pose, b: &Pose, eps: f64) -> bool {
        (a.translation - b.translation).norm() < eps
            && (a.rotation.coords - b.rotation.coords).norm() < eps
    }

    #[test]
    fn identity_compose_left_is_identity() {
        let p = Pose::new(
            Vector3::new(1.0, 2.0, 3.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2),
        );
        let result = Pose::identity().compose(&p);
        assert!(approx_pose_eq(&result, &p, 1e-12));
    }

    #[test]
    fn compose_with_inverse_is_identity() {
        let p = Pose::new(
            Vector3::new(0.5, -1.0, 2.0),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.4),
        );
        let result = p.compose(&p.inverse());
        assert!(approx_pose_eq(&result, &Pose::identity(), 1e-12));
    }

    #[test]
    fn compose_order_matches_matrix_product() {
        // a is a 90° rotation about z; b is a +1 m translation along x.
        let a = Pose::from_rotation(UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2));
        let b = Pose::from_translation(Vector3::new(1.0, 0.0, 0.0));

        // a.compose(&b) applies b first (move +x) then a (rotate +90° about z)
        // — equivalently `a * b` — so the new origin in `a`'s frame is at
        // (cos90·1, sin90·1, 0) = (0, 1, 0).
        let ab = a.compose(&b);
        assert!((ab.translation - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-12);

        // b.compose(&a) applies a first (rotation, doesn't move origin)
        // then b (translate +x in the rotated frame's parent), so the
        // origin lands at (1, 0, 0).
        let ba = b.compose(&a);
        assert!((ba.translation - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
    }
}
