//! Kinematic chain: a tree of [`Link`]s connected by [`JointSpec`]s.
//!
//! A chain is intentionally minimal: it carries only what's needed for
//! forward kinematics, Jacobian assembly, and DLS inverse kinematics —
//! geometry (per-link fixed offsets), joint specifications (axis,
//! kind, limits), and parent links. Mass, inertia, and collision
//! geometry live in higher layers.
//!
//! ## Topology
//!
//! Exactly one link must be the root (have `parent: None`). Every
//! other link must reference a parent that's already in the chain.
//! [`KinematicChain::topo_order`] does a breadth-first walk from the
//! root and **panics** if it finds a cycle or more than one root —
//! cycles in a kinematic tree are a programming bug, not a runtime
//! input, and forward kinematics has no sensible fallback.

use atomr_physical_core::JointId;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::pose::Pose;

/// A link identifier — a string newtype with the same hashing /
/// equality semantics as the upstream [`JointId`].
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LinkId(pub String);

impl LinkId {
    /// Construct a link id from anything convertible into a String.
    pub fn new(s: impl Into<String>) -> Self {
        Self(s.into())
    }

    /// Borrow the id as a string slice.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<&str> for LinkId {
    fn from(s: &str) -> Self {
        Self(s.to_string())
    }
}

impl From<String> for LinkId {
    fn from(s: String) -> Self {
        Self(s)
    }
}

impl std::fmt::Display for LinkId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// The kind of motion a joint produces relative to its axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum JointKind {
    /// Rotational joint: position is an angle in radians about `axis`.
    Revolute,
    /// Linear joint: position is a displacement in metres along `axis`.
    Prismatic,
    /// Rigid joint: identity transform; does not contribute a DOF.
    Fixed,
}

/// One joint's static specification: identity, kind, axis, and limits.
#[derive(Debug, Clone)]
pub struct JointSpec {
    /// Stable joint identifier (reusing the upstream [`JointId`]).
    pub id: JointId,
    /// What kind of motion this joint produces.
    pub kind: JointKind,
    /// The joint's axis in the **parent link's** frame. Should be a
    /// unit vector; the joint transform formulas assume `axis.norm() ≈ 1`.
    pub axis: Vector3<f64>,
    /// Inclusive joint range `(min, max)` — radians for [`Revolute`],
    /// metres for [`Prismatic`]. Ignored for [`Fixed`].
    ///
    /// [`Revolute`]: JointKind::Revolute
    /// [`Prismatic`]: JointKind::Prismatic
    /// [`Fixed`]: JointKind::Fixed
    pub limits: (f64, f64),
}

impl JointSpec {
    /// Construct a [`Revolute`](JointKind::Revolute) joint about `axis`
    /// with `(min, max)` radian limits.
    pub fn revolute(id: JointId, axis: Vector3<f64>, limits: (f64, f64)) -> Self {
        Self {
            id,
            kind: JointKind::Revolute,
            axis,
            limits,
        }
    }

    /// Construct a [`Prismatic`](JointKind::Prismatic) joint along
    /// `axis` with `(min, max)` metre limits.
    pub fn prismatic(id: JointId, axis: Vector3<f64>, limits: (f64, f64)) -> Self {
        Self {
            id,
            kind: JointKind::Prismatic,
            axis,
            limits,
        }
    }

    /// Construct a [`Fixed`](JointKind::Fixed) joint — identity
    /// transform regardless of position.
    pub fn fixed(id: JointId) -> Self {
        Self {
            id,
            kind: JointKind::Fixed,
            axis: Vector3::z(),
            limits: (0.0, 0.0),
        }
    }
}

/// One link in a kinematic chain.
///
/// Each link except the root carries a [`JointSpec`] describing the
/// joint **above** it (the joint that connects this link to
/// `parent`). The link's frame is reached from its parent's frame by
/// applying `transform_from_parent` followed by the joint transform
/// implied by the current joint position.
#[derive(Debug, Clone)]
pub struct Link {
    /// This link's identifier.
    pub id: LinkId,
    /// The parent link's id, or `None` for the root.
    pub parent: Option<LinkId>,
    /// Fixed offset from the parent's frame to this link's pre-joint
    /// frame. For the root link this is typically `Pose::identity()`.
    pub transform_from_parent: Pose,
    /// The joint that connects this link to `parent`, if any. `None`
    /// for the root link.
    pub joint: Option<JointSpec>,
}

impl Link {
    /// A root link: no parent, no joint, identity offset.
    pub fn root(id: impl Into<LinkId>) -> Self {
        Self {
            id: id.into(),
            parent: None,
            transform_from_parent: Pose::identity(),
            joint: None,
        }
    }

    /// A child link with the given parent, fixed offset, and joint.
    pub fn child(
        id: impl Into<LinkId>,
        parent: impl Into<LinkId>,
        transform_from_parent: Pose,
        joint: JointSpec,
    ) -> Self {
        Self {
            id: id.into(),
            parent: Some(parent.into()),
            transform_from_parent,
            joint: Some(joint),
        }
    }
}

/// A tree of links connected by joints — the static structure that
/// forward kinematics, the Jacobian, and IK all operate on.
#[derive(Debug, Clone, Default)]
pub struct KinematicChain {
    /// The chain's links, in insertion order. Topology is recovered
    /// from each link's `parent` pointer; insertion order is not
    /// required to be topological.
    pub links: Vec<Link>,
}

impl KinematicChain {
    /// Construct an empty chain.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style: append a link.
    pub fn with_link(mut self, link: Link) -> Self {
        self.links.push(link);
        self
    }

    /// Append a link in place.
    pub fn add_link(&mut self, link: Link) {
        self.links.push(link);
    }

    /// Returns the link ids in topological order: root first, then
    /// each link after its parent.
    ///
    /// # Panics
    ///
    /// Panics if the chain has more than one root, has no root, or
    /// contains a cycle. These are programming-time errors — see the
    /// module docstring for the rationale.
    pub fn topo_order(&self) -> Vec<LinkId> {
        // Find the unique root.
        let roots: Vec<&Link> = self.links.iter().filter(|l| l.parent.is_none()).collect();
        assert!(
            roots.len() <= 1,
            "kinematic chain has {} roots; expected at most 1",
            roots.len()
        );
        let root = match roots.first() {
            Some(r) => *r,
            None => return Vec::new(),
        };

        // BFS from the root, walking child pointers.
        let mut order = Vec::with_capacity(self.links.len());
        let mut queue: std::collections::VecDeque<LinkId> = std::collections::VecDeque::new();
        queue.push_back(root.id.clone());
        while let Some(id) = queue.pop_front() {
            assert!(
                !order.contains(&id),
                "kinematic chain contains a cycle through {id}"
            );
            order.push(id.clone());
            for child in self.links.iter().filter(|l| l.parent.as_ref() == Some(&id)) {
                queue.push_back(child.id.clone());
            }
        }

        assert_eq!(
            order.len(),
            self.links.len(),
            "kinematic chain has disconnected links (some link's parent is not in the chain)"
        );
        order
    }

    /// Total active (non-[`Fixed`](JointKind::Fixed)) joint count.
    pub fn dof(&self) -> usize {
        self.active_joints().len()
    }

    /// The ordered list of active [`JointSpec`]s — topological order,
    /// skipping fixed joints. The order matches the joint-position
    /// slice passed to forward kinematics / IK.
    pub fn active_joints(&self) -> Vec<&JointSpec> {
        let order = self.topo_order();
        let mut out = Vec::with_capacity(order.len());
        for id in &order {
            if let Some(link) = self.find_link(id) {
                if let Some(joint) = &link.joint {
                    if joint.kind != JointKind::Fixed {
                        out.push(joint);
                    }
                }
            }
        }
        out
    }

    /// Look up a link by id, returning `None` if absent.
    pub fn find_link(&self, id: &LinkId) -> Option<&Link> {
        self.links.iter().find(|l| &l.id == id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn x_axis() -> Vector3<f64> {
        Vector3::x()
    }

    #[test]
    fn topo_order_returns_root_first() {
        let chain = KinematicChain::new()
            .with_link(Link::root("base"))
            .with_link(Link::child(
                "arm",
                "base",
                Pose::identity(),
                JointSpec::revolute(JointId::from("j1"), x_axis(), (-1.0, 1.0)),
            ));
        let order = chain.topo_order();
        assert_eq!(order, vec![LinkId::from("base"), LinkId::from("arm")]);
    }

    #[test]
    fn dof_counts_only_active_joints() {
        let revolute = KinematicChain::new()
            .with_link(Link::root("base"))
            .with_link(Link::child(
                "arm",
                "base",
                Pose::identity(),
                JointSpec::revolute(JointId::from("j1"), x_axis(), (-1.0, 1.0)),
            ));
        assert_eq!(revolute.dof(), 1);

        let fixed = KinematicChain::new()
            .with_link(Link::root("base"))
            .with_link(Link::child(
                "arm",
                "base",
                Pose::identity(),
                JointSpec::fixed(JointId::from("j1")),
            ));
        assert_eq!(fixed.dof(), 0);
    }

    #[test]
    fn find_link_returns_some_for_known_id() {
        let chain = KinematicChain::new().with_link(Link::root("base"));
        assert!(chain.find_link(&LinkId::from("base")).is_some());
        assert!(chain.find_link(&LinkId::from("ghost")).is_none());
    }

    #[test]
    #[should_panic(expected = "roots")]
    fn topo_order_panics_with_two_roots() {
        let chain = KinematicChain::new()
            .with_link(Link::root("base1"))
            .with_link(Link::root("base2"));
        let _ = chain.topo_order();
    }
}
