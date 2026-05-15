//! ROS 2 / `geometry_msgs` interop — **placeholder**.
//!
//! This module is gated behind the `ros2` cargo feature and is wired
//! up as a stub so that downstream crates can depend on the feature
//! today without breaking when the conversions actually land.
//!
//! Planned API (none of this is implemented yet):
//!
//! - `From<&Pose>` for `geometry_msgs::msg::Pose` — convert our
//!   `(Vector3, UnitQuaternion)` split into ROS 2's pose message.
//! - `TryFrom<&geometry_msgs::msg::Pose>` for `Pose` — the reverse
//!   direction, returning [`crate::error::KinematicsError`] for
//!   degenerate / non-unit quaternions.
//! - Joint-state ⇆ `sensor_msgs::msg::JointState` conversion for
//!   chains.
//!
//! The conversions will be feature-gated under `ros2` so the core
//! pure-math API stays free of ROS deps.

#![cfg(feature = "ros2")]

// Intentionally empty — see module docstring for the plan.
