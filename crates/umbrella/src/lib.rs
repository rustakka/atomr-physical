//! **atomr-physical** — physical sensing, output (low-bandwidth
//! `Command` dispatch and Sunshine/Moonlight video projection), and
//! ROS2-integrated robotics as native
//! [atomr](https://github.com/rustakka/atomr) actors, with a
//! first-class Python API overlay.
//!
//! This umbrella crate re-exports each subsystem behind a feature flag,
//! mirroring the convention used by the `atomr` and `atomr-agents`
//! umbrellas. [`core`] is always present; [`sensing`], [`actuation`],
//! [`robotics`], [`ros2`], [`control`], [`kinematics`], [`hal`],
//! [`projection`], and [`testkit`] are opt-in. The `hal-*` features
//! and `full-linux` are Linux-only because they pull in `socketcan`
//! and `linux-embedded-hal`.
//!
//! ```toml
//! [dependencies]
//! # Defaults: sensing + actuation + robotics
//! atomr-physical = "0.1"
//!
//! # Add the ROS2 bridge and test doubles:
//! # atomr-physical = { version = "0.1", features = ["ros2", "testkit"] }
//!
//! # Sunshine/Moonlight video projection (pulls reqwest + mdns-sd):
//! # atomr-physical = { version = "0.1", features = ["projection"] }
//! ```

#[doc(inline)]
pub use atomr_physical_core as core;

#[cfg(feature = "sensing")]
#[doc(inline)]
pub use atomr_physical_sensing as sensing;

#[cfg(feature = "actuation")]
#[doc(inline)]
pub use atomr_physical_actuation as actuation;

#[cfg(feature = "robotics")]
#[doc(inline)]
pub use atomr_physical_robotics as robotics;

#[cfg(feature = "ros2")]
#[doc(inline)]
pub use atomr_physical_ros2 as ros2;

#[cfg(feature = "control")]
#[doc(inline)]
pub use atomr_physical_control as control;

#[cfg(feature = "kinematics")]
#[doc(inline)]
pub use atomr_physical_kinematics as kinematics;

#[cfg(feature = "hal")]
#[doc(inline)]
pub use atomr_physical_hal as hal;

#[cfg(feature = "projection")]
#[doc(inline)]
pub use atomr_physical_projection as projection;

#[cfg(feature = "testkit")]
#[doc(inline)]
pub use atomr_physical_testkit as testkit;

/// The recommended glob-import surface for applications.
pub mod prelude {
    pub use atomr_physical_core::prelude::*;
}
