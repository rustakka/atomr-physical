//! **atomr-physical** — physical sensing, output, and ROS2-integrated
//! robotics as native [atomr](https://github.com/rustakka/atomr)
//! actors, with a first-class Python API overlay.
//!
//! This umbrella crate re-exports each subsystem behind a feature flag,
//! mirroring the convention used by the `atomr` and `atomr-agents`
//! umbrellas. [`core`] is always present; [`sensing`], [`actuation`],
//! [`robotics`], [`ros2`], and [`testkit`] are opt-in.
//!
//! ```toml
//! [dependencies]
//! # Defaults: sensing + actuation + robotics
//! atomr-physical = "0.1"
//!
//! # Add the ROS2 bridge and test doubles:
//! # atomr-physical = { version = "0.1", features = ["ros2", "testkit"] }
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
