//! Strongly-typed identifiers for physical-layer entities.
//!
//! Every id is a string newtype: cheap to clone, stable across
//! serialization, and impossible to mix up at a call site (a
//! [`SensorId`] will not type-check where an [`ActuatorId`] is
//! expected).

use serde::{Deserialize, Serialize};

macro_rules! id_newtype {
    ($(#[$m:meta])* $name:ident, $prefix:literal) => {
        $(#[$m])*
        #[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
        pub struct $name(String);

        impl $name {
            /// Generate a fresh random id of the form `<prefix>-<uuid>`.
            pub fn new() -> Self {
                Self(format!("{}-{}", $prefix, uuid::Uuid::new_v4()))
            }

            /// Borrow the id as a string slice.
            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }

        impl From<String> for $name {
            fn from(s: String) -> Self {
                Self(s)
            }
        }

        impl From<&str> for $name {
            fn from(s: &str) -> Self {
                Self(s.to_string())
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(&self.0)
            }
        }
    };
}

id_newtype!(
    /// Identifies any physical device — a sensor, an actuator, or a
    /// composite node that does both.
    DeviceId, "dev"
);
id_newtype!(
    /// Identifies a sensor — an input device producing
    /// [`Reading`](crate::Reading)s.
    SensorId, "sen"
);
id_newtype!(
    /// Identifies an actuator — an output device accepting
    /// [`Command`](crate::Command)s.
    ActuatorId, "act"
);
id_newtype!(
    /// Identifies a robot — a supervised tree of sensor and actuator
    /// actors.
    RobotId, "rob"
);
id_newtype!(
    /// Identifies a single articulated joint within a robot.
    JointId, "jnt"
);
