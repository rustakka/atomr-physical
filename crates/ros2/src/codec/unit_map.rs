//! The `Unit` ↔ message-type compatibility table.
//!
//! Each curated ROS2 message type imposes a constraint on the physical
//! [`Unit`] the [`Quantity`](atomr_physical_core::Quantity) it carries
//! may use — `sensor_msgs/msg/Temperature` is degrees Celsius,
//! `geometry_msgs/msg/Twist` is linear / angular velocity, and so on.
//! The table is pure data: a codec calls [`check_unit`] before it
//! encodes, so a unit mismatch surfaces as a [`Ros2Error`] rather than
//! as a physically meaningless message on the wire.

use atomr_physical_core::Unit;

use crate::error::Ros2Error;

/// The unit constraint a curated message type imposes on the quantity it
/// carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnitConstraint {
    /// Any unit is acceptable — e.g. `std_msgs/msg/Float64`, which is a
    /// bare number with no physical dimension of its own.
    Any,
    /// Only one of these units is acceptable.
    OneOf(&'static [Unit]),
    /// The message type is not in the curated unit table — the codec
    /// layer makes no claim about it.
    Unlisted,
}

impl UnitConstraint {
    /// Whether `unit` satisfies this constraint.
    ///
    /// An [`Unlisted`](UnitConstraint::Unlisted) message type accepts
    /// nothing — a codec for it must not rely on this table.
    pub fn accepts(&self, unit: Unit) -> bool {
        match self {
            UnitConstraint::Any => true,
            UnitConstraint::OneOf(units) => units.contains(&unit),
            UnitConstraint::Unlisted => false,
        }
    }
}

/// The [`UnitConstraint`] a curated message type imposes.
///
/// The set here is the bridge's curated builtin message surface; it
/// grows as concrete codecs are added. A message type not listed here
/// returns [`UnitConstraint::Unlisted`].
pub fn unit_constraint(message_type: &str) -> UnitConstraint {
    use Unit::*;
    match message_type {
        // Bare numbers — no physical dimension of their own.
        "std_msgs/msg/Float64" | "std_msgs/msg/Float64MultiArray" => UnitConstraint::Any,
        // A temperature reading is degrees Celsius.
        "sensor_msgs/msg/Temperature" => UnitConstraint::OneOf(&[Celsius]),
        // Joint state carries position / velocity / effort across joints.
        "sensor_msgs/msg/JointState" => UnitConstraint::OneOf(&[
            Radian,
            RadianPerSecond,
            NewtonMetre,
            Metre,
            MetrePerSecond,
            Newton,
        ]),
        // A twist is linear + angular velocity.
        "geometry_msgs/msg/Twist" => UnitConstraint::OneOf(&[MetrePerSecond, RadianPerSecond]),
        // Battery state: terminal voltage, current, state-of-charge.
        "sensor_msgs/msg/BatteryState" => UnitConstraint::OneOf(&[Volt, Ampere, Percent]),
        _ => UnitConstraint::Unlisted,
    }
}

/// Check that `unit` is acceptable for `message_type`, for use inside a
/// codec's `encode_*` path.
///
/// Returns `Ok(())` when the unit fits or the message type is
/// [`Unlisted`](UnitConstraint::Unlisted) (the codec owns the decision
/// in that case). Returns [`Ros2Error::Encode`] when the curated table
/// rejects the unit.
pub fn check_unit(endpoint: &str, message_type: &str, unit: Unit) -> Result<(), Ros2Error> {
    let constraint = unit_constraint(message_type);
    if matches!(constraint, UnitConstraint::Unlisted) || constraint.accepts(unit) {
        Ok(())
    } else {
        Err(Ros2Error::Encode {
            endpoint: endpoint.to_string(),
            message_type: message_type.to_string(),
            reason: format!(
                "unit {:?} ({}) is not accepted by {message_type}",
                unit,
                unit.symbol()
            ),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn float64_accepts_any_unit() {
        assert!(unit_constraint("std_msgs/msg/Float64").accepts(Unit::Volt));
        assert!(unit_constraint("std_msgs/msg/Float64").accepts(Unit::Radian));
    }

    #[test]
    fn temperature_requires_celsius() {
        let c = unit_constraint("sensor_msgs/msg/Temperature");
        assert!(c.accepts(Unit::Celsius));
        assert!(!c.accepts(Unit::Pascal));
    }

    #[test]
    fn twist_requires_velocity_units() {
        let c = unit_constraint("geometry_msgs/msg/Twist");
        assert!(c.accepts(Unit::MetrePerSecond));
        assert!(c.accepts(Unit::RadianPerSecond));
        assert!(!c.accepts(Unit::Metre));
    }

    #[test]
    fn unlisted_type_is_unlisted() {
        assert_eq!(unit_constraint("some_pkg/msg/Custom"), UnitConstraint::Unlisted);
    }

    #[test]
    fn check_unit_passes_for_a_good_fit() {
        assert!(check_unit("/arm/temp", "sensor_msgs/msg/Temperature", Unit::Celsius).is_ok());
    }

    #[test]
    fn check_unit_rejects_a_bad_fit() {
        let err = check_unit("/arm/temp", "sensor_msgs/msg/Temperature", Unit::Pascal).unwrap_err();
        match err {
            Ros2Error::Encode {
                endpoint,
                message_type,
                ..
            } => {
                assert_eq!(endpoint, "/arm/temp");
                assert_eq!(message_type, "sensor_msgs/msg/Temperature");
            }
            other => panic!("expected Encode error, got {other:?}"),
        }
    }

    #[test]
    fn check_unit_defers_on_unlisted_types() {
        // An unlisted type imposes no constraint here — the codec owns it.
        assert!(check_unit("/x", "some_pkg/msg/Custom", Unit::Scalar).is_ok());
    }
}
