//! Physical quantities and SI-aligned units.
//!
//! atomr-physical keeps quantities explicit at the type boundary so a
//! sensor reading or an actuation setpoint always carries its unit. The
//! [`Unit`] enum is intentionally small — extend it as new device
//! classes land rather than passing bare `f64`s around.

use serde::{Deserialize, Serialize};

/// A measured-or-commanded physical unit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Unit {
    /// Dimensionless ratio or count.
    Scalar,
    /// Metres (length / position).
    Metre,
    /// Metres per second (linear velocity).
    MetrePerSecond,
    /// Metres per second squared (linear acceleration).
    MetrePerSecondSquared,
    /// Radians (angular position).
    Radian,
    /// Radians per second (angular velocity).
    RadianPerSecond,
    /// Newtons (force).
    Newton,
    /// Newton-metres (torque).
    NewtonMetre,
    /// Degrees Celsius (temperature).
    Celsius,
    /// Pascals (pressure).
    Pascal,
    /// Volts (electric potential).
    Volt,
    /// Amperes (electric current).
    Ampere,
    /// Percent, `0.0`–`100.0` — e.g. duty cycle or battery state-of-charge.
    Percent,
}

impl Unit {
    /// A short human-readable symbol for the unit.
    pub fn symbol(&self) -> &'static str {
        match self {
            Unit::Scalar => "",
            Unit::Metre => "m",
            Unit::MetrePerSecond => "m/s",
            Unit::MetrePerSecondSquared => "m/s²",
            Unit::Radian => "rad",
            Unit::RadianPerSecond => "rad/s",
            Unit::Newton => "N",
            Unit::NewtonMetre => "N·m",
            Unit::Celsius => "°C",
            Unit::Pascal => "Pa",
            Unit::Volt => "V",
            Unit::Ampere => "A",
            Unit::Percent => "%",
        }
    }
}

/// A scalar physical quantity: a value paired with its [`Unit`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quantity {
    /// The numeric magnitude, expressed in `unit`.
    pub value: f64,
    /// The unit `value` is expressed in.
    pub unit: Unit,
}

impl Quantity {
    /// Construct a quantity from a value and unit.
    pub fn new(value: f64, unit: Unit) -> Self {
        Self { value, unit }
    }

    /// A dimensionless scalar quantity.
    pub fn scalar(value: f64) -> Self {
        Self::new(value, Unit::Scalar)
    }

    /// Returns `true` if `other` carries the same unit as `self`.
    pub fn same_unit(&self, other: &Quantity) -> bool {
        self.unit == other.unit
    }
}

impl std::fmt::Display for Quantity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}{}", self.value, self.unit.symbol())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantity_display_carries_symbol() {
        assert_eq!(Quantity::new(1.5, Unit::Radian).to_string(), "1.5rad");
        assert_eq!(Quantity::scalar(3.0).to_string(), "3");
    }

    #[test]
    fn same_unit_compares_dimension() {
        let a = Quantity::new(1.0, Unit::Metre);
        let b = Quantity::new(2.0, Unit::Metre);
        let c = Quantity::new(2.0, Unit::Radian);
        assert!(a.same_unit(&b));
        assert!(!a.same_unit(&c));
    }
}
