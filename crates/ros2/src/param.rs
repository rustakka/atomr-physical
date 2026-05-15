//! ROS2 parameter declarations and values.
//!
//! The bridge mirrors atomr-physical configuration — `SamplingPolicy`,
//! `SafetyEnvelope`, `Calibration` — as ROS2 parameters, and applies
//! external parameter changes back onto the running actors. This module
//! defines the offline declaration ([`Ros2ParamDecl`]) and the pure-data
//! value type ([`ParamValue`]) that crosses the transport boundary; the
//! live `Ros2ParamActor` lands with the `rclrs` feature.

use serde::{Deserialize, Serialize};

/// The type tag of a ROS2 parameter.
///
/// Mirrors the subset of `rcl_interfaces` parameter types the bridge
/// curates — the scalar types plus the homogeneous arrays.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ParamType {
    /// A boolean parameter.
    Bool,
    /// A 64-bit signed integer parameter.
    Int,
    /// A double-precision float parameter.
    Double,
    /// A string parameter.
    Str,
    /// An array of integers.
    IntArray,
    /// An array of doubles.
    DoubleArray,
    /// An array of strings.
    StrArray,
    /// An array of booleans.
    BoolArray,
}

/// The value of a ROS2 parameter — the pure-data type that crosses the
/// transport boundary in both directions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ParamValue {
    /// A boolean value.
    Bool(bool),
    /// A 64-bit signed integer value.
    Int(i64),
    /// A double-precision float value.
    Double(f64),
    /// A string value.
    Str(String),
    /// An array of integers.
    IntArray(Vec<i64>),
    /// An array of doubles.
    DoubleArray(Vec<f64>),
    /// An array of strings.
    StrArray(Vec<String>),
    /// An array of booleans.
    BoolArray(Vec<bool>),
}

impl ParamValue {
    /// The [`ParamType`] this value carries.
    pub fn param_type(&self) -> ParamType {
        match self {
            ParamValue::Bool(_) => ParamType::Bool,
            ParamValue::Int(_) => ParamType::Int,
            ParamValue::Double(_) => ParamType::Double,
            ParamValue::Str(_) => ParamType::Str,
            ParamValue::IntArray(_) => ParamType::IntArray,
            ParamValue::DoubleArray(_) => ParamType::DoubleArray,
            ParamValue::StrArray(_) => ParamType::StrArray,
            ParamValue::BoolArray(_) => ParamType::BoolArray,
        }
    }
}

/// Declares a ROS2 parameter the bridge mirrors.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ros2ParamDecl {
    /// The parameter name, e.g. `shoulder.sampling_period_ms`.
    pub name: String,
    /// The parameter's default value — also pins its [`ParamType`].
    pub default: ParamValue,
    /// A human-readable description, surfaced through ROS2 parameter
    /// descriptors.
    pub description: String,
    /// Whether the parameter is read-only — mirrored out, but external
    /// changes are rejected.
    pub read_only: bool,
}

impl Ros2ParamDecl {
    /// Declare a read-write parameter with a default value.
    pub fn new(name: impl Into<String>, default: ParamValue) -> Self {
        Self {
            name: name.into(),
            default,
            description: String::new(),
            read_only: false,
        }
    }

    /// Builder-style: attach a human-readable description.
    pub fn with_description(mut self, description: impl Into<String>) -> Self {
        self.description = description.into();
        self
    }

    /// Builder-style: mark the parameter read-only — mirrored out, but
    /// external writes are rejected.
    pub fn read_only(mut self) -> Self {
        self.read_only = true;
        self
    }

    /// The [`ParamType`] this parameter carries, taken from its default.
    pub fn param_type(&self) -> ParamType {
        self.default.param_type()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn value_reports_its_type() {
        assert_eq!(ParamValue::Double(1.0).param_type(), ParamType::Double);
        assert_eq!(
            ParamValue::StrArray(vec!["a".into()]).param_type(),
            ParamType::StrArray
        );
    }

    #[test]
    fn decl_builder_sets_description_and_read_only() {
        let decl = Ros2ParamDecl::new("shoulder.period_ms", ParamValue::Int(100))
            .with_description("sampling period")
            .read_only();
        assert!(decl.read_only);
        assert_eq!(decl.description, "sampling period");
        assert_eq!(decl.param_type(), ParamType::Int);
    }

    #[test]
    fn param_decl_round_trips_through_json() {
        let decl = Ros2ParamDecl::new("envelope.max", ParamValue::Double(2.5))
            .with_description("safety envelope upper bound");
        let json = serde_json::to_string(&decl).unwrap();
        let back: Ros2ParamDecl = serde_json::from_str(&json).unwrap();
        assert_eq!(decl, back);
    }
}
