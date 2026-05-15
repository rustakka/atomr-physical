//! The curated builtin codecs.
//!
//! Each codec maps a [`Reading`] / [`Command`] to and from a
//! **structured** [`Ros2Payload`] whose shape matches the ROS2 message's
//! field layout (`std_msgs/msg/Float64` is `{ "data": f64 }`,
//! `sensor_msgs/msg/Temperature` carries a header, and so on). The
//! codecs are pure Rust — they touch no `rosidl` types — so they are
//! available with the `rclrs` feature off and are unit-tested here. The
//! `rclrs` transport materialises a structured payload into a concrete
//! `rosidl` message at the wire.
//!
//! [`register_builtin`] installs the set into a [`CodecRegistry`];
//! `CodecRegistry::builtin()` calls it. Downstream crates add more via
//! [`CodecRegistry::register`].

use std::sync::Arc;

use atomr_physical_core::{ActuatorId, Command, ControlMode, Quantity, Reading, Unit};
use serde_json::{json, Value};

use crate::codec::{check_unit, CodecRegistry, MessageCodec, Ros2Payload};
use crate::endpoint::Ros2Endpoint;
use crate::error::Ros2Error;

/// Install the curated builtin codecs into `registry`.
pub fn register_builtin(registry: &mut CodecRegistry) {
    registry.register(Arc::new(Float64Codec));
    registry.register(Arc::new(Float64MultiArrayCodec));
    registry.register(Arc::new(TemperatureCodec));
    registry.register(Arc::new(TwistCodec));
}

/// Pull a numeric field out of a structured payload, or a decode error.
fn number_field(
    payload: &Ros2Payload,
    field: &str,
    endpoint: &str,
    message_type: &str,
) -> Result<f64, Ros2Error> {
    payload
        .as_structured()
        .and_then(|v| v.get(field))
        .and_then(Value::as_f64)
        .ok_or_else(|| Ros2Error::Decode {
            endpoint: endpoint.to_string(),
            message_type: message_type.to_string(),
            reason: format!("missing or non-numeric `{field}` field"),
        })
}

// ---------------------------------------------------------------------
// std_msgs/msg/Float64 — a bare scalar, both directions.
// ---------------------------------------------------------------------

/// Codec for `std_msgs/msg/Float64` — `{ "data": f64 }`.
#[derive(Debug)]
pub struct Float64Codec;

impl MessageCodec for Float64Codec {
    fn message_type(&self) -> &str {
        "std_msgs/msg/Float64"
    }

    fn encode_reading(&self, endpoint: &Ros2Endpoint, reading: &Reading) -> Result<Ros2Payload, Ros2Error> {
        check_unit(&endpoint.topic, self.message_type(), reading.quantity.unit)?;
        Ok(Ros2Payload::structured(json!({ "data": reading.quantity.value })))
    }

    fn decode_command(
        &self,
        actuator: &ActuatorId,
        endpoint: &Ros2Endpoint,
        payload: &Ros2Payload,
    ) -> Result<Command, Ros2Error> {
        let value = number_field(payload, "data", &endpoint.topic, self.message_type())?;
        Ok(Command {
            actuator: actuator.clone(),
            mode: ControlMode::Position,
            setpoint: Quantity::new(value, Unit::Scalar),
            issued_ms: 0,
        })
    }
}

// ---------------------------------------------------------------------
// std_msgs/msg/Float64MultiArray — `{ "data": [f64, …] }`. atomr-physical
// readings / commands are scalar, so the bridge maps to / from a
// one-element array.
// ---------------------------------------------------------------------

/// Codec for `std_msgs/msg/Float64MultiArray` — `{ "data": [f64] }`.
#[derive(Debug)]
pub struct Float64MultiArrayCodec;

impl MessageCodec for Float64MultiArrayCodec {
    fn message_type(&self) -> &str {
        "std_msgs/msg/Float64MultiArray"
    }

    fn encode_reading(&self, endpoint: &Ros2Endpoint, reading: &Reading) -> Result<Ros2Payload, Ros2Error> {
        check_unit(&endpoint.topic, self.message_type(), reading.quantity.unit)?;
        Ok(Ros2Payload::structured(
            json!({ "data": [reading.quantity.value] }),
        ))
    }

    fn decode_command(
        &self,
        actuator: &ActuatorId,
        endpoint: &Ros2Endpoint,
        payload: &Ros2Payload,
    ) -> Result<Command, Ros2Error> {
        let value = payload
            .as_structured()
            .and_then(|v| v.get("data"))
            .and_then(Value::as_array)
            .and_then(|arr| arr.first())
            .and_then(Value::as_f64)
            .ok_or_else(|| Ros2Error::Decode {
                endpoint: endpoint.topic.clone(),
                message_type: self.message_type().to_string(),
                reason: "expected a non-empty numeric `data` array".into(),
            })?;
        Ok(Command {
            actuator: actuator.clone(),
            mode: ControlMode::Position,
            setpoint: Quantity::new(value, Unit::Scalar),
            issued_ms: 0,
        })
    }
}

// ---------------------------------------------------------------------
// sensor_msgs/msg/Temperature — a header-bearing sensor reading. Encode
// only: temperature is a measured quantity, published, not commanded.
// ---------------------------------------------------------------------

/// Codec for `sensor_msgs/msg/Temperature` — carries a `std_msgs/Header`
/// (`frame_id` + `stamp`), `temperature`, and `variance`.
#[derive(Debug)]
pub struct TemperatureCodec;

impl MessageCodec for TemperatureCodec {
    fn message_type(&self) -> &str {
        "sensor_msgs/msg/Temperature"
    }

    fn encode_reading(&self, endpoint: &Ros2Endpoint, reading: &Reading) -> Result<Ros2Payload, Ros2Error> {
        check_unit(&endpoint.topic, self.message_type(), reading.quantity.unit)?;
        Ok(Ros2Payload::structured(json!({
            "header": {
                "frame_id": reading.frame.clone().unwrap_or_default(),
                "stamp_ms": reading.timestamp_ms,
            },
            "temperature": reading.quantity.value,
            "variance": 0.0,
        })))
    }
}

// ---------------------------------------------------------------------
// geometry_msgs/msg/Twist — linear + angular velocity. A scalar reading
// maps to the component its unit selects (`MetrePerSecond` → linear.x,
// `RadianPerSecond` → angular.z); the rest are zero.
// ---------------------------------------------------------------------

/// Codec for `geometry_msgs/msg/Twist` — `{ linear: {x,y,z}, angular:
/// {x,y,z} }`, the component chosen by the quantity's unit.
#[derive(Debug)]
pub struct TwistCodec;

impl TwistCodec {
    /// Build a zeroed twist with `value` placed by `unit`.
    fn twist_with(unit: Unit, value: f64) -> Result<Value, Ros2Error> {
        let (mut lx, mut az) = (0.0, 0.0);
        match unit {
            Unit::MetrePerSecond => lx = value,
            Unit::RadianPerSecond => az = value,
            other => {
                return Err(Ros2Error::Encode {
                    endpoint: String::new(),
                    message_type: "geometry_msgs/msg/Twist".into(),
                    reason: format!("unit {other:?} does not map to a Twist component"),
                })
            }
        }
        Ok(json!({
            "linear": { "x": lx, "y": 0.0, "z": 0.0 },
            "angular": { "x": 0.0, "y": 0.0, "z": az },
        }))
    }
}

impl MessageCodec for TwistCodec {
    fn message_type(&self) -> &str {
        "geometry_msgs/msg/Twist"
    }

    fn encode_reading(&self, endpoint: &Ros2Endpoint, reading: &Reading) -> Result<Ros2Payload, Ros2Error> {
        check_unit(&endpoint.topic, self.message_type(), reading.quantity.unit)?;
        let twist = Self::twist_with(reading.quantity.unit, reading.quantity.value).map_err(|e| {
            Ros2Error::Encode {
                endpoint: endpoint.topic.clone(),
                message_type: self.message_type().to_string(),
                reason: e.to_string(),
            }
        })?;
        Ok(Ros2Payload::structured(twist))
    }

    fn decode_command(
        &self,
        actuator: &ActuatorId,
        endpoint: &Ros2Endpoint,
        payload: &Ros2Payload,
    ) -> Result<Command, Ros2Error> {
        let structured = payload.as_structured().ok_or_else(|| Ros2Error::Decode {
            endpoint: endpoint.topic.clone(),
            message_type: self.message_type().to_string(),
            reason: "twist payload was not structured".into(),
        })?;
        // A non-zero linear.x is a velocity command; a non-zero
        // angular.z is an angular-velocity command; linear.x wins a tie.
        let lx = structured
            .pointer("/linear/x")
            .and_then(Value::as_f64)
            .unwrap_or(0.0);
        let az = structured
            .pointer("/angular/z")
            .and_then(Value::as_f64)
            .unwrap_or(0.0);
        let (value, unit) = if lx != 0.0 {
            (lx, Unit::MetrePerSecond)
        } else {
            (az, Unit::RadianPerSecond)
        };
        Ok(Command {
            actuator: actuator.clone(),
            mode: ControlMode::Velocity,
            setpoint: Quantity::new(value, unit),
            issued_ms: 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_physical_core::SensorId;

    fn reading(value: f64, unit: Unit) -> Reading {
        Reading {
            sensor: SensorId::from("s1"),
            quantity: Quantity::new(value, unit),
            timestamp_ms: 1234,
            frame: Some("base_link".into()),
        }
    }

    #[test]
    fn builtin_set_is_registered() {
        let registry = CodecRegistry::builtin();
        for ty in [
            "std_msgs/msg/Float64",
            "std_msgs/msg/Float64MultiArray",
            "sensor_msgs/msg/Temperature",
            "geometry_msgs/msg/Twist",
        ] {
            assert!(registry.contains(ty), "{ty} should be registered");
        }
    }

    #[test]
    fn float64_round_trips_a_scalar() {
        let codec = Float64Codec;
        let endpoint = Ros2Endpoint::publish("/t", "std_msgs/msg/Float64");
        let payload = codec
            .encode_reading(&endpoint, &reading(2.5, Unit::Scalar))
            .unwrap();
        assert_eq!(payload.as_structured().unwrap()["data"], json!(2.5));

        let command = codec
            .decode_command(&ActuatorId::from("a1"), &endpoint, &payload)
            .unwrap();
        assert_eq!(command.actuator, ActuatorId::from("a1"));
        assert_eq!(command.setpoint.value, 2.5);
    }

    #[test]
    fn float64_multi_array_uses_a_one_element_array() {
        let codec = Float64MultiArrayCodec;
        let endpoint = Ros2Endpoint::publish("/t", "std_msgs/msg/Float64MultiArray");
        let payload = codec
            .encode_reading(&endpoint, &reading(7.0, Unit::Scalar))
            .unwrap();
        assert_eq!(payload.as_structured().unwrap()["data"], json!([7.0]));
        let command = codec
            .decode_command(&ActuatorId::from("a1"), &endpoint, &payload)
            .unwrap();
        assert_eq!(command.setpoint.value, 7.0);
    }

    #[test]
    fn temperature_carries_header_and_rejects_a_bad_unit() {
        let codec = TemperatureCodec;
        let endpoint = Ros2Endpoint::publish("/t", "sensor_msgs/msg/Temperature");
        let payload = codec
            .encode_reading(&endpoint, &reading(21.5, Unit::Celsius))
            .unwrap();
        let v = payload.as_structured().unwrap();
        assert_eq!(v["temperature"], json!(21.5));
        assert_eq!(v["header"]["frame_id"], json!("base_link"));
        assert_eq!(v["header"]["stamp_ms"], json!(1234));

        // A non-Celsius reading is rejected by the unit table.
        assert!(codec
            .encode_reading(&endpoint, &reading(21.5, Unit::Pascal))
            .is_err());
    }

    #[test]
    fn twist_maps_a_scalar_by_unit() {
        let codec = TwistCodec;
        let endpoint = Ros2Endpoint::subscribe("/cmd_vel", "geometry_msgs/msg/Twist");

        let linear = codec
            .encode_reading(&endpoint, &reading(0.4, Unit::MetrePerSecond))
            .unwrap();
        assert_eq!(linear.as_structured().unwrap()["linear"]["x"], json!(0.4));

        let command = codec
            .decode_command(&ActuatorId::from("a1"), &endpoint, &linear)
            .unwrap();
        assert_eq!(command.setpoint.unit, Unit::MetrePerSecond);
        assert_eq!(command.setpoint.value, 0.4);

        let angular = codec
            .encode_reading(&endpoint, &reading(1.2, Unit::RadianPerSecond))
            .unwrap();
        assert_eq!(angular.as_structured().unwrap()["angular"]["z"], json!(1.2));
    }
}
