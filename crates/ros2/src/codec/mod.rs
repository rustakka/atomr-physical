//! The message-codec layer — encoding atomr-physical value types to and
//! from ROS2 wire payloads.
//!
//! A `message_type` string like `"sensor_msgs/msg/JointState"` cannot
//! itself produce a typed publisher: `rosidl` message structs are
//! generated **statically**, per interface package, at build time. The
//! [`CodecRegistry`] bridges that gap — it maps a message-type string to
//! a [`MessageCodec`] that knows how to encode/decode that concrete
//! type.
//!
//! The registry is **downstream-extensible**: the [`MessageCodec`] trait
//! and [`CodecRegistry::register`] are public, so a crate can add codecs
//! for its own message types. The curated builtin set
//! ([`CodecRegistry::builtin`]) maps `Reading` / `Command` to and from
//! **structured** payloads — `Ros2Payload`s whose shape mirrors the ROS2
//! message field layout — so the codecs are pure Rust and available with
//! the `rclrs` feature off. The `rclrs` transport materialises a
//! structured payload into a concrete `rosidl` message at the wire.
//!
//! The whole module — the trait, the registry, the builtin codecs,
//! [`CodecValue`], and [`Ros2Payload`] — is unit-testable with no ROS2
//! toolchain.

mod builtin;
mod payload;
mod unit_map;

use std::collections::HashMap;
use std::sync::Arc;

use atomr_physical_core::{ActuatorId, Command, Reading};
use serde_json::Value;

use crate::endpoint::Ros2Endpoint;
use crate::error::Ros2Error;

pub use payload::Ros2Payload;
pub use unit_map::{check_unit, unit_constraint, UnitConstraint};

/// A generic structured value — the atomr-side representation of a ROS2
/// service request/response or action goal/feedback/result, for message
/// shapes that do not map onto a [`Reading`] or [`Command`].
#[derive(Debug, Clone, PartialEq)]
pub struct CodecValue(Value);

impl CodecValue {
    /// Wrap a [`serde_json::Value`] as a codec value.
    pub fn new(value: impl Into<Value>) -> Self {
        Self(value.into())
    }

    /// An empty codec value — a JSON object with no fields.
    pub fn empty() -> Self {
        Self(Value::Object(serde_json::Map::new()))
    }

    /// Borrow the underlying value.
    pub fn as_value(&self) -> &Value {
        &self.0
    }

    /// Consume the codec value, returning the underlying value.
    pub fn into_value(self) -> Value {
        self.0
    }
}

impl From<Value> for CodecValue {
    fn from(value: Value) -> Self {
        Self(value)
    }
}

impl CodecValue {
    /// Wrap this value as a structured [`Ros2Payload`] — the offline
    /// interchange from a service / action handler back to the
    /// transport contract.
    pub fn into_payload(self) -> Ros2Payload {
        Ros2Payload::structured(self.0)
    }
}

impl Ros2Payload {
    /// Reinterpret a structured payload as a [`CodecValue`] — the
    /// offline interchange from the transport contract into a service /
    /// action handler. Returns `None` for a payload that wraps a native
    /// `rclrs` message instead.
    pub fn as_codec_value(&self) -> Option<CodecValue> {
        self.as_structured().cloned().map(CodecValue)
    }
}

/// Encodes atomr-physical value types to and from one concrete ROS2
/// message / service / action type.
///
/// A codec implements only the directions its message type supports —
/// a topic message codec overrides [`encode_reading`](MessageCodec::encode_reading)
/// and [`decode_command`](MessageCodec::decode_command); a service or
/// action codec overrides [`encode_payload`](MessageCodec::encode_payload)
/// and [`decode_payload`](MessageCodec::decode_payload). The directions
/// it leaves unimplemented return [`Ros2Error::UnsupportedOperation`].
pub trait MessageCodec: Send + Sync + std::fmt::Debug {
    /// The ROS2 message / service / action type this codec handles,
    /// e.g. `"sensor_msgs/msg/Temperature"`. Used as the registry key.
    fn message_type(&self) -> &str;

    /// Encode a sensor reading for a publishing topic endpoint.
    fn encode_reading(&self, _endpoint: &Ros2Endpoint, _reading: &Reading) -> Result<Ros2Payload, Ros2Error> {
        Err(Ros2Error::UnsupportedOperation {
            message_type: self.message_type().to_string(),
            operation: "encode_reading",
        })
    }

    /// Decode an inbound message on a subscribing topic endpoint into a
    /// command for `actuator` — the device the topic is bound to, which
    /// the codec stamps onto the [`Command`] it produces.
    fn decode_command(
        &self,
        _actuator: &ActuatorId,
        _endpoint: &Ros2Endpoint,
        _payload: &Ros2Payload,
    ) -> Result<Command, Ros2Error> {
        Err(Ros2Error::UnsupportedOperation {
            message_type: self.message_type().to_string(),
            operation: "decode_command",
        })
    }

    /// Encode a generic value into a payload — a service request /
    /// response, or an action goal / feedback / result.
    fn encode_payload(&self, _value: &CodecValue) -> Result<Ros2Payload, Ros2Error> {
        Err(Ros2Error::UnsupportedOperation {
            message_type: self.message_type().to_string(),
            operation: "encode_payload",
        })
    }

    /// Decode a generic payload into a generic value.
    fn decode_payload(&self, _payload: &Ros2Payload) -> Result<CodecValue, Ros2Error> {
        Err(Ros2Error::UnsupportedOperation {
            message_type: self.message_type().to_string(),
            operation: "decode_payload",
        })
    }
}

/// A registry mapping ROS2 message-type strings to [`MessageCodec`]s.
///
/// Build one with [`CodecRegistry::builtin`] for the curated set, then
/// [`register`](CodecRegistry::register) any additional codecs your
/// application needs.
#[derive(Default, Clone, Debug)]
pub struct CodecRegistry {
    codecs: HashMap<String, Arc<dyn MessageCodec>>,
}

impl CodecRegistry {
    /// An empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// A registry pre-populated with the curated builtin codecs.
    ///
    /// The builtin codecs map `Reading` / `Command` to and from
    /// **structured** payloads matching each ROS2 message's field
    /// layout, so they are pure Rust and available with `rclrs` off. The
    /// `rclrs` transport materialises a structured payload into a
    /// concrete `rosidl` message at the wire.
    pub fn builtin() -> Self {
        let mut registry = Self::new();
        builtin::register_builtin(&mut registry);
        registry
    }

    /// Register a codec, keyed by its [`message_type`](MessageCodec::message_type).
    ///
    /// A later registration for the same message type replaces the
    /// earlier one — this is the extension point downstream crates use
    /// to override a builtin codec or add their own.
    pub fn register(&mut self, codec: Arc<dyn MessageCodec>) {
        self.codecs.insert(codec.message_type().to_string(), codec);
    }

    /// The codec registered for `message_type`, if any.
    pub fn get(&self, message_type: &str) -> Option<&Arc<dyn MessageCodec>> {
        self.codecs.get(message_type)
    }

    /// The codec registered for `message_type`, or
    /// [`Ros2Error::UnknownMessageType`] if none is.
    pub fn require(&self, message_type: &str) -> Result<&Arc<dyn MessageCodec>, Ros2Error> {
        self.get(message_type)
            .ok_or_else(|| Ros2Error::UnknownMessageType(message_type.to_string()))
    }

    /// Whether a codec is registered for `message_type`.
    pub fn contains(&self, message_type: &str) -> bool {
        self.codecs.contains_key(message_type)
    }

    /// The message types this registry can encode / decode. Iteration
    /// order is unspecified.
    pub fn registered_types(&self) -> impl Iterator<Item = &str> {
        self.codecs.keys().map(String::as_str)
    }

    /// Number of registered codecs.
    pub fn len(&self) -> usize {
        self.codecs.len()
    }

    /// Whether the registry holds no codecs.
    pub fn is_empty(&self) -> bool {
        self.codecs.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_physical_core::{ActuatorId, ControlMode, Quantity, SensorId, Unit};
    use serde_json::json;

    /// A test codec for a fictional scalar topic type — exercises the
    /// downstream extension point without needing `rosidl`.
    #[derive(Debug)]
    struct TestScalarCodec {
        message_type: String,
    }

    impl MessageCodec for TestScalarCodec {
        fn message_type(&self) -> &str {
            &self.message_type
        }

        fn encode_reading(
            &self,
            endpoint: &Ros2Endpoint,
            reading: &Reading,
        ) -> Result<Ros2Payload, Ros2Error> {
            check_unit(&endpoint.topic, &self.message_type, reading.quantity.unit)?;
            Ok(Ros2Payload::structured(json!({ "data": reading.quantity.value })))
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
                .and_then(Value::as_f64)
                .ok_or_else(|| Ros2Error::Decode {
                    endpoint: endpoint.topic.clone(),
                    message_type: self.message_type.clone(),
                    reason: "missing or non-numeric `data` field".into(),
                })?;
            Ok(Command {
                actuator: actuator.clone(),
                mode: ControlMode::Position,
                setpoint: Quantity::new(value, Unit::Scalar),
                issued_ms: 0,
            })
        }
    }

    fn scalar_codec() -> Arc<dyn MessageCodec> {
        Arc::new(TestScalarCodec {
            message_type: "test_msgs/msg/Scalar".to_string(),
        })
    }

    #[test]
    fn builtin_registry_carries_the_curated_codecs() {
        // The builtin codecs are structured-payload codecs — pure Rust,
        // available with `rclrs` off.
        let registry = CodecRegistry::builtin();
        assert!(!registry.is_empty());
        assert!(registry.contains("std_msgs/msg/Float64"));
        assert!(registry.contains("sensor_msgs/msg/Temperature"));
    }

    #[test]
    fn register_then_get_round_trips() {
        let mut registry = CodecRegistry::new();
        registry.register(scalar_codec());
        assert_eq!(registry.len(), 1);
        assert!(registry.contains("test_msgs/msg/Scalar"));
        assert!(registry.get("test_msgs/msg/Scalar").is_some());
        assert_eq!(
            registry.registered_types().collect::<Vec<_>>(),
            vec!["test_msgs/msg/Scalar"]
        );
    }

    #[test]
    fn require_errors_on_unknown_type() {
        let registry = CodecRegistry::new();
        match registry.require("nope/msg/Missing") {
            Err(Ros2Error::UnknownMessageType(t)) => assert_eq!(t, "nope/msg/Missing"),
            other => panic!("expected UnknownMessageType, got {other:?}"),
        }
    }

    #[test]
    fn a_later_registration_replaces_an_earlier_one() {
        let mut registry = CodecRegistry::new();
        registry.register(scalar_codec());
        registry.register(scalar_codec());
        // Same message type — still one entry, not two.
        assert_eq!(registry.len(), 1);
    }

    #[test]
    fn registered_codec_encodes_a_reading_and_decodes_a_command() {
        let mut registry = CodecRegistry::new();
        registry.register(scalar_codec());
        let codec = registry.require("test_msgs/msg/Scalar").unwrap();

        let endpoint = Ros2Endpoint::publish("/test/scalar", "test_msgs/msg/Scalar");
        let reading = Reading {
            sensor: SensorId::from("s1"),
            quantity: Quantity::new(4.2, Unit::Scalar),
            timestamp_ms: 0,
            frame: None,
        };
        let payload = codec.encode_reading(&endpoint, &reading).unwrap();
        assert_eq!(payload.as_structured(), Some(&json!({ "data": 4.2 })));

        let command = codec
            .decode_command(&ActuatorId::from("a1"), &endpoint, &payload)
            .unwrap();
        assert_eq!(command.setpoint.value, 4.2);
    }

    #[test]
    fn unimplemented_directions_return_unsupported_operation() {
        let codec = scalar_codec();
        // The scalar codec implements topic directions but not payload
        // directions.
        match codec.encode_payload(&CodecValue::empty()) {
            Err(Ros2Error::UnsupportedOperation {
                message_type,
                operation,
            }) => {
                assert_eq!(message_type, "test_msgs/msg/Scalar");
                assert_eq!(operation, "encode_payload");
            }
            other => panic!("expected UnsupportedOperation, got {other:?}"),
        }
    }

    #[test]
    fn codec_value_round_trips_its_inner_value() {
        let value = CodecValue::new(json!({ "success": true, "message": "homed" }));
        assert_eq!(value.as_value()["success"], json!(true));
        assert_eq!(value.into_value(), json!({ "success": true, "message": "homed" }));
    }
}
