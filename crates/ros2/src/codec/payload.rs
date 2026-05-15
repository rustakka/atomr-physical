//! [`Ros2Payload`] — the opaque ROS2 wire payload.

use serde_json::Value;

/// An opaque ROS2 wire payload.
///
/// Offline — and as the interchange form before the `rclrs` transport
/// materialises a concrete message — a payload is a **structured value**:
/// a tagged [`serde_json::Value`] capturing the message's field layout.
/// It round-trips losslessly in tests with no ROS2 toolchain.
///
/// With the `rclrs` feature a payload can additionally wrap a concrete
/// `rosidl`-generated message; that variant lands with the transport
/// core. Code that needs the structured form uses
/// [`as_structured`](Ros2Payload::as_structured), which returns `None`
/// for a payload that wraps a native message instead.
#[derive(Debug, Clone, PartialEq)]
pub struct Ros2Payload {
    repr: PayloadRepr,
}

/// The internal representation of a payload. Kept private so the public
/// surface stays stable as the `rclrs`-native variant is added.
#[derive(Debug, Clone, PartialEq)]
enum PayloadRepr {
    /// A tagged structured value — the offline / interchange form.
    Structured(Value),
}

impl Ros2Payload {
    /// Wrap a structured [`serde_json::Value`] as a payload.
    pub fn structured(value: impl Into<Value>) -> Self {
        Self {
            repr: PayloadRepr::Structured(value.into()),
        }
    }

    /// An empty structured payload — a JSON object with no fields.
    pub fn empty() -> Self {
        Self::structured(Value::Object(serde_json::Map::new()))
    }

    /// Borrow the structured value, if this payload carries one.
    ///
    /// Always `Some` for payloads built offline. A `None` would mean the
    /// payload wraps a concrete `rosidl` message instead — an `rclrs`-only
    /// form that the transport core materialises.
    pub fn as_structured(&self) -> Option<&Value> {
        match &self.repr {
            PayloadRepr::Structured(value) => Some(value),
        }
    }

    /// Consume the payload, returning its structured value if it carries
    /// one.
    pub fn into_structured(self) -> Option<Value> {
        match self.repr {
            PayloadRepr::Structured(value) => Some(value),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn structured_payload_exposes_its_value() {
        let payload = Ros2Payload::structured(json!({ "data": 1.5 }));
        assert_eq!(payload.as_structured(), Some(&json!({ "data": 1.5 })));
    }

    #[test]
    fn empty_payload_is_an_empty_object() {
        let payload = Ros2Payload::empty();
        assert_eq!(payload.as_structured(), Some(&json!({})));
    }

    #[test]
    fn into_structured_returns_the_value() {
        let payload = Ros2Payload::structured(json!({ "x": [1, 2, 3] }));
        assert_eq!(payload.into_structured(), Some(json!({ "x": [1, 2, 3] })));
    }

    #[test]
    fn payloads_compare_by_structured_value() {
        let a = Ros2Payload::structured(json!({ "data": 1.0 }));
        let b = Ros2Payload::structured(json!({ "data": 1.0 }));
        let c = Ros2Payload::structured(json!({ "data": 2.0 }));
        assert_eq!(a, b);
        assert_ne!(a, c);
    }
}
