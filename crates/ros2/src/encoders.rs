//! Typed payload + encoder registry for the rclrs bridge.
//!
//! The bridge's default publish path writes the single-`Quantity`
//! value of a `Reading` into the first float field of the bound
//! message type. That works for `std_msgs/Float64` and
//! `sensor_msgs/Temperature` but is too coarse for multi-field
//! messages like `sensor_msgs/Imu`. This module adds a small registry
//! of typed encoders so multi-field messages can be published with
//! structured payloads while keeping the single-float shortcut as the
//! default fallback.
//!
//! The payload structs ([`ImuPayload`], [`JointStatePayload`],
//! [`TwistPayload`], [`EncoderPayload`]) are visible from the public
//! API regardless of the `rclrs` feature so callers can construct
//! them in offline / cross-platform code paths. The encoder traits
//! and concrete encoders are feature-gated because they reference the
//! `rclrs::DynamicMessage` API directly.

// ---------------------------------------------------------------------------
// Payload types — feature-agnostic
// ---------------------------------------------------------------------------

/// Payload for `sensor_msgs/msg/Imu`.
///
/// Quaternion order is `[w, x, y, z]` to match the rest of
/// atomr-physical's orientation conventions; the encoder maps onto
/// the ROS `geometry_msgs/Quaternion` field order `x, y, z, w`
/// internally.
#[derive(Debug, Clone)]
pub struct ImuPayload {
    /// Orientation as a unit quaternion `[w, x, y, z]`.
    pub orientation: [f64; 4],
    /// Body-frame angular velocity in rad/s, `[x, y, z]`.
    pub angular_velocity: [f64; 3],
    /// Body-frame linear acceleration in m/s², `[x, y, z]`.
    pub linear_acceleration: [f64; 3],
    /// Row-major 3x3 covariance for orientation, or `None` to leave
    /// the field zeroed (ROS convention: zeroed = unknown).
    pub orientation_covariance: Option<[f64; 9]>,
    /// Row-major 3x3 covariance for angular velocity.
    pub angular_velocity_covariance: Option<[f64; 9]>,
    /// Row-major 3x3 covariance for linear acceleration.
    pub linear_acceleration_covariance: Option<[f64; 9]>,
    /// Optional frame id for the `std_msgs/Header`.
    pub frame_id: Option<String>,
    /// `header.stamp.sec`.
    pub stamp_sec: i32,
    /// `header.stamp.nanosec`.
    pub stamp_nanosec: u32,
}

/// Payload for `sensor_msgs/msg/JointState`.
#[derive(Debug, Clone)]
pub struct JointStatePayload {
    /// Joint names; written into the `name` string sequence.
    pub names: Vec<String>,
    /// Joint positions in radians (revolute) or metres (prismatic).
    pub positions: Vec<f64>,
    /// Joint velocities.
    pub velocities: Vec<f64>,
    /// Joint efforts (torque / force).
    pub efforts: Vec<f64>,
    /// Optional `header.frame_id`.
    pub frame_id: Option<String>,
    /// `header.stamp.sec`.
    pub stamp_sec: i32,
    /// `header.stamp.nanosec`.
    pub stamp_nanosec: u32,
}

/// Payload for `geometry_msgs/msg/Twist`.
#[derive(Debug, Clone)]
pub struct TwistPayload {
    /// Linear velocity `[x, y, z]` in m/s.
    pub linear: [f64; 3],
    /// Angular velocity `[x, y, z]` in rad/s.
    pub angular: [f64; 3],
}

/// What a [`MessageEncoder`] consumes.
///
/// `Scalar` is the default fallback used by
/// `Ros2BridgeHandle::publish_reading` when no typed encoder is
/// registered for a topic's message type.
#[derive(Debug, Clone)]
pub enum EncoderPayload {
    /// A bare scalar — what the default `FloatScalarEncoder` consumes.
    Scalar(f64),
    /// An IMU sample.
    Imu(ImuPayload),
    /// A joint-state snapshot.
    JointState(JointStatePayload),
    /// A twist command.
    Twist(TwistPayload),
}

// ---------------------------------------------------------------------------
// Encoder trait + concrete encoders — feature-gated on `rclrs`
// ---------------------------------------------------------------------------

#[cfg(feature = "rclrs")]
mod rclrs_impl {
    use super::*;
    use rclrs::{
        ArrayValueMut, DynamicMessage, DynamicMessageViewMut, SequenceValueMut, SimpleValueMut,
        ValueMut,
    };
    use rosidl_runtime_rs::{Sequence, String as RosString};

    /// Writes a typed [`EncoderPayload`] into a freshly-minted
    /// `DynamicMessage`. Encoders are stored in a `HashMap<String,
    /// Arc<dyn MessageEncoder>>` on the bridge, keyed by ROS message
    /// type (e.g. `"sensor_msgs/msg/Imu"`).
    ///
    /// Encoder implementations are best-effort: a missing field
    /// should never panic or return an error — log via `tracing` and
    /// continue, so the publish pipeline stays alive even if the
    /// running message type doesn't match the payload exactly.
    pub trait MessageEncoder: Send + Sync {
        /// Write `payload` into `message`.
        fn encode(
            &self,
            message: &mut DynamicMessage,
            payload: &EncoderPayload,
        ) -> Result<(), String>;
    }

    /// Default encoder: writes the payload's scalar value into the
    /// first `f64` / `f32` simple field of the message. This matches
    /// the legacy single-float-field shortcut used by
    /// `std_msgs/Float64`, `sensor_msgs/Temperature`, etc.
    ///
    /// Used as the fallback when no encoder is registered for a
    /// topic's message type, so existing single-field flows keep
    /// working without changes.
    pub struct FloatScalarEncoder;

    impl MessageEncoder for FloatScalarEncoder {
        fn encode(
            &self,
            message: &mut DynamicMessage,
            payload: &EncoderPayload,
        ) -> Result<(), String> {
            let value = match payload {
                EncoderPayload::Scalar(v) => *v,
                // For non-scalar payloads, fall back to writing 0.0 —
                // the caller has paired a structured payload with a
                // single-float message type, which is almost
                // certainly a configuration mistake but shouldn't
                // crash the bridge.
                other => {
                    tracing::warn!(
                        "FloatScalarEncoder received non-scalar payload {:?}; writing 0.0",
                        other
                    );
                    0.0
                }
            };
            write_first_float_field_in_msg(message, value);
            Ok(())
        }
    }

    /// Encoder for `sensor_msgs/msg/Imu`.
    ///
    /// Writes:
    /// - `header.stamp.{sec, nanosec}` and `header.frame_id`,
    /// - `orientation.{x, y, z, w}` from `[w, x, y, z]`,
    /// - `angular_velocity.{x, y, z}`,
    /// - `linear_acceleration.{x, y, z}`,
    /// - the three 9-element covariance arrays when supplied.
    ///
    /// Missing fields are skipped with a `trace!` log — running
    /// against a type that doesn't have these fields will still
    /// publish (with whatever defaults the message ships with) rather
    /// than crashing the publisher loop.
    pub struct ImuEncoder;

    impl MessageEncoder for ImuEncoder {
        fn encode(
            &self,
            message: &mut DynamicMessage,
            payload: &EncoderPayload,
        ) -> Result<(), String> {
            let imu = match payload {
                EncoderPayload::Imu(p) => p,
                EncoderPayload::Scalar(v) => {
                    // Degrade gracefully: a scalar publish through an
                    // IMU-registered topic still drops the value into
                    // the first float slot, so legacy callers don't
                    // silently lose data when an encoder is
                    // registered.
                    write_first_float_field_in_msg(message, *v);
                    return Ok(());
                }
                other => {
                    tracing::warn!("ImuEncoder received non-IMU payload {:?}; skipping", other);
                    return Ok(());
                }
            };

            with_nested_msg(message, "header", |hdr| {
                write_header(hdr, imu.stamp_sec, imu.stamp_nanosec, imu.frame_id.as_deref());
            });

            with_nested_msg(message, "orientation", |q| {
                // ROS quaternion field order is x, y, z, w.
                write_double_in_view(q, "x", imu.orientation[1]);
                write_double_in_view(q, "y", imu.orientation[2]);
                write_double_in_view(q, "z", imu.orientation[3]);
                write_double_in_view(q, "w", imu.orientation[0]);
            });

            with_nested_msg(message, "angular_velocity", |v| {
                write_vector3(v, imu.angular_velocity);
            });

            with_nested_msg(message, "linear_acceleration", |v| {
                write_vector3(v, imu.linear_acceleration);
            });

            if let Some(cov) = &imu.orientation_covariance {
                write_double_fixed_array(message, "orientation_covariance", cov);
            }
            if let Some(cov) = &imu.angular_velocity_covariance {
                write_double_fixed_array(message, "angular_velocity_covariance", cov);
            }
            if let Some(cov) = &imu.linear_acceleration_covariance {
                write_double_fixed_array(message, "linear_acceleration_covariance", cov);
            }

            Ok(())
        }
    }

    /// Encoder for `sensor_msgs/msg/JointState`.
    ///
    /// The `name`, `position`, `velocity` and `effort` fields are
    /// unbounded sequences in the ROS IDL, so the encoder replaces
    /// each sequence wholesale with one sized to the payload.
    pub struct JointStateEncoder;

    impl MessageEncoder for JointStateEncoder {
        fn encode(
            &self,
            message: &mut DynamicMessage,
            payload: &EncoderPayload,
        ) -> Result<(), String> {
            let js = match payload {
                EncoderPayload::JointState(p) => p,
                EncoderPayload::Scalar(v) => {
                    write_first_float_field_in_msg(message, *v);
                    return Ok(());
                }
                other => {
                    tracing::warn!(
                        "JointStateEncoder received non-JointState payload {:?}; skipping",
                        other
                    );
                    return Ok(());
                }
            };

            with_nested_msg(message, "header", |hdr| {
                write_header(hdr, js.stamp_sec, js.stamp_nanosec, js.frame_id.as_deref());
            });

            write_string_sequence(message, "name", &js.names);
            write_double_sequence(message, "position", &js.positions);
            write_double_sequence(message, "velocity", &js.velocities);
            write_double_sequence(message, "effort", &js.efforts);

            Ok(())
        }
    }

    /// Encoder for `geometry_msgs/msg/Twist`.
    ///
    /// Writes `linear.{x, y, z}` and `angular.{x, y, z}`. Other
    /// twist-shaped types (`TwistStamped`, `TwistWithCovariance`) are
    /// out of scope for this encoder.
    pub struct TwistEncoder;

    impl MessageEncoder for TwistEncoder {
        fn encode(
            &self,
            message: &mut DynamicMessage,
            payload: &EncoderPayload,
        ) -> Result<(), String> {
            let tw = match payload {
                EncoderPayload::Twist(p) => p,
                EncoderPayload::Scalar(v) => {
                    write_first_float_field_in_msg(message, *v);
                    return Ok(());
                }
                other => {
                    tracing::warn!(
                        "TwistEncoder received non-Twist payload {:?}; skipping",
                        other
                    );
                    return Ok(());
                }
            };

            with_nested_msg(message, "linear", |v| write_vector3(v, tw.linear));
            with_nested_msg(message, "angular", |v| write_vector3(v, tw.angular));
            Ok(())
        }
    }

    // -----------------------------------------------------------------
    // Field-writing helpers
    //
    // rclrs has *two* distinct dynamic-message view types:
    //   - `DynamicMessage`           — owned, top-level
    //   - `DynamicMessageViewMut`    — borrowed, used for nested
    //                                  submessages reached via
    //                                  `SimpleValueMut::Message(_)`.
    // Both expose `get_mut(name) -> Option<ValueMut>`, but they are
    // not interchangeable. The helpers below come in two flavors — a
    // `_in_msg` family for the top-level message and a `_in_view`
    // family for nested views — and an `with_nested_msg` adapter that
    // walks one level down.
    // -----------------------------------------------------------------

    /// Walk the message's fields and write `value` into the first
    /// floating-point primitive (top-level form). Mirrors the legacy
    /// `write_first_float_field` in lib.rs's `rclrs_spin` module.
    pub(crate) fn write_first_float_field_in_msg(message: &mut DynamicMessage, value: f64) {
        let field_names: Vec<String> =
            message.iter().map(|(name, _)| name.to_string()).collect();
        for name in field_names {
            if let Some(field) = message.get_mut(&name) {
                if let ValueMut::Simple(simple) = field {
                    match simple {
                        SimpleValueMut::Double(slot) => {
                            *slot = value;
                            return;
                        }
                        SimpleValueMut::Float(slot) => {
                            *slot = value as f32;
                            return;
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    /// Top-level: write an `f64`/`f32` into a named field.
    fn write_double_in_msg(view: &mut DynamicMessage, name: &str, value: f64) {
        match view.get_mut(name) {
            Some(ValueMut::Simple(SimpleValueMut::Double(slot))) => *slot = value,
            Some(ValueMut::Simple(SimpleValueMut::Float(slot))) => *slot = value as f32,
            Some(_) => tracing::trace!("'{name}' is not a float field"),
            None => tracing::trace!("field '{name}' not found"),
        }
    }

    /// Nested view: write an `f64`/`f32` into a named field.
    fn write_double_in_view(view: &mut DynamicMessageViewMut<'_>, name: &str, value: f64) {
        match view.get_mut(name) {
            Some(ValueMut::Simple(SimpleValueMut::Double(slot))) => *slot = value,
            Some(ValueMut::Simple(SimpleValueMut::Float(slot))) => *slot = value as f32,
            Some(_) => tracing::trace!("'{name}' is not a float field"),
            None => tracing::trace!("field '{name}' not found in nested view"),
        }
    }

    /// Nested view: write an `i32` into a named field.
    fn write_i32_in_view(view: &mut DynamicMessageViewMut<'_>, name: &str, value: i32) {
        match view.get_mut(name) {
            Some(ValueMut::Simple(SimpleValueMut::Int32(slot))) => *slot = value,
            Some(_) => tracing::trace!("'{name}' is not an i32 field"),
            None => tracing::trace!("field '{name}' not found in nested view"),
        }
    }

    /// Nested view: write a `u32` into a named field.
    fn write_u32_in_view(view: &mut DynamicMessageViewMut<'_>, name: &str, value: u32) {
        match view.get_mut(name) {
            Some(ValueMut::Simple(SimpleValueMut::Uint32(slot))) => *slot = value,
            Some(_) => tracing::trace!("'{name}' is not a u32 field"),
            None => tracing::trace!("field '{name}' not found in nested view"),
        }
    }

    /// Nested view: write a string into a named field.
    fn write_string_in_view(view: &mut DynamicMessageViewMut<'_>, name: &str, value: &str) {
        match view.get_mut(name) {
            Some(ValueMut::Simple(SimpleValueMut::String(slot))) => {
                *slot = RosString::from(value);
            }
            Some(_) => tracing::trace!("'{name}' is not a string field"),
            None => tracing::trace!("field '{name}' not found in nested view"),
        }
    }

    /// Borrow a nested submessage field by name (top-level form) and
    /// hand a mutable view to `f`. Missing or wrong-shape fields are
    /// silently traced and skipped.
    fn with_nested_msg<F>(message: &mut DynamicMessage, field: &str, f: F)
    where
        F: FnOnce(&mut DynamicMessageViewMut<'_>),
    {
        match message.get_mut(field) {
            Some(ValueMut::Simple(SimpleValueMut::Message(mut view))) => f(&mut view),
            Some(_) => tracing::trace!("nested field '{field}' is not a submessage"),
            None => tracing::trace!("nested field '{field}' not found"),
        }
    }

    /// Borrow a nested submessage field of a nested view, one level
    /// deeper than [`with_nested_msg`]. Used for things like
    /// `header.stamp` inside `Imu.header`.
    fn with_nested_in_view<F>(view: &mut DynamicMessageViewMut<'_>, field: &str, f: F)
    where
        F: FnOnce(&mut DynamicMessageViewMut<'_>),
    {
        match view.get_mut(field) {
            Some(ValueMut::Simple(SimpleValueMut::Message(mut inner))) => f(&mut inner),
            Some(_) => tracing::trace!("nested field '{field}' is not a submessage (in view)"),
            None => tracing::trace!("nested field '{field}' not found (in view)"),
        }
    }

    /// Top-level helper: write a row-major fixed-size double array.
    fn write_double_fixed_array(view: &mut DynamicMessage, name: &str, values: &[f64]) {
        let Some(slot) = view.get_mut(name) else {
            tracing::trace!("array field '{name}' not found");
            return;
        };
        match slot {
            ValueMut::Array(ArrayValueMut::DoubleArray(arr)) => {
                let n = arr.len().min(values.len());
                arr[..n].copy_from_slice(&values[..n]);
            }
            ValueMut::Array(ArrayValueMut::FloatArray(arr)) => {
                let n = arr.len().min(values.len());
                for i in 0..n {
                    arr[i] = values[i] as f32;
                }
            }
            _ => tracing::trace!("'{name}' is not a fixed-size float array"),
        }
    }

    /// Top-level helper: replace an unbounded `Sequence<f64>` field
    /// with a fresh sequence populated from `values`.
    fn write_double_sequence(view: &mut DynamicMessage, name: &str, values: &[f64]) {
        let Some(slot) = view.get_mut(name) else {
            tracing::trace!("sequence field '{name}' not found");
            return;
        };
        match slot {
            ValueMut::Sequence(SequenceValueMut::DoubleSequence(seq)) => {
                let mut fresh = Sequence::<f64>::new(values.len());
                fresh.as_mut_slice().copy_from_slice(values);
                *seq = fresh;
            }
            ValueMut::Sequence(SequenceValueMut::FloatSequence(seq)) => {
                let mut fresh = Sequence::<f32>::new(values.len());
                for (i, v) in values.iter().enumerate() {
                    fresh.as_mut_slice()[i] = *v as f32;
                }
                *seq = fresh;
            }
            _ => tracing::trace!("'{name}' is not a double sequence"),
        }
    }

    /// Top-level helper: replace an unbounded `Sequence<String>`
    /// field with a fresh sequence populated from `values`.
    fn write_string_sequence(view: &mut DynamicMessage, name: &str, values: &[String]) {
        let Some(slot) = view.get_mut(name) else {
            tracing::trace!("string sequence field '{name}' not found");
            return;
        };
        match slot {
            ValueMut::Sequence(SequenceValueMut::StringSequence(seq)) => {
                let mut fresh = Sequence::<RosString>::new(values.len());
                for (i, v) in values.iter().enumerate() {
                    fresh.as_mut_slice()[i] = RosString::from(v.as_str());
                }
                *seq = fresh;
            }
            _ => tracing::trace!("'{name}' is not a string sequence"),
        }
    }

    /// Write a `geometry_msgs/Vector3` (or anything else with
    /// `x`/`y`/`z` doubles) into a nested view.
    fn write_vector3(view: &mut DynamicMessageViewMut<'_>, v: [f64; 3]) {
        write_double_in_view(view, "x", v[0]);
        write_double_in_view(view, "y", v[1]);
        write_double_in_view(view, "z", v[2]);
    }

    /// Write a `std_msgs/Header` into a nested view: nested
    /// `stamp.{sec, nanosec}` and a top-level `frame_id` string.
    fn write_header(
        view: &mut DynamicMessageViewMut<'_>,
        sec: i32,
        nanosec: u32,
        frame_id: Option<&str>,
    ) {
        with_nested_in_view(view, "stamp", |st| {
            write_i32_in_view(st, "sec", sec);
            write_u32_in_view(st, "nanosec", nanosec);
        });
        if let Some(id) = frame_id {
            write_string_in_view(view, "frame_id", id);
        }
    }

    // Quiet unused-helper warnings when the encoders happen not to
    // exercise every path on a given build. The functions are kept
    // because they round out the matched pair (msg / view) helpers.
    #[allow(dead_code)]
    fn _ensure_helpers_used() {
        let _ = write_double_in_msg as fn(&mut DynamicMessage, &str, f64);
    }
}

#[cfg(feature = "rclrs")]
pub use rclrs_impl::*;
