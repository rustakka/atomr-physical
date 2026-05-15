//! Error taxonomy for the physical layer.

use thiserror::Error;

/// Result alias used across atomr-physical.
pub type Result<T> = std::result::Result<T, PhysicalError>;

/// Errors raised by sensing, actuation, robotics, and the ROS2 bridge.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum PhysicalError {
    /// A device id did not resolve to a registered device.
    #[error("unknown device: {0}")]
    UnknownDevice(String),

    /// A sensor read failed at the hardware / driver boundary.
    #[error("sensor read failed for {device}: {reason}")]
    SensorRead {
        /// The sensor that failed to read.
        device: String,
        /// Driver-level explanation.
        reason: String,
    },

    /// An actuation command was rejected before dispatch.
    #[error("actuation rejected for {device}: {reason}")]
    ActuationRejected {
        /// The actuator that rejected the command.
        device: String,
        /// Why the command was rejected.
        reason: String,
    },

    /// A command setpoint fell outside the actuator's safe envelope.
    #[error("setpoint {value} out of safe range [{min}, {max}] for {device}")]
    OutOfRange {
        /// The actuator whose envelope was violated.
        device: String,
        /// The offending setpoint value.
        value: f64,
        /// Envelope lower bound.
        min: f64,
        /// Envelope upper bound.
        max: f64,
    },

    /// A unit conversion was requested between incompatible dimensions.
    #[error("incompatible units: cannot convert {from} to {to}")]
    UnitMismatch {
        /// The unit being converted from.
        from: &'static str,
        /// The unit being converted to.
        to: &'static str,
    },

    /// The device is not in a state that accepts the requested operation.
    #[error("device {device} not ready: {reason}")]
    NotReady {
        /// The device that is not ready.
        device: String,
        /// What state it is in instead.
        reason: String,
    },

    /// The ROS2 bridge could not be established or lost its session.
    #[error("ros2 bridge error: {0}")]
    Ros2Bridge(String),

    /// A timeout elapsed waiting on a device.
    #[error("timed out after {millis} ms waiting on {device}")]
    Timeout {
        /// The device that did not respond in time.
        device: String,
        /// How long the caller waited, in milliseconds.
        millis: u64,
    },

    /// Catch-all for driver / transport faults.
    #[error("device fault: {0}")]
    Fault(String),

    /// A virtual display could not be brought up or torn down.
    #[error("display {display} unavailable: {reason}")]
    DisplayUnavailable {
        /// The display id or connector name.
        display: String,
        /// Why the operation failed.
        reason: String,
    },

    /// A Sunshine server process failed to spawn or terminated abnormally.
    #[error("sunshine instance failed: {reason}")]
    SunshineSpawn {
        /// Driver / OS level explanation.
        reason: String,
    },

    /// A remote Moonlight client could not be paired against a Sunshine
    /// instance.
    #[error("pairing rejected for {client}: {reason}")]
    PairingRejected {
        /// The client identifier or hostname.
        client: String,
        /// Why pairing was rejected.
        reason: String,
    },

    /// Port allocation for a Sunshine instance failed (out of windows,
    /// or all probed ports are occupied).
    #[error("port allocation failed: need {needed} usable ports")]
    PortExhausted {
        /// How many ports the allocator was looking for.
        needed: u16,
    },

    /// A required kernel module is not loaded and could not be loaded
    /// automatically.
    #[error("kernel module {module}: {reason}")]
    KernelModule {
        /// The module name (e.g. `vkms`).
        module: &'static str,
        /// Remediation guidance.
        reason: String,
    },

    /// A remote node (Moonlight client / Pi / Jetson) could not be
    /// reached or driven.
    #[error("remote node error: {reason}")]
    RemoteNode {
        /// Transport-level explanation.
        reason: String,
    },
}
