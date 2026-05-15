//! ROS2 service endpoints — request/response bindings.
//!
//! A service maps onto atomr's `ask` pattern. atomr-physical can either
//! **serve** a service (external clients call in, a handler actor
//! replies) or **call** one (an atomr actor `ask`s an external server).
//! This module defines the offline endpoint type that records that
//! binding; the live wiring lands with the `rclrs` feature.

use serde::{Deserialize, Serialize};

/// Whether atomr-physical hosts a service or calls it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ServiceRole {
    /// atomr-physical **serves** the service — external clients call in,
    /// and a handler actor produces the response.
    Server,
    /// atomr-physical **calls** the service — it lives on an external
    /// ROS2 node, and an atomr actor `ask`s it.
    Client,
}

/// A ROS2 service endpoint bound to an atomr-physical handler.
///
/// Records the service name, the service type (`package/srv/Type`), and
/// the [`ServiceRole`]. Plain serde data — buildable and assertable with
/// no ROS2 toolchain.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ros2ServiceEndpoint {
    /// The fully-qualified ROS2 service name, e.g. `/arm/home`.
    pub service: String,
    /// The ROS2 service type, e.g. `std_srvs/srv/Trigger`.
    pub service_type: String,
    /// Whether atomr-physical serves or calls this service.
    pub role: ServiceRole,
}

impl Ros2ServiceEndpoint {
    /// A service atomr-physical **serves** — external clients call in.
    pub fn server(service: impl Into<String>, service_type: impl Into<String>) -> Self {
        Self {
            service: service.into(),
            service_type: service_type.into(),
            role: ServiceRole::Server,
        }
    }

    /// A service atomr-physical **calls** — it is an external server.
    pub fn client(service: impl Into<String>, service_type: impl Into<String>) -> Self {
        Self {
            service: service.into(),
            service_type: service_type.into(),
            role: ServiceRole::Client,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn factories_set_role() {
        assert_eq!(
            Ros2ServiceEndpoint::server("/arm/home", "std_srvs/srv/Trigger").role,
            ServiceRole::Server
        );
        assert_eq!(
            Ros2ServiceEndpoint::client("/arm/calibrate", "std_srvs/srv/SetBool").role,
            ServiceRole::Client
        );
    }

    #[test]
    fn service_endpoint_round_trips_through_json() {
        let ep = Ros2ServiceEndpoint::server("/arm/home", "std_srvs/srv/Trigger");
        let json = serde_json::to_string(&ep).unwrap();
        let back: Ros2ServiceEndpoint = serde_json::from_str(&json).unwrap();
        assert_eq!(ep, back);
    }
}
