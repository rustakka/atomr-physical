//! ROS2 bridge for atomr-physical.
//!
//! This crate maps atomr-physical's actor world onto ROS2's
//! topic / service / action graph: a `SensorActor`'s reading stream
//! becomes a published topic, an `ActuatorActor`'s command mailbox
//! becomes a subscription, and a `RobotActor` becomes a ROS2 node.
//!
//! The bridge surface here is transport-agnostic and builds with **no
//! ROS2 installation**. The `rclrs` feature (Phase 2) links the
//! [`rclrs`](https://github.com/ros2-rust/ros2_rust) client library and
//! implements [`Ros2Bridge::spin`] against a live ROS2 graph; until
//! then the bridge records its plan so a node graph can be planned,
//! inspected, validated, and unit-tested offline. See
//! [`docs/ros2-bridge.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/ros2-bridge.md).
//!
//! # Layout
//!
//! The offline planning modules (`endpoint`, `topic_map`, `service`,
//! `action`, `param`, `plan`, `qos`, `clock`, `codec`, `validate`,
//! `error`, `bridge`) are private — their types surface through the
//! crate-root re-exports below. Two modules are public:
//!
//! - [`transport`] — the [`Ros2Event`] / [`Ros2Command`] channel
//!   contract and the [`Ros2Transport`] seam.
//! - [`actors`] — the Model 2 orchestration actors and the device seam.
//!
//! Key re-exported types: [`Ros2Bridge`], [`Ros2Plan`], [`TopicMap`],
//! [`Ros2Endpoint`], [`QosProfile`], the [`MessageCodec`] trait and
//! [`CodecRegistry`], and [`Ros2Error`].

mod action;
mod bridge;
mod clock;
mod codec;
mod endpoint;
mod error;
mod param;
mod plan;
mod qos;
mod service;
mod topic_map;
mod validate;

pub mod actors;
pub mod transport;

/// Re-export of the atomr actor runtime this crate builds on.
pub use atomr_core as actor;

pub use action::{ActionRole, GoalId, Ros2ActionEndpoint};
pub use actors::{
    ActionHandler, ActuatorActorSink, CommandSink, ParamStore, ReadingSource, Ros2ActionActor, Ros2ActionMsg,
    Ros2NodeActor, Ros2NodeMsg, Ros2ParamActor, Ros2ParamMsg, Ros2PublisherActor, Ros2PublisherMsg,
    Ros2ServiceActor, Ros2ServiceMsg, Ros2SubscriberActor, Ros2SubscriberMsg, Ros2Wiring, SensorActorSource,
    ServiceHandler,
};
pub use bridge::{Ros2Bridge, Ros2BridgeHandle};
pub use clock::Ros2ClockSource;
pub use codec::{
    check_unit, unit_constraint, CodecRegistry, CodecValue, MessageCodec, Ros2Payload, UnitConstraint,
};
pub use endpoint::{Ros2Direction, Ros2Endpoint};
pub use error::Ros2Error;
pub use param::{ParamType, ParamValue, Ros2ParamDecl};
pub use plan::Ros2Plan;
pub use qos::{Durability, History, QosProfile, Reliability};
pub use service::{Ros2ServiceEndpoint, ServiceRole};
pub use topic_map::TopicMap;
#[cfg(feature = "rclrs")]
pub use transport::RclrsTransport;
pub use transport::{ReqId, Ros2Command, Ros2Event, Ros2Link, Ros2Transport};
pub use validate::{
    validate_action_endpoint, validate_endpoint, validate_param_decl, validate_plan,
    validate_service_endpoint, validate_topic_map, ValidationError, ValidationIssue,
};
