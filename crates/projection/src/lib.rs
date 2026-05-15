//! Projection output actors for atomr-physical.
//!
//! Extends the output surface from low-bandwidth [`Command`] dispatch to
//! full **video projection** — the live screen of a virtual display
//! streamed to one or more remote Moonlight clients via spawned Sunshine
//! server processes.
//!
//! A [`ProjectionActor`] is the supervisor at the top of the projection
//! subtree. It owns:
//!
//! 1. A [`VkmsDisplayManager`] that creates / tears down headless
//!    virtual displays via the kernel's vkms driver.
//! 2. A pool of supervised [`SunshineInstanceActor`] children — one
//!    `sunshine` binary per active stream window.
//! 3. A [`ClientProvisioner`] that handles the Moonlight pairing dance
//!    over Sunshine's local HTTPS API.
//! 4. An [`MdnsBroadcaster`] that advertises each instance as a
//!    `_nvstream._tcp.local.` service for remote-node discovery.
//!
//! The atomr actor runtime is re-exported as [`actor`] so downstream
//! crates have a single import path for it.

mod actor;
mod config_template;
mod display;
mod mdns;
mod pairing;
mod ports;
mod sunshine;

pub use actor::{
    BandwidthTier, PairingTicket, ProjectionActor, ProjectionActorRef, ProjectionHandle,
    ProjectionMsg, ProjectionSpec,
};
pub use config_template::{render_config, runtime_config_dir, write_instance_config, SunshineConfigParams};
pub use display::{DisplayHandle, DisplaySpec, VkmsDisplayManager};
pub use mdns::{MdnsBroadcaster, MdnsRegistration};
pub use pairing::{ClientProvisioner, PairingRecord};
pub use ports::{PortAllocator, PortWindow};
pub use sunshine::{
    SunshineInstanceActor, SunshineInstanceMsg, SunshineInstanceRef, SunshineInstanceSummary,
};

/// Re-export of the atomr actor runtime this crate builds on.
pub use atomr_core as actor_runtime;
