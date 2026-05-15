//! Hardware abstraction layer for atomr-physical.
//!
//! This crate provides supervised **bus actors** (CAN, I2C) plus a
//! direct SPI device handle, and concrete **device drivers** (BNO085
//! IMU, ODrive motor controller, MIT-protocol QDD joint, AS5048A
//! encoder) that talk to real hardware through those buses.
//!
//! Everything is feature-gated. The default build is platform-neutral
//! and pulls in no transport-layer crate — only the atomr-physical
//! core types — so the workspace compiles cleanly on macOS, Windows,
//! and bare CI runners without Linux-only deps.
//!
//! ## Features
//!
//! - `can` — pull in `socketcan` and expose [`bus::can::CanBusActor`].
//! - `i2c` — pull in `linux-embedded-hal` and expose
//!   [`bus::i2c::I2cBusActor`].
//! - `spi` — pull in `linux-embedded-hal` and expose
//!   [`bus::spi::SpiDevice`].
//! - `bno085` — implies `i2c`; enables the BNO085 driver.
//! - `odrive` — implies `can`; enables the ODrive driver.
//! - `qdd-mit` — implies `can`; enables the MIT QDD driver.
//! - `as5048a` — implies `spi`; enables the AS5048A encoder driver.
//! - `all-drivers` — pulls in every driver above.
//!
//! ## Two-form contract
//!
//! Bus actors follow the same two-form pattern the rest of the
//! workspace uses: build the actor offline, then call
//! `.spawn(system, name)` (or `.spawn_under(ctx, name)`) to promote it
//! into a live supervised actor. The returned `*Ref` handle is the
//! mailbox-typed interface; cheap to clone, safe to share.

pub mod bus;
pub mod drivers;
pub mod error;

#[cfg(test)]
mod loopback;

pub use crate::error::{HalError, Result};

/// Re-export of the atomr actor runtime this crate builds on. Matches
/// the convention used by `atomr-physical-sensing` so downstream
/// consumers have a single import path.
pub use atomr_core as actor;

/// Recommended glob-import surface for downstream crates.
pub mod prelude {
    pub use crate::error::{HalError, Result};

    #[cfg(feature = "can")]
    pub use crate::bus::can::{CanBusActor, CanBusActorRef, FilteredCanReceiver};
    #[cfg(feature = "i2c")]
    pub use crate::bus::i2c::{I2cBusActor, I2cBusActorRef};
    #[cfg(feature = "spi")]
    pub use crate::bus::spi::{SpiDevice, SpiMode};

    #[cfg(feature = "bno085")]
    pub use crate::drivers::bno085::{
        spawn_poller, Bno085AccelX, Bno085AccelY, Bno085AccelZ, Bno085Driver, Bno085GyroX,
        Bno085GyroY, Bno085GyroZ, Bno085PollerRef, Bno085QuatW, Bno085QuatX, Bno085QuatY,
        Bno085QuatZ, Bno085Snapshot,
    };
    #[cfg(feature = "odrive")]
    pub use crate::drivers::odrive::OdriveAxis;
    #[cfg(feature = "qdd-mit")]
    pub use crate::drivers::qdd_mit::{MitParams, QddMitFeedbackSensor, QddMitJoint};
    #[cfg(feature = "as5048a")]
    pub use crate::drivers::as5048a::As5048aEncoder;
}
