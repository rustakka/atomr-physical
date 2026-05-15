//! Supervised I2C bus owner over `linux-embedded-hal::I2cdev`.
//!
//! One [`I2cBusActor`] owns the I2C character device. Concurrent driver
//! requests are serialised by the actor's mailbox so the kernel-level
//! address re-selection is safe.

use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use embedded_hal::i2c::I2c;
use linux_embedded_hal::I2cdev;
use tokio::sync::oneshot;

use crate::error::{HalError, Result};

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

/// A supervised I2C bus actor.
///
/// Construct with [`I2cBusActor::new`] and promote with
/// [`spawn`](Self::spawn) / [`spawn_under`](Self::spawn_under).
#[derive(Clone)]
pub struct I2cBusActor {
    device_path: String,
}

impl I2cBusActor {
    /// Build an I2C bus actor that will open `device_path` (e.g.
    /// `"/dev/i2c-1"`) on `pre_start`.
    pub fn new(device_path: impl Into<String>) -> Self {
        Self {
            device_path: device_path.into(),
        }
    }

    /// Promote into a supervised atomr actor.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<I2cBusActorRef, ActorSystemError> {
        let props = self.into_runner_props();
        let inner = system.actor_of(props, name)?;
        Ok(I2cBusActorRef { inner })
    }

    /// Promote into a supervised atomr actor as a child of `ctx`.
    pub fn spawn_under<P: Actor>(self, ctx: &mut Context<P>, name: &str) -> Result<I2cBusActorRef> {
        let props = self.into_runner_props();
        let inner = ctx
            .spawn(props, name)
            .map_err(|e| HalError::Bus(format!("i2c bus child spawn failed: {e}")))?;
        Ok(I2cBusActorRef { inner })
    }

    fn into_runner_props(self) -> Props<I2cBusRunner> {
        let path = self.device_path;
        Props::create(move || I2cBusRunner {
            device_path: path.clone(),
            dev: None,
        })
    }
}

/// Typed handle to a spawned [`I2cBusActor`].
#[derive(Clone)]
pub struct I2cBusActorRef {
    inner: ActorRef<I2cBusMsg>,
}

impl I2cBusActorRef {
    /// Read `len` bytes starting at `register` on the target `addr`.
    pub async fn read_register(&self, addr: u8, register: u8, len: usize) -> Result<Vec<u8>> {
        self.inner
            .ask_with(
                |reply| I2cBusMsg::ReadRegister {
                    addr,
                    register,
                    len,
                    reply,
                },
                ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_hal)?
    }

    /// Write `bytes` to `addr` in a single transaction.
    pub async fn write(&self, addr: u8, bytes: Vec<u8>) -> Result<()> {
        self.inner
            .ask_with(|reply| I2cBusMsg::Write { addr, bytes, reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_hal)?
    }

    /// Write `bytes` to `register` on `addr`.
    pub async fn write_register(&self, addr: u8, register: u8, bytes: Vec<u8>) -> Result<()> {
        self.inner
            .ask_with(
                |reply| I2cBusMsg::WriteRegister {
                    addr,
                    register,
                    bytes,
                    reply,
                },
                ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_hal)?
    }

    /// Write `write` and then read `read_len` bytes back, as one
    /// repeated-start transaction.
    pub async fn write_then_read(&self, addr: u8, write: Vec<u8>, read_len: usize) -> Result<Vec<u8>> {
        self.inner
            .ask_with(
                |reply| I2cBusMsg::WriteThenRead {
                    addr,
                    write,
                    read_len,
                    reply,
                },
                ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_hal)?
    }

    /// Probe the bus actor for liveness.
    pub async fn health_check(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| I2cBusMsg::Health { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_hal)?
    }

    /// Construct an [`I2cBusActorRef`] directly from a fabricated
    /// mailbox. Used by the loopback test double.
    #[cfg(test)]
    pub(crate) fn from_actor_ref(inner: ActorRef<I2cBusMsg>) -> Self {
        Self { inner }
    }
}

/// The mailbox protocol of a live [`I2cBusActor`].
pub enum I2cBusMsg {
    /// Read `len` bytes from `register` on the device at `addr`.
    ReadRegister {
        /// Device address.
        addr: u8,
        /// Register offset.
        register: u8,
        /// Bytes to read after the register pointer write.
        len: usize,
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<Vec<u8>>>,
    },
    /// Write a raw byte buffer.
    Write {
        /// Device address.
        addr: u8,
        /// Bytes to write.
        bytes: Vec<u8>,
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Write `bytes` to `register` on `addr`.
    WriteRegister {
        /// Device address.
        addr: u8,
        /// Register offset.
        register: u8,
        /// Payload following the register pointer.
        bytes: Vec<u8>,
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Write-then-read transaction with a repeated start.
    WriteThenRead {
        /// Device address.
        addr: u8,
        /// Bytes to write before the read phase.
        write: Vec<u8>,
        /// How many bytes to read.
        read_len: usize,
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<Vec<u8>>>,
    },
    /// Probe for liveness.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
}

fn ask_to_hal(e: atomr_core::actor::AskError) -> HalError {
    HalError::Bus(format!("i2c bus actor ask failed: {e:?}"))
}

/// Internal Actor implementation backing a spawned [`I2cBusActor`].
struct I2cBusRunner {
    device_path: String,
    dev: Option<I2cdev>,
}

#[async_trait]
impl Actor for I2cBusRunner {
    type Msg = I2cBusMsg;

    async fn pre_start(&mut self, _ctx: &mut Context<Self>) {
        match I2cdev::new(&self.device_path) {
            Ok(dev) => self.dev = Some(dev),
            Err(e) => {
                tracing::error!(path = %self.device_path, error = %e, "failed to open I2C device");
            }
        }
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: I2cBusMsg) {
        match msg {
            I2cBusMsg::ReadRegister {
                addr,
                register,
                len,
                reply,
            } => {
                let result = self.with_dev(|dev| {
                    let mut buf = vec![0u8; len];
                    dev.write_read(addr, &[register], &mut buf)
                        .map_err(|e| HalError::Bus(format!("i2c read_register: {e}")))?;
                    Ok(buf)
                });
                let _ = reply.send(result);
            }
            I2cBusMsg::Write { addr, bytes, reply } => {
                let result = self.with_dev(|dev| {
                    dev.write(addr, &bytes)
                        .map_err(|e| HalError::Bus(format!("i2c write: {e}")))
                });
                let _ = reply.send(result);
            }
            I2cBusMsg::WriteRegister {
                addr,
                register,
                bytes,
                reply,
            } => {
                let result = self.with_dev(|dev| {
                    let mut payload = Vec::with_capacity(bytes.len() + 1);
                    payload.push(register);
                    payload.extend_from_slice(&bytes);
                    dev.write(addr, &payload)
                        .map_err(|e| HalError::Bus(format!("i2c write_register: {e}")))
                });
                let _ = reply.send(result);
            }
            I2cBusMsg::WriteThenRead {
                addr,
                write,
                read_len,
                reply,
            } => {
                let result = self.with_dev(|dev| {
                    let mut buf = vec![0u8; read_len];
                    dev.write_read(addr, &write, &mut buf)
                        .map_err(|e| HalError::Bus(format!("i2c write_then_read: {e}")))?;
                    Ok(buf)
                });
                let _ = reply.send(result);
            }
            I2cBusMsg::Health { reply } => {
                let _ = reply.send(if self.dev.is_some() {
                    Ok(())
                } else {
                    Err(HalError::NotConfigured("i2c device not open".into()))
                });
            }
        }
    }
}

impl I2cBusRunner {
    fn with_dev<T, F>(&mut self, f: F) -> Result<T>
    where
        F: FnOnce(&mut I2cdev) -> Result<T>,
    {
        match self.dev.as_mut() {
            Some(dev) => tokio::task::block_in_place(|| f(dev)),
            None => Err(HalError::NotConfigured("i2c device not open".into())),
        }
    }
}
