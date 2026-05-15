//! In-process loopback bus doubles for hal tests.
//!
//! Each double mirrors the public surface of its production
//! counterpart so a driver under test can be wired up exactly as it
//! would be in production — just substituting the real socket /
//! character device with an in-memory queue.
//!
//! These types are gated behind `#[cfg(test)]` in the crate root, so
//! they never widen the public API.

#[cfg(feature = "can")]
pub use can::LoopbackCanBus;
#[cfg(feature = "i2c")]
pub use i2c::LoopbackI2cBus;
#[cfg(feature = "spi")]
pub use spi::LoopbackSpiDevice;

#[cfg(feature = "can")]
mod can {
    use std::sync::Arc;

    use async_trait::async_trait;
    use atomr_core::actor::{Actor, ActorSystem, Context, Props};
    use parking_lot::Mutex;
    use socketcan::CanFrame;
    use tokio::sync::broadcast;

    use crate::bus::can::{CanBusActorRef, CanBusMsg};

    /// An in-process loopback CAN bus.
    ///
    /// `as_ref` returns a [`CanBusActorRef`] driver code can use the
    /// same way it would use a real one. `pop_frame` lets the test
    /// drain frames sent by the driver under test; `inject_frame`
    /// publishes a frame to all subscribers.
    pub struct LoopbackCanBus {
        sent: Arc<Mutex<Vec<CanFrame>>>,
        broadcast_tx: broadcast::Sender<CanFrame>,
        actor_ref: CanBusActorRef,
        _system: ActorSystem,
    }

    impl LoopbackCanBus {
        /// Build a fresh loopback CAN bus on its own private actor
        /// system. The system stays alive as long as the loopback
        /// handle does.
        pub async fn new() -> Self {
            let sent: Arc<Mutex<Vec<CanFrame>>> = Arc::new(Mutex::new(Vec::new()));
            let (broadcast_tx, _) = broadcast::channel::<CanFrame>(64);

            let system = atomr_core::actor::ActorSystem::create(
                "hal-loopback-can",
                atomr_config::Config::reference(),
            )
            .await
            .expect("failed to create loopback ActorSystem");

            let sent_for_factory = sent.clone();
            let props = Props::create(move || LoopbackCanRunner {
                sent: sent_for_factory.clone(),
            });
            let inner = system
                .actor_of(props, "loopback-can-runner")
                .expect("failed to spawn loopback can runner");
            let actor_ref = CanBusActorRef::from_channels(inner, broadcast_tx.clone());
            Self {
                sent,
                broadcast_tx,
                actor_ref,
                _system: system,
            }
        }

        /// Obtain a `CanBusActorRef` driver code can use.
        pub fn as_ref(&self) -> CanBusActorRef {
            self.actor_ref.clone()
        }

        /// Pop the oldest sent frame from the loopback buffer.
        pub fn pop_frame(&self) -> Option<CanFrame> {
            let mut guard = self.sent.lock();
            if guard.is_empty() {
                None
            } else {
                Some(guard.remove(0))
            }
        }

        /// Publish a frame to all subscribers — used to simulate
        /// inbound traffic.
        #[allow(dead_code)]
        pub fn inject_frame(&self, frame: CanFrame) {
            let _ = self.broadcast_tx.send(frame);
        }
    }

    struct LoopbackCanRunner {
        sent: Arc<Mutex<Vec<CanFrame>>>,
    }

    #[async_trait]
    impl Actor for LoopbackCanRunner {
        type Msg = CanBusMsg;

        async fn handle(&mut self, _ctx: &mut Context<Self>, msg: CanBusMsg) {
            match msg {
                CanBusMsg::Send { frame, reply } => {
                    self.sent.lock().push(frame);
                    let _ = reply.send(Ok(()));
                }
                CanBusMsg::Health { reply } => {
                    let _ = reply.send(Ok(()));
                }
            }
        }
    }
}

#[cfg(feature = "i2c")]
mod i2c {
    use std::collections::VecDeque;
    use std::sync::Arc;

    use async_trait::async_trait;
    use atomr_core::actor::{Actor, ActorSystem, Context, Props};
    use parking_lot::Mutex;

    use crate::bus::i2c::{I2cBusActorRef, I2cBusMsg};
    use crate::error::HalError;

    /// In-process I2C bus double with a queue of canned read responses.
    ///
    /// `pre_populate` and `queue_response` let tests stage the data the
    /// driver under test will receive on its next read.
    pub struct LoopbackI2cBus {
        responses: Arc<Mutex<VecDeque<Vec<u8>>>>,
        writes: Arc<Mutex<Vec<(u8, Vec<u8>)>>>,
        actor_ref: I2cBusActorRef,
        _system: ActorSystem,
    }

    impl LoopbackI2cBus {
        /// Build a fresh loopback I2C bus.
        pub async fn new() -> Self {
            let responses: Arc<Mutex<VecDeque<Vec<u8>>>> = Arc::new(Mutex::new(VecDeque::new()));
            let writes: Arc<Mutex<Vec<(u8, Vec<u8>)>>> = Arc::new(Mutex::new(Vec::new()));

            let system = atomr_core::actor::ActorSystem::create(
                "hal-loopback-i2c",
                atomr_config::Config::reference(),
            )
            .await
            .expect("failed to create loopback ActorSystem");

            let responses_for_factory = responses.clone();
            let writes_for_factory = writes.clone();
            let props = Props::create(move || LoopbackI2cRunner {
                responses: responses_for_factory.clone(),
                writes: writes_for_factory.clone(),
            });
            let inner = system
                .actor_of(props, "loopback-i2c-runner")
                .expect("failed to spawn loopback i2c runner");
            let actor_ref = I2cBusActorRef::from_actor_ref(inner);
            Self {
                responses,
                writes,
                actor_ref,
                _system: system,
            }
        }

        /// Obtain an `I2cBusActorRef` driver code can use.
        pub fn as_ref(&self) -> I2cBusActorRef {
            self.actor_ref.clone()
        }

        /// Push a canned read response to the back of the queue.
        pub fn queue_response(&self, bytes: Vec<u8>) {
            self.responses.lock().push_back(bytes);
        }

        /// Snapshot of `(addr, bytes_written)` for every write so far.
        #[allow(dead_code)]
        pub fn writes(&self) -> Vec<(u8, Vec<u8>)> {
            self.writes.lock().clone()
        }
    }

    struct LoopbackI2cRunner {
        responses: Arc<Mutex<VecDeque<Vec<u8>>>>,
        writes: Arc<Mutex<Vec<(u8, Vec<u8>)>>>,
    }

    impl LoopbackI2cRunner {
        fn pop_response(&self, len: usize) -> std::result::Result<Vec<u8>, HalError> {
            let mut guard = self.responses.lock();
            let resp = guard
                .pop_front()
                .ok_or_else(|| HalError::Bus("loopback i2c: no canned response".into()))?;
            // Truncate or zero-pad to the requested length so the
            // caller's `len` argument is always honoured.
            let mut out = vec![0u8; len];
            let n = len.min(resp.len());
            out[..n].copy_from_slice(&resp[..n]);
            Ok(out)
        }
    }

    #[async_trait]
    impl Actor for LoopbackI2cRunner {
        type Msg = I2cBusMsg;

        async fn handle(&mut self, _ctx: &mut Context<Self>, msg: I2cBusMsg) {
            match msg {
                I2cBusMsg::ReadRegister {
                    addr,
                    register,
                    len,
                    reply,
                } => {
                    self.writes.lock().push((addr, vec![register]));
                    let _ = reply.send(self.pop_response(len));
                }
                I2cBusMsg::Write { addr, bytes, reply } => {
                    self.writes.lock().push((addr, bytes));
                    let _ = reply.send(Ok(()));
                }
                I2cBusMsg::WriteRegister {
                    addr,
                    register,
                    bytes,
                    reply,
                } => {
                    let mut full = Vec::with_capacity(bytes.len() + 1);
                    full.push(register);
                    full.extend(bytes);
                    self.writes.lock().push((addr, full));
                    let _ = reply.send(Ok(()));
                }
                I2cBusMsg::WriteThenRead {
                    addr,
                    write,
                    read_len,
                    reply,
                } => {
                    self.writes.lock().push((addr, write));
                    let _ = reply.send(self.pop_response(read_len));
                }
                I2cBusMsg::Health { reply } => {
                    let _ = reply.send(Ok(()));
                }
            }
        }
    }
}

#[cfg(feature = "spi")]
mod spi {
    use std::collections::VecDeque;

    use crate::bus::spi::{LoopbackInner, SpiDevice};

    /// In-process SPI device double — driver code receives canned
    /// responses pre-staged by the test.
    pub struct LoopbackSpiDevice {
        device: SpiDevice,
    }

    impl LoopbackSpiDevice {
        /// Build a fresh loopback SPI device with an empty response queue.
        pub fn new() -> Self {
            let inner = LoopbackInner {
                responses: VecDeque::new(),
                written: Vec::new(),
            };
            let device = SpiDevice::from_loopback(inner);
            Self { device }
        }

        /// Builder-style: queue a canned response for the next transfer.
        pub fn with_response(self, bytes: Vec<u8>) -> Self {
            // Rebuild the inner queue with the new response appended.
            // We do this through `transfer` so we can also be used by
            // chained calls. Implementation: pop the current inner,
            // push, reinstall.
            //
            // To keep the API simple, we go through `SpiDevice`'s
            // `from_loopback` constructor again — but we'd lose the
            // previously queued bytes. Instead, mutate the inner
            // queue directly through a helper on `SpiDevice`.
            push_loopback_response(&self.device, bytes);
            self
        }

        /// Borrow a clone of the underlying device handle.
        pub fn device(&self) -> SpiDevice {
            self.device.clone()
        }

        /// Snapshot of every byte buffer written/transferred so far.
        #[allow(dead_code)]
        pub fn written(&self) -> Vec<Vec<u8>> {
            self.device.loopback_written()
        }
    }

    fn push_loopback_response(device: &SpiDevice, bytes: Vec<u8>) {
        // Use a private helper on SpiDevice so the queue stays
        // encapsulated in bus::spi.
        device.loopback_push_response(bytes);
    }
}
