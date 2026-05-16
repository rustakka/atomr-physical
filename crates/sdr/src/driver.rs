//! [`HackRfDriver`] — the thin async wrapper around `rs_hackrf::HackRf`.
//!
//! This is the seam between rs-hackrf's blocking USB API and
//! atomr-physical's async world. It implements
//! [`atomr_physical_core::Device`] so the registry can introspect it,
//! but **not** [`Sensor`](atomr_physical_core::Sensor) or
//! [`Actuator`](atomr_physical_core::Actuator) — the
//! single-`Reading` / single-`Command` shapes those traits define
//! can't carry a streaming IQ flow. Instead the [`crate::SdrActor`]
//! adapter speaks SDR-shaped messages over its mailbox.
//!
//! ## State machine
//!
//! `rs_hackrf::HackRf::into_streaming_reader` consumes the device. To
//! survive a stop/start cycle the driver therefore tracks three
//! states:
//!
//! * `Idle(HackRf)`   — open, not streaming. Config writes go through
//!   the [`HackRf`] handle directly.
//! * `Streaming { … }` — the streaming reader owns the device on a
//!   dedicated thread; runtime tune / gain go through
//!   [`AsyncReadControlHandle`]. Sample-rate, baseband-filter, and
//!   antenna-port changes are **not** supported live (rs-hackrf
//!   doesn't expose them on the control handle); applying them
//!   requires `stop_rx` → `apply` → `start_rx`.
//! * `Closed`         — placeholder used while a transition is in
//!   progress, and the resting state after `stop_rx` (the device has
//!   to be re-opened by serial on the next `start_rx`).

use std::sync::Arc;
use std::time::Duration;

use arc_swap::ArcSwap;
use async_trait::async_trait;
use atomr_physical_core::{
    Capability, Device, DeviceDescriptor, DeviceId, DeviceKind, PhysicalError, Result, Unit,
};
use chrono::Utc;
use rs_hackrf::{AsyncReadControlHandle, AsyncReadHandle, HackRf};
use tokio::sync::{mpsc, Mutex};

use crate::config::SdrParams;
use crate::error::{SdrError, SdrResult};
use crate::iq::IqChunk;

/// Default model string we advertise in the device descriptor when
/// the user didn't supply one.
pub const DEFAULT_MODEL: &str = "hackrf-one";

/// The backend contract every SDR driver implements.
///
/// This is the seam that lets the actor run against both
/// [`HackRfDriver`] (real hardware) and `MockSdrDriver` (in the
/// testkit) without conditional code in the runner.
#[async_trait]
pub trait SdrBackend: Device {
    /// Apply every field of `params` to hardware. Only valid while
    /// idle.
    async fn apply(&self, params: &SdrParams) -> SdrResult<()>;
    /// Apply the live-tunable subset (centre / gains / amp) without
    /// interrupting the RX stream.
    async fn tune_live(&self, params: &SdrParams) -> SdrResult<()>;
    /// Begin RX, forwarding each chunk into `sink` as an [`IqChunk`].
    async fn start_rx(&self, sink: mpsc::Sender<IqChunk>) -> SdrResult<()>;
    /// Stop RX. Idempotent.
    async fn stop_rx(&self) -> SdrResult<()>;
    /// Submit a TX burst.
    async fn transmit(&self, samples: &[i8]) -> SdrResult<()>;
    /// The currently-active parameter set.
    fn params(&self) -> SdrParams;
}

/// The driver wrapping a single HackRF device.
pub struct HackRfDriver {
    descriptor: DeviceDescriptor,
    /// Optional serial number — captured at open so we can re-acquire
    /// the same physical device after a stop/start cycle.
    serial: Option<String>,
    state: Mutex<DriverState>,
    params: ArcSwap<SdrParams>,
}

enum DriverState {
    /// The device handle is gone (after `stop_rx`, or briefly during
    /// a state transition). `start_rx` re-opens by serial.
    Closed,
    /// Open, available for config writes; **not** streaming.
    Idle(HackRf),
    /// Streaming. The device handle has been consumed by
    /// [`HackRf::into_streaming_reader`] and now lives inside the
    /// rs-hackrf streaming thread.
    Streaming {
        control: AsyncReadControlHandle,
        /// Background task forwarding chunks from the rs-hackrf
        /// std::sync::mpsc receiver into the runner's tokio mpsc.
        /// `None` once the task has been awaited.
        forwarder: Option<tokio::task::JoinHandle<()>>,
    },
}

impl HackRfDriver {
    /// Open a device by index, with an optional serial filter, and
    /// produce a driver. Falls back to "any device" when `serial` is
    /// `None`.
    pub fn open(serial: Option<String>) -> SdrResult<Self> {
        let (device, resolved_serial) = open_device(serial.as_deref())?;
        let id = serial
            .clone()
            .unwrap_or_else(|| resolved_serial.clone().unwrap_or_else(|| "hackrf-0".into()));
        let descriptor = DeviceDescriptor::new(DeviceId::from(id), DeviceKind::Composite, DEFAULT_MODEL)
            .with_capability(Capability::new("iq_stream", Unit::Iq))
            .with_capability(Capability::new("rf_tx", Unit::Iq));
        Ok(Self {
            descriptor,
            serial: resolved_serial.or(serial),
            state: Mutex::new(DriverState::Idle(device)),
            params: ArcSwap::from_pointee(SdrParams::default_rx()),
        })
    }

    /// Open the first available device, no serial filter.
    pub fn open_first() -> SdrResult<Self> {
        Self::open(None)
    }

    /// List the serial numbers of every connected HackRF device.
    pub fn probe() -> SdrResult<Vec<String>> {
        HackRf::list_devices().map_err(SdrError::from)
    }

    /// Read the connected board's ID, firmware string, and serial — a
    /// concise dump suitable for the `sdr info` CLI subcommand.
    pub async fn info(&self) -> SdrResult<HackRfInfo> {
        let state = self.state.lock().await;
        match &*state {
            DriverState::Idle(device) => Ok(HackRfInfo {
                board_id: device.board_id().map_err(SdrError::from)?,
                version: device.version().map_err(SdrError::from)?,
                serial: device
                    .board_partid_serialno()
                    .map_err(SdrError::from)?
                    .2,
                usb_api_version: device.usb_api_version(),
            }),
            DriverState::Streaming { .. } => Err(SdrError::BadState(
                "info: device is streaming — stop_rx first",
            )),
            DriverState::Closed => Err(SdrError::BadState(
                "info: device handle is closed (call start_rx or re-open)",
            )),
        }
    }

    /// Push every field of `params` to hardware. Only valid while
    /// idle — call [`tune_live`](Self::tune_live) instead during
    /// streaming.
    pub async fn apply(&self, params: &SdrParams) -> SdrResult<()> {
        params.validate()?;
        let state = self.state.lock().await;
        let device = match &*state {
            DriverState::Idle(d) => d,
            DriverState::Streaming { .. } => {
                return Err(SdrError::BadState("apply: cannot reconfigure during streaming"));
            }
            DriverState::Closed => {
                return Err(SdrError::BadState("apply: device closed — open first"));
            }
        };
        device.set_freq(params.centre_hz).map_err(SdrError::from)?;
        device
            .set_sample_rate(params.sample_rate_hz)
            .map_err(SdrError::from)?;
        if let Some(bw) = params.baseband_filter_hz {
            device
                .set_baseband_filter_bandwidth(bw)
                .map_err(SdrError::from)?;
        }
        device
            .set_lna_gain(params.lna_gain_db as u32)
            .map_err(SdrError::from)?;
        device
            .set_vga_gain(params.vga_gain_db as u32)
            .map_err(SdrError::from)?;
        device
            .set_amp_enable(params.amp_enable)
            .map_err(SdrError::from)?;
        device
            .set_antenna_enable(params.antenna_port_pwr)
            .map_err(SdrError::from)?;
        self.params.store(Arc::new(params.clone()));
        Ok(())
    }

    /// Apply the subset of parameters that the rs-hackrf streaming
    /// thread accepts mid-stream: centre frequency, LNA / VGA gain,
    /// RF amp. Returns `BadState` if not currently streaming. Other
    /// fields in `params` are recorded into [`Self::params`] but not
    /// pushed to the device — the caller must `stop_rx` → `apply` →
    /// `start_rx` to take them.
    pub async fn tune_live(&self, params: &SdrParams) -> SdrResult<()> {
        params.validate()?;
        let state = self.state.lock().await;
        let control = match &*state {
            DriverState::Streaming { control, .. } => control,
            _ => return Err(SdrError::BadState("tune_live: not streaming")),
        };
        control.tune(params.centre_hz).map_err(SdrError::from)?;
        control
            .set_lna_gain(params.lna_gain_db as u32)
            .map_err(SdrError::from)?;
        control
            .set_vga_gain(params.vga_gain_db as u32)
            .map_err(SdrError::from)?;
        control
            .set_amp_enable(params.amp_enable)
            .map_err(SdrError::from)?;
        self.params.store(Arc::new(params.clone()));
        Ok(())
    }

    /// Begin RX streaming. The driver consumes the open `HackRf` into
    /// rs-hackrf's streaming reader, then spawns a background tokio
    /// blocking task that drains the reader's std::sync::mpsc
    /// receiver and forwards each chunk into `sink` as an [`IqChunk`].
    ///
    /// Returns `BadState` if the driver isn't currently `Idle`. The
    /// caller is responsible for `apply`ing parameters before
    /// starting; an unconfigured device falls back to whatever
    /// defaults the HackRF retained from its last power-cycle.
    pub async fn start_rx(&self, sink: mpsc::Sender<IqChunk>) -> SdrResult<()> {
        let mut state = self.state.lock().await;
        // Re-open if we're Closed (post stop_rx).
        if matches!(*state, DriverState::Closed) {
            let (device, _) = open_device(self.serial.as_deref())?;
            *state = DriverState::Idle(device);
        }
        let device = match std::mem::replace(&mut *state, DriverState::Closed) {
            DriverState::Idle(d) => d,
            DriverState::Streaming { control, forwarder } => {
                // Restore and bail — must stop_rx first.
                *state = DriverState::Streaming { control, forwarder };
                return Err(SdrError::BadState("start_rx: already streaming"));
            }
            DriverState::Closed => unreachable!("just re-opened above"),
        };
        let handle = device
            .into_streaming_reader(0, 0)
            .map_err(SdrError::from)?;
        let control = handle.control_handle();
        let params = self.params.load_full();
        let forwarder = spawn_forwarder(handle, sink, params);
        *state = DriverState::Streaming {
            control,
            forwarder: Some(forwarder),
        };
        Ok(())
    }

    /// Stop RX streaming. Idempotent — calling on a non-streaming
    /// driver is a no-op.
    pub async fn stop_rx(&self) -> SdrResult<()> {
        let mut state = self.state.lock().await;
        let previous = std::mem::replace(&mut *state, DriverState::Closed);
        match previous {
            DriverState::Idle(_) | DriverState::Closed => Ok(()),
            DriverState::Streaming { control, mut forwarder } => {
                control.stop();
                if let Some(handle) = forwarder.take() {
                    // The forwarder's recv() returns None once the
                    // streaming thread exits; awaiting joins cleanly.
                    let _ = tokio::time::timeout(Duration::from_secs(2), handle).await;
                }
                Ok(())
            }
        }
    }

    /// Submit a transmit burst. Returns `Err(Unsupported)` — the
    /// active rs-hackrf 0.4 backend is RX-only. The signature is
    /// stable so call sites can target it today and not break when
    /// TX lands.
    pub async fn transmit(&self, _samples: &[i8]) -> SdrResult<()> {
        Err(SdrError::Unsupported(
            "TX not supported by rs-hackrf 0.4 — RX-only backend",
        ))
    }

    /// Snapshot the currently-active parameter set (the last one that
    /// `apply` or `tune_live` accepted).
    pub fn params(&self) -> SdrParams {
        (*self.params.load_full()).clone()
    }

    /// The serial number captured at open, if the device advertised
    /// one over USB.
    pub fn serial(&self) -> Option<&str> {
        self.serial.as_deref()
    }
}

#[async_trait]
impl SdrBackend for HackRfDriver {
    async fn apply(&self, params: &SdrParams) -> SdrResult<()> {
        HackRfDriver::apply(self, params).await
    }
    async fn tune_live(&self, params: &SdrParams) -> SdrResult<()> {
        HackRfDriver::tune_live(self, params).await
    }
    async fn start_rx(&self, sink: mpsc::Sender<IqChunk>) -> SdrResult<()> {
        HackRfDriver::start_rx(self, sink).await
    }
    async fn stop_rx(&self) -> SdrResult<()> {
        HackRfDriver::stop_rx(self).await
    }
    async fn transmit(&self, samples: &[i8]) -> SdrResult<()> {
        HackRfDriver::transmit(self, samples).await
    }
    fn params(&self) -> SdrParams {
        HackRfDriver::params(self)
    }
}

#[async_trait]
impl Device for HackRfDriver {
    fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    async fn health_check(&self) -> Result<()> {
        // Same triage as `info`, minus formatting — a working USB
        // round-trip on the board-id endpoint is the cheapest "is
        // anyone there" probe rs-hackrf exposes.
        let state = self.state.lock().await;
        match &*state {
            DriverState::Idle(d) => {
                d.board_id()
                    .map_err(|e| PhysicalError::from(SdrError::from(e)))?;
                Ok(())
            }
            DriverState::Streaming { .. } => Ok(()),
            DriverState::Closed => Err(PhysicalError::NotReady {
                device: self.descriptor.id.to_string(),
                reason: "device handle is closed".into(),
            }),
        }
    }
}

/// Concise device summary returned by [`HackRfDriver::info`].
#[derive(Debug, Clone)]
pub struct HackRfInfo {
    /// Raw board ID byte (see `rs_hackrf::transport::board_id_name`).
    pub board_id: u8,
    /// Firmware version string.
    pub version: String,
    /// 32-character hex-formatted serial number.
    pub serial: String,
    /// USB API version (bcdDevice).
    pub usb_api_version: u16,
}

/// Open by serial (or first available if `serial` is `None`),
/// returning the device handle and the resolved serial.
fn open_device(serial: Option<&str>) -> SdrResult<(HackRf, Option<String>)> {
    match serial {
        None => {
            let device = HackRf::open_first().map_err(SdrError::from)?;
            let resolved = device
                .board_partid_serialno()
                .ok()
                .map(|(_, _, s)| s);
            Ok((device, resolved))
        }
        Some(target) => {
            let serials = HackRf::list_devices().map_err(SdrError::from)?;
            let idx = serials
                .iter()
                .position(|s| s.contains(target))
                .ok_or_else(|| {
                    SdrError::Transport(format!("no HackRF with serial containing {target:?}"))
                })?;
            let device = HackRf::open_by_index(idx).map_err(SdrError::from)?;
            Ok((device, Some(serials[idx].clone())))
        }
    }
}

/// Spawn a tokio blocking task that drains the rs-hackrf streaming
/// reader and forwards each chunk as an [`IqChunk`] into `sink`.
///
/// Exits cleanly when the reader returns `None` (the rs-hackrf
/// streaming thread has been stopped) or when `sink` is dropped.
fn spawn_forwarder(
    handle: AsyncReadHandle,
    sink: mpsc::Sender<IqChunk>,
    initial_params: Arc<SdrParams>,
) -> tokio::task::JoinHandle<()> {
    tokio::task::spawn_blocking(move || {
        let mut sequence: u64 = 0;
        loop {
            match handle.recv() {
                None => break,
                Some(Err(e)) => {
                    tracing::warn!(error = ?e, "hackrf streaming error; continuing");
                    continue;
                }
                Some(Ok(bytes)) => {
                    let samples: Vec<i8> = bytes.into_iter().map(|b| b as i8).collect();
                    let chunk = IqChunk {
                        sequence,
                        captured_at: Utc::now(),
                        centre_hz: initial_params.centre_hz,
                        sample_rate_hz: initial_params.sample_rate_hz,
                        samples: Arc::from(samples),
                    };
                    sequence = sequence.wrapping_add(1);
                    // Use blocking_send so we propagate backpressure
                    // — the runner's mailbox is a tokio channel.
                    if sink.blocking_send(chunk).is_err() {
                        // Receiver dropped — runner is gone.
                        break;
                    }
                }
            }
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn driver_info_struct_is_debug() {
        let info = HackRfInfo {
            board_id: 2,
            version: "fw".into(),
            serial: "deadbeef".into(),
            usb_api_version: 0x0106,
        };
        // Compile-only check that Debug is implemented; print to
        // /dev/null avoids polluting test output.
        let _ = format!("{info:?}");
    }
}
