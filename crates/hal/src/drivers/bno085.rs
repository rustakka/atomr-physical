//! BNO085 9-axis IMU driver over I2C (simplified SH-2 client).
//!
//! The BNO085 speaks the **SH-2** protocol on top of Bosch's **HCP**
//! transport. A full HCP/SH-2 stack tracks channels, sequence numbers,
//! product-id queries, and feature-response handshakes. This driver
//! implements **only** the subset needed to bring up rotation-vector,
//! calibrated-gyro, and accelerometer reports at a fixed rate, then
//! parse those reports out of an inbound packet stream.
//!
//! ## What is simplified
//!
//! - We send the SH-2 **Set Feature Command** (report ID `0xFD`) once
//!   on the SHTP **Control** channel (channel 2) for each of the three
//!   reports we want, with a 5 ms inter-feature gap. We do **not** wait
//!   for the corresponding Get Feature Response — the device starts
//!   streaming reports on the **Input Sensor Reports** channel
//!   (channel 3) regardless.
//! - We don't track HCP sequence numbers. The peripheral does, but it
//!   tolerates a stuck sequence number on outbound packets at the cost
//!   of one error frame per request.
//! - On poll, we read a single 4-byte SHTP header, then drain the
//!   advertised payload. The first report-id byte of an input-report
//!   payload is then decoded. Multi-report packet de-multiplexing
//!   (several reports concatenated in one packet) is supported.
//!
//! The Quaternion is reported as Q14 fixed-point per the SH-2
//! datasheet (Reference Manual rev 1.4 §6.5.18).

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_physical_core::{
    Capability, Device, DeviceDescriptor, DeviceId, DeviceKind, PhysicalError, Quantity, Reading,
    Result, Sensor, SensorId, Unit,
};
use parking_lot::RwLock;
use tokio::sync::oneshot;

use crate::bus::i2c::I2cBusActorRef;
use crate::error::HalError;

/// Default I2C address (ADR pin tied low).
pub const DEFAULT_ADDRESS: u8 = 0x4A;

/// Report ids we care about on the Input Sensor Reports channel.
const REPORT_ID_ACCELEROMETER: u8 = 0x01;
const REPORT_ID_GYROSCOPE: u8 = 0x02;
const REPORT_ID_ROTATION_VECTOR: u8 = 0x05;

/// SHTP channel for control commands.
const SHTP_CHANNEL_CONTROL: u8 = 2;

/// SH-2 Set Feature Command id (length 17 bytes after the report id).
const SH2_SET_FEATURE: u8 = 0xFD;

/// Snapshot of the latest reports drained from the device.
#[derive(Debug, Clone, Default)]
pub struct Bno085Snapshot {
    /// `[w, x, y, z]` rotation vector. Zero until first valid sample.
    pub orientation: [f64; 4],
    /// `[gx, gy, gz]` calibrated gyroscope, rad/s.
    pub angular_velocity: [f64; 3],
    /// `[ax, ay, az]` linear acceleration, m/s².
    pub linear_acceleration: [f64; 3],
    /// `true` once at least one rotation-vector report has been
    /// observed.
    pub valid: bool,
    /// Wall-clock time of the most recent snapshot update, ms since
    /// the Unix epoch.
    pub last_update_ms: i64,
}

/// BNO085 IMU driver. The driver itself only mutates the shared
/// snapshot; per-axis sensors hold an `Arc<Bno085Driver>` and read
/// from it.
pub struct Bno085Driver {
    bus: I2cBusActorRef,
    address: u8,
    snapshot: Arc<RwLock<Bno085Snapshot>>,
}

impl Bno085Driver {
    /// Build a driver bound to `bus` at the given I2C `address` (the
    /// BNO085 defaults to `0x4A` when ADR is tied low, `0x4B` otherwise).
    pub fn new(bus: I2cBusActorRef, address: u8) -> Self {
        Self {
            bus,
            address,
            snapshot: Arc::new(RwLock::new(Bno085Snapshot::default())),
        }
    }

    /// Snapshot the most recently parsed report state.
    pub fn snapshot(&self) -> Bno085Snapshot {
        self.snapshot.read().clone()
    }

    /// Enable the three sensor reports at `report_rate_hz`.
    ///
    /// Implementation detail: SH-2 Set-Feature commands carry a 17-byte
    /// payload (`report_id, flags, change_sensitivity[2], report_interval[4], batch_interval[4], sensor_specific[4]`).
    /// We zero everything but `report_id` and `report_interval`.
    pub async fn initialize(&self, report_rate_hz: u32) -> Result<()> {
        let interval_us: u32 = if report_rate_hz == 0 {
            0
        } else {
            1_000_000 / report_rate_hz
        };
        for report in [REPORT_ID_ACCELEROMETER, REPORT_ID_GYROSCOPE, REPORT_ID_ROTATION_VECTOR] {
            let packet = build_set_feature_packet(report, interval_us);
            self.bus
                .write(self.address, packet)
                .await
                .map_err(PhysicalError::from)?;
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        Ok(())
    }

    /// Read one SHTP packet from the device and decode any reports
    /// inside it into the shared snapshot.
    pub async fn poll_once(&self) -> Result<()> {
        // Read the 4-byte header first.
        let header = self
            .bus
            .write_then_read(self.address, Vec::new(), 4)
            .await
            .map_err(PhysicalError::from)?;
        if header.len() < 4 {
            return Err(PhysicalError::from(HalError::Frame(
                "bno085 short header".into(),
            )));
        }
        let length = ((header[0] as u16) | (((header[1] as u16) & 0x7F) << 8)) as usize;
        if length < 4 {
            // Empty packet — nothing to decode.
            return Ok(());
        }
        let channel = header[2];
        // Total packet length includes the 4-byte header itself.
        let payload_len = length - 4;
        if payload_len == 0 {
            return Ok(());
        }
        let payload = self
            .bus
            .write_then_read(self.address, Vec::new(), 4 + payload_len)
            .await
            .map_err(PhysicalError::from)?;
        // The full read repeats the header, so skip the first 4 bytes
        // when interpreting payload contents.
        let body = if payload.len() >= 4 {
            &payload[4..]
        } else {
            &payload[..]
        };
        if channel == 3 {
            // Input Sensor Reports — walk concatenated reports.
            decode_input_reports(body, &self.snapshot);
        }
        Ok(())
    }
}

#[async_trait]
impl Device for Bno085Driver {
    fn descriptor(&self) -> &DeviceDescriptor {
        BNO085_DESCRIPTOR.with(|d| {
            // SAFETY: the thread-local descriptor is a once-init.
            unsafe { &*d.get() }
        })
    }

    async fn health_check(&self) -> Result<()> {
        self.bus.health_check().await.map_err(PhysicalError::from)
    }
}

// Per-thread once-init descriptor avoids cloning the same metadata for
// each per-axis sensor — every per-axis wrapper points at the shared
// driver, but the driver itself doesn't carry per-axis metadata.
thread_local! {
    static BNO085_DESCRIPTOR: std::cell::UnsafeCell<DeviceDescriptor> = std::cell::UnsafeCell::new(
        DeviceDescriptor::new(
            DeviceId::from("bno085"),
            DeviceKind::Composite,
            "bno085",
        )
        .with_capability(Capability::new("orientation_quaternion", Unit::Scalar))
        .with_capability(Capability::new("angular_velocity", Unit::RadianPerSecond))
        .with_capability(Capability::new("linear_acceleration", Unit::MetrePerSecondSquared)),
    );
}

/// SH-2 Set Feature Command packet (21 bytes total — 4-byte SHTP
/// header + 17-byte SH-2 payload). Sent on channel 2.
fn build_set_feature_packet(report_id: u8, report_interval_us: u32) -> Vec<u8> {
    let payload_len = 21u16; // total including header
    let mut buf = Vec::with_capacity(21);
    // SHTP header.
    buf.push((payload_len & 0xFF) as u8);
    buf.push(((payload_len >> 8) & 0xFF) as u8);
    buf.push(SHTP_CHANNEL_CONTROL);
    buf.push(0); // sequence number — leave at 0 (see module docs).
                 // SH-2 payload.
    buf.push(SH2_SET_FEATURE);
    buf.push(report_id);
    buf.push(0); // feature flags
    buf.push(0); // change sensitivity LSB
    buf.push(0); // change sensitivity MSB
    buf.extend_from_slice(&report_interval_us.to_le_bytes());
    buf.extend_from_slice(&0u32.to_le_bytes()); // batch interval
    buf.extend_from_slice(&0u32.to_le_bytes()); // sensor-specific cfg
    debug_assert_eq!(buf.len(), 21);
    buf
}

/// Decode all reports concatenated in a single Input Sensor Reports
/// payload. Each report begins with a 5-byte header
/// `[report_id, seq, status, delay_lsb, delay_msb]` followed by the
/// fixed-size body for that report id.
fn decode_input_reports(payload: &[u8], snapshot: &Arc<RwLock<Bno085Snapshot>>) {
    let mut cursor = 0usize;
    let now_ms = chrono::Utc::now().timestamp_millis();
    while cursor + 5 < payload.len() {
        let report_id = payload[cursor];
        let body_start = cursor + 5;
        match report_id {
            REPORT_ID_ACCELEROMETER => {
                if body_start + 6 > payload.len() {
                    break;
                }
                let x = q_point_to_f64(read_i16_le(&payload[body_start..]), 8);
                let y = q_point_to_f64(read_i16_le(&payload[body_start + 2..]), 8);
                let z = q_point_to_f64(read_i16_le(&payload[body_start + 4..]), 8);
                let mut s = snapshot.write();
                s.linear_acceleration = [x, y, z];
                s.last_update_ms = now_ms;
                cursor = body_start + 6;
            }
            REPORT_ID_GYROSCOPE => {
                if body_start + 6 > payload.len() {
                    break;
                }
                let x = q_point_to_f64(read_i16_le(&payload[body_start..]), 9);
                let y = q_point_to_f64(read_i16_le(&payload[body_start + 2..]), 9);
                let z = q_point_to_f64(read_i16_le(&payload[body_start + 4..]), 9);
                let mut s = snapshot.write();
                s.angular_velocity = [x, y, z];
                s.last_update_ms = now_ms;
                cursor = body_start + 6;
            }
            REPORT_ID_ROTATION_VECTOR => {
                if body_start + 10 > payload.len() {
                    break;
                }
                let i = q_point_to_f64(read_i16_le(&payload[body_start..]), 14);
                let j = q_point_to_f64(read_i16_le(&payload[body_start + 2..]), 14);
                let k = q_point_to_f64(read_i16_le(&payload[body_start + 4..]), 14);
                let real = q_point_to_f64(read_i16_le(&payload[body_start + 6..]), 14);
                let mut s = snapshot.write();
                s.orientation = [real, i, j, k];
                s.valid = true;
                s.last_update_ms = now_ms;
                cursor = body_start + 10;
            }
            _ => {
                // Unknown report — bail out; without a length table we
                // can't safely re-sync. Subsequent polls will resync on
                // the next SHTP packet boundary.
                break;
            }
        }
    }
}

fn read_i16_le(b: &[u8]) -> i16 {
    if b.len() < 2 {
        0
    } else {
        i16::from_le_bytes([b[0], b[1]])
    }
}

fn q_point_to_f64(value: i16, q: u32) -> f64 {
    (value as f64) / ((1u32 << q) as f64)
}

/// Spawn a background poller actor that ticks `driver.poll_once()` at
/// `rate_hz`.
pub fn spawn_poller(
    driver: Arc<Bno085Driver>,
    system: &ActorSystem,
    name: &str,
    rate_hz: u32,
) -> std::result::Result<Bno085PollerRef, ActorSystemError> {
    let rate_hz = rate_hz.max(1);
    let driver_for_factory = driver.clone();
    let props = Props::create(move || Bno085Poller {
        driver: driver_for_factory.clone(),
        period: Duration::from_millis(1000 / rate_hz as u64),
    });
    let inner = system.actor_of(props, name)?;
    Ok(Bno085PollerRef { inner })
}

/// Typed handle to a spawned BNO085 poller.
#[derive(Clone)]
pub struct Bno085PollerRef {
    inner: ActorRef<Bno085PollerMsg>,
}

impl Bno085PollerRef {
    /// Probe for liveness.
    pub async fn health_check(&self) -> Result<()> {
        self.inner
            .ask_with(
                |reply| Bno085PollerMsg::Health { reply },
                Duration::from_secs(5),
            )
            .await
            .map_err(|e| PhysicalError::Fault(format!("bno085 poller ask failed: {e:?}")))?
    }
}

/// Mailbox protocol for the poller actor.
pub enum Bno085PollerMsg {
    /// Internal tick fired by the polling loop.
    Tick,
    /// Probe for liveness.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
}

struct Bno085Poller {
    driver: Arc<Bno085Driver>,
    period: Duration,
}

#[async_trait]
impl Actor for Bno085Poller {
    type Msg = Bno085PollerMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        let me = ctx.self_ref().clone();
        let period = self.period;
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(period);
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            interval.tick().await; // skip the immediate tick.
            loop {
                interval.tick().await;
                if me.is_terminated() {
                    break;
                }
                me.tell(Bno085PollerMsg::Tick);
            }
        });
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: Bno085PollerMsg) {
        match msg {
            Bno085PollerMsg::Tick => {
                if let Err(e) = self.driver.poll_once().await {
                    tracing::warn!(error = %e, "bno085 poll failed");
                }
            }
            Bno085PollerMsg::Health { reply } => {
                let _ = reply.send(Ok(()));
            }
        }
    }
}

// ---------------------------------------------------------------------
// Per-axis sensor wrappers.
//
// Each wrapper points at the shared driver and exposes one scalar
// component of the snapshot through the [`Sensor`] contract trait so
// these can be plugged into atomr-physical-sensing actors.
// ---------------------------------------------------------------------

macro_rules! per_axis_sensor {
    ($name:ident, $cap:literal, $unit:expr, |$s:ident| $extract:expr) => {
        #[doc = concat!("Per-axis BNO085 sensor exposing ", $cap, ".")]
        pub struct $name {
            driver: Arc<Bno085Driver>,
            descriptor: DeviceDescriptor,
        }

        impl $name {
            #[doc = "Build a wrapper around the shared driver."]
            pub fn new(driver: Arc<Bno085Driver>, id: impl Into<String>) -> Self {
                let descriptor = DeviceDescriptor::new(
                    DeviceId::from(id.into()),
                    DeviceKind::Sensor,
                    concat!("bno085-", $cap),
                )
                .with_capability(Capability::new($cap, $unit));
                Self { driver, descriptor }
            }
        }

        #[async_trait]
        impl Device for $name {
            fn descriptor(&self) -> &DeviceDescriptor {
                &self.descriptor
            }

            async fn health_check(&self) -> Result<()> {
                self.driver.health_check().await
            }
        }

        #[async_trait]
        impl Sensor for $name {
            async fn read(&self) -> Result<Reading> {
                let $s = self.driver.snapshot();
                let value: f64 = $extract;
                let sensor_id = SensorId::from(self.descriptor.id.as_str());
                Ok(Reading::now(sensor_id, Quantity::new(value, $unit)))
            }
        }
    };
}

per_axis_sensor!(Bno085QuatW, "quat_w", Unit::Scalar, |s| s.orientation[0]);
per_axis_sensor!(Bno085QuatX, "quat_x", Unit::Scalar, |s| s.orientation[1]);
per_axis_sensor!(Bno085QuatY, "quat_y", Unit::Scalar, |s| s.orientation[2]);
per_axis_sensor!(Bno085QuatZ, "quat_z", Unit::Scalar, |s| s.orientation[3]);
per_axis_sensor!(Bno085GyroX, "gyro_x", Unit::RadianPerSecond, |s| s
    .angular_velocity[0]);
per_axis_sensor!(Bno085GyroY, "gyro_y", Unit::RadianPerSecond, |s| s
    .angular_velocity[1]);
per_axis_sensor!(Bno085GyroZ, "gyro_z", Unit::RadianPerSecond, |s| s
    .angular_velocity[2]);
per_axis_sensor!(Bno085AccelX, "accel_x", Unit::MetrePerSecondSquared, |s| s
    .linear_acceleration[0]);
per_axis_sensor!(Bno085AccelY, "accel_y", Unit::MetrePerSecondSquared, |s| s
    .linear_acceleration[1]);
per_axis_sensor!(Bno085AccelZ, "accel_z", Unit::MetrePerSecondSquared, |s| s
    .linear_acceleration[2]);

#[cfg(test)]
mod tests {
    use super::*;

    use crate::loopback::LoopbackI2cBus;

    /// Build a canned SHTP packet on channel 3 carrying a single
    /// rotation-vector report.
    fn rotation_vector_packet(i: i16, j: i16, k: i16, real: i16) -> Vec<u8> {
        // 4-byte SHTP header + 15-byte input report (5 header + 10 body).
        let total_len: u16 = 4 + 15;
        let mut p = Vec::with_capacity(total_len as usize);
        p.push((total_len & 0xFF) as u8);
        p.push(((total_len >> 8) & 0xFF) as u8);
        p.push(3); // channel = Input Sensor Reports
        p.push(0); // seq num
        // Input report header.
        p.push(REPORT_ID_ROTATION_VECTOR);
        p.push(0); // seq
        p.push(0); // status
        p.push(0); // delay lsb
        p.push(0); // delay msb
        // Body: i, j, k, real as Q14.
        p.extend_from_slice(&i.to_le_bytes());
        p.extend_from_slice(&j.to_le_bytes());
        p.extend_from_slice(&k.to_le_bytes());
        p.extend_from_slice(&real.to_le_bytes());
        p
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn poll_decodes_canned_rotation_vector() {
        let bus = LoopbackI2cBus::new().await;
        // We expect two `write_then_read` calls per poll. The loopback
        // bus returns the same canned response each time.
        let canned = rotation_vector_packet(0x4000, 0x0000, 0x0000, 0x4000);
        bus.queue_response(canned.clone());
        bus.queue_response(canned);
        let driver = Bno085Driver::new(bus.as_ref(), DEFAULT_ADDRESS);
        driver.poll_once().await.unwrap();
        let snap = driver.snapshot();
        assert!(snap.valid);
        // 0x4000 / 2^14 = 1.0 exactly.
        let expected_one = 1.0_f64;
        assert!((snap.orientation[0] - expected_one).abs() < 1e-9, "{:?}", snap.orientation);
        assert!((snap.orientation[1] - expected_one).abs() < 1e-9);
        assert!(snap.orientation[2].abs() < 1e-9);
        assert!(snap.orientation[3].abs() < 1e-9);
    }
}
