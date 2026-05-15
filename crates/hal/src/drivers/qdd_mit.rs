//! MIT mini-cheetah / QDD CAN protocol driver.
//!
//! Drives "T-Motor"-style quasi-direct-drive joints (AK10-9, AK60-6,
//! AK80-9, …) over the MIT mini-cheetah firmware's 8-byte packed CAN
//! frame format:
//!
//! ```text
//!   position : 16 bits  ([p_min, p_max] radians)
//!   velocity : 12 bits  ([v_min, v_max] rad/s)
//!   kp       : 12 bits  ([kp_min, kp_max])
//!   kd       : 12 bits  ([kd_min, kd_max])
//!   torque   : 12 bits  ([t_min, t_max] N·m)
//! ```
//!
//! Magic control frames `enter_motor_mode` / `exit_motor_mode` /
//! `zero_position` are exposed as inherent methods.

use async_trait::async_trait;
use atomr_physical_core::{
    Actuator, Capability, Command, CommandAck, ControlMode, Device, DeviceDescriptor, DeviceId,
    DeviceKind, PhysicalError, Quantity, Reading, Result, Sensor, SensorId, Unit,
};
use socketcan::{CanFrame, EmbeddedFrame, Id, StandardId};

use crate::bus::can::CanBusActorRef;
use crate::error::HalError;

/// Encoded range for one MIT-protocol joint. Defaults match the
/// AK10-9 / AK60-6 family.
#[derive(Debug, Clone, Copy)]
pub struct MitParams {
    /// Lowest commanded position, radians.
    pub p_min: f64,
    /// Highest commanded position, radians.
    pub p_max: f64,
    /// Lowest commanded velocity, rad/s.
    pub v_min: f64,
    /// Highest commanded velocity, rad/s.
    pub v_max: f64,
    /// Lowest stiffness gain.
    pub kp_min: f64,
    /// Highest stiffness gain.
    pub kp_max: f64,
    /// Lowest damping gain.
    pub kd_min: f64,
    /// Highest damping gain.
    pub kd_max: f64,
    /// Lowest feed-forward torque, N·m.
    pub t_min: f64,
    /// Highest feed-forward torque, N·m.
    pub t_max: f64,
}

impl Default for MitParams {
    fn default() -> Self {
        MitParams {
            p_min: -12.5,
            p_max: 12.5,
            v_min: -45.0,
            v_max: 45.0,
            kp_min: 0.0,
            kp_max: 500.0,
            kd_min: 0.0,
            kd_max: 5.0,
            t_min: -18.0,
            t_max: 18.0,
        }
    }
}

/// Quantise `x` from a real range to an unsigned integer of `bits` width.
fn float_to_uint(x: f64, x_min: f64, x_max: f64, bits: u32) -> u32 {
    let span = (x_max - x_min).max(f64::EPSILON);
    let clamped = x.clamp(x_min, x_max);
    let max = ((1u64 << bits) - 1) as f64;
    (((clamped - x_min) * max / span).round() as i64).clamp(0, max as i64) as u32
}

/// Dequantise an unsigned integer from `bits` width back to its real range.
fn uint_to_float(x: u32, x_min: f64, x_max: f64, bits: u32) -> f64 {
    let span = x_max - x_min;
    let max = ((1u64 << bits) - 1) as f64;
    (x as f64) * span / max + x_min
}

fn pack_command(p: f64, v: f64, kp: f64, kd: f64, t: f64, params: &MitParams) -> [u8; 8] {
    let p_int = float_to_uint(p, params.p_min, params.p_max, 16);
    let v_int = float_to_uint(v, params.v_min, params.v_max, 12);
    let kp_int = float_to_uint(kp, params.kp_min, params.kp_max, 12);
    let kd_int = float_to_uint(kd, params.kd_min, params.kd_max, 12);
    let t_int = float_to_uint(t, params.t_min, params.t_max, 12);

    let mut buf = [0u8; 8];
    buf[0] = ((p_int >> 8) & 0xFF) as u8;
    buf[1] = (p_int & 0xFF) as u8;
    buf[2] = ((v_int >> 4) & 0xFF) as u8;
    buf[3] = (((v_int & 0xF) << 4) as u32 | ((kp_int >> 8) & 0xF)) as u8;
    buf[4] = (kp_int & 0xFF) as u8;
    buf[5] = ((kd_int >> 4) & 0xFF) as u8;
    buf[6] = (((kd_int & 0xF) << 4) as u32 | ((t_int >> 8) & 0xF)) as u8;
    buf[7] = (t_int & 0xFF) as u8;
    buf
}

/// Decode an 8-byte MIT-protocol reply into `(motor_id, position,
/// velocity, current)`. Reply frames echo the motor id in byte 0 and
/// pack `(position, velocity, current)` as 16+12+12 bits in bytes 1..7.
pub fn decode_reply(data: &[u8], params: &MitParams) -> Option<(u8, f64, f64, f64)> {
    if data.len() < 6 {
        return None;
    }
    let motor_id = data[0];
    let p_int = ((data[1] as u32) << 8) | (data[2] as u32);
    let v_int = ((data[3] as u32) << 4) | ((data[4] as u32) >> 4);
    let i_int = (((data[4] as u32) & 0xF) << 8) | (data[5] as u32);
    let position = uint_to_float(p_int, params.p_min, params.p_max, 16);
    let velocity = uint_to_float(v_int, params.v_min, params.v_max, 12);
    let current = uint_to_float(i_int, -40.0, 40.0, 12);
    Some((motor_id, position, velocity, current))
}

/// A MIT-protocol joint controller.
pub struct QddMitJoint {
    bus: CanBusActorRef,
    descriptor: DeviceDescriptor,
    motor_id: u8,
    params: MitParams,
}

impl QddMitJoint {
    /// Build a driver bound to `bus` at `motor_id`.
    pub fn new(bus: CanBusActorRef, id: impl Into<String>, motor_id: u8, params: MitParams) -> Self {
        let device_id: String = id.into();
        let descriptor = DeviceDescriptor::new(
            DeviceId::from(device_id),
            DeviceKind::Actuator,
            "qdd-mit",
        )
        .with_capability(Capability::new("joint_position", Unit::Radian))
        .with_capability(Capability::new("joint_velocity", Unit::RadianPerSecond))
        .with_capability(Capability::new("joint_torque", Unit::NewtonMetre));
        Self {
            bus,
            descriptor,
            motor_id,
            params,
        }
    }

    /// The motor id (CAN id) this joint listens on.
    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }

    fn frame(&self, payload: [u8; 8]) -> std::result::Result<CanFrame, HalError> {
        let id = StandardId::new(self.motor_id as u16)
            .ok_or_else(|| HalError::Frame(format!("motor id out of range: {}", self.motor_id)))?;
        CanFrame::new(Id::Standard(id), &payload)
            .ok_or_else(|| HalError::Frame("failed to construct CAN frame".into()))
    }

    async fn send(&self, payload: [u8; 8]) -> Result<()> {
        let frame = self.frame(payload).map_err(PhysicalError::from)?;
        self.bus.send_frame(frame).await.map_err(PhysicalError::from)
    }

    /// Send the MIT `enter motor mode` magic frame
    /// (`FF FF FF FF FF FF FF FC`).
    pub async fn enter_motor_mode(&self) -> Result<()> {
        self.send([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]).await
    }

    /// Send the MIT `exit motor mode` magic frame
    /// (`FF FF FF FF FF FF FF FD`).
    pub async fn exit_motor_mode(&self) -> Result<()> {
        self.send([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]).await
    }

    /// Send the MIT `zero position` magic frame
    /// (`FF FF FF FF FF FF FF FE`).
    pub async fn zero_position(&self) -> Result<()> {
        self.send([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]).await
    }
}

#[async_trait]
impl Device for QddMitJoint {
    fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    async fn health_check(&self) -> Result<()> {
        // A bus subscriber is implicitly created here; we have no
        // heartbeat to wait for, so just confirm the bus is reachable.
        self.bus.health_check().await.map_err(PhysicalError::from)
    }
}

#[async_trait]
impl Actuator for QddMitJoint {
    async fn apply(&self, command: Command) -> Result<CommandAck> {
        let actuator_id = command.actuator.clone();
        let v = command.setpoint.value;
        let payload = match command.mode {
            ControlMode::Position => pack_command(v, 0.0, 50.0, 1.0, 0.0, &self.params),
            ControlMode::Velocity => pack_command(0.0, v, 0.0, 1.0, 0.0, &self.params),
            ControlMode::Effort => pack_command(0.0, 0.0, 0.0, 0.0, v, &self.params),
            ControlMode::Duty => {
                return Err(PhysicalError::ActuationRejected {
                    device: self.descriptor.id.to_string(),
                    reason: "QddMitJoint does not accept ControlMode::Duty".into(),
                });
            }
            other => {
                return Err(PhysicalError::ActuationRejected {
                    device: self.descriptor.id.to_string(),
                    reason: format!("QddMitJoint does not accept {other:?}"),
                });
            }
        };
        self.send(payload).await?;
        Ok(CommandAck::accepted(actuator_id))
    }
}

/// Per-joint reply sensor — subscribes to the bus, decodes reply frames
/// whose first byte echoes our motor id, and exposes the latest position.
pub struct QddMitFeedbackSensor {
    bus: CanBusActorRef,
    descriptor: DeviceDescriptor,
    motor_id: u8,
    params: MitParams,
}

impl QddMitFeedbackSensor {
    /// Build a feedback sensor bound to `bus` for `motor_id`.
    pub fn new(bus: CanBusActorRef, id: impl Into<String>, motor_id: u8, params: MitParams) -> Self {
        let device_id: String = id.into();
        let descriptor = DeviceDescriptor::new(
            DeviceId::from(device_id),
            DeviceKind::Sensor,
            "qdd-mit-feedback",
        )
        .with_capability(Capability::new("joint_position", Unit::Radian));
        Self {
            bus,
            descriptor,
            motor_id,
            params,
        }
    }
}

#[async_trait]
impl Device for QddMitFeedbackSensor {
    fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    async fn health_check(&self) -> Result<()> {
        self.bus.health_check().await.map_err(PhysicalError::from)
    }
}

#[async_trait]
impl Sensor for QddMitFeedbackSensor {
    async fn read(&self) -> Result<Reading> {
        let mut rx = self.bus.subscribe();
        loop {
            let frame = rx
                .recv()
                .await
                .map_err(|e| PhysicalError::Fault(format!("qdd feedback recv: {e}")))?;
            let data = frame.data();
            if let Some((id, position, _v, _i)) = decode_reply(data, &self.params) {
                if id == self.motor_id {
                    let sensor_id = SensorId::from(self.descriptor.id.as_str());
                    return Ok(Reading::now(sensor_id, Quantity::new(position, Unit::Radian)));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_physical_core::{ActuatorId, Quantity};
    use socketcan::Frame as _;

    use crate::loopback::LoopbackCanBus;

    #[test]
    fn float_to_uint_round_trip_within_one_lsb() {
        let params = MitParams::default();
        let values = [-12.5, -1.0, 0.0, 0.5, 12.5];
        for v in values {
            let q = float_to_uint(v, params.p_min, params.p_max, 16);
            let back = uint_to_float(q, params.p_min, params.p_max, 16);
            let lsb = (params.p_max - params.p_min) / ((1u64 << 16) - 1) as f64;
            assert!(
                (back - v).abs() <= lsb,
                "v={v} back={back} lsb={lsb}"
            );
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn position_command_packs_one_radian_within_one_lsb() {
        let bus = LoopbackCanBus::new().await;
        let bus_ref = bus.as_ref();
        let params = MitParams::default();
        let joint = QddMitJoint::new(bus_ref, "j1", 1, params);
        let cmd = Command::now(
            ActuatorId::from("j1"),
            ControlMode::Position,
            Quantity::new(1.0, Unit::Radian),
        );
        joint.apply(cmd).await.unwrap();
        let frame = bus.pop_frame().expect("frame emitted");
        assert_eq!(frame.raw_id(), 1);
        let p_int = ((frame.data()[0] as u32) << 8) | (frame.data()[1] as u32);
        let decoded = uint_to_float(p_int, params.p_min, params.p_max, 16);
        let lsb = (params.p_max - params.p_min) / ((1u64 << 16) - 1) as f64;
        assert!((decoded - 1.0).abs() <= lsb, "decoded={decoded}");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn enter_motor_mode_sends_magic_frame() {
        let bus = LoopbackCanBus::new().await;
        let bus_ref = bus.as_ref();
        let joint = QddMitJoint::new(bus_ref, "j1", 5, MitParams::default());
        joint.enter_motor_mode().await.unwrap();
        let frame = bus.pop_frame().expect("frame emitted");
        assert_eq!(frame.raw_id(), 5);
        assert_eq!(
            frame.data(),
            &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC][..]
        );
    }
}
