//! ODrive (CAN-protocol) axis driver.
//!
//! Implements the [`atomr_physical_core::Actuator`] contract over the
//! shared CAN bus actor. The ODrive's 11-bit ID layout is
//! `(node_id << 5) | command_id`; this driver maps
//! [`ControlMode::Position`] / [`Velocity`](ControlMode::Velocity) /
//! [`Effort`](ControlMode::Effort) onto the firmware's
//! `Set_Input_Pos` / `Set_Input_Vel` / `Set_Input_Torque` commands.
//!
//! `health_check` waits up to 200 ms for a `Heartbeat` frame on the
//! shared broadcast.

use std::time::Duration;

use async_trait::async_trait;
use atomr_physical_core::{
    Actuator, Capability, Command, CommandAck, ControlMode, Device, DeviceDescriptor, DeviceId,
    DeviceKind, PhysicalError, Result, Unit,
};
use socketcan::{CanFrame, EmbeddedFrame, Id, StandardId};

use crate::bus::can::CanBusActorRef;
use crate::error::HalError;

/// ODrive CAN command IDs (firmware ≥ 0.5.4 mainline mapping).
const CMD_HEARTBEAT: u32 = 0x01;
const CMD_SET_INPUT_POS: u32 = 0x0C;
const CMD_SET_INPUT_VEL: u32 = 0x0D;
const CMD_SET_INPUT_TORQUE: u32 = 0x0E;

const HEARTBEAT_TIMEOUT: Duration = Duration::from_millis(200);

/// One ODrive axis (axis 0 on a single-axis board).
pub struct OdriveAxis {
    bus: CanBusActorRef,
    descriptor: DeviceDescriptor,
    node_id: u8,
}

impl OdriveAxis {
    /// Build a driver bound to `bus` at the given `node_id`.
    pub fn new(bus: CanBusActorRef, id: impl Into<String>, node_id: u8) -> Self {
        let device_id: String = id.into();
        let descriptor = DeviceDescriptor::new(
            DeviceId::from(device_id),
            DeviceKind::Actuator,
            "odrive-axis",
        )
        .with_capability(Capability::new("joint_position", Unit::Radian))
        .with_capability(Capability::new("joint_velocity", Unit::RadianPerSecond))
        .with_capability(Capability::new("joint_torque", Unit::NewtonMetre));
        Self {
            bus,
            descriptor,
            node_id,
        }
    }

    /// The configured ODrive node id (0..32).
    pub fn node_id(&self) -> u8 {
        self.node_id
    }

    fn frame_id(&self, cmd: u32) -> u32 {
        ((self.node_id as u32) << 5) | (cmd & 0x1F)
    }

    fn build_frame(&self, cmd: u32, payload: [u8; 8]) -> std::result::Result<CanFrame, HalError> {
        let id_raw = self.frame_id(cmd);
        let id = StandardId::new(id_raw as u16)
            .ok_or_else(|| HalError::Frame(format!("standard id out of range: {id_raw:#x}")))?;
        CanFrame::new(Id::Standard(id), &payload)
            .ok_or_else(|| HalError::Frame("failed to construct CAN frame".into()))
    }

    fn encode_set_input_pos(setpoint_rev: f32) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0..4].copy_from_slice(&setpoint_rev.to_le_bytes());
        // vel_ff and torque_ff are int16_t scaled by 1/1000 — leave at 0.
        buf
    }

    fn encode_set_input_vel(vel_rev_s: f32) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0..4].copy_from_slice(&vel_rev_s.to_le_bytes());
        // torque_ff (f32) — leave at 0.
        buf
    }

    fn encode_set_input_torque(torque_nm: f32) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0..4].copy_from_slice(&torque_nm.to_le_bytes());
        buf
    }
}

#[async_trait]
impl Device for OdriveAxis {
    fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    async fn health_check(&self) -> Result<()> {
        // Match on this node's Heartbeat command id; the (node_id<<5) |
        // 0x01 pattern is unique per axis on the shared bus.
        let mut rx = self
            .bus
            .subscribe_filter(0x7FF, self.frame_id(CMD_HEARTBEAT));
        match tokio::time::timeout(HEARTBEAT_TIMEOUT, rx.recv()).await {
            Ok(Ok(_frame)) => Ok(()),
            Ok(Err(e)) => Err(PhysicalError::from(e)),
            Err(_) => Err(PhysicalError::from(HalError::Timeout("odrive heartbeat".into()))),
        }
    }
}

#[async_trait]
impl Actuator for OdriveAxis {
    async fn apply(&self, command: Command) -> Result<CommandAck> {
        let actuator_id = command.actuator.clone();
        let setpoint = command.setpoint.value as f32;
        let (cmd, payload) = match command.mode {
            ControlMode::Position => {
                // ODrive position is in revolutions; convert if the
                // command came in radians.
                let rev = match command.setpoint.unit {
                    Unit::Radian => setpoint / (2.0 * std::f32::consts::PI),
                    _ => setpoint,
                };
                (CMD_SET_INPUT_POS, Self::encode_set_input_pos(rev))
            }
            ControlMode::Velocity => {
                let rev_s = match command.setpoint.unit {
                    Unit::RadianPerSecond => setpoint / (2.0 * std::f32::consts::PI),
                    _ => setpoint,
                };
                (CMD_SET_INPUT_VEL, Self::encode_set_input_vel(rev_s))
            }
            ControlMode::Effort => (
                CMD_SET_INPUT_TORQUE,
                Self::encode_set_input_torque(setpoint),
            ),
            ControlMode::Duty => {
                return Err(PhysicalError::ActuationRejected {
                    device: self.descriptor.id.to_string(),
                    reason: "ODrive driver does not accept ControlMode::Duty".into(),
                });
            }
            other => {
                return Err(PhysicalError::ActuationRejected {
                    device: self.descriptor.id.to_string(),
                    reason: format!("ODrive driver does not accept {other:?}"),
                });
            }
        };
        let frame = self.build_frame(cmd, payload).map_err(PhysicalError::from)?;
        self.bus
            .send_frame(frame)
            .await
            .map_err(PhysicalError::from)?;
        Ok(CommandAck::accepted(actuator_id))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_physical_core::{ActuatorId, Quantity};
    use socketcan::Frame as _;

    use crate::loopback::LoopbackCanBus;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn effort_command_emits_torque_frame() {
        let bus = LoopbackCanBus::new().await;
        let bus_ref = bus.as_ref();
        let axis = OdriveAxis::new(bus_ref, "odrive-0", 3);
        let cmd = Command::now(
            ActuatorId::from("odrive-0"),
            ControlMode::Effort,
            Quantity::new(1.5, Unit::NewtonMetre),
        );
        axis.apply(cmd).await.unwrap();
        let frame = bus.pop_frame().expect("frame emitted");
        // Node 3 -> raw id 3 << 5 | 0x0E = 0x6E.
        assert_eq!(frame.raw_id(), (3 << 5) | CMD_SET_INPUT_TORQUE);
        let mut tx = [0u8; 4];
        tx.copy_from_slice(&frame.data()[0..4]);
        let value = f32::from_le_bytes(tx);
        assert!((value - 1.5).abs() < 1e-6, "torque payload {value}");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn position_command_converts_radians_to_revolutions() {
        let bus = LoopbackCanBus::new().await;
        let bus_ref = bus.as_ref();
        let axis = OdriveAxis::new(bus_ref, "odrive-0", 0);
        let cmd = Command::now(
            ActuatorId::from("odrive-0"),
            ControlMode::Position,
            Quantity::new(2.0 * std::f64::consts::PI, Unit::Radian),
        );
        axis.apply(cmd).await.unwrap();
        let frame = bus.pop_frame().expect("frame emitted");
        assert_eq!(frame.raw_id(), CMD_SET_INPUT_POS);
        let mut tx = [0u8; 4];
        tx.copy_from_slice(&frame.data()[0..4]);
        let rev = f32::from_le_bytes(tx);
        assert!((rev - 1.0).abs() < 1e-5, "rev payload {rev}");
    }
}
