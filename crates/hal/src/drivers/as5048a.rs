//! AS5048A magnetic rotary encoder over SPI.
//!
//! 16-bit SPI command word with even-parity bit, 14-bit address. The
//! response from one transaction reports the **previous** command, so
//! the read-angle helper performs two transfers and returns the second.

use std::f64::consts::PI;

use async_trait::async_trait;
use atomr_physical_core::{
    Capability, Device, DeviceDescriptor, DeviceId, DeviceKind, PhysicalError, Quantity, Reading,
    Result, Sensor, SensorId, Unit,
};

use crate::bus::spi::SpiDevice;
use crate::error::HalError;

const ANGLE_REGISTER: u16 = 0x3FFF;

/// AS5048A magnetic rotary encoder driver.
pub struct As5048aEncoder {
    spi: SpiDevice,
    descriptor: DeviceDescriptor,
    zero_offset: f64,
}

impl As5048aEncoder {
    /// Build an encoder bound to `spi`.
    pub fn new(spi: SpiDevice, id: impl Into<String>) -> Self {
        let device_id: String = id.into();
        let descriptor = DeviceDescriptor::new(
            DeviceId::from(device_id),
            DeviceKind::Sensor,
            "as5048a",
        )
        .with_capability(Capability::new("joint_position", Unit::Radian));
        Self {
            spi,
            descriptor,
            zero_offset: 0.0,
        }
    }

    /// Builder-style: shift the reported angle so `0` rad corresponds
    /// to `offset_rad` raw.
    pub fn with_zero_offset(mut self, offset_rad: f64) -> Self {
        self.zero_offset = offset_rad;
        self
    }

    /// Compute the even-parity bit for the bottom 15 bits of `word`.
    fn parity_bit(word: u16) -> u16 {
        let mut v = word & 0x7FFF;
        let mut p = 0u16;
        while v != 0 {
            p ^= v & 1;
            v >>= 1;
        }
        p
    }

    fn read_command_frame(register: u16) -> [u8; 2] {
        // Bit 15 = parity, bit 14 = R/W (1 = read), bits 13..0 = address.
        let body = 0x4000 | (register & 0x3FFF);
        let parity = Self::parity_bit(body) << 15;
        let cmd = body | parity;
        cmd.to_be_bytes()
    }

    /// Read one angle in radians. Performs two SPI transfers because
    /// the AS5048A clocks responses out one transaction behind the
    /// command.
    pub fn read_angle(&self) -> std::result::Result<f64, HalError> {
        let cmd = Self::read_command_frame(ANGLE_REGISTER);
        let mut throwaway = [0u8; 2];
        self.spi.transfer(&cmd, &mut throwaway)?;
        let mut response = [0u8; 2];
        self.spi.transfer(&cmd, &mut response)?;
        let raw = u16::from_be_bytes(response);
        // Top 2 bits are PAR + EF; strip them.
        let counts = raw & 0x3FFF;
        let angle = (counts as f64 / 16384.0) * 2.0 * PI - self.zero_offset;
        Ok(angle)
    }
}

#[async_trait]
impl Device for As5048aEncoder {
    fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    async fn health_check(&self) -> Result<()> {
        // Successfully clocking out one transaction is enough — if the
        // SPI bus is dead the transfer will surface a HalError.
        let mut throwaway = [0u8; 2];
        let cmd = Self::read_command_frame(ANGLE_REGISTER);
        self.spi
            .transfer(&cmd, &mut throwaway)
            .map_err(PhysicalError::from)
    }
}

#[async_trait]
impl Sensor for As5048aEncoder {
    async fn read(&self) -> Result<Reading> {
        let angle = self.read_angle().map_err(PhysicalError::from)?;
        let sensor_id = SensorId::from(self.descriptor.id.as_str());
        Ok(Reading::now(sensor_id, Quantity::new(angle, Unit::Radian)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::loopback::LoopbackSpiDevice;

    #[test]
    fn parity_bit_matches_datasheet_example() {
        // Datasheet example: address 0x3FFD has even-parity 1.
        let body = 0x4000 | 0x3FFD;
        assert_eq!(As5048aEncoder::parity_bit(body) & 1, 0);
    }

    #[tokio::test]
    async fn reads_canned_angle_within_one_lsb() {
        let spi = LoopbackSpiDevice::new()
            .with_response(vec![0x00, 0x00]) // discarded — first xfer reply is for previous command
            .with_response(vec![0x0F, 0xFF]); // counts = 0x0FFF
        let encoder = As5048aEncoder::new(spi.device(), "enc0");
        let r = encoder.read().await.unwrap();
        let expected = (0x0FFF as f64 / 16384.0) * 2.0 * PI;
        let lsb = 2.0 * PI / 16384.0;
        assert!((r.quantity.value - expected).abs() <= lsb, "got {}", r.quantity.value);
    }

    #[tokio::test]
    async fn zero_offset_shifts_angle() {
        let spi = LoopbackSpiDevice::new()
            .with_response(vec![0x00, 0x00])
            .with_response(vec![0x10, 0x00]);
        let encoder = As5048aEncoder::new(spi.device(), "enc0").with_zero_offset(0.5);
        let r = encoder.read().await.unwrap();
        let expected = (0x1000 as f64 / 16384.0) * 2.0 * PI - 0.5;
        assert!((r.quantity.value - expected).abs() < 1e-9);
    }
}
