//! SDR configuration value types.
//!
//! These are plain `Serialize` / `Deserialize` structs that mirror the
//! HackRF parameter surface. They live separately from the actor so
//! they can be loaded from a config file or sent over an RPC without
//! pulling the actor runtime in.

use serde::{Deserialize, Serialize};

use crate::error::{SdrError, SdrResult};

/// HackRF One frequency floor — the datasheet quotes ~1 MHz.
pub const MIN_CENTRE_HZ: u64 = 1_000_000;
/// HackRF One frequency ceiling — the datasheet quotes 6 GHz.
pub const MAX_CENTRE_HZ: u64 = 6_000_000_000;
/// Minimum supported sample rate. The MAX5864 ADC can in principle run
/// lower, but the libhackrf documentation recommends >= 2 MS/s.
pub const MIN_SAMPLE_RATE_HZ: u32 = 2_000_000;
/// Maximum supported sample rate (20 MS/s).
pub const MAX_SAMPLE_RATE_HZ: u32 = 20_000_000;
/// LNA gain ceiling, in dB.
pub const MAX_LNA_GAIN_DB: u8 = 40;
/// VGA gain ceiling, in dB.
pub const MAX_VGA_GAIN_DB: u8 = 62;

/// HackRF parameter set. Constructed once, then mutated on tune.
///
/// Every field is in SI units (no implicit MHz / kHz). Out-of-range
/// values are caught by [`SdrParams::validate`] *before* they reach
/// the driver.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SdrParams {
    /// Centre frequency, in Hz. Range: 1 MHz .. 6 GHz.
    pub centre_hz: u64,
    /// Sample rate, in Hz. Range: 2 MS/s .. 20 MS/s.
    pub sample_rate_hz: u32,
    /// Baseband filter bandwidth, in Hz. `None` lets the driver
    /// auto-derive from the sample rate (75 % of effective rate, the
    /// libhackrf default).
    pub baseband_filter_hz: Option<u32>,
    /// LNA gain, in dB. Range: 0 .. 40, step 8.
    pub lna_gain_db: u8,
    /// VGA / baseband gain, in dB. Range: 0 .. 62, step 2.
    pub vga_gain_db: u8,
    /// Enable the external RF amplifier (+14 dB) ahead of the LNA.
    pub amp_enable: bool,
    /// Enable the antenna-port bias-T (DC power to a powered antenna
    /// or external LNA). WARNING — only turn on when the device
    /// downstream of the antenna port actually wants DC.
    pub antenna_port_pwr: bool,
}

impl SdrParams {
    /// A sane default RX configuration: 100 MHz, 4 MS/s, modest gains,
    /// no amp, no bias-T. Good enough to see broadcast FM in
    /// `inspectrum` without front-end damage risk.
    pub fn default_rx() -> Self {
        Self {
            centre_hz: 100_000_000,
            sample_rate_hz: 4_000_000,
            baseband_filter_hz: None,
            lna_gain_db: 16,
            vga_gain_db: 20,
            amp_enable: false,
            antenna_port_pwr: false,
        }
    }

    /// Builder-style centre frequency override.
    pub fn with_centre_hz(mut self, hz: u64) -> Self {
        self.centre_hz = hz;
        self
    }

    /// Builder-style sample rate override.
    pub fn with_sample_rate_hz(mut self, hz: u32) -> Self {
        self.sample_rate_hz = hz;
        self
    }

    /// Builder-style LNA gain override.
    pub fn with_lna_gain_db(mut self, db: u8) -> Self {
        self.lna_gain_db = db;
        self
    }

    /// Builder-style VGA gain override.
    pub fn with_vga_gain_db(mut self, db: u8) -> Self {
        self.vga_gain_db = db;
        self
    }

    /// Builder-style RF amp toggle.
    pub fn with_amp_enable(mut self, on: bool) -> Self {
        self.amp_enable = on;
        self
    }

    /// Builder-style antenna-port bias-T toggle.
    pub fn with_antenna_port_pwr(mut self, on: bool) -> Self {
        self.antenna_port_pwr = on;
        self
    }

    /// Validate every field against the HackRF One's documented
    /// envelope. Returns `Err(SdrError::InvalidParameter)` with a
    /// human-readable reason at the first violation.
    pub fn validate(&self) -> SdrResult<()> {
        if !(MIN_CENTRE_HZ..=MAX_CENTRE_HZ).contains(&self.centre_hz) {
            return Err(SdrError::InvalidParameter(format!(
                "centre_hz {} out of range [{}, {}]",
                self.centre_hz, MIN_CENTRE_HZ, MAX_CENTRE_HZ
            )));
        }
        if !(MIN_SAMPLE_RATE_HZ..=MAX_SAMPLE_RATE_HZ).contains(&self.sample_rate_hz) {
            return Err(SdrError::InvalidParameter(format!(
                "sample_rate_hz {} out of range [{}, {}]",
                self.sample_rate_hz, MIN_SAMPLE_RATE_HZ, MAX_SAMPLE_RATE_HZ
            )));
        }
        if self.lna_gain_db > MAX_LNA_GAIN_DB {
            return Err(SdrError::InvalidParameter(format!(
                "lna_gain_db {} exceeds {}",
                self.lna_gain_db, MAX_LNA_GAIN_DB
            )));
        }
        if self.lna_gain_db % 8 != 0 {
            return Err(SdrError::InvalidParameter(format!(
                "lna_gain_db {} must be a multiple of 8",
                self.lna_gain_db
            )));
        }
        if self.vga_gain_db > MAX_VGA_GAIN_DB {
            return Err(SdrError::InvalidParameter(format!(
                "vga_gain_db {} exceeds {}",
                self.vga_gain_db, MAX_VGA_GAIN_DB
            )));
        }
        if self.vga_gain_db % 2 != 0 {
            return Err(SdrError::InvalidParameter(format!(
                "vga_gain_db {} must be a multiple of 2",
                self.vga_gain_db
            )));
        }
        Ok(())
    }
}

impl Default for SdrParams {
    fn default() -> Self {
        Self::default_rx()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_valid() {
        SdrParams::default_rx().validate().unwrap();
    }

    #[test]
    fn centre_out_of_range_is_rejected() {
        let p = SdrParams::default_rx().with_centre_hz(MAX_CENTRE_HZ + 1);
        assert!(p.validate().is_err());
        let p = SdrParams::default_rx().with_centre_hz(MIN_CENTRE_HZ - 1);
        assert!(p.validate().is_err());
    }

    #[test]
    fn lna_step_enforced() {
        let p = SdrParams::default_rx().with_lna_gain_db(7);
        assert!(p.validate().is_err());
        let p = SdrParams::default_rx().with_lna_gain_db(8);
        assert!(p.validate().is_ok());
    }

    #[test]
    fn vga_step_enforced() {
        let p = SdrParams::default_rx().with_vga_gain_db(3);
        assert!(p.validate().is_err());
        let p = SdrParams::default_rx().with_vga_gain_db(4);
        assert!(p.validate().is_ok());
    }

    #[test]
    fn sample_rate_floor_enforced() {
        let p = SdrParams::default_rx().with_sample_rate_hz(MIN_SAMPLE_RATE_HZ - 1);
        assert!(p.validate().is_err());
    }
}
