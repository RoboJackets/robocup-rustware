//!
//! Fuse bit information for the atmega32a chip
//! 

/// The startup time on the atmega32a microcontroller
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum StartupTime {
    /// BOD Enabled (6 Clock Cycles)
    Bod = 0,
    /// Fast rising power (6 Clock Cycles + 4.1ms)
    FastRising = 1,
    /// Slowly rising power (6 Clock Cycles + 65ms)
    SlowRising = 2,
}

impl From<u8> for StartupTime {
    fn from(value: u8) -> Self {
        match (value >> 4) & 0b11 {
            0 => StartupTime::Bod,
            1 => StartupTime::FastRising,
            _ => StartupTime::SlowRising,
        }
    }
}

/// The selected clock source for the atmega32a microcontroller
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ClockSource {
    /// An external resonator
    ExternalResonator,
    /// An external low-frequency resonator
    ExternalLowFrequency,
    /// External RC Oscillator
    ExternalRc,
    /// Calibrated internal RC Oscillator
    InternalRc,
    /// External Clock
    External,
}

impl From<u8> for ClockSource {
    fn from(value: u8) -> Self {
        match value & 0b1111 {
            0b1010..=0b1111 => ClockSource::ExternalResonator,
            0b1001 => ClockSource::ExternalLowFrequency,
            0b0101..=0b1000 => ClockSource::ExternalRc,
            0b0001..=0b0100 => ClockSource::InternalRc, 
            _ => ClockSource::External,
        }
    }
}

/// The lower fuse bits set on the atmega32a microcontroller
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct LowerFuseBits {
    /// Is the brown out trigger level set
    pub brown_out_trigger_level: bool,
    /// Is the brown out detector enabled
    pub brown_out_detector_enable: bool,
    /// What is the level of the startup time
    pub startup_time: StartupTime,
    /// What is the currently selected clock source
    pub clock_source: ClockSource,
}

impl From<u8> for LowerFuseBits {
    fn from(value: u8) -> Self {
        Self {
            brown_out_trigger_level: value & (0b1 << 7) == 0,
            brown_out_detector_enable: value & (0b1 << 7) == 0,
            startup_time: StartupTime::from(value),
            clock_source: ClockSource::from(value),
        }
    }
}

/// The selected boot size for the microcontroller
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum BootSize {
    /// 256 Words
    W256 = 0b11,
    /// 512 Words
    W512 = 0b10,
    /// 1024 Words
    W1024 = 0b01,
    /// 2048 Words
    W2048 = 0b00,
}

impl From<u8> for BootSize {
    fn from(value: u8) -> Self {
        match (value >> 1) & 0b11 {
            0 => BootSize::W2048,
            1 => BootSize::W1024,
            2 => BootSize::W512,
            _ => BootSize::W256,
        }
    }
}

/// The upper fuse bits set on the atmega32a microcontroller
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct UpperFuseBits {
    /// Is ocd enabled
    pub ocd_enabled: bool,
    /// Is jtag programming enabled
    pub jtag_enabled: bool,
    /// Is spi programming enabled
    pub spi_enabled: bool,
    /// The oscillator options
    pub clk_options: bool,
    /// Is the EEPROM preserved between chip erases
    pub eeprom_preserved: bool,
    /// The boot size selector
    pub boot_size: BootSize,
    /// The reset vector selector
    pub reset_vector: bool,
}

impl From<u8> for UpperFuseBits {
    fn from(value: u8) -> Self {
        Self {
            ocd_enabled: value & (0b1 << 7) == 0,
            jtag_enabled: value & (0b1 << 6) == 0,
            spi_enabled: value & (0b1 << 5) == 0,
            clk_options: value & (0b1 << 4) == 0,
            eeprom_preserved: value & (0b1 << 3) == 0,
            boot_size: BootSize::from(value),
            reset_vector: value & 0b1 == 0,
        }
    }
}