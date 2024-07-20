//!
//! Driver for the Kicker Module for Robojackets Robocup
//! 

use core::{fmt::Debug, marker::PhantomData};

use embedded_hal::{
    digital::v2::OutputPin,
    blocking::delay::DelayUs,
};

use hal_extensions::spi::ConfigurableSpi;

use robojackets_robocup_rtp::control_message::ControlMessage;

/// Voltage threshold for kicker board to be considered charged
const IS_CHARGED_CUTOFF: u8 = 230;
/// The flag to supply to allow charging of the kicker
const CHARG_ALLOWED_FLAG: u8 = 1 << 4;
/// The mask to determine how powerful the kick should be
const KICK_POWER_MASK: u8 = 0x0F;
/// The mask to determine whether breakbeam is tripped (1) or not (0)
const BREAKBEAM_TRIPPED: u8 = 1 << 7;
/// Volatage mask for the kicker voltage
const VOLTAGE_MASK: u8 = 0x7F;
/// The scalar to multiply the returned voltage by
const VOLTAGE_SCALAR: u8 = 2;

/// The type of kick to execute on the kicker board
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KickType {
    // The next performed kick will chip the ball
    Chip = 1 << 7,
    // The next performed kick will kick the ball
    Kick = 0 << 7,
}

/// What triggers the kicker to execute
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KickTrigger {
    // The kicker should activate immediately
    Immediate = 0b10 << 5,
    // The kicker should activate when breakbeam is triggered
    Breakbeam = 0b01 << 5,
    // The kicker should not activate
    Cancel = 0b11 << 5,
}

/// Error from operating the kicker
#[derive(Debug)]
pub enum KickerError<SPI: core::fmt::Debug, GPIOE: core::fmt::Debug> {
    // Error with the spi
    SpiError(SPI),
    // Error with the gpio
    GpioError(GPIOE),
}

/// Driver for the Robojackets Robocup Kicker Board
pub struct Kicker<SPI, CSN, SPIE: core::fmt::Debug, GPIOE: core::fmt::Debug> where 
    SPI: ConfigurableSpi<SPIE>,
    CSN: OutputPin<Error=GPIOE> {
    // Peripherals
    // The chip select connected to the kicker board
    csn: CSN,

    // State
    // True if the breakbeam has been tripped
    ball_sensed: bool,
    // True if the kicker has a non-zero voltage
    is_healthy: bool,
    // Current voltage stored in the kicker (volts)
    current_voltage: u8,
    // The type of kick to perform
    kick_type: KickType,
    // The trigger for the next kick
    kick_trigger: KickTrigger,
    // True if the kicker board can safely charge the capacitors
    charge_allowed: bool,
    // 8-bit encoding of the kick's strength (0-255)
    kick_strength: u8,

    // A placeholder for the SPI used to communicate with the kicker
    phantom: PhantomData<(SPI, SPIE)>,
}

impl<SPI, CSN, SPIE: core::fmt::Debug, GPIOE: core::fmt::Debug> Kicker<SPI, CSN, SPIE, GPIOE> where 
    SPI: ConfigurableSpi<SPIE>,
    CSN: OutputPin<Error=GPIOE> {
    /// Initialize a new kicker driver
    pub fn new(csn: CSN) -> Self {
        Self {
            csn,
            ball_sensed: false,
            is_healthy: true,
            current_voltage: 0,
            kick_type: KickType::Chip,
            kick_trigger: KickTrigger::Cancel,
            charge_allowed: false,
            kick_strength: 0,
            phantom: PhantomData,
        }
    }

    /// Set the kick type for the kicker driver
    pub fn set_kick_type(&mut self, kick_type: KickType) {
        self.kick_type = kick_type;
    }

    /// Set the kicker to kick immediately
    pub fn kick_immediately(&mut self, strength: u8) {
        self.kick_trigger = KickTrigger::Immediate;
        self.kick_strength = strength;
    }

    /// Set the kicker to kick on breakbeam
    pub fn kick_on_breakbeam(&mut self, strength: u8) {
        self.kick_trigger = KickTrigger::Breakbeam;
        self.kick_strength = strength;
    }

    /// Tell the kicker not to kick
    pub fn cancel_kick(&mut self) {
        self.kick_trigger = KickTrigger::Cancel;
    }

    /// Set whether or not the kicker is allowed to charge
    pub fn set_charge_allowed(&mut self, charge_allowed: bool) {
        self.charge_allowed = charge_allowed;
    }

    /// Set the parameters for the kicker from a given control message
    pub fn set_from_control_message(&mut self, control_message: &ControlMessage) {
        self.kick_type = match control_message.shoot_mode {
            true => KickType::Chip,
            false => KickType::Kick,
        };

        self.kick_trigger = match *control_message.trigger_mode {
            0b01 => KickTrigger::Breakbeam,
            0b10 => KickTrigger::Immediate,
            0b00 | 0b11 | _ => KickTrigger::Cancel,
        };

        self.kick_strength = *control_message.kick_strength;
    }

    /// Get whether or not the kicker has ball sense
    pub fn has_ball_sense(&self) -> bool {
        self.ball_sensed
    }

    /// Get whether or no the kicker is healthy
    pub fn is_healthy(&self) -> bool {
        self.is_healthy
    }

    /// Get the current voltage of the kicker
    pub fn get_voltage(&self) -> u8 {
        self.current_voltage
    }

    /// Get whether or not the kicker is currently charged
    pub fn is_charged(&self) -> bool {
        self.current_voltage > IS_CHARGED_CUTOFF
    }

    /// Write the current commands to the kicker.  The packet has the 
    /// following format:
    /// 
    /// |---------------------------------------|
    /// | (7) | (6) (5) | (4) | (3) (2) (1) (0) |
    /// |---------------------------------------| 
    /// 
    /// Bits 0-3
    ///     Power of kick
    ///         0 - 15
    ///         0 is min power
    ///         15 is max power
    /// 
    /// Bit 4
    ///     Charge Allowed
    ///         Whether the kicker can start charging the capacitors
    ///         1 Charge allowed
    ///         0 Charge not allowed
    /// 
    /// Bits 5-6
    ///     Type of kick activation
    ///         0b01 Kick on breakbeam
    ///         0b10 Kick immediately
    ///         0b11 Cancel all current kick commands
    /// 
    /// Bit 7
    ///     Type of kick
    ///         1 Chip
    ///         0 Kick
    pub fn service(&mut self, spi: &mut SPI, delay: &mut impl DelayUs<u32>) -> Result<(), KickerError<SPIE, GPIOE>> {
        let mut command = [0x00];

        // Set the kick type bits in the command
        command[0] |= self.kick_type as u8;

        // Set the kick trigger bits in the command
        command[0] |= self.kick_trigger as u8;

        // Set the charge allowed bit in the command
        if self.charge_allowed {
            command[0] |= CHARG_ALLOWED_FLAG
        }

        // Set the kick strength bits in the command
        let kick_strength = ((self.kick_strength as f32) / 255.0 * 15.0) as u8;
        command[0] |= kick_strength & KICK_POWER_MASK;

        spi.set_frequency(100_000);
        self.csn.set_low().map_err(KickerError::GpioError)?;
        // Delay at least 10us
        delay.delay_us(50);
        spi.transfer(&mut command).map_err(KickerError::SpiError)?;
        // Delay at least 10us
        delay.delay_us(50);
        self.csn.set_high().map_err(KickerError::GpioError)?;

        // Decode current_voltage and ball_sense from the response from kicker
        self.current_voltage = (command[0] & VOLTAGE_MASK) * VOLTAGE_SCALAR;
        self.ball_sensed = (command[0] & BREAKBEAM_TRIPPED) != 0;

        // Assume the kicker is healthy if we get some voltage back
        self.is_healthy = self.current_voltage > 0;

        Ok(())
    }
}
