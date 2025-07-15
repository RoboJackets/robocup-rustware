//!
//! Driver for interfacing with the kicker module
//!

#![no_std]
#![deny(missing_docs)]

use core::fmt::Debug;

use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};

use robojackets_robocup_rtp::control_message::{ControlMessage, ShootMode, TriggerMode};

/// Voltage threshold for the kicker board to be considered charged.
pub const CHARGE_CUTOFF: u8 = 230;

/// The mask for setting the kicker power
const KICK_POWER_MASK: u8 = 0x0F;
/// The mask for determining the voltage of the kicker
const VOLTAGE_MASK: u8 = 0x7F;
/// The scale factor for the voltage received from the kicker
const VOLTAGE_SCALE: u8 = 2;

/// The type of kick to perform
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum KickType {
    /// Kick the ball
    Kick = 0,
    /// Chip the ball
    Chip = 1 << 7,
}

impl From<ShootMode> for KickType {
    fn from(value: ShootMode) -> Self {
        match value {
            ShootMode::Kick => Self::Kick,
            ShootMode::Chip => Self::Chip,
        }
    }
}

/// The trigger for the kicker to perform a kick
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum KickTrigger {
    /// The kicker should not be kicking
    Disabled = 0b11 << 5,
    /// The kicker should kick when breakbeam is triggered
    Breakbeam = 1 << 5,
    /// The kicker should kick immediately
    Immediate = 1 << 6,
}

impl From<TriggerMode> for KickTrigger {
    fn from(value: TriggerMode) -> Self {
        match value {
            TriggerMode::Immediate => Self::Immediate,
            TriggerMode::OnBreakBeam => Self::Breakbeam,
            TriggerMode::StandDown => Self::Disabled,
        }
    }
}

/// The command to send to the kicker
///
/// The kicker packet definition is as follows:
/// |---------------------------------------|
/// | (7) | (6) (5) | (4) | (3) (2) (1) (0) |
/// |---------------------------------------|
///
/// Bits 0-3
///  Power of kick
///      0 - 15
///      0 is min power
///      15 is max power
///
/// Bits 4
///  Charge Allowed
///      Whether the kicker can start charging the caps
///      1 Charge allowed
///      0 Charge not allowed
///
/// Bits 5-6
///  Type of kick activation
///      0b01 Kick on breakbeam
///      0b10 Kick immediately
///      0b11 Cancel all current kick commands
///
/// Bits 7
///  Type of kick
///      1 Chip
///      0 Kick
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KickerCommand {
    /// What type of kick should be performed
    pub kick_type: KickType,
    /// What should trigger the kick
    pub kick_trigger: KickTrigger,
    /// The desired strength of a kick
    pub kick_strength: f32,
    /// Should the kicker be charging the capacitors
    pub charge_allowed: bool,
}

impl Default for KickerCommand {
    fn default() -> Self {
        Self {
            kick_type: KickType::Kick,
            kick_trigger: KickTrigger::Disabled,
            kick_strength: 0.0,
            charge_allowed: false,
        }
    }
}

impl From<KickerCommand> for u8 {
    fn from(value: KickerCommand) -> Self {
        let mut command = 0x00;
        command |= value.kick_type as u8;
        command |= value.kick_trigger as u8;
        command |= (value.kick_strength / 255.0 * 15.0) as u8 & KICK_POWER_MASK;
        if value.charge_allowed {
            command |= 1 << 4;
        }
        command
    }
}

impl From<ControlMessage> for KickerCommand {
    fn from(value: ControlMessage) -> Self {
        Self {
            kick_type: value.shoot_mode.into(),
            kick_trigger: value.trigger_mode.into(),
            kick_strength: value.kick_strength as f32 * 255.0 / 15.0,
            charge_allowed: value.trigger_mode != TriggerMode::StandDown,
        }
    }
}

/// The state returned from interacting with the kicker
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KickerState {
    /// the current voltage of the kicker
    pub current_voltage: u8,
    /// did the kicker detect a ball
    pub ball_sensed: bool,
    /// Is the kicker healthy
    pub healthy: bool,
}

impl Default for KickerState {
    fn default() -> Self {
        Self {
            current_voltage: 0,
            ball_sensed: false,
            healthy: true,
        }
    }
}

impl From<u8> for KickerState {
    fn from(value: u8) -> Self {
        Self {
            current_voltage: (value & VOLTAGE_MASK) * VOLTAGE_SCALE,
            ball_sensed: value & (1 << 7) != 0,
            healthy: value != 0,
        }
    }
}

/// Error from operating the kicker controller
#[derive(Debug)]
pub enum KickerServiceError<SPIE: Debug, GPIOE: Debug> {
    /// An error occurred with the SPI peripheral
    Spi(SPIE),
    /// An error occurred with the GPIO peripheral
    Gpio(GPIOE),
}

/// Driver to control the kicker board
pub struct Kicker<CS, RESET> {
    /// The current state of the kicker
    pub state: KickerState,
    /// The chip select controlled by the kicker controller
    cs: CS,
    /// The reset pin of the kicker (needs to be held high)
    reset: RESET,
}

impl<CS: OutputPin<Error = GPIOE>, RESET: OutputPin<Error = GPIOE>, GPIOE: Debug>
    Kicker<CS, RESET>
{
    /// Create a new kicker control driver
    pub fn new(cs: CS, mut reset: RESET) -> Self {
        reset.set_high().unwrap();

        Self {
            state: KickerState::default(),
            cs,
            reset,
        }
    }

    /// Free the underlying peripherals of the kicker controller
    pub fn destroy(self) -> (CS, RESET) {
        (self.cs, self.reset)
    }

    /// Service the kicker by sending a command and receiving the kicker
    /// state
    pub fn service<SPIE: Debug>(
        &mut self,
        command: KickerCommand,
        spi: &mut (impl Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>),
    ) -> Result<KickerState, KickerServiceError<SPIE, GPIOE>> {
        let mut buffer = [command.into()];
        self.cs.set_low().map_err(KickerServiceError::Gpio)?;
        spi.transfer(&mut buffer).map_err(KickerServiceError::Spi)?;
        self.cs.set_high().map_err(KickerServiceError::Gpio)?;

        let status = KickerState::from(buffer[0]);
        self.state = status;

        Ok(status)
    }
}
