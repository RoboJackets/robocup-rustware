//!
//! Driver for operating the Kicker Module
//!
//! Note: This is not the same as the kicker programmer module and should (realistically)
//! only be used during the actual operations of the kicker module.  A separate project
//! and example will be used as the kicker programmer
//!

#![no_std]
#![crate_type = "lib"]

use core::marker::PhantomData;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::DelayUs;

pub mod command;
pub use command::KickType;
use command::{KickCommand, KickActivation};

/// Voltage threshold for the kicker board to be considered charged
/// voltage is mapped [0, 255]
pub const IS_CHARGED_CUTOFF: u8 = 230;
/// Whether the Breakbeam is tripped (1) or not (0)
pub const BREAKBEAM_TRIPPED: u8 = 1 << 7;
/// Voltage of the Kicker (mask)
pub const VOLTAGE_MASK: u8 = 0x7F;
/// How much to multiply the voltage returned
pub const VOLTAGE_SCALE: u8 = 2;

/// Errors that can occur when using the kicker module
pub enum KickerError<SPIE, GPIOE> {
    GpioError(GPIOE),
    SpiError(SPIE),
    SpiGpioError((SPIE, GPIOE)),
}

/// Driver to interface with the kicker module
pub struct Kicker<CSN, SPI, DELAY, GPIOE, SPIE> where
    CSN: OutputPin<Error=GPIOE>,
    SPI: Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>,
    DELAY: DelayUs<u32>,
{
    ball_sensed: bool,
    is_healthy: bool,
    current_voltage: u8,
    kick_type: KickType,
    kick_activation: KickActivation,
    charge_allowed: bool,
    kick_strength: u8,
    csn: CSN,
    _phantom: PhantomData<(SPI, DELAY)>,
}

impl<CSN, SPI, DELAY, GPIOE, SPIE> Kicker<CSN, SPI, DELAY, GPIOE, SPIE> where
    CSN: OutputPin<Error=GPIOE>,
    SPI: Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>,
    DELAY: DelayUs<u32>,
{
    pub fn new(csn: CSN) -> Self {
        Self {
            ball_sensed: false,
            is_healthy: false,
            current_voltage: 0,
            kick_type: KickType::Kick,
            kick_activation: KickActivation::Cancel,
            charge_allowed: false,
            kick_strength: 0u8,
            csn,
            _phantom: PhantomData,
        }
    }

    pub fn set_kick_type(&mut self, kick_type: KickType) {
        self.kick_type = kick_type;
    }

    pub fn set_kick(&mut self, strength: u8) {
        self.kick_activation = KickActivation::Immediate;
        self.kick_strength = strength;
    }

    pub fn set_kick_on_breakbeam(&mut self, strength: u8) {
        self.kick_activation = KickActivation::Breakbeam;
        self.kick_strength = strength;
    }

    pub fn cancel_breakbeam(&mut self) {
        self.kick_activation = KickActivation::Cancel;
        self.kick_strength = 0;
    }

    pub fn has_ball_sense(&mut self) -> bool {
        self.ball_sensed
    }

    pub fn get_voltage(&mut self) -> u8 {
        self.current_voltage
    }

    pub fn is_charged(&mut self) -> bool {
        self.current_voltage > IS_CHARGED_CUTOFF
    }

    pub fn set_charge_allowed(&mut self, charge_allowed: bool) {
        self.charge_allowed = charge_allowed;
    }

    pub fn service(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerError<SPIE, GPIOE>> {
        let mut command = KickCommand::new()
                        .kick_type(self.kick_type)
                        .activation(self.kick_activation)
                        .charge_allowed(self.charge_allowed)
                        .kick_power(self.kick_strength)
                        .build();

        // TODO: Possibly Set SPI Frequency to 100_000

        self.spi_transfer(&mut command, spi, delay)?;

        self.current_voltage = (command[0] & VOLTAGE_MASK) * VOLTAGE_SCALE;
        self.ball_sensed = (command[0] & BREAKBEAM_TRIPPED) != 0;
        self.is_healthy = self.current_voltage > 0;

        Ok(())
    }

    fn spi_transfer(&mut self, buffer: &mut [u8], spi: &mut SPI, delay: &mut DELAY)
        -> Result<(), KickerError<SPIE, GPIOE>> {
        self.csn.set_low().map_err(KickerError::GpioError)?;
        delay.delay_us(50);
        let spi_err = spi.transfer(buffer);
        delay.delay_us(50);
        let gpio_err = self.csn.set_high();

        match (spi_err, gpio_err) {
            (Err(spie), Err(gpioe)) => Err(KickerError::SpiGpioError((spie, gpioe))),
            (Err(e), _) => Err(KickerError::SpiError(e)), 
            (_, Err(e)) => Err(KickerError::GpioError(e)),
            _ => Ok(())
        }
    }
}