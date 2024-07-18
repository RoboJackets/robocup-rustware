//!
//! Spi wrapper around the embedded-hal trait to allow for
//! spi modes and frequencies to be modified
//! 

use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::spi::Mode;

use teensy4_bsp::{
    board::{LpspiPins, LPSPI_FREQUENCY},
    hal::lpspi::{LpspiError, Lpspi, Status},
};

use teensy4_pins::t41::*;

use hal_extensions::spi::ConfigurableSpi;

/// The fourth spi bus on the teensy 4.1
pub struct Spi4(Lpspi<LpspiPins<P11, P12, P13, P10>, 4>);


impl Spi4 {
    /// Initialize a new instance of the fourth spi bus from the teensy 4.1 peripherals
    pub fn new(mut spi: Lpspi<LpspiPins<P11, P12, P13, P10>, 4>, mode: Mode, frequency: u32) -> Self {
        spi.disabled(|spi| {
            spi.set_mode(mode);
            spi.set_clock_hz(LPSPI_FREQUENCY, frequency);
        });
        Self(spi)
    }
}

impl Transfer<u8> for Spi4 {
    type Error = LpspiError;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        let mut status = self.0.status();
        while status.contains(Status::BUSY) {
            status = self.0.status();
        }
        self.0.transfer(words)
    }
}

impl Write<u8> for Spi4 {
    type Error = LpspiError;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut status = self.0.status();
        while status.contains(Status::BUSY) {
            status = self.0.status();
        }
        self.0.write(words)
    }
}

impl ConfigurableSpi<LpspiError> for Spi4 {
    fn set_mode(&mut self, mode: Mode) {
        self.0.disabled(|spi| {
            spi.set_mode(mode);
        })
    }

    fn set_frequency(&mut self, frequency: u32) {
        self.0.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, frequency)
        })
    }
}

/// The third spi bus on the Teensy 4.1
pub struct Spi3(Lpspi<LpspiPins<P26, P39, P27, P38>, 3>);

impl Spi3 {
    /// Initialize a new spi bus 3 from the teensy 4.1 peripherals
    pub fn new(mut spi: Lpspi<LpspiPins<P26, P39, P27, P38>, 3>, mode: Mode, frequency: u32) -> Self {
        spi.disabled(|spi| {
            spi.set_mode(mode);
            spi.set_clock_hz(LPSPI_FREQUENCY, frequency);
        });
        Self(spi)
    }
}

impl Transfer<u8> for Spi3 {
    type Error = LpspiError;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        let mut status = self.0.status();
        while status.contains(Status::BUSY) {
            status = self.0.status();
        }
        self.0.transfer(words)
    }
}

impl Write<u8> for Spi3 {
    type Error = LpspiError;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut status = self.0.status();
        while status.contains(Status::BUSY) {
            status = self.0.status();
        }
        self.0.write(words)
    }
}

impl ConfigurableSpi<LpspiError> for Spi3 {
    fn set_mode(&mut self, mode: Mode) {
        self.0.disabled(|spi| {
            spi.set_mode(mode);
        });
    }

    fn set_frequency(&mut self, frequency: u32) {
        self.0.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, frequency);
        })
    }
}