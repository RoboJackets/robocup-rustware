//!
//! Driver for the MCP23017 io-expander chip.
//! 

#![no_std]

mod register;

pub mod error;
pub use error::IOExpanderError;

pub mod pin;
pub use pin::Pin;

use embedded_hal::blocking::i2c::{Write, Read};
use register::Register;

pub const IO_EXPANDER_ADDRESS: u8 = 0x00;

#[inline(always)]
pub const fn buffer_to_u16(buffer: [u8; 2]) -> u16 {
    u16::from_le_bytes(buffer)
}

#[inline(always)]
pub const fn u16_to_buffer(value: u16) -> [u8; 2] {
    value.to_le_bytes()
}

pub struct IOExpander<I2C> {
    i2c: I2C,
    // Cached Values
    cached_word: u16,
    cached_output_mask: u16,
    cached_pullup_mask: u16,
    cached_polarity_mask: u16,
}

impl<I2C: Write<Error=E> + Read<Error=E>, E> IOExpander<I2C> {
    pub fn digital_word_read(&mut self) -> Result<u16, IOExpanderError<E>> {
        self.cached_word = self.read_register(Register::GPIO)?;
        Ok(self.cached_word)
    }

    pub fn digital_word_write(&mut self, word: u16) -> Result<(), IOExpanderError<E>> {
        self.cached_word = word;
        self.write_register(Register::GPIO, word)
    }

    pub fn set_polarity_mask(&mut self, polarity_mask: u16) -> Result<(), IOExpanderError<E>> {
        self.cached_polarity_mask = polarity_mask;
        self.write_register(Register::IPOL, polarity_mask)
    }

    pub fn set_output_mask(&mut self, output_mask: u16) -> Result<(), IOExpanderError<E>> {
        self.cached_output_mask;
        self.write_register(Register::IODIR, output_mask)
    }

    pub fn set_pullup_mask(&mut self, pullup_mask: u16) -> Result<(), IOExpanderError<E>> {
        self.cached_pullup_mask = pullup_mask;
        self.write_register(Register::GPPU, pullup_mask)
    }

    fn read_register(&mut self, register: Register) -> Result<u16, IOExpanderError<E>> {
        let mut buffer = [0u8; 2];
        self.i2c.write(IO_EXPANDER_ADDRESS, &[register.address()]).map_err(IOExpanderError::I2CError)?;
        self.i2c.read(IO_EXPANDER_ADDRESS, &mut buffer).map_err(IOExpanderError::I2CError)?;
        Ok(buffer_to_u16(buffer))
    }

    fn write_register(&mut self, register: Register, value: u16) -> Result<(), IOExpanderError<E>> {
        let buffer = u16_to_buffer(value);
        self.i2c.write(IO_EXPANDER_ADDRESS, &[register.address(), buffer[0], buffer[1]]).map_err(IOExpanderError::I2CError)
    }
}
