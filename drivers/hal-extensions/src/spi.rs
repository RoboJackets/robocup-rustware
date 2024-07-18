//!
//! Hal extensions for SPI
//! 

use embedded_hal::{
    spi::Mode,
    blocking::spi::{Transfer, Write},
};

/// A configurable spi peripheral is an spi peripheral that can
/// have its frequency and mode modified
pub trait ConfigurableSpi<E: core::fmt::Debug>: Transfer<u8, Error=E> + Write<u8, Error=E> {
    // Set the mode of the configurable spi
    fn set_mode(&mut self, mode: Mode);

    // Set the frequency of the configurable spi
    fn set_frequency(&mut self, frequency: u32);
}