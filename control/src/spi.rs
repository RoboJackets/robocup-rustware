//!
//! Fake SPI Designed to send data at significantly lower
//! speeds than the hardware SPI.  Please only use this for
//! the kicker because the onboard spi should work so much
//! better.
//!
//! The entire reason I (Nate) created this Fake SPI is that the
//! minimum SPI frequency is about 500 KHz.  This is, unfortunately
//! too fast to program the current Atmega32a chip on the kicker so
//! I'm going to attempt to use digital outputs to simulate an SPI
//! peripheral.
//!

use core::convert::Infallible;

use teensy4_pins::t41::*;

use teensy4_bsp::board::PERCLK_FREQUENCY;
use teensy4_bsp::hal::{
    gpio::{Input, Output},
    pit::Pit3,
    timer::Blocking,
};

use embedded_hal::blocking::spi::{Transfer, Write};

/// A Fake SPI implemented using a number of digital output pins.
///
/// Please do not use this for anything but the kicker.  It has no
/// hardware acceleration so it is super slow.
///
/// Note: The frequency for this fake SPI is 100 KHz.
pub struct FakeSpi {
    pub clk: Output<P2>,
    pub mosi: Output<P3>,
    pub miso: Input<P4>,
    pub delay: Blocking<Pit3, PERCLK_FREQUENCY>,
}

impl FakeSpi {
    /// Create a new Fake SPI driver
    pub fn new(
        clk: Output<P2>,
        mosi: Output<P3>,
        miso: Input<P4>,
        delay: Blocking<Pit3, PERCLK_FREQUENCY>,
    ) -> Self {
        clk.clear();
        mosi.clear();

        Self {
            clk,
            mosi,
            miso,
            delay,
        }
    }

    /// Free the peripherals for the fake SPI driver
    pub fn free(
        self,
    ) -> (
        Output<P2>,
        Output<P3>,
        Input<P4>,
        Blocking<Pit3, PERCLK_FREQUENCY>,
    ) {
        (self.clk, self.mosi, self.miso, self.delay)
    }
}

impl Write<u8> for FakeSpi {
    type Error = Infallible;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        // The Fake SPI is always in SPI Mode 0, which means that data
        // is transmitted on falling edge and sampled on rising edge
        self.clk.clear();
        for word in words {
            for bit in (0..8).rev() {
                if *word & (0b1 << bit) != 0 {
                    self.mosi.set();
                } else {
                    self.mosi.clear();
                }
                self.delay.block_us(1);
                self.clk.set();
                self.delay.block_us(1);
                self.clk.clear();
            }
        }
        Ok(())
    }
}

impl Transfer<u8> for FakeSpi {
    type Error = Infallible;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        // The Fake SPI is always in SPI Mode 0, which means that data
        // is transmitted on falling edge and sampled on rising edge
        self.clk.clear();
        for word in words.iter_mut() {
            for bit in (0..8).rev() {
                if *word & (0b1 << bit) != 0 {
                    self.mosi.set();
                } else {
                    self.mosi.clear();
                }
                self.delay.block_us(1);
                self.clk.set();
                if self.miso.is_set() {
                    *word |= 0b1 << bit;
                } else {
                    *word &= !(0b1 << bit);
                }
                self.delay.block_us(1);
                self.clk.clear();
            }
        }
        self.mosi.clear();
        Ok(words)
    }
}
