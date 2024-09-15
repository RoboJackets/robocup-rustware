//!
//! Driver for in-system programming of the kicker board
//! 

#![no_std]

use core::fmt::Debug;

#[deny(missing_docs)]

use embedded_hal::{
    digital::v2::OutputPin,
    blocking::spi::{Transfer, Write},
    blocking::delay::{DelayMs, DelayUs},
};

/// The expected vendor code for the kicker board chip.
const EXPECTED_VENDOR_CODE: u8 = 0x1E;
/// The expected combined part family and memory size for the kicker board chip.
const EXPECTED_PART_FAILY_AND_MEMORY_SIZE: u8 = 0xF0;
/// The expected part number for the kicker board chip.
const EXPECTED_PART_NUMBER: u8 = 0xFF;
/// The page size of the kicker board chip in words (1 word = 2 bytes)
const ATMEGA_PAGESIZE: usize = 64;
/// The number of pages in the kicker board chip's memory
const ATMEGA_NUM_PAGES: usize = 256;

// Instructions //
/// Write the high portion of a word
const HIGH_BYTE: u8 = 0b0100_1000;
/// Write the low portion of a word
const LOW_BYTE: u8 = 0b0100_0000;
/// Write the flash memory page
const WRITE_PAGE: u8 = 0b0100_1100;

/// Error from Programming the Kicker
#[derive(Debug)]
pub enum KickerProgrammerError<GPIOE: Debug, SPIE: Debug> {
    /// The command to enable Programming failed to return the
    /// expected value
    FailedToEnableProgramming, 
    /// The command to erase the chip failed to return the expected
    /// value
    FailedToEraseChip,
    /// The kicker board chip identified incorrectly (there was likely a
    /// problem with enabling programming or erasing the chip)
    InvalidIdentity,
    /// Error with the GPIO Peripheral
    Gpio(GPIOE),
    /// Error with the SPI Peripheral
    Spi(SPIE),
}

/// Kicker Programmer Driver
pub struct KickerProgrammer<CS, RESET> {
    /// The chip select connected to the driver
    cs: CS,
    /// The reset pin connected to the driver
    reset: RESET,
}

impl<CS, RESET, GPIOE> KickerProgrammer<CS, RESET> where 
    CS: OutputPin<Error=GPIOE>,
    RESET: OutputPin<Error=GPIOE>,
    GPIOE: Debug,
{
    /// Instantiate a new Kicker Driver
    pub fn new(cs: CS, reset: RESET) -> Self {
        Self {
            cs,
            reset,
        }
    }

    /// Recover the Peripherals from the Kicker Programmer Driver
    pub fn destroy(self) -> (CS, RESET) {
        (self.cs, self.reset)
    }

    /// Program the kicker
    pub fn program<SPIE: Debug>(
        &mut self,
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        // TODO: if this isn't working it may be necessary to set reset high
        // wait 100ms, set reset low again, and then enable programming.
        
        // Begin programming by pulling reset low and waiting at least
        // 20ms before issuing the first command
        self.reset.set_low().map_err(KickerProgrammerError::Gpio)?;
        delay.delay_ms(25);

        // Enable programming on the kicker chip
        self.enable_programming(spi, delay)?;

        // Erase the current contents of the kicker chip
        self.erase_chip(spi, delay)?;

        // Check the vendor code, part family, flash size, and part
        // number of the kicker chip
        self.check_device(spi, delay)?;

        // TODO: Load the file buffer
        // TODO: Program by pages from the file buffer

        // TODO: Check that all bytes were programmed correctly

        // TODO: Determine if this delay is necessary
        delay.delay_ms(100);
        self.reset.set_high().map_err(KickerProgrammerError::Gpio)?;

        Ok(())
    }

    /// Enable programming of the Kicker
    fn enable_programming<SPIE: Debug>(
        &mut self,
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        let mut buffer = [0xAC, 0x53, 0x00, 0x00];
        self.write(&mut buffer, spi, delay)?;

        if buffer[1] != 0xAC || buffer[2] != 0x53 {
            Err(KickerProgrammerError::FailedToEnableProgramming)
        } else {
            Ok(())
        }
    }

    /// Erase the contents of the kicker
    fn erase_chip<SPIE: Debug>(
        &mut self,
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        let mut buffer = [0xAC, 0x80, 0x00, 0x00];
        self.write(&mut buffer, spi, delay)?;

        if buffer[1] != 0xAC || buffer[2] != 0x80 {
            Err(KickerProgrammerError::FailedToEraseChip)
        } else {
            Ok(())
        }
    }

    /// Check the Vendor Code, Part Family, Flash Size, and Part Number
    /// of the kicker chip.
    fn check_device<SPIE: Debug>(
        &mut self,
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        let mut buffer = [0x30, 0x00, 0x00, 0x00];
        self.write(&mut buffer, spi, delay)?;

        if buffer[1] != 0x30 || buffer[2] != 0x00 || buffer[3] != EXPECTED_VENDOR_CODE {
            return Err(KickerProgrammerError::InvalidIdentity);
        }

        buffer = [0x30, 0x00, 0x01, 0x00];
        self.write(&mut buffer, spi, delay)?;

        if buffer[0] != 0x00 || buffer[1] != 0x30 || buffer[2] != 0x00 || buffer[3] != EXPECTED_PART_FAILY_AND_MEMORY_SIZE {
            return Err(KickerProgrammerError::InvalidIdentity);
        }

        buffer = [0x30, 0x00, 0x02, 0x00];
        self.write(&mut buffer, spi, delay)?;

        if buffer[0] != 000 || buffer[1] != 0x30 || buffer[2] != 0x00 || buffer[3] != EXPECTED_PART_NUMBER {
            return Err(KickerProgrammerError::InvalidIdentity);
        }

        Ok(())
    }

    /// Load a page into memory in order low, then high
    fn load_memory_page<SPIE: Debug>(
        &mut self,
        data: &[u8],
        page_number: usize,
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        // Load the page data to memory
        for address in 0..core::cmp::min(ATMEGA_PAGESIZE, data.len() / 2) {
            let mut low_byte_buffer = [LOW_BYTE, 0x00, (address & 0x3F) as u8, data[2*address]];
            self.write(&mut low_byte_buffer, spi, delay)?;

            let mut high_byte_buffer = [HIGH_BYTE, 0x00, (address & 0x3F) as u8, data[2*address+1]];
            self.write(&mut high_byte_buffer, spi, delay)?;
        }

        // Write the page into memory
        let mut buffer = [WRITE_PAGE, (page_number >> 2) as u8, (page_number << 6) as u8, 0x00];
        self.write(&mut buffer, spi, delay)?;

        // The maximum programming time for the atmega32a is 4.5ms
        delay.delay_ms(5);

        Ok(())
    }

    /// Write data over the SPI
    fn write<SPIE: Debug>(
        &mut self,
        buffer: &mut [u8],
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        delay.delay_us(10);
        self.cs.set_low().map_err(KickerProgrammerError::Gpio)?;
        spi.transfer(buffer).map_err(KickerProgrammerError::Spi)?;
        delay.delay_us(1);
        self.cs.set_high().map_err(KickerProgrammerError::Gpio)?;
        delay.delay_us(10);
        Ok(())
    }
}