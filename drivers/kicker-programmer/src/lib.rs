//!
//! Driver for in-system programming of the kicker board
//! 

#![no_std]

use core::{cmp::min, fmt::Debug};

#[deny(missing_docs)]

use embedded_hal::{
    digital::v2::OutputPin,
    blocking::spi::{Transfer, Write},
    blocking::delay::{DelayMs, DelayUs},
};


mod kicker_bin;
use kicker_bin::KICKER_BYTES;
mod kicker_bin_kick_on_bb;
use kicker_bin_kick_on_bb::KICKER_BYTES_ON_BB;

/// The expected vendor code for the kicker board chip.
const EXPECTED_VENDOR_CODE: u8 = 0x1E;
/// The expected combined part family and memory size for the kicker board chip.
const EXPECTED_PART_FAILY_AND_MEMORY_SIZE: u8 = 0x95;
/// The expected part number for the kicker board chip.
const EXPECTED_PART_NUMBER: u8 = 0x02;
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
/// Read the high portion of a word
const READ_HIGH_BYTE: u8 = 0x28;
/// Read the low portion of a word
const READ_LOW_BYTE: u8 = 0x20;

/// Error from Programming the Kicker
#[derive(Debug)]
pub enum KickerProgrammerError<GPIOE: Debug, SPIE: Debug> {
    /// The command to enable Programming failed to return the
    /// expected value
    FailedToEnableProgramming{ bytes: [u8; 4] },
    /// The command to erase the chip failed to return the expected
    /// value
    FailedToEraseChip,
    /// The kicker board chip identified incorrectly (there was likely a
    /// problem with enabling programming or erasing the chip)
    InvalidIdentity { register: u8, expected: u8, found: u8 },
    /// The binary that is attempting to be programmed is larger than the
    /// maximum flash size of the kicker chip
    InvalidBinarySize,
    /// The binary differs from the programmed value on page `page_num` offset
    /// `offset`
    BinaryDiffers { page_number: usize, offset: u8, low_byte: bool, },
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

    /// Program the kicker with a kick-on-breakbeam program so the kicker will
    /// activate whenever the breakbeam is interrupted.
    pub fn program_kick_on_breakbeam<SPIE: Debug>(
        &mut self,
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        self.program(&KICKER_BYTES, spi, delay)
    }

    /// Program the kicker with full functionality to control kicking via
    /// spi commands
    pub fn program_kicker<SPIE: Debug>(
        &mut self,
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        self.program(&KICKER_BYTES_ON_BB, spi, delay)
    }

    /// Program the kicker
    fn program<SPIE: Debug>(
        &mut self,
        kicker_program: &[u8],
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        log::info!("Attempting to Enable Programming");
        let mut try_count = 0;
        let mut enabled = self.enable_programming(spi, delay);
        while enabled.is_err() && try_count < 20 {
            self.reset.set_high().map_err(KickerProgrammerError::Gpio)?;
            delay.delay_ms(100);
            self.reset.set_low().map_err(KickerProgrammerError::Gpio)?;

            delay.delay_ms(100);

            enabled = self.enable_programming(spi, delay);
            try_count += 1;
        }

        if enabled.is_err() {
            log::error!("Unable to Enable Programming");
            return enabled;
        }
        log::info!("Programming Enabled");

        // Erase the current contents of the kicker chip
        log::info!("Erasing the Atmega32a Chip");
        self.erase_chip(spi, delay)?;

        // Check the vendor code, part family, flash size, and part
        // number of the kicker chip
        log::info!("Checking the identity of the kicker board chip");
        self.check_device(spi, delay)?;

        // Load the file buffer
        log::info!("Programming the Kicker");
        // let kicker_program = include_bytes!("../bin/kicker.nib");
        // The total number of pages is twice the binary length divided by the number of pages
        // because the total number of pages is in words (which are two bytes in length)
        let mut num_pages = kicker_program.len() * 2 / ATMEGA_NUM_PAGES;
        if num_pages * ATMEGA_NUM_PAGES < kicker_program.len() * 2 {
            num_pages += 1;
        }
        for page in 0..num_pages {
            log::info!("Programming Page {}", page);
            self.load_memory_page(
                &kicker_program[(page * 2 * ATMEGA_PAGESIZE)..min((page+1) * 2 * ATMEGA_PAGESIZE, kicker_program.len())],
                page,
                spi,
                delay,
            )?;
        }

        log::info!("Programming Complete!");

        delay.delay_ms(2_000);

        // Check that all bytes were programmed correctly
        log::info!("Validating the Kicker Program");
        for page in 0..num_pages {
            log::info!("Checking Page {}", page);
            self.check_memory_page(
                &kicker_program[(page * 2 * ATMEGA_PAGESIZE)..min((page + 1) * 2 * ATMEGA_PAGESIZE, kicker_program.len())],
                page,
                spi,
                delay,
            )?;
        }

        log::info!("The Kicker has been programmed");

        delay.delay_ms(100);
        self.exit_programming(spi, delay)?;

        Ok(())
    }

    /// Enable programming of the Kicker
    fn enable_programming<SPIE: Debug>(
        &mut self,
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        self.reset.set_high().map_err(KickerProgrammerError::Gpio)?;
        delay.delay_ms(100u32);
        self.reset.set_low().map_err(KickerProgrammerError::Gpio)?;
        delay.delay_ms(100u32);

        let mut buffer = [0xAC, 0x53, 0x00, 0x00];
        self.write(&mut buffer, spi, delay)?;

        if buffer[2] != 0x53 {
            Err(KickerProgrammerError::FailedToEnableProgramming { bytes: buffer })
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

        delay.delay_ms(120);

        if buffer[2] != 0x80 {
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

        log::info!("Buffer: {:?}", buffer);
        if buffer[3] != EXPECTED_VENDOR_CODE {
            return Err(KickerProgrammerError::InvalidIdentity { register: 0x00, expected: EXPECTED_VENDOR_CODE, found: buffer[3]});
        }

        buffer = [0x30, 0x00, 0x01, 0x00];
        self.write(&mut buffer, spi, delay)?;

        log::info!("Buffer: {:?}", buffer);
        if buffer[3] != EXPECTED_PART_FAILY_AND_MEMORY_SIZE {
            return Err(KickerProgrammerError::InvalidIdentity { register: 0x01, expected: EXPECTED_PART_FAILY_AND_MEMORY_SIZE, found: buffer[3] });
        }

        buffer = [0x30, 0x00, 0x02, 0x00];
        self.write(&mut buffer, spi, delay)?;

        log::info!("Buffer: {:?}", buffer);
        if buffer[3] != EXPECTED_PART_NUMBER {
            return Err(KickerProgrammerError::InvalidIdentity { register: 0x02, expected: EXPECTED_PART_NUMBER, found: buffer[3]});
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
        log::info!("Data Length: {}\n Pagesize: {}", data.len(), ATMEGA_PAGESIZE);
        log::info!("{}", min(ATMEGA_PAGESIZE, data.len() / 2));
        // Load the page data to memory
        for address in 0..min(ATMEGA_PAGESIZE, data.len() / 2) {
            let mut low_byte_buffer = [LOW_BYTE, 0x00, (address & 0x3F) as u8, data[2*address]];
            self.write(&mut low_byte_buffer, spi, delay)?;

            delay.delay_ms(20);

            let mut high_byte_buffer = [HIGH_BYTE, 0x00, (address & 0x3F) as u8, data[2*address+1]];
            self.write(&mut high_byte_buffer, spi, delay)?;

            delay.delay_ms(20);
        }

        // Write the page into memory
        let mut buffer = [WRITE_PAGE, (page_number >> 2) as u8, (page_number << 6) as u8, 0x00];
        self.write(&mut buffer, spi, delay)?;

        // The maximum programming time for the atmega32a is 4.5ms
        delay.delay_ms(20);

        Ok(())
    }

    fn check_memory_page<SPIE: Debug>(
        &mut self,
        data: &[u8],
        page_number: usize,
        spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        delay: &mut (impl DelayMs<u32> + DelayUs<u32>)
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        for address in 0..core::cmp::min(ATMEGA_PAGESIZE, data.len() / 2) {
            let mut low_byte_buffer = [
                READ_LOW_BYTE,
                (page_number >> 2) as u8,
                (page_number as u8) << 6 | (address as u8) & 0x3F,
                0x00,
            ];
            self.write(&mut low_byte_buffer, spi, delay)?;
            if data[2*address] != low_byte_buffer[3] {
                return Err(KickerProgrammerError::BinaryDiffers {
                    page_number,
                    offset: address as u8,
                    low_byte: true,
                });
            }

            let mut high_byte_buffer = [
                READ_HIGH_BYTE,
                (page_number >> 2) as u8,
                (page_number as u8) << 6 | (address as u8) & 0x3F,
                0x00,
            ];
            self.write(&mut high_byte_buffer, spi, delay)?;
            if data[2*address+1] != high_byte_buffer[3] {
                return Err(KickerProgrammerError::BinaryDiffers {
                    page_number,
                    offset: address as u8,
                    low_byte: false,
                });
            }
        }

        Ok(())
    }

    fn exit_programming<SPIE: Debug>(
        &mut self,
        _spi: &mut (impl Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>),
        _delay: &mut (impl DelayMs<u32> + DelayUs<u32>),
    ) -> Result<(), KickerProgrammerError<GPIOE, SPIE>> {
        self.reset.set_high().map_err(KickerProgrammerError::Gpio)
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