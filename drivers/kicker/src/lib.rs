#![no_std]

use core::fmt::Debug;
use core::marker::PhantomData;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::DelayUs;

pub mod error;
use error::KickerBoardError;
use error::{convert_gpio_error, convert_spi_error, convert_error};

/// Voltage Threshold for kicker board to be considered charge (voltage mapped to 0-255)
pub const IS_CHARGED_CUTOFF: u8 = 230;

/// Vendor Code for the AVR910 Chip
const ATMEL_VENDOR_CODE: u8 = 0x1e;

/// Family Mask of the AVR910 Chip
const AVR_FAMILY_MASK: u8 = 0xf0;

/// Family ID of the AVR910 Chip
const AVR_FAMILY_ID: u8 = 0x90;

/// ID of the ATMEGA DEVICE
const ATMEGA_DEVICE_ID: u8 = 0x02;

/// Page Size of the ATMEGA in words (1 word = 2 Bytes)
const ATMEGA_PAGESIZE: u8 = 64;

/// Number of pages possible on the ATMEGA device
const ATMEGA_NUM_PAGES: u16 = 256;

/// Kicker Board Driver.
/// 
/// The Kicker Board Driver takes care of the programming and communication between
/// the main board and the kicker board.
/// 
pub struct KickerBoard<E: Debug, CSN: OutputPin<Error = E>, RESET: OutputPin<Error = E>, SPI: Transfer<u8, Error = E> + Write<u8, Error = E>, DELAY: DelayUs<u32>> {
    csn: CSN,
    reset: RESET,
    pub verbose: bool,
    pub ball_sensed: bool,
    pub is_healthy: bool,
    pub current_voltage: u8,
    pub is_kick: bool,
    pub kick_immediate: bool,
    pub kick_breakbeam: bool,
    pub cancel_kick: bool,
    pub charge_allowed: bool,
    pub kick_strength: u8,
    phantom: PhantomData<(SPI, DELAY)>,
}

impl<E, CSN, RESET, SPI, DELAY> KickerBoard<E, CSN, RESET, SPI, DELAY>
    where E: Debug, CSN: OutputPin<Error = E>, RESET: OutputPin<Error = E>,
    SPI: Write<u8, Error = E> + Transfer<u8, Error = E>, DELAY: DelayUs<u32> {
    pub fn new(csn: CSN, reset: RESET, verbose: bool) -> Result<Self, KickerBoardError<E>> {
        Ok(Self {
            csn,
            reset,
            verbose,
            ball_sensed: false,
            is_healthy: true,
            current_voltage: 0u8,
            is_kick: false,
            kick_immediate: false,
            kick_breakbeam: false,
            cancel_kick: true,
            charge_allowed: false,
            kick_strength: 0u8,
            phantom: PhantomData,
        })
    }

    pub fn init(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        let mut try_count = 0;
        let mut enabled = false;
        self.csn.set_high().map_err(convert_gpio_error)?;

        while !enabled && try_count < 20 {
            self.reset.set_high().map_err(convert_gpio_error)?;
            delay.delay_us(20_000);
            self.reset.set_low().map_err(convert_gpio_error)?;

            // Wait at least 20 ms
            delay.delay_us(20_000);

            // Enable SPI Serial Programming
            enabled = self.enable_programming(spi, delay)?;

            try_count += 1;
        }

        if !enabled {
            return Err(KickerBoardError::UnableToEnableProgramming);
        }

        Ok(())
    }

    pub fn program(&mut self, binary: &[u8], pages: u8, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        self.chip_erase(spi, delay)?;

        todo!()

        Ok(())
    }

    pub fn read_vendor_code() -> Result<(), KickerBoardError<E>> {
        todo!()
    }

    pub fn read_part_family_and_flash_size() -> Result<(), KickerBoardError<E>> {
        todo!()
    }

    pub fn read_part_number() -> Result<(), KickerBoardError<E>> {
        todo!()
    }

    fn write_fuse_bits_low(&mut self, delay: &mut DELAY, spi: &mut SPI) -> Result<(), KickerBoardError<E>> {
        delay.delay_us(10);

        self.csn.set_low().map_err(convert_gpio_error)?;
        spi.write(&[0xac, 0xa0, 0x33, 0xe4]).map_err(convert_spi_error)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        delay.delay_us(10);

        Ok(())
    }

    fn check_memory() -> Result<(), KickerBoardError<E>> {
        todo!()
    }

    fn exit_programming(&mut self, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        self.reset.set_low().map_err(convert_gpio_error)?;
        delay.delay_us(100_000);
        self.reset.set_high().map_err(convert_gpio_error)?;
        Ok(())
    }

    fn enable_programming(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<bool, KickerBoardError<E>> {
        delay.delay_us(10);
        self.csn.set_low().map_err(convert_gpio_error)?;
        spi.write(&[0xac, 0x53]).map_err(convert_spi_error)?;
        let mut response = [0x00];
        spi.transfer(&mut response).map_err(convert_spi_error)?;
        spi.write(&[0x00]).map_err(convert_spi_error)?;
        self.csn.set_high().map_err(convert_gpio_error)?;
        delay.delay_us(10);

        if response[0] == 0x53 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn poll() -> Result<(), KickerBoardError<E>> {
        todo!()
    }

    fn chip_erase(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        delay.delay_us(10);

        // Issue Chip Erase Command
        self.csn.set_low().map_err(convert_gpio_error)?;
        spi.write(&[0xac, 0x80, 0x00, 0x00]).map_err(convert_spi_error)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        delay.delay_us(10);

        // Delay for 9 ms
        delay.delay_us(15_000);

        Ok(())
    }

    fn load_memory_page() -> Result<(), KickerBoardError<E>> {
        todo!()
    }

    fn write_flash_memory_byte(&mut self, address: u16, data: u8, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        delay.delay_us(10);

        self.csn.set_low().map_err(convert_gpio_error)?;
        let data = [
            0x4c,
            ((address & 0xff00) >> 8) as u8,
            (address & 0x003f) as u8,
            data,
        ];
        spi.write(&data).map_err(convert_spi_error)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        delay.delay_us(10);

        Ok(())
    }

    fn write_flash_memory_page(&mut self, page_number: u8, page_offset: u8, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        delay.delay_us(10);

        self.csn.set_low().map_err(convert_gpio_error)?;
        spi.write(&[0x4c, page_number >> 2, page_number << 6, 0x00])
    }

    fn read_program_memory() -> Result<u8, KickerBoardError<E>> {
        todo!()
    }
}