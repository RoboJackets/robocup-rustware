#![no_std]

use core::fmt::Debug;
use core::marker::PhantomData;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::DelayUs;


pub mod error;
use error::KickerBoardError;
use error::{convert_gpio_error, convert_spi_error, convert_error};

pub mod command;
use command::{Command, KickerCommand};

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

/// Response to determine the voltage of the kicker board
const VOLTAGE_MASK: u8 = 0x7f;

/// Scale to multiply incoming voltage by
const VOLTAGE_SCALE: u8 = 2;

/// Whether or not the breakbeam has been tripped
const BREAKBEAM_TRIPPED: u8 = 1 << 7;

/// For a good number of commands the high or low byte of the command written to the 
/// device will be returned.  Use this enum to control that
pub enum ReturnByte {
    High = 0x40,
    Low = 0x44,
}

pub enum KickType {
    Kick,
    Chip,
}

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
    pub kick_type: KickType,
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
            kick_type: KickType::Kick,
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

    pub fn program(&mut self, binary: &[u8], pages: u8, page_size: u8, num_pages: u8, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        self.chip_erase(spi, delay)?;

        let mut page_offset = 0u8;
        let mut page_number = 0u8;
        let mut c;
        let mut return_byte = ReturnByte::Low;
        let mut binary_loc = 0usize;

        let mut lc_offset = 0xffu8;

        while binary_loc < binary.len() {
            c = binary[binary_loc];
            binary_loc += 1;

            if page_offset == page_size {
                self.write_flash_memory_page(page_number, spi, delay)?;
                lc_offset = 0xff;

                page_number += 1;
                if page_number > num_pages {
                    return Err(KickerBoardError::MaxBinaryExceeded);
                }
                page_offset = 0;
            }

            if lc_offset == 0xff && c != 0xff {
                lc_offset = page_offset;
            }

            // Write low byte
            match return_byte {
                ReturnByte::High => {
                    self.load_memory_page(ReturnByte::High, page_offset, c, spi, delay)?;
                    return_byte = ReturnByte::Low;
                },
                ReturnByte::Low => {
                    self.load_memory_page(ReturnByte::Low, page_offset, c, spi, delay)?;
                    return_byte = ReturnByte::High;
                    page_offset += 1;
                }
            }
        }

        Ok(())
    }

    pub fn read_vendor_code(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<u8, KickerBoardError<E>> {
        self.read_register(0x00, spi, delay)
    }

    pub fn read_part_family_and_flash_size(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<u8, KickerBoardError<E>> {
        self.read_register(0x01, spi, delay)
    }

    pub fn read_part_number(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<u8, KickerBoardError<E>> {
        self.read_register(0x02, spi, delay)
    }

    fn read_register(&mut self, register: u8, spi: &mut SPI, delay: &mut DELAY) -> Result<u8, KickerBoardError<E>> {
        delay.delay_us(10);
        self.csn.set_low().map_err(convert_gpio_error)?;
        spi.write(&[0x30, 0x00, register]).map_err(convert_spi_error)?;
        let mut response = [0x00];
        spi.transfer(&mut response).map_err(convert_spi_error)?;
        self.csn.set_high().map_err(convert_gpio_error)?;
        delay.delay_us(10);

        Ok(response[0])
    }

    fn write_fuse_bits_low(&mut self, delay: &mut DELAY, spi: &mut SPI) -> Result<(), KickerBoardError<E>> {
        delay.delay_us(10);

        self.csn.set_low().map_err(convert_gpio_error)?;
        spi.write(&[0xac, 0xa0, 0x33, 0xe4]).map_err(convert_spi_error)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        delay.delay_us(10);

        Ok(())
    }

    fn check_memory(&mut self, page_size: u8, num_pages: u8, binary: &[u8], spi: &mut SPI, delay: &mut DELAY) -> Result<bool, KickerBoardError<E>> {
        let mut success = true;

        for page in 0..num_pages {
            for offset in 0..page_size {
                let c = binary[(page * page_size + offset) as usize];

                // TODO: Handle Error With Read Program Memory
                let response = self.read_program_memory(ReturnByte::Low, page, offset, spi).unwrap();

                if c != response {
                    success = false;
                }

                let c = binary[(page * page_size + offset + 1) as usize];

                // TODO: Handle Error With Read Program Memory
                let response = self.read_program_memory(ReturnByte::High, page, offset, spi).unwrap();

                if c != response {
                    success = false;
                }
            }
        }

        Ok(success)
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

    // This seems to just wait 5 ticks
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

    fn load_memory_page(&mut self, return_byte: ReturnByte, address: u8, data: u8, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        delay.delay_us(10);
        self.csn.set_low().map_err(convert_gpio_error)?;
        spi.write(&[return_byte as u8, 0x00, address & 0x3f, data]).map_err(convert_spi_error)?;
        self.csn.set_high().map_err(convert_gpio_error)?;
        delay.delay_us(10);

        Ok(())
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

    fn write_flash_memory_page(&mut self, page_number: u8, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        delay.delay_us(10);
        self.csn.set_low().map_err(convert_gpio_error)?;
        spi.write(&[0x4C, page_number >> 2, page_number << 6, 0x00]).map_err(convert_spi_error)?;
        self.csn.set_high().map_err(convert_gpio_error)?;
        // TODO: Make sure this is 10 us + 5 ticks
        delay.delay_us(15);

        Ok(())
    }

    fn read_program_memory(&mut self, return_byte: ReturnByte, page_number: u8, page_offset: u8, spi: &mut SPI) -> Result<u8, KickerBoardError<E>> {
        self.csn.set_low().map_err(convert_gpio_error)?;
        spi.write(&[return_byte as u8, page_number >> 2, (page_number << 6) | (page_offset & 0x3f)]).map_err(convert_spi_error)?;
        let mut response = [0x00];
        spi.transfer(&mut response).map_err(convert_spi_error)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        return Ok(response[0]);
    }

    pub fn service(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), KickerBoardError<E>> {
        let mut command = 0x00;

        match self.kick_type {
            KickType::Kick => command |= KickerCommand::NormalKick as u8,
            KickType::Chip => command |= KickerCommand::ChipKick as u8,
        }

        if self.kick_immediate {
            command |= KickerCommand::KickImmediate as u8;
            self.kick_immediate = false;
        }

        if self.kick_breakbeam {
            command |= KickerCommand::KickOnBreakbeam as u8;
            self.kick_breakbeam = false;
        }

        if self.cancel_kick {
            command |= KickerCommand::CancelKick as u8;
            self.cancel_kick = false;
        }

        if self.charge_allowed {
            command |= KickerCommand::ChargeAllowed as u8;
        }

        // TODO: Set SPI Frequency to 100_000 Hz

        self.csn.set_low().map_err(convert_gpio_error)?;
        delay.delay_us(50);
        spi.write(&[command]).map_err(convert_spi_error)?;
        let mut response = [0u8];
        spi.transfer(&mut response).map_err(convert_spi_error)?;
        delay.delay_us(50);
        self.csn.set_high().map_err(convert_gpio_error)?;

        self.current_voltage = response[0] & VOLTAGE_MASK * VOLTAGE_SCALE;

        self.ball_sensed = (response[0] & VOLTAGE_SCALE) > 0;

        // The kicker board is healthy if we get voltage back
        self.is_healthy = self.current_voltage > 0;

        Ok(())
    }
}