//!
//! Driver for programming the Kicker Module
//! 
//! Note: This should be instantiated to program the kicker and can then be
//! decomposed to use the underlying hardware for the kicker driver.
//! 

#![no_std]
#![crate_type = "lib"]

mod binary;

use core::marker::PhantomData;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::{DelayUs, DelayMs};

pub const ATMEL_VENDOR_CODE: u8 = 0x1E;
pub const AVR_FAMILY_ID: u8 = 0x90;
pub const ATMEGA_DEVICE_ID: u8 = 0x02;

// Pagesize in words (1 word = 2 bytes)
pub const ATMEGA_PAGESIZE: usize = 64;
pub const ATMEGA_NUM_PAGES: usize = 256;
pub const MAX_BINARY_SIZE: usize = 2 * 64 * 256;

pub enum KickerProgramError<SPIE, GPIOE> {
    InvalidVendorId{expected: u8, found: u8},
    InvalidPartFamily{expected: u8, found: u8},
    InvalidDeviceId{expected: u8, found: u8},
    InvalidBinarySize{max: usize, found: usize},
    FailedToProgramKicker,
    GpioError(GPIOE),
    SpiError(SPIE),
    SpiGpioError((SPIE, GPIOE)),
}

pub enum MemoryByte {
    Low,
    High,
}

impl MemoryByte {
    pub fn read(self) -> u8 {
        match self {
            Self::Low => 0x20,
            Self::High => 0x28,
        }
    }

    pub fn write(self) -> u8 {
        match self {
            Self::Low => 0x40,
            Self::High => 0x48,
        }
    }
}

pub struct KickerProgrammer<CSN, RESET, SPI, DELAY, GPIOE, SPIE> where
    CSN: OutputPin<Error=GPIOE>,
    RESET: OutputPin<Error=GPIOE>,
    SPI: Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>,
    DELAY: DelayUs<u32> + DelayMs<u32> {
    spi: SPI,
    delay: DELAY,
    csn: CSN,
    reset: RESET,
}

impl<CSN, RESET, SPI, DELAY, GPIOE, SPIE> Kicker<CSN, RESET, SPI, DELAY, GPIOE, SPIE> where
    CSN: OutputPin<Error=GPIOE>,
    RESET: OutputPin<Error=GPIOE>,
    SPI: Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>,
    DELAY: DelayUs<u32> + DelayMs<u32> {
    pub fn new(csn: CSN, reset: RESET, spi: SPI, delay: DELAY) -> Self {
        Self {
            spi,
            delay,
            csn,
            reset,
        }
    }

    /// Check that the vendor code of the kicker is valid.
    pub fn check_vendor_code(&mut self) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        let vendor_code = self.read_register(0x00)?;
        match vendor_code {
            ATMEL_VENDOR_CODE => Ok(()),
            _ => Err(KickerProgramError::InvalidVendorId { expected: ATMEL_VENDOR_CODE, found: vendor_code }),
        }
    }

    /// Check that the part family of the kicker is valid.
    pub fn check_part_family(&mut self) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        let part_family = self.read_register(0x01)?;
        match part_family {
            AVR_FAMILY_ID => Ok(()),
            _ => Err(KickerProgramError::InvalidPartFamily { expected: AVR_FAMILY_ID, found: part_family }),
        }
    }

    /// Check that the part number of the kicker is valid.
    pub fn check_part_number(&mut self) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        let part_number = self.read_register(0x02)?;
        match part_number {
            ATMEGA_DEVICE_ID => Ok(()),
            _ => Err(KickerProgramError::InvalidDeviceId { expected: AVR_FAMILY_ID, found: part_number }),
        }
    }

    /// Determine whether the program to flash is the same as the program already flashed to the kicker.
    pub fn programs_different(&mut self, program: &[u8]) -> Result<bool, KickerProgramError<SPIE, GPIOE>> {
        if program.len() > MAX_BINARY_SIZE {
            return Err(KickerProgramError::InvalidBinarySize { max: MAX_BINARY_SIZE, found: program.len() });
        }

        let mut program_pointer = 0;
        'page: for page in 0..ATMEGA_NUM_PAGES {
            for offset in 0..ATMEGA_PAGESIZE {
                let low_byte = self.read_program_memory(MemoryByte::Low, page, offset)?;
                if low_byte != program[program_pointer] {
                    return (Ok(true));
                }
                program_pointer += 1;
                if program_pointer >= program.len() {
                    break 'page;
                }

                let high_byte = self.read_program_memory(MemoryByte::High, page, offset)?;
                if high_byte != program[program_pointer] {
                    return (Ok(true));
                }
                program_pointer += 1;
                if program_pointer >= program.len() {
                    break 'page;
                }
            }
        }
        Ok(false)
    }

    /// Program the Kicker
    pub fn program(&mut self, program: &[u8]) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        if program.len() > MAX_BINARY_SIZE {
            return Err(KickerProgramError::InvalidBinarySize { max: MAX_BINARY_SIZE, found: program.len() });
        }

        self.erase()?;

        let mut program_pointer = 0;
        'page: for page in ATMEGA_NUM_PAGES {
            for offset in ATMEGA_PAGESIZE {
                // Write Low Byte
                self.load_memory_page(MemoryByte::Low, offset, page[program_pointer])?;
                program_pointer += 1;
                if program_pointer >= program.len() {
                    self.write_page(page)?;
                    self.poll()?;
                    break 'page;
                }

                self.load_memory_page(MemoryByte::High, offset, page[program_pointer])?;
                program_pointer += 1;
                if program_pointer >= program.len() {
                    self.write_page(page)?;
                    self.poll()?;
                    break 'page;
                }
            }
            self.write_page(page)?;
            self.poll()?;
        }

        let success = self.programs_different(program)?;

        self.exit_programming()?;

        match success {
            true => Ok(()),
            false => Err(KickerProgramError::FailedToProgramKicker),
        }
    }

    /// Load data into memory at a specific page offset.
    fn load_memory_page(&mut self, high_low: MemoryByte, address: u8, data: u8) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        self.delay.delay_us(10);
        self.write(&[
            high_low.write(),
            0x00,
            address & 0x3F,
            data,
        ])?;
        self.delay.delay_us(10);
        Ok(())
    }

    fn write_page(&mut self, page: usize) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        self.delay.delay_us(10);
        self.write(&[
            0x4C,
            (page >> 2) as u8,
            (page << 6) as u8,
            0x00
        ]);
        self.delay.delay_us(10);
        Ok(())
    }

    fn poll(&mut self) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        // TODO: Determine why this code was commented out in the old firmware code
        // let mut buffer: [u8; 4] = [0xF0, 0x00, 0x00, 0x00];
        // while buffer[3] & 0x01 != 0 {
        //     buffer = [0xF0, 0x00, 0x00, 0x00];
        //     self.transfer(&mut buffer)?;
        // }

        self.delay.delay_ms(5);
        Ok(())
    }

    /// Erase the contents of the kicker
    fn erase(&mut self) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        self.delay.delay_us(10);
        self.write(&[0xAc, 0x80, 0x00, 0x00])?;
        self.delay.delay_us(10);

        // Delay at least 9 ms
        self.delay.delay_ms(15);
        Ok(())
    }

    /// Read the high or low byte from a given location in memory
    /// 
    /// high_low:
    ///     0b0100_0000 => low byte
    ///     0b0100_0100 => high byte
    fn read_program_memory(&mut self, high_low: MemoryByte, page_number: usize, page_offset: usize) -> Result<u8, KickerProgramError<SPIE, GPIOE>> {
        let mut buffer = [
            high_low.read(),
            (page_number >> 2) as u8,
            ((page_number << 6) | (page_offset & 0x3F)) as u8,
            0x00,
        ];

        self.transfer(&mut buffer)?;
        Ok(buffer[3])
    }

    fn exit_programming(&mut self) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        self.reset.set_low().map_err(KickerProgramError::GpioError)?;
        self.delay.delay_us(100_000);
        self.reset.set_high().map_err(KickerProgramError::GpioError)?;
        Ok(())
    }

    fn read_register(&mut self, register: u8) -> Result<u8, KickerProgramError<SPIE, GPIOE>> {
        self.delay.delay_us(10);

        let mut buffer = [0x30, 0x00, register, 0x00];
        self.transfer(&mut buffer)?;
        
        self.delay.delay_us(10);
        Ok(buffer[3])
    }

    fn transfer(&mut self, buffer: &mut [u8]) -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        self.csn.set_low().map_err(KickerProgramError::GpioError)?;
        let spi_err = self.spi.transfer(buffer);
        let gpio_err = self.csn.set_high();
        self.delay.delay_us(1);

        match (spi_err, gpio_err) {
            (Err(spi), Err(gpio)) => Err(KickerProgramError::SpiGpioError((spi, gpio))),
            (Err(err), _) => Err(KickerProgramError::SpiError(err)),
            (_, Err(err)) => Err(KickerProgramError::GpioError(err)),
            (_, _) => Ok(()),
        }
    }

    fn write(&mut self, data: &[u8], spi: &mut SPI, delay: &mut DELAY)
        -> Result<(), KickerProgramError<SPIE, GPIOE>> {
        self.csn.set_low().map_err(KickerProgramError::GpioError)?;
        let spi_err = spi.write(data);
        let gpio_err = self.csn.set_high();

        match (spi_err, gpio_err) {
            (Err(spi), Err(gpio)) => Err(KickerProgramError::SpiGpioError((spie, gpio))),
            (Err(err), _) => Err(KickerProgramError::SpiError(err)),
            (_, Err(err)) => Err(KickerProgramError::GpioError(err)),
            (_, _) => Ok(()),
        }
    }
}