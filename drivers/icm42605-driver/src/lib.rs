//!
//! Driver for the ICM42605 IMU.
//!
//! [Datasheet](https://invensense.tdk.com/wp-content/uploads/2020/09/DS-000292-ICM-42605-v1.5.pdf)
//!

#![no_std]

use core::marker::PhantomData;

#[cfg(feature = "i2c")]
use embedded_hal::blocking::i2c::{Write, Read};

#[cfg(feature = "spi")]
use embedded_hal::blocking::spi::{Transfer, Write};
#[cfg(feature = "spi")]
use embedded_hal::digital::v2::OutputPin;

use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use registers::{Bank, BankSelect, WHO_AM_I};

pub mod registers;

#[cfg(feature = "i2c")]
const ICM_ADDR: u8 = 0b1101000;
const WHO_AM_I_EXPECTED: u8 = 0x42;
const LSB_TO_G: f32 = 16.0 / 32768.0;

const LSB_TO_DPS: f32 = 1000.0 / 32768.0;

#[inline]
pub fn reading_to_gyro(high: u8, low: u8) -> f32 {
    let value = i16::from_be_bytes([high, low]);
    ((value as f32) * LSB_TO_DPS).to_radians()
}

#[inline]
pub fn reading_to_accel(high: u8, low: u8) -> f32 {
    let value = i16::from_be_bytes([high, low]);
    (value as f32) * LSB_TO_G
}

#[cfg(feature = "i2c")]
pub struct IMU<I2C> {
    i2c: I2C,
    current_bank: Bank,
}

#[cfg(feature = "spi")]
pub enum IMUError<SPIE, GPIOE> {
    SpiError(SPIE),
    GpioError(GPIOE),
    SpiGpioError(SPIE, GPIOE),
}

#[cfg(feature = "spi")]
pub struct IMU<SPI, CSN, DELAY, SPIE, GPIOE> where
    SPI: Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>,
    CSN: OutputPin<Error=GPIOE>,
    DELAY: DelayMs<u8> + DelayUs<u8> {
    csn: CSN,
    delay: DELAY,
    current_bank: Bank,
    phantom: PhantomData<SPI>,
}

#[cfg(feature = "i2c")]
impl<I2C: i2c::Write<Error = E> + i2c::Read<Error = E>, E> IMU<I2C> {
    pub fn new(i2c: I2C, delay: &mut impl DelayMs<u8>) -> Result<Self, E> {
        let mut this = Self {
            i2c,
            current_bank: Default::default(),
        };

        log::info!("Checking whomai 1.");
        while this.read(Bank::Bank0, WHO_AM_I)? != WHO_AM_I_EXPECTED {
            log::warn!("Whoami was not what expected.");
            delay.delay_ms(100);
        }

        log::info!("Resetting 1.");
        // Reset
        this.write(
            registers::DeviceConfig::BANK,
            registers::DeviceConfig::ADDR,
            registers::DeviceConfig::SOFT_RESET_CONFIG.bits()
                | registers::DeviceConfig::SPI_MODE_0_AND_3.bits(),
        )?;

        delay.delay_ms(10);

        log::info!("Checking whomai 2.");
        // Check whoami again
        while this.read(Bank::Bank0, WHO_AM_I)? != WHO_AM_I_EXPECTED {
            log::warn!("Whoami was not what expected.");
            delay.delay_ms(100);
        }

        delay.delay_ms(100);

        log::info!("Turn on gyro and accelerometer.");
        // Turn on the gyro and accel
        this.write(
            registers::PowerManagement::BANK,
            registers::PowerManagement::ADDR,
            registers::PowerManagement::GYRO_LOW_NOISE.bits()
                | registers::PowerManagement::ACCEL_LOW_NOISE.bits(),
        )?;

        // From 14.36:
        // Gyroscope needs to be kept ON for a minimum of 45ms. When transitioning from OFF to any of the other modes, do not issue any register writes for 200μs.
        delay.delay_ms(100);

        log::info!("Configuring gyro and accelerometer.");
        // Configure gyro and accelerometer
        this.write(
            registers::GyroConfig::BANK,
            registers::GyroConfig::ADDR,
            registers::GyroConfig::GYRO_FS_1000.bits()
                | registers::GyroConfig::GYRO_ODR_1kHz.bits(),
        )?;

        this.write(
            registers::AccelConfig::BANK,
            registers::AccelConfig::ADDR,
            registers::AccelConfig::ACCEL_FS_16.bits()
                | registers::AccelConfig::ACCEL_ODR_1kHz.bits(),
        )?;

        Ok(this)
    }

    pub fn gyro_z(&mut self) -> Result<f32, E> {
        let hi = self.read(Bank::Bank0, registers::GYRO_DATA_Z1)?;
        let lo = self.read(Bank::Bank0, registers::GYRO_DATA_Z0)?;

        Ok(reading_to_gyro(hi, lo))
    }

    pub fn accel_x(&mut self) -> Result<f32, E> {
        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_X1)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_X0)?;

        Ok(reading_to_accel(hi, lo))
    }

    pub fn accel_y(&mut self) -> Result<f32, E> {
        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_Y1)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_Y0)?;

        Ok(reading_to_accel(hi, lo))
    }

    fn raw_write(&mut self, register_addr: u8, data: u8) -> Result<(), E> {
        self.i2c.write(ICM_ADDR, &[register_addr, data])
    }

    fn switch_bank(&mut self, new_bank: Bank) -> Result<(), E> {
        self.raw_write(BankSelect::ADDR, new_bank.value())?;
        self.current_bank = new_bank;
        Ok(())
    }

    fn read(&mut self, bank: Bank, address: u8) -> Result<u8, E> {
        if bank != self.current_bank {
            self.switch_bank(bank)?;
        }
        self.i2c.write(ICM_ADDR, &[address])?;

        let mut buf = [0];
        self.i2c.read(ICM_ADDR, &mut buf)?;

        Ok(buf[0])
    }

    fn write(&mut self, bank: Bank, address: u8, value: u8) -> Result<(), E> {
        if bank != self.current_bank {
            self.switch_bank(bank)?;
        }

        self.raw_write(address, value)?;
        Ok(())
    }
}

#[cfg(feature = "spi")]
impl<SPI, CSN, DELAY, SPIE, GPIOE> IMU<SPI, CSN, DELAY, SPIE, GPIOE> where
    SPI: Transfer<u8, Error=SPIE> + Write<u8, Error=SPIE>,
    CSN: OutputPin<Error=GPIOE>,
    DELAY: DelayMs<u8> + DelayUs<u8> {
    pub fn new(csn: CSN, delay: DELAY, spi: &mut SPI) -> Result<Self, IMUError<SPIE, GPIOE>> {
        let mut this = Self {
            csn,
            delay,
            current_bank: Default::default(),
            phantom: PhantomData,
        };

        this.write(
            registers::DeviceConfig::BANK,
            registers::DeviceConfig::ADDR,
            registers::DeviceConfig::SOFT_RESET_CONFIG.bits()
                | registers::DeviceConfig::SPI_MODE_0_AND_3.bits(),
            spi,
        )?;

        this.delay.delay_ms(10);

        while this.read(Bank::Bank0, WHO_AM_I, spi)? != WHO_AM_I_EXPECTED {
            this.delay.delay_ms(100)
        }

        this.delay.delay_ms(100);

        this.write(
            registers::PowerManagement::BANK,
            registers::PowerManagement::ADDR,
            registers::PowerManagement::GYRO_LOW_NOISE.bits()
                | registers::PowerManagement::ACCEL_LOW_NOISE.bits(),
            spi
        )?;

        // From 14.36:
        // Gyroscope needs to be kept ON for a minimum of 45ms. When transitioning from OFF to any of the other modes, do not issue any register writes for 200μs.
        this.delay.delay_ms(100);

        // Configure Gyro and Accelerometer
        this.write(
            registers::GyroConfig::BANK,
            registers::GyroConfig::ADDR,
            registers::GyroConfig::GYRO_FS_1000.bits()
                | registers::GyroConfig::GYRO_ODR_1kHz.bits(),
            spi,
        )?;

        this.write(
            registers::AccelConfig::BANK,
            registers::AccelConfig::ADDR,
            registers::AccelConfig::ACCEL_FS_16.bits()
                | registers::AccelConfig::ACCEL_ODR_1kHz.bits(),
            spi,
        )?;

        Ok(this)
    }

    pub fn gyro_z(&mut self, spi: &mut SPI) -> Result<f32, IMUError<SPIE, GPIOE>> {
        let hi = self.read(Bank::Bank0, registers::GYRO_DATA_Z1, spi)?;
        let lo = self.read(Bank::Bank0, registers::GYRO_DATA_Z0, spi)?;

        Ok(reading_to_gyro(hi, lo))
    }

    pub fn accel_x(&mut self, spi: &mut SPI) -> Result<f32, IMUError<SPIE, GPIOE>> {
        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_X1, spi)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_X0, spi)?;

        Ok(reading_to_accel(hi, lo))
    }

    pub fn accel_y(&mut self, spi: &mut SPI) -> Result<f32, IMUError<SPIE, GPIOE>> {
        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_Y1, spi)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_Y0, spi)?;

        Ok(reading_to_accel(hi, lo))
    }

    fn read(&mut self, bank: Bank, address: u8, spi: &mut SPI) -> Result<u8, IMUError<SPIE, GPIOE>> {
        if bank != self.current_bank {
            self.switch_bank(bank, spi)?;
        }

        let mut buffer = [address, 0x00];
        self.safe_transfer(&mut buffer, spi)?;
        Ok(buffer[1])
    }

    fn write(&mut self, bank: Bank, address: u8, value: u8, spi: &mut SPI) -> Result<(), IMUError<SPIE, GPIOE>> {
        if bank != self.current_bank {
            self.switch_bank(bank, spi)?;
        }

        self.write(bank, address, value, spi)
    }

    fn switch_bank(&mut self, new_bank: Bank, spi: &mut SPI) -> Result<(), IMUError<SPIE, GPIOE>> {
        self.safe_write(&[BankSelect::ADDR, new_bank.value()], spi)
    }

    fn safe_write(&mut self, data: &[u8], spi: &mut SPI)
        -> Result<(), IMUError<SPIE, GPIOE>> {
        self.csn.set_low().map_err(IMUError::GpioError)?;
        let spi_err = spi.write(data);
        let gpio_err = self.csn.set_high();

        self.delay.delay_us(1);

        match (spi_err, gpio_err) {
            (Err(spi), Err(gpio)) => Err(IMUError::SpiGpioError(spi, gpio)),
            (Err(err), _) => Err(IMUError::SpiError(err)),
            (_, Err(err)) => Err(IMUError::GpioError(err)),
            (_, _) => Ok(())
        }
    }

    fn safe_transfer(&mut self, data: &mut [u8], spi: &mut SPI)
        -> Result<(), IMUError<SPIE, GPIOE>> {
        self.csn.set_low().map_err(IMUError::GpioError)?;
        let spi_err = spi.transfer(data);
        let gpio_err = self.csn.set_high();

        self.delay.delay_us(1);

        match (spi_err, gpio_err) {
            (Err(spi), Err(gpio)) => Err(IMUError::SpiGpioError(spi, gpio)),
            (Err(err), _) => Err(IMUError::SpiError(err)),
            (_, Err(err)) => Err(IMUError::GpioError(err)),
            (_, _) => Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn test_reading_to_gyro() {
        let high_reading = 0x12;
        let low_reading = 0x34;

        assert_eq!(reading_to_gyro(high_reading, low_reading), 2.4820662);
    }

    #[test]
    fn test_reading_to_accel() {
        let high_reading = 0x12;
        let low_reading = 0x34;

        assert_eq!(reading_to_accel(high_reading, low_reading), 2.2753906);
    }
}