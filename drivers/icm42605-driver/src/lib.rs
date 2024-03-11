//!
//! Driver for the ICM42605 IMU.
//!
//! [Datasheet](https://invensense.tdk.com/wp-content/uploads/2020/09/DS-000292-ICM-42605-v1.5.pdf)
//!

#![no_std]

use embedded_hal::blocking::{delay::DelayMs, i2c};
use registers::{Bank, BankSelect, WHO_AM_I};

pub mod registers;

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

pub struct IMU<I2C> {
    i2c: I2C,
    current_bank: Bank,
}

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