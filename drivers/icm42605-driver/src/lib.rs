//!
//! Driver for the ICM42605 IMU.
//!
//! [Datasheet](https://invensense.tdk.com/wp-content/uploads/2020/09/DS-000292-ICM-42605-v1.5.pdf)
//!

#![no_std]
#![deny(missing_docs)]

use core::fmt::Debug;

use embedded_hal::blocking::{delay::DelayMs, i2c};
use registers::{Bank, BankSelect, WHO_AM_I};

mod registers;

const ICM_ADDR: u8 = 0b1101000;
const WHO_AM_I_EXPECTED: u8 = 0x42;
const LSB_TO_G: f32 = 16.0 / 32768.0;

const LSB_TO_DPS: f32 = 1000.0 / 32768.0;

#[inline]
/// Convert the high and low bits obtained from the IMU into a gyrometer
/// reading (in degrees per second)
pub fn reading_to_gyro(high: u8, low: u8) -> f32 {
    let value = i16::from_be_bytes([high, low]);
    ((value as f32) * LSB_TO_DPS).to_radians()
}

#[inline]
/// Convert the high and low bits obtained from the IMU into an accelerometer
/// reading (in g's)
pub fn reading_to_accel(high: u8, low: u8) -> f32 {
    let value = i16::from_be_bytes([high, low]);
    (value as f32) * LSB_TO_G
}

#[derive(Debug)]
/// An error from operating the IMU
pub enum ImuError<E: Debug> {
    /// The IMU was not initialized before being used
    Uninitialized,
    /// The IMU did not return a valid device identification when prompted
    InvalidDeviceId,
    /// An error ocurred with the I2C peripheral
    I2c(E),
}

/// A driver for the icm42605 imu
pub struct IMU<I2C> {
    /// The i2c peripheral
    i2c: I2C,
    /// The currently selected register bank
    current_bank: Bank,
    /// Whether or not the IMU has been initialized
    pub initialized: bool,
}

impl<I2C: i2c::Write<Error = E> + i2c::Read<Error = E>, E: Debug> IMU<I2C> {
    /// Acquire the necessary peripherals for the IMU driver.
    /// 
    /// Note: This does not initialize the IMU and `init` must be called before
    /// using the IMU
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            current_bank: Default::default(),
            initialized: false,
        }
    }

    /// Initialize the IMU driver
    pub fn init(&mut self, delay: &mut impl DelayMs<u8>) -> Result<(), ImuError<E>> {
        if self.read(Bank::Bank0, WHO_AM_I)? != WHO_AM_I_EXPECTED {
            return Err(ImuError::InvalidDeviceId);
        }

        // Reset Device
        self.write(
            registers::DeviceConfig::BANK,
            registers::DeviceConfig::ADDR,
            registers::DeviceConfig::SOFT_RESET_CONFIG.bits()
                | registers::DeviceConfig::SPI_MODE_0_AND_3.bits(),
        )?;

        delay.delay_ms(10);

        // Check whoami again
        if self.read(Bank::Bank0, WHO_AM_I)? != WHO_AM_I_EXPECTED {
            return Err(ImuError::InvalidDeviceId);
        }

        delay.delay_ms(100);

        // Turn on the gyro and accel
        self.write(
            registers::PowerManagement::BANK,
            registers::PowerManagement::ADDR,
            registers::PowerManagement::GYRO_LOW_NOISE.bits()
                | registers::PowerManagement::ACCEL_LOW_NOISE.bits(),
        )?;

        // From 14.36:
        // Gyroscope needs to be kept ON for a minimum of 45ms. When transitioning from OFF to any of the other modes, do not issue any register writes for 200μs.
        delay.delay_ms(100);

        // Configure gyro and accelerometer
        self.write(
            registers::GyroConfig::BANK,
            registers::GyroConfig::ADDR,
            registers::GyroConfig::GYRO_FS_1000.bits()
                | registers::GyroConfig::GYRO_ODR_1kHz.bits(),
        )?;

        self.write(
            registers::AccelConfig::BANK,
            registers::AccelConfig::ADDR,
            registers::AccelConfig::ACCEL_FS_16.bits()
                | registers::AccelConfig::ACCEL_ODR_1kHz.bits(),
        )?;

        Ok(())
    }

    /// Read the gyro velocity in the z direction
    pub fn gyro_z(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized { return Err(ImuError::Uninitialized); }

        let hi = self.read(Bank::Bank0, registers::GYRO_DATA_Z1)?;
        let lo = self.read(Bank::Bank0, registers::GYRO_DATA_Z0)?;

        Ok(reading_to_gyro(hi, lo))
    }

    /// Read the acceleration in the x direction
    pub fn accel_x(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized { return Err(ImuError::Uninitialized); }
       
        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_X1)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_X0)?;

        Ok(reading_to_accel(hi, lo))
    }

    /// Read the acceleration in the y direction
    pub fn accel_y(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized { return Err(ImuError::Uninitialized); }

        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_Y1)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_Y0)?;

        Ok(reading_to_accel(hi, lo))
    }

    /// Write the raw register address and data over the i2c line
    fn raw_write(&mut self, register_addr: u8, data: u8) -> Result<(), ImuError<E>> {
        self.i2c.write(ICM_ADDR, &[register_addr, data]).map_err(ImuError::I2c)
    }
    
    /// Change the IMU register bank that is current targeted
    fn switch_bank(&mut self, new_bank: Bank) -> Result<(), ImuError<E>> {
        self.raw_write(BankSelect::ADDR, new_bank.value())?;
        self.current_bank = new_bank;
        Ok(())
    }
    
    /// Read the data contained in a specific address of a given bank in the IMU
    fn read(&mut self, bank: Bank, address: u8) -> Result<u8, ImuError<E>> {
        if bank != self.current_bank {
            self.switch_bank(bank)?;
        }
        self.i2c.write(ICM_ADDR, &[address]).map_err(ImuError::I2c)?;

        let mut buf = [0];
        self.i2c.read(ICM_ADDR, &mut buf).map_err(ImuError::I2c)?;

        Ok(buf[0])
    }

    /// Write a value to a specific address of a given bank in the imu
    fn write(&mut self, bank: Bank, address: u8, value: u8) -> Result<(), ImuError<E>> {
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