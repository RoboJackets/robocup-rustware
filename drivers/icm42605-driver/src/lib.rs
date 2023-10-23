#![no_std]

use embedded_hal::blocking::{i2c, delay::DelayMs};
use registers::{Bank, BankSelect, WHO_AM_I};

pub mod registers;

const WHO_AM_I_EXPECTED: u8 = 0x42;
const LSB_TO_G: f64 = 4.0 / 32768.0;

pub struct Icm42605<I2C> {
    i2c: I2C,
    current_bank: Bank,
}

impl<
        I2C: i2c::Write<Error = E> + i2c::Read<Error = E> + 'static,
        E
    > Icm42605<I2C>
{
    pub fn new(i2c: I2C, delay: &mut impl DelayMs<u8>) -> Result<Self, E> {
        let mut this = Self {
            i2c,
            current_bank: Default::default(),
        };

        while this.read(Bank::Bank0, WHO_AM_I)? != WHO_AM_I_EXPECTED {
            // TODO: log???
            delay.delay_ms(100);
        }

        // Reset
        this.write(
            registers::DeviceConfig::BANK,
            registers::DeviceConfig::ADDR,
            registers::DeviceConfig::SOFT_RESET_CONFIG.bits()
                | registers::DeviceConfig::SPI_MODE_0_AND_3.bits(),
        )?;

        delay.delay_ms(100);

        // Check whoami again
        while this.read(Bank::Bank0, WHO_AM_I)? != WHO_AM_I_EXPECTED {
            // TODO: log???
            delay.delay_ms(100);
        }

        delay.delay_ms(100);

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

    pub fn gyro_z(&mut self) -> Result<f64, E> {
        let hi = self.read(Bank::Bank0, registers::GYRO_DATA_Z1)?;
        let lo = self.read(Bank::Bank0, registers::GYRO_DATA_Z0)?;

        // Reinterpret the bits as a signed 16-bit integer
        let total = u16::from_ne_bytes([hi, lo]);

        // +-1000dps max, for 16-bit signed integers, comes out to 32.8lsb/deg.
        const LSB_TO_DPS: f64 = 1000.0 / 32768.0;
        const DEG_TO_RADIANS: f64 = core::f64::consts::PI / 180.0;

        Ok(total as f64 * (DEG_TO_RADIANS * LSB_TO_DPS))
    }

    pub fn accel_x(&mut self) -> Result<f64, E> {
        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_X1)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_X0)?;

        // Reinterpret the bits as a signed 16-bit integer
        let total = u16::from_ne_bytes([hi, lo]);

        // +-4g max, for 16-bit signed integers, comes out to 8192lsb/g.
        Ok(total as f64 * LSB_TO_G)
    }

    pub fn accel_y(&mut self) -> Result<f64, E> {
        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_Y1)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_Y0)?;

        // Reinterpret the bits as a signed 16-bit integer
        let total = u16::from_ne_bytes([hi, lo]);

        // +-4g max, for 16-bit signed integers, comes out to 8192lsb/g.
        Ok(total as f64 * LSB_TO_G)
    }

    fn switch_bank(&mut self, new_bank: Bank) -> Result<(), E> {
        self.i2c.write(BankSelect::ADDR, &[new_bank.value()])?;
        self.current_bank = new_bank;
        Ok(())
    }

    fn read(&mut self, bank: Bank, address: u8) -> Result<u8, E> {
        if bank != self.current_bank {
            self.switch_bank(bank)?;
        }

        let mut buf = [0];
        self.i2c.read(address, &mut buf)?;
        Ok(buf[0])
    }

    fn write(&mut self, bank: Bank, address: u8, value: u8) -> Result<(), E> {
        if bank != self.current_bank {
            self.switch_bank(bank)?;
        }

        self.i2c.write(address, &[value])?;
        Ok(())
    }
}
