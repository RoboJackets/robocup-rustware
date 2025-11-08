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

use rtic_monotonics::{systick::*, Monotonic};

//Kalman filter libraries
use kfilter::kalman::{Kalman1M, KalmanFilter, KalmanPredict, KalmanUpdate};
use kfilter::measurement::LinearMeasurement;
use kfilter::system::LinearNoInputSystem;
use kfilter::Kalman1MLinearNoInput;
use nalgebra::{Matrix1, Matrix1x2, Matrix2, Matrix2x1, SMatrix, Vector2, SVector};

// The correct type alias for a filter with a 2-dimensional state and f32 numbers.
type ImuKalmanFilter = Kalman1MLinearNoInput<f32, 2, 1>;

mod registers;

const ICM_ADDR: u8 = 0b1101000;
const WHO_AM_I_EXPECTED: u8 = 0x42;
const LSB_TO_G: f32 = 16.0 / 32768.0;

const LSB_TO_DPS: f32 = 1000.0 / 32768.0;

const TICK_PERIOD: rtic_monotonics::systick::fugit::Duration<u32, 1, 1000> = <rtic_monotonics::systick::Systick as Monotonic>::TICK_PERIOD;
const SECONDS_PER_TICK: f32 = TICK_PERIOD.to_micros() as f32 / 1_000_000.0; 

/// Convert the high and low bits obtained from the IMU into a gyrometer
/// reading (in degrees per second).
#[inline]
pub fn reading_to_gyro(high: u8, low: u8) -> f32 {
    let value = i16::from_be_bytes([high, low]);
    ((value as f32) * LSB_TO_DPS).to_radians()
}
#[allow(missing_docs)]
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

    //Offsets determined during calibration
    offset_gz: f32,
    offset_ax: f32,
    offset_ay: f32,

    /// Kalman filter state
    /// State for the gyro Z-axis filter
    x_gz: Vector2<f32>,
    p_gz: Matrix2<f32>,

    /// State for the accel X-axis filter
    x_ax: Vector2<f32>,
    p_ax: Matrix2<f32>,

    /// State for the accel Y-axis filter
    x_ay: Vector2<f32>,
    p_ay: Matrix2<f32>, 

    //Timstamp of last update. For dt calculation
    /// Timestamp of the last gyro Z update.
    last_update_gz: Option<u32>,
    /// Timestamp of the last accel X update.
    last_update_ax: Option<u32>,
    /// Timestamp of the last accel Y update.
    last_update_ay: Option<u32>,

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
            offset_ax: 0.0,
            offset_ay: 0.0,
            offset_gz: 0.0,
            //Kalman filter initializations
            x_gz: Vector2::zeros(),
            p_gz: Matrix2::identity(),
            x_ax: Vector2::zeros(),
            p_ax: Matrix2::identity(),
            x_ay: Vector2::zeros(),
            p_ay: Matrix2::identity(),
            last_update_gz: None,
            last_update_ax: None,
            last_update_ay: None,            
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

        self.initialized = true;

        //1 second calibration to determine offsets
        //MAKE SURE IMU IS STILL/FLAT during this time
        self.calibrate_offsets(delay, 1000);

        //Initialize KALMAN FILTERS

        let initial_covariance = Matrix2::identity() * 1000.0; // High initial uncertainty

        self.x_gz = Vector2::zeros();
        self.p_gz = initial_covariance;

        self.x_ax = Vector2::zeros();
        self.p_ax = initial_covariance;

        self.x_ay = Vector2::zeros();
        self.p_ay = initial_covariance;

        self.initialized = true;

        Ok(())
    }

    /// Read the gyro velocity in the z direction
    pub fn gyro_z(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized {
            return Err(ImuError::Uninitialized);
        }

        // CALCULATE DT
        let now_tick = Systick::now().ticks();

        
        let dt_ticks: f32 =  (now_tick - self.last_update_gz.unwrap_or(now_tick)) as f32; // Convert milliseconds to seconds
        let dt: f32 = dt_ticks * SECONDS_PER_TICK;

        // CREATE MATRICES FOR THIS STEP
        let f = Matrix2::new(1.0, dt, 0.0, 1.0);
        let q = Matrix2::new(0.1, 0.0, 0.0, 1.0);
        let h = Matrix1x2::new(1.0, 0.0);
        let r = Matrix1::new(1.0);

        // CREATE THE FILTER COMPONENTS
        // We create the system object, passing in the F, Q, and last state `x`.
        let system: LinearNoInputSystem<f32, 2> = LinearNoInputSystem::new(f, q, self.x_gz);
        // We create the measurement handler object.
        let measurement_handler = LinearMeasurement::new(h, r, Matrix1::zeros());

        // CREATE A TEMPORARY FILTER
        // Use `new_custom` to build the filter from our components and the last covariance `P`.
        let mut kf = Kalman1M::new_custom(system, self.p_gz, measurement_handler);

        // RUN PREDICT AND UPDATE
        kf.predict();
        let raw_value = self.raw_gyro_z()? - self.offset_gz;
        kf.update(Matrix1::new(raw_value));

        // SAVE THE NEW STATE FOR THE NEXT CALL
        self.x_gz = *kf.state();
        self.p_gz = *kf.covariance();

        //update last update time
        self.last_update_gz = Some(now_tick);

        // RETURN THE FILTERED VALUE
        Ok(self.x_gz[0])
      }

    /// Read the acceleration in the x direction
    pub fn accel_x(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized {
            return Err(ImuError::Uninitialized);
        }

        let now_tick = Systick::now().ticks();
        let dt_ticks: f32 = (now_tick - self.last_update_ax.unwrap_or(now_tick)) as f32; // Convert milliseconds to seconds
        let dt: f32 = dt_ticks * SECONDS_PER_TICK;

        let f = Matrix2::new(1.0, dt, 0.0, 1.0);
        let q = Matrix2::new(0.1, 0.0, 0.0, 1.0);
        let h = Matrix1x2::new(1.0, 0.0);
        let r = Matrix1::new(1.0);

        let system: LinearNoInputSystem<f32, 2> = LinearNoInputSystem::new(f, q, self.x_ax);

        let measurement_handler = LinearMeasurement::new(h, r, Matrix1::zeros());

        let mut kf = Kalman1M::new_custom(system, self.p_ax, measurement_handler);

        kf.predict();
        let raw_value = self.raw_accel_x()? - self.offset_ax;
        kf.update(Matrix1::new(raw_value));

        self.x_ax = *kf.state();
        self.p_ax = *kf.covariance();

        //update last update time
        self.last_update_ax = Some(now_tick);

        Ok(self.x_ax[0])
        }

    /// Read the acceleration in the y direction
    pub fn accel_y(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized {
            return Err(ImuError::Uninitialized);
        }
        
        let now_tick = Systick::now().ticks();
        let dt_ticks: f32 = (now_tick - self.last_update_ay.unwrap_or(now_tick)) as f32; // Convert milliseconds to seconds
        let dt: f32 = dt_ticks * SECONDS_PER_TICK;

        let f = Matrix2::new(1.0, dt, 0.0, 1.0);
        let q = Matrix2::new(0.1, 0.0, 0.0, 1.0);
        let h = Matrix1x2::new(1.0, 0.0);
        let r = Matrix1::new(1.0);

        let system: LinearNoInputSystem<f32, 2> = LinearNoInputSystem::new(f, q, self.x_ay);

        let measurement_handler = LinearMeasurement::new(h, r, Matrix1::zeros());
        let mut kf = Kalman1M::new_custom(system, self.p_ay, measurement_handler);

        kf.predict();
        let raw_value = self.raw_accel_y()? - self.offset_ay;
        kf.update(Matrix1::new(raw_value));

        self.x_ay = *kf.state();
        self.p_ay = *kf.covariance();

        //update last update time
        self.last_update_ay = Some(now_tick);

        Ok(self.x_ay[0])
    }


    /// Write the raw register address and data over the i2c line
    fn raw_write(&mut self, register_addr: u8, data: u8) -> Result<(), ImuError<E>> {
        self.i2c
            .write(ICM_ADDR, &[register_addr, data])
            .map_err(ImuError::I2c)
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
        self.i2c
            .write(ICM_ADDR, &[address])
            .map_err(ImuError::I2c)?;

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

    
    /// Without the Kalman Filter, read the gyro velocity in the z direction
    fn raw_gyro_z(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized {
            return Err(ImuError::Uninitialized);
        }

        let hi = self.read(Bank::Bank0, registers::GYRO_DATA_Z1)?;
        let lo = self.read(Bank::Bank0, registers::GYRO_DATA_Z0)?;

        Ok(reading_to_gyro(hi, lo))
    }

    /// Without the Kalman Filter, read the acceleration in the x direction
    fn raw_accel_x(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized {
            return Err(ImuError::Uninitialized);
        }

        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_X1)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_X0)?;

        Ok(reading_to_accel(hi, lo))
    }

    /// Without the Kalman Filte,read the acceleration in the y direction
    fn raw_accel_y(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized {
            return Err(ImuError::Uninitialized);
        }

        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_Y1)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_Y0)?;

        Ok(reading_to_accel(hi, lo))
    }

    
    /// Calibrate the offsets for the gyro and accelerometer
    fn calibrate_offsets(&mut self, delay: &mut impl DelayMs<u8>, cal_time_ms: u32) {
        let t0 = Systick::now().ticks();
        let mut count: i64 = 0;
        let mut running_sum_gz: f32 = 0.0;
        let mut running_sum_ax: f32 = 0.0;
        let mut running_sum_ay: f32 = 0.0;

        while Systick::now().ticks() - t0 < cal_time_ms {
            // collect raw values for 1 second
            //MAKE SURE IMU IS STILL/FLAT during this time
            running_sum_gz += self.raw_gyro_z().unwrap_or_default();
            running_sum_ax += self.raw_accel_x().unwrap_or_default();
            running_sum_ay += self.raw_accel_y().unwrap_or_default();
            count += 1;
            delay.delay_ms(5); //maybe change this later? Seems to work fine
        }

        self.offset_gz = running_sum_gz / count as f32;
        self.offset_ax = running_sum_ax / count as f32;
        self.offset_ay = running_sum_ay / count as f32;
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
