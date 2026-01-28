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
use kfilter::kalman::{Kalman1M, KalmanFilter, KalmanPredict};
use kfilter::measurement::LinearMeasurement;
use kfilter::system::LinearNoInputSystem;
use nalgebra::{Matrix1, Matrix1x2, Matrix2, UnitQuaternion, Vector2, Vector3};

mod registers;

const ICM_ADDR: u8 = 0b1101000;
const WHO_AM_I_EXPECTED: u8 = 0x42;
const LSB_TO_G: f32 = 16.0 / 32768.0;
const LSB_TO_DPS: f32 = 1000.0 / 32768.0;

const TICK_PERIOD: rtic_monotonics::systick::fugit::Duration<u32, 1, 1000> =
    <rtic_monotonics::systick::Systick as Monotonic>::TICK_PERIOD;
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

    /// Gyro Bias: Average noise XYZ when still
    gyro_bias: Vector3<f32>,

    /// Accel Rotation: Rotates sensor frame to robot frame (fix tilt)
    accel_correction: UnitQuaternion<f32>,

    /// Accel Scale: Scales gravity to exactly 1.0g
    accel_scale: f32,

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
            // Initialize Calibration to "Identity" (No effect)
            gyro_bias: Vector3::zeros(),
            accel_correction: UnitQuaternion::identity(),
            accel_scale: 1.0,
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
        //MAKE SURE IMU IS STILL during this time
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

        let dt_ticks: f32 = (now_tick - self.last_update_gz.unwrap_or(now_tick)) as f32; // Convert milliseconds to seconds
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
        let corrected_vec = self.get_corrected_gyro_vector()?;
        kf.update(Matrix1::new(corrected_vec.z));

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
        let corrected_vec = self.get_corrected_accel_vector()?;
        kf.update(Matrix1::new(corrected_vec.x));

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
        let dt_ticks: f32 = (now_tick - self.last_update_ay.unwrap_or(now_tick)) as f32;
        let dt: f32 = dt_ticks * SECONDS_PER_TICK;

        let f = Matrix2::new(1.0, dt, 0.0, 1.0);
        let q = Matrix2::new(0.1, 0.0, 0.0, 1.0);
        let h = Matrix1x2::new(1.0, 0.0);
        let r = Matrix1::new(1.0);

        let system: LinearNoInputSystem<f32, 2> = LinearNoInputSystem::new(f, q, self.x_ay);

        let measurement_handler = LinearMeasurement::new(h, r, Matrix1::zeros());
        let mut kf = Kalman1M::new_custom(system, self.p_ay, measurement_handler);

        kf.predict();
        let corrected_vec = self.get_corrected_accel_vector()?;
        kf.update(Matrix1::new(corrected_vec.y));

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

    /// Without the Kalman Filter, read the gyro velocity in the x direction
    fn raw_gyro_x(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized {
            return Err(ImuError::Uninitialized);
        }

        let hi = self.read(Bank::Bank0, registers::GYRO_DATA_X1)?;
        let lo = self.read(Bank::Bank0, registers::GYRO_DATA_X0)?;

        Ok(reading_to_gyro(hi, lo))
    }

    /// Without the Kalman Filter, read the gyro velocity in the x direction
    fn raw_gyro_y(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized {
            return Err(ImuError::Uninitialized);
        }

        let hi = self.read(Bank::Bank0, registers::GYRO_DATA_Y1)?;
        let lo = self.read(Bank::Bank0, registers::GYRO_DATA_Y0)?;

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

    /// Without the Kalman Filter, read the acceleration in the z direction. This is gravity!
    fn raw_accel_z(&mut self) -> Result<f32, ImuError<E>> {
        if !self.initialized {
            return Err(ImuError::Uninitialized);
        }

        let hi = self.read(Bank::Bank0, registers::ACCEL_DATA_Z1)?;
        let lo = self.read(Bank::Bank0, registers::ACCEL_DATA_Z0)?;

        Ok(reading_to_accel(hi, lo))
    }

    /// Calibrate the offsets for the gyro and accelerometer
    /// IMU axes are adjusted to robot frame
    /// MAKE SURE IMU IS STILL during this time
    fn calibrate_offsets(&mut self, delay: &mut impl DelayMs<u8>, cal_time_ms: u32) {
        let t0 = Systick::now().ticks();
        let mut count: i64 = 0;

        let mut sum_gyro = Vector3::zeros();
        let mut sum_accel = Vector3::zeros();

        let max_ticks = (cal_time_ms as f32 / 1000.0 / SECONDS_PER_TICK) as u32;

        while (Systick::now().ticks().wrapping_sub(t0)) < max_ticks {
            // Read raw axes
            let gx = self.raw_gyro_x().unwrap_or(0.0);
            let gy = self.raw_gyro_y().unwrap_or(0.0);
            let gz = self.raw_gyro_z().unwrap_or(0.0);

            let ax = self.raw_accel_x().unwrap_or(0.0);
            let ay = self.raw_accel_y().unwrap_or(0.0);
            let az = self.raw_accel_z().unwrap_or(0.0);

            sum_gyro += Vector3::new(gx, gy, gz);
            sum_accel += Vector3::new(ax, ay, az);

            count += 1;
            delay.delay_ms(5);
        }

        if count == 0 {
            return;
        }

        // GYRO BIAS (Average noise)
        self.gyro_bias = sum_gyro / (count as f32);

        // ACCEL CALIBRATION (Rotation & Scaling)
        let avg_accel = sum_accel / (count as f32);

        // Calculate Scaling: Force magnitude to be exactly 1.0
        let current_magnitude = avg_accel.norm();
        if current_magnitude > 0.001 {
            self.accel_scale = 1.0 / current_magnitude;
        }

        // Calculate Rotation: Rotate measured direction to point UP/DOWN (Z-axis)
        // Assuming +Z is "Up/Down" relative to the robot chassis.
        let measured_direction = avg_accel.normalize();
        let target_direction = Vector3::z_axis(); // Points to +Z (0, 0, 1)

        // This creates a rotation that aligns "Measured" -> "Target"
        self.accel_correction =
            UnitQuaternion::rotation_between(&measured_direction, &target_direction)
                .unwrap_or(UnitQuaternion::identity());
    }

    /// Reads raw Accel, applies Scaling, then Rotation
    fn get_corrected_accel_vector(&mut self) -> Result<Vector3<f32>, ImuError<E>> {
        let ax = self.raw_accel_x()?;
        let ay = self.raw_accel_y()?;
        let az = self.raw_accel_z()?;

        let raw = Vector3::new(ax, ay, az);

        // Scale (fix magnitude error)
        let scaled = raw * self.accel_scale;

        // Rotate (fix mounting tilt)
        let rotated = self.accel_correction * scaled;

        Ok(rotated)
    }

    /// Reads raw Gyro, subtracts Bias, then applies Rotation
    fn get_corrected_gyro_vector(&mut self) -> Result<Vector3<f32>, ImuError<E>> {
        let gx = self.raw_gyro_x()?;
        let gy = self.raw_gyro_y()?;
        let gz = self.raw_gyro_z()?;

        let raw = Vector3::new(gx, gy, gz);

        // Remove Bias
        let unbiased = raw - self.gyro_bias;

        // Rotate (Align to robot frame)
        let rotated = self.accel_correction * unbiased;

        Ok(rotated)
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
