//!
//! IMU-Related Tasks for the 2nd edition of the robojackets control
//! board
//! 

use core::cell::RefCell;
use embassy_stm32::{i2c::{self, I2c}, mode::Async, bind_interrupts, peripherals};
use embassy_sync::{blocking_mutex::{NoopMutex, raw::NoopRawMutex}, pubsub::Publisher};
use static_cell::StaticCell;
use icm42605::Imu;

bind_interrupts!(pub struct ImuIrqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

/// The IMU Connected to the robot
pub static IMU: StaticCell<NoopMutex<RefCell<Imu<I2c<'static, Async, i2c::Master>>>>> = StaticCell::new();

/// For now we will read from the IMU at 100 Hz
pub const IMU_READ_FREQUENCY_HZ: u64 = 100;

/// Data published by the IMU
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub gyro_z: f32,
    pub accel_x: f32,
    pub accel_y: f32,
}

#[embassy_executor::task]
pub async fn read_imu_data(
    imu: &'static mut NoopMutex<RefCell<Imu<I2c<'static, Async, i2c::Master>>>>,
    data_publisher: Publisher<'static, NoopRawMutex, ImuData, 4, 1, 1>
) {
    loop {
        let imu = imu.get_mut();

        let gyro_z = imu.borrow_mut().gyro_z().unwrap_or(0.0);
        let accel_x = imu.borrow_mut().accel_x().unwrap_or(0.0);
        let accel_y = imu.borrow_mut().accel_y().unwrap_or(0.0);

        // Publish the imu data
        data_publisher.publish_immediate(ImuData { gyro_z, accel_x, accel_y });

        // Add a delay to control the read frequency (e.g., 100ms)
        embassy_time::Timer::after(embassy_time::Duration::from_hz(IMU_READ_FREQUENCY_HZ)).await;
    }
}