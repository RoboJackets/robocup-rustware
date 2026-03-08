//!
//! Tasks to read the battery voltage of the robot.
//! 
//! This module is also important for turning off the robot when the battery gets too low.
//! 

use embassy_stm32::{adc::{self, Adc}, peripherals::{ADC1, PB0}, Peri};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::Publisher};

/// The frequency to check the battery voltage (in Hz)
pub const BATTERY_CHECK_FREQUENCY_HZ: u64 = 3;

/// The Minimum safe battery voltage (in volts)
pub const MIN_BATTERY_VOLTAGE: f32 = 2.8;
/// The resolution of the ADC
pub const ADC_RESOLUTION: f32 = 65535.0;
/// The reference voltage for the ADC (in volts)
pub const ADC_REFERENCE_VOLTAGE: f32 = 3.3;

#[embassy_executor::task]
pub async fn monitor_battery_voltage(
    mut adc: Adc<'static, ADC1>,
    mut battery_pin: Peri<'static, PB0>,
    battery_voltage_publisher: Publisher<'static, NoopRawMutex, f32, 4, 2, 1>,
    power_off_publisher: Publisher<'static, NoopRawMutex, (), 4, 2, 2>,
) {
    // The voltage window applies a moving average to our voltage readings to filter noise
    let mut voltage_window = [ADC_REFERENCE_VOLTAGE; 5];
    let mut voltage_idx = 0;

    loop {
        // Read from the ADC and update the voltage window
        let reading = adc.blocking_read(&mut battery_pin, adc::SampleTime::CYCLES32_5);
        let voltage = (reading as f32 / ADC_RESOLUTION) * ADC_REFERENCE_VOLTAGE;
        voltage_window[voltage_idx] = voltage;
        voltage_idx = (voltage_idx + 1) % voltage_window.len();
        
        // Publish the average voltage
        battery_voltage_publisher.publish_immediate(voltage_window.iter().sum::<f32>() / voltage_window.len() as f32);

        // Turn off the robot if our voltage dips below the min battery voltage threshold
        if voltage_window.iter().all(|&v| v < MIN_BATTERY_VOLTAGE) {
            power_off_publisher.publish_immediate(());
        }

        // Wait to check the battery voltage again
        embassy_time::Timer::after(embassy_time::Duration::from_hz(BATTERY_CHECK_FREQUENCY_HZ)).await;
    }
}