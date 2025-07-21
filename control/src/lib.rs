//!
//! Library of Constants and Definitions to make it
//! a bit easier to tweak the performance of the robots
//!

#![no_std]
#![feature(type_alias_impl_trait)]

extern crate alloc;

pub mod radio;
use imxrt_hal::lpuart::{self, Lpuart};
pub use radio::*;

pub mod clock;
pub use clock::*;

pub mod robot;
pub use robot::robot_config::*;

pub mod peripherals;
pub use peripherals::*;

pub mod errors;
pub use errors::*;

pub mod motors;

use rtic::Mutex;
use rtic_sync::channel::Receiver;

pub mod spi;

/// At 2.372V, our batteries have depleted to 18.5V (see voltage divider in schematics)
pub const MIN_BATTERY_VOLTAGE: f32 = 2.384615;

/// The current state of the program.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Default Operation
    Default,
    /// Idling
    Idle,
    /// Testing the IMU
    IMUTesting,
    /// Benchmarking the radio receive
    ReceiveBenchmark,
    /// Benchmarking the radio sending
    SendBenchmark,
    /// Programming the kicker with kick-on-breakbeam
    ProgramKickOnBreakbeam,
    /// Programming the kicker with normal operations
    ProgramKicker,
    /// Testing the Kicker
    KickerTesting,
}

impl Default for State {
    fn default() -> Self {
        Self::Default
    }
}

/// Interrupt Handler for the Motor Uarts
#[inline]
pub fn motor_interrupt<
    UART: Mutex<T = Lpuart<PINS, INTERFACE>>,
    VELOCITY: Mutex<T = i32>,
    PINS,
    const INTERFACE: u8,
>(
    mut uart: UART,
    mut velocity: VELOCITY,
    motor_rx: &mut Receiver<'static, [u8; 4], 3>,
    idx: &mut usize,
    reading: &mut bool,
    buffer: &mut [u8; 4],
) {
    if let Ok(setpoint) = motor_rx.try_recv() {
        *buffer = setpoint;
        *idx = 0;
        *reading = false;
    }

    if *reading {
        uart.lock(|uart| {
            let data = uart.read_data();
            buffer[*idx] = data.into();
            uart.clear_status(lpuart::Status::W1C);
            *idx += 1;
            if *idx == buffer.len() {
                *idx = 0;
                *reading = false;
                velocity.lock(|velocity| *velocity = i32::from_le_bytes(*buffer));
            }
        });
    } else {
        if *idx == buffer.len() {
            *idx = 0;
            *reading = true;
            uart.lock(|uart| {
                uart.clear_status(lpuart::Status::W1C);
                uart.disable(|uart| uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL));
            });
        } else {
            uart.lock(|uart| {
                uart.write_byte(buffer[*idx]);
                uart.clear_status(lpuart::Status::W1C);
                *idx += 1;
            });
        }
    }
}
