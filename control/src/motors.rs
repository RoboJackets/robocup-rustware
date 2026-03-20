//!
//! Helper functions for working with the motor board
//!

use imxrt_hal::lpuart::{self, Lpuart};
use rtic::Mutex;
use rtic_sync::channel::{Receiver, Sender};

#[inline]
/// Send a command over a motor's uart
pub fn send_command<PINS, const INTERFACE: u8>(
    setpoint: i32,
    tx: &mut Sender<'static, [u8; 4], 3>,
    uart: &mut Lpuart<PINS, INTERFACE>,
    motor: u8,
) {
    let mut command = [0u8; 4];
    command[..].copy_from_slice(&setpoint.to_le_bytes());
    let _ = tx.try_send(command);
    uart.disable(|uart| uart.set_interrupts(lpuart::Interrupts::TRANSMIT_COMPLETE));
    uart.clear_status(lpuart::Status::W1C);
    match motor {
        0 => uart.write_byte(0x11),
        1 => uart.write_byte(0x22),
        2 => uart.write_byte(0x33),
        3 => uart.write_byte(0x44),
        4 => uart.write_byte(0x55),
        _ => (),
    }
}

#[inline]
/// Interrupt Handler for the Motor Uarts
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
