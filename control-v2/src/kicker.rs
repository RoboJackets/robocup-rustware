//!
//! Wrapper for tasks related to the kicker
//! 

use core::cell::RefCell;
use embassy_sync::{blocking_mutex::{NoopMutex, raw::NoopRawMutex}, pubsub::{Publisher, Subscriber, WaitResult}};
use embassy_stm32::{mode::Blocking, spi, gpio};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use static_cell::StaticCell;
use kicker_controller::{Kicker, KickerCommand, KickerState, KickType, KickTrigger};

use crate::radio::control_command::ControlMessage;

/// The Kicker SPI Bus
pub static KICKER_SPI: StaticCell<NoopMutex<RefCell<spi::Spi<'static, Blocking, spi::mode::Master>>>> = StaticCell::new();

/// The frequency to service the kicker at
pub const KICKER_SERVICE_FREQUENCY_HZ: u64 = 10;

#[embassy_executor::task]
pub async fn service_kicker(
    mut kicker: Kicker<gpio::Output<'static>, SpiDevice<'static, NoopRawMutex, spi::Spi<'static, embassy_stm32::mode::Blocking, spi::mode::Master>, gpio::Output<'static>>>,
    mut command_receiver: Subscriber<'static, NoopRawMutex, ControlMessage, 4, 2, 1>,
    kicker_status_publisher: Publisher<'static, NoopRawMutex, KickerState, 4, 2, 1>,
    mut power_off_subscriber: Subscriber<'static, NoopRawMutex, (), 4, 2, 2>,
) {
    let mut kicker_command = KickerCommand {
        kick_type: KickType::Kick,
        kick_trigger: KickTrigger::Disabled,
        kick_strength: 0.0,
        charge_allowed: false,
    };
    let mut power_off = false;

    loop {
        // Check if we have low battery and should turn off the kicker
        if let Some(WaitResult::Message(())) = power_off_subscriber.try_next_message() {
            power_off = true;
        }
        if !power_off {
            // Check for a new kicker command
            if let Some(command) = command_receiver.try_next_message() {
                if let WaitResult::Message(command) = command {
                    kicker_command = command.into();
                }
            }

            // Service the kicker and publish the updated state
            if let Ok(kicker_state) = kicker.service(kicker_command.clone()) {
                kicker_status_publisher.publish_immediate(kicker_state);
            }
        } else {
            for _ in 0..5 {
                // If we have low battery kick the kicker
                if let Ok(kicker_state) = kicker.service(KickerCommand {
                    kick_type: KickType::Kick,
                    kick_trigger: KickTrigger::Immediate,
                    kick_strength: 1.0,
                    charge_allowed: false,
                }) {
                    kicker_status_publisher.publish_immediate(kicker_state);
                }

                embassy_time::Timer::after(embassy_time::Duration::from_millis(333)).await;
            }
        }

        // Sleep until it's time to service the kicker again
        embassy_time::Timer::after(embassy_time::Duration::from_hz(KICKER_SERVICE_FREQUENCY_HZ)).await;
    }
}