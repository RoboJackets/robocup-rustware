//!
//! Radio Tasks
//!

use alloc::format;
use core::cell::RefCell;
use defmt::error;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::{exti::ExtiInput, gpio::Output, mode::Async, spi};
use embassy_sync::{
    blocking_mutex::{NoopMutex, raw::NoopRawMutex},
    pubsub::{Publisher, Subscriber, WaitResult},
};
use embassy_time::{Delay, Duration, Timer};
use kicker_controller::KickerState;
use rf24::radio::{RF24, prelude::*};
use ssd1306::{
    Ssd1306, mode::BufferedGraphicsMode, prelude::I2CInterface, size::DisplaySize128x64,
};
use static_cell::StaticCell;

use control_command::{CONTROL_MESSAGE_SIZE, ControlMessage};
use robot_status::{ROBOT_STATUS_SIZE, RobotStatusMessageBuilder};

use crate::{Team, graphics};
use common::packing::Packable;

/// The addresses for the radio communication
pub mod addresses;
/// The control command message
pub mod control_command;
/// The robot status message
pub mod robot_status;

/// The channel for the radio
pub const CHANNEL: u8 = 0x74;
/// The PA Level of the radio
pub const PA_LEVEL: rf24::PaLevel = rf24::PaLevel::Min;
/// The amount of time to wait before sending a response to a control message (in ms)
///
/// From my calculations, at 60Hz the time between the base station sending the last packet
/// and receiving the last message from the robots is around 16.667 milliseconds.  From this,
/// I figure we can give half of the time to the base station transmitting and the other
/// half to the robot responding, so each robot can wait 8 milliseconds from receiving a
/// command before they should respond
pub const RESPONSE_DELAY_MS: u64 = 8;

/// The SPI Bus
pub static RADIO_SPI: StaticCell<NoopMutex<RefCell<spi::Spi<'static, Async, spi::mode::Master>>>> =
    StaticCell::new();

/// Helper method to initialize the radio and handle errors
pub fn init_radio(
    radio: &mut RF24<
        SpiDevice<
            'static,
            NoopRawMutex,
            spi::Spi<'static, Async, spi::mode::Master>,
            Output<'static>,
        >,
        Output<'static>,
        Delay,
    >,
    display: &mut Ssd1306<
        I2CInterface<I2c<'static, Async, i2c::Master>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
    robot_id: u8,
    team: Team,
) {
    let radio_id = addresses::ROBOT_ADDRESSES[team as usize][robot_id as usize];
    let base_station_id = addresses::BASE_STATION_ADDRESSES[team as usize][(robot_id > 2) as usize];

    // Initialize the radio
    if let Err(err) = radio.init() {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("Radio Init: {:?}", err));

        error!("Error Initializing Radio");
    }

    // Set the PA Level
    if let Err(err) = radio.set_pa_level(PA_LEVEL) {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("Radio PA: {:?}", err));

        error!("Error Setting Radio PA Level");
    }

    if let Err(err) = radio.set_auto_retries(0, 0) {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("Auto Retries: {:?}", err));

        error!("Error setting auto retries");
    }

    if let Err(err) = radio.set_auto_ack(false) {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("Auto Ack: {:?}", err));

        error!("Error setting auto ack");
    }

    if let Err(err) = radio.set_crc_length(rf24::CrcLength::Bit8) {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("CRC Length: {:?}", err));

        error!("Error setting CRC length");
    }

    // Set the data rate
    if let Err(err) = radio.set_data_rate(rf24::DataRate::Mbps1) {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("Radio Data Rate: {:?}", err));

        error!("Error Setting Radio Data Rate");
    }

    // Set the radio channel
    if let Err(err) = radio.set_channel(CHANNEL) {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("Radio Channel: {:?}", err));

        error!("Error Setting Radio Channel");
    }

    // Set the TX address of the radio
    if let Err(err) = radio.as_tx(Some(&base_station_id)) {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("Radio TX Addr: {:?}", err));

        error!("Error Setting Radio TX Address");
    }

    // Set the RX address of the radio
    if let Err(err) = radio.open_rx_pipe(1, &radio_id) {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("Radio RX Addr: {:?}", err));

        error!("Error Setting Radio RX Address");
    }

    // Enable Interrupts
    if let Err(err) = radio.set_status_flags(
        rf24::StatusFlags::new()
            .with_rx_dr(true)
            .with_tx_ds(false)
            .with_tx_df(false),
    ) {
        // Display Error
        let _ = graphics::draw_error_screen(display, &format!("Radio Status Flags: {:?}", err));

        error!("Error Setting Radio Status Flags");
    }
}

#[embassy_executor::task]
pub async fn receive_radio_data(
    mut irq: ExtiInput<'static>,
    mut radio: RF24<
        SpiDevice<
            'static,
            NoopRawMutex,
            spi::Spi<'static, Async, spi::mode::Master>,
            Output<'static>,
        >,
        Output<'static>,
        Delay,
    >,
    robot_id: u8,
    team: Team,
    command_channel: Publisher<'static, NoopRawMutex, ControlMessage, 4, 2, 1>,
    mut kicker_status_subscriber: Subscriber<'static, NoopRawMutex, KickerState, 4, 2, 1>,
    mut battery_voltage_subscriber: Subscriber<'static, NoopRawMutex, f32, 4, 2, 1>,
) {
    let mut buffer = [0u8; CONTROL_MESSAGE_SIZE];
    let mut response_buffer = [0u8; ROBOT_STATUS_SIZE];
    let mut status = RobotStatusMessageBuilder::new()
        .robot_id(robot_id)
        .team(team)
        .build();
    let radio_id = addresses::ROBOT_ADDRESSES[team as usize][robot_id as usize];
    let base_station_id = addresses::BASE_STATION_ADDRESSES[team as usize][(robot_id > 2) as usize];
    loop {
        irq.wait_for_falling_edge().await;

        // Read data from the radio and do something with it
        if let Ok(n) = radio.read(&mut buffer, Some(CONTROL_MESSAGE_SIZE as u8)) {
            if n == CONTROL_MESSAGE_SIZE as u8 {
                if let Ok(control_message) = ControlMessage::unpack(&buffer[..]) {
                    // Check if the message is for us
                    if control_message.robot_id == robot_id && control_message.team == team {
                        // Publish the command to the channel
                        command_channel.publish_immediate(control_message);
                    }
                }
            }
        }

        // Check for a new kicker status message
        if let Some(WaitResult::Message(state)) = kicker_status_subscriber.try_next_message() {
            status.ball_sense_status = state.ball_sensed;
            status.kick_healthy = state.healthy;
            status.kick_status = true;
        }

        // Check for a new battery voltage message
        if let Some(WaitResult::Message(battery_voltage)) =
            battery_voltage_subscriber.try_next_message()
        {
            // Kind of an approximation as a percentage of 3.7V (the nominal voltage of a single cell)
            status.battery_voltage = ((battery_voltage / 3.7) * 100.0) as u8;
        }
        status.pack(&mut response_buffer).unwrap();

        // Wait to send response
        Timer::after(Duration::from_millis(RESPONSE_DELAY_MS)).await;

        // Response with status data
        radio.as_tx(Some(&base_station_id)).unwrap();
        radio.send(&response_buffer, true).unwrap();
        radio.as_tx(None).unwrap();
        radio.open_rx_pipe(0, &radio_id).unwrap();
        radio.as_rx().unwrap();
    }
}
