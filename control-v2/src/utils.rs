//!
//! Testing Utilities for the robot
//!

use common::{dribbler::DribblerCommand, motor::MotorCommand};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    pubsub::{Publisher, Subscriber, WaitResult},
};
use kicker_controller::KickerState;

use crate::radio::control_command::ControlMessage;

#[embassy_executor::task]
pub async fn radio_testing_task(
    mut command_subscriber: Subscriber<'static, NoopRawMutex, ControlMessage, 4, 2, 1>,
    kicker_status_publisher: Publisher<'static, NoopRawMutex, KickerState, 4, 2, 1>,
    battery_voltage_publisher: Publisher<'static, NoopRawMutex, f32, 4, 2, 1>,
) {
    let mut current_voltage = 12.5;
    let mut ball_sensed = true;
    let mut healthy = false;

    loop {
        // Wait for a command to be received
        if let WaitResult::Message(_) = command_subscriber.next_message().await {
            defmt::info!("Received Command");

            // Publish a test kicker state
            current_voltage += 0.1;
            ball_sensed = !ball_sensed;
            healthy = !healthy;
            let kicker_state = KickerState {
                current_voltage: current_voltage as u8,
                ball_sensed: ball_sensed,
                healthy: healthy,
            };
            kicker_status_publisher.publish_immediate(kicker_state);

            // Publish a test battery voltage
            battery_voltage_publisher.publish_immediate(current_voltage);
        }
    }
}

const TICKS_PER_SECOND: i32 = 6000;

#[embassy_executor::task]
pub async fn motor_testing_task(
    motor_command_publishers: [Publisher<'static, NoopRawMutex, MotorCommand, 4, 1, 1>; 4],
    dribbler_publisher: Publisher<'static, NoopRawMutex, DribblerCommand, 4, 1, 1>,
) {
    let mut current_speed = TICKS_PER_SECOND;
    let mut dribble_on = false;
    loop {
        for i in 0..4 {
            motor_command_publishers[i].publish_immediate(MotorCommand::Move {
                ticks_per_second: current_speed,
            });

            dribbler_publisher.publish_immediate(DribblerCommand::Move {
                percent: if dribble_on { 100 } else { 0 },
            })
        }

        current_speed = (current_speed + TICKS_PER_SECOND) % (2 * TICKS_PER_SECOND);
        dribble_on = !dribble_on;

        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
    }
}
