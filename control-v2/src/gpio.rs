//!
//! Logic related to GPIO handling (specifically for the dip switches)
//! 

use embassy_stm32::{exti::ExtiInput, gpio::Output};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::Publisher};

use crate::Team;

/// Determine the id of the robot based on the state of the dip switches
pub fn decode_robot_id(s1: bool, s2: bool, s3: bool) -> u8 {
    match (s1, s2, s3) {
        (false, false, false) => 0,
        (false, false, true) => 1,
        (false, true, false) => 2,
        (false, true, true) => 3,
        (true, false, false) => 4,
        (true, false, true) => 5,
        (true, true, false) => 6,
        (true, true, true) => 7,
    }
}

/// Determine the team of the robot based on the state of the dip switch 4
pub fn decode_team(s4: bool) -> Team {
    if s4 {
        Team::Yellow
    } else {
        Team::Blue
    }
}

/// The amount of time to wait before turning off the robot's power.
/// 
/// This is important so we discharge the kicker before we turn off the robot
pub const POWER_OFF_DELAY_MS: u64 = 1000;

#[embassy_executor::task]
pub async fn power_switch(
    mut irq: ExtiInput<'static>,
    power_off_publisher: Publisher<'static, NoopRawMutex, (), 4, 2, 2>,
    mut kill_n: Output<'static>,
) {
    irq.wait_for_low().await;
    power_off_publisher.publish_immediate(());

    embassy_time::Timer::after(embassy_time::Duration::from_millis(POWER_OFF_DELAY_MS)).await;
    kill_n.set_low();
}
