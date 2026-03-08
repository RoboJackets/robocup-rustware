#![no_std]

extern crate alloc;

use defmt::Format;

/// IMU Related Functionality
pub mod imu;
/// Graphics Related Functionality
pub mod graphics;
/// Radio Related Functionality
pub mod radio;
/// Gpio Handler Logic
pub mod gpio;
/// Motor Related Functionality
pub mod motor;
/// Kicker Related Functionality
pub mod kicker;
/// The Controller for the robot
pub mod control;
/// Battery Related Functionality
pub mod battery;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Format)]
/// The team the Robots are on
pub enum Team {
    /// Blue Team
    Blue = 0,
    /// Yellow Team
    Yellow = 1,
}

impl Into<bool> for Team {
    fn into(self) -> bool {
        match self {
            Team::Blue => false,
            Team::Yellow => true,
        }
    }
}