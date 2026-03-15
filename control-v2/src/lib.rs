#![no_std]

extern crate alloc;

use defmt::Format;

/// Battery Related Functionality
pub mod battery;
/// The Controller for the robot
pub mod control;
/// Gpio Handler Logic
pub mod gpio;
/// Graphics Related Functionality
pub mod graphics;
/// IMU Related Functionality
pub mod imu;
/// Kicker Related Functionality
pub mod kicker;
/// Motor Related Functionality
pub mod motor;
/// Radio Related Functionality
pub mod radio;
/// Helpful Testing Utilities
pub mod utils;

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
