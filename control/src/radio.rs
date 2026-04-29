//!
//! Radio Constants and Configuration Information
//!

use rtic_nrf24l01::config::power_amplifier::PowerAmplifier;

pub mod addresses;
pub mod control_message;
pub mod robot_status;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// The Team the robots are on
pub enum Team {
    /// The Blue Team
    Blue = 0,
    /// The Yellow Team
    Yellow = 1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Error from attempting to pack a message
pub enum PackingError {
    /// The buffer provided was too small
    InvalidBufferSize,
}

/// Radio Channel (f_(MHz) = 2400 + CHANNEL)
pub const CHANNEL: u8 = 106;
/// Base Radio Amplification Level
pub const BASE_AMPLIFICATION_LEVEL: PowerAmplifier = PowerAmplifier::PALow;
