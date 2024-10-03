//!
//! Radio Constants and Configuration Information
//!

use rtic_nrf24l01::config::power_amplifier::PowerAmplifier;

/// Radio Channel (f_(MHz) = 2400 + CHANNEL)
pub const CHANNEL: u8 = 106;
/// Base Radio Amplification Level
pub const BASE_AMPLIFICATION_LEVEL: PowerAmplifier = PowerAmplifier::PAMax;
