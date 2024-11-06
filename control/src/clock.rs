//!
//! Clock Constants and Configuration
//!

use teensy4_bsp::{board::PERCLK_FREQUENCY, hal::gpt::ClockSource};


/// Frequency of the Motion Control Refernce Clock
pub const GPT_1_FREQUENCY: u32 = 1_000_000;
/// Divider for the Motion Control Reference Clock
pub const GPT_1_DIVIDER: u32 = PERCLK_FREQUENCY / GPT_1_FREQUENCY;
/// Frequency of the GPT Clocks
pub const GPT_FREQUENCY: u32 = 1_000;
/// Reference clock for the GPT clocks
pub const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
/// Divider for the GPT clocks
pub const GPT_DIVIDER: u32 = PERCLK_FREQUENCY / GPT_FREQUENCY;
