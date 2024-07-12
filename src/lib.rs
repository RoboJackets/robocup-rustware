#![no_std]
#![feature(type_alias_impl_trait)]

extern crate alloc;

pub mod motion_control;
pub mod collect;

#[cfg(not(any(
    feature = "robot-0",
    feature = "robot-1",
    feature = "robot-2",
    feature = "robot-3",
    feature = "robot-4",
    feature = "robot-5",
)))]
pub const ROBOT_ID: u8 = 0;

#[cfg(feature = "robot-0")]
pub const ROBOT_ID: u8 = 0;
#[cfg(feature = "robot-1")]
pub const ROBOT_ID: u8 = 1;
#[cfg(feature = "robot-2")]
pub const ROBOT_ID: u8 = 2;
#[cfg(feature = "robot-3")]
pub const ROBOT_ID: u8 = 3;
#[cfg(feature = "robot-4")]
pub const ROBOT_ID: u8 = 4;
#[cfg(feature = "robot-5")]
pub const ROBOT_ID: u8 = 5;


// Clock Parameters
use teensy4_bsp::hal::gpt::ClockSource;
use teensy4_bsp::board::PERCLK_FREQUENCY;

/// Frequency of the GPT Clocks
pub const GPT_FREQUENCY: u32 = 1_000;
/// Reference clock for the GPT clocks
pub const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
/// Divider for the GPT clocks
pub const GPT_DIVIDER: u32 = PERCLK_FREQUENCY / GPT_FREQUENCY;