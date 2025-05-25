//!
//! Library of Constants and Definitions to make it
//! a bit easier to tweak the performance of the robots
//!

#![no_std]
#![feature(type_alias_impl_trait)]

extern crate alloc;

pub mod radio;
pub use radio::*;

pub mod clock;
pub use clock::*;

pub mod robot;
pub use robot::robot_config::*;

pub mod peripherals;
pub use peripherals::*;

pub mod errors;
pub use errors::*;

pub mod spi;

/// The current state of the program.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Default Operation
    Default,
    /// Idling
    Idle,
    /// Testing the IMU
    IMUTesting,
    /// Benchmarking the radio receive
    ReceiveBenchmark,
    /// Benchmarking the radio sending
    SendBenchmark,
    /// Programming the kicker with kick-on-breakbeam
    ProgramKickOnBreakbeam,
    /// Programming the kicker with normal operations
    ProgramKicker,
    /// Testing the Kicker
    KickerTesting,
}

impl Default for State {
    fn default() -> Self {
        Self::Default
    }
}
