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
