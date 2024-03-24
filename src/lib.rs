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