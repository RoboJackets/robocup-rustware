#![no_std]
#![feature(type_alias_impl_trait)]

#[macro_use]
extern crate alloc;

pub mod motion_control;

pub const ROBOT_RADIO_ADDRESSES: [[u8; 5]; 6] = [
    [0xC3, 0xC3, 0xC3, 0xC3, 0xC1],
    [0xC3, 0xC3, 0xC3, 0xC3, 0xC2],
    [0xC3, 0xC3, 0xC3, 0xC3, 0xC3],
    [0xC3, 0xC3, 0xC3, 0xC3, 0xC4],
    [0xC3, 0xC3, 0xC3, 0xC3, 0xC5],
    [0xC3, 0xC3, 0xC3, 0xC3, 0xC6]
];

pub const BASE_STATION_ADDRESS: [u8; 5] = [0xE7, 0xE7, 0xE7, 0xE7, 0xE7];

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