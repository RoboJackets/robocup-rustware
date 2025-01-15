//!
//! Library for the motor-controller firmware
//! 

#![no_std]
#![feature(type_alias_impl_trait)]

use stm32f0xx_hal::gpio::{gpioa::{PA0, PA1, PA2}, Input, PullDown};

/// The frequency of tim2 clock
pub const TIM2_CLOCK_HZ: u32 = 24_000_000;

/// The first hal sensor
pub type HS1 = PA0<Input<PullDown>>;
/// The second hal sensor
pub type HS2 = PA1<Input<PullDown>>;
/// The third hal sensor
pub type HS3 = PA2<Input<PullDown>>;