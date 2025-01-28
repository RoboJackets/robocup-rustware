//!
//! Library for the motor-controller firmware
//! 

#![no_std]
#![feature(type_alias_impl_trait)]

use stm32f0xx_hal::gpio::{gpioa::{PA0, PA1, PA2}, Input, PullDown};
use embedded_hal::PwmPin;

/// The frequency of tim2 clock
pub const TIM2_CLOCK_HZ: u32 = 8_000_000;

/// The first hal sensor
pub type HS1 = PA0<Input<PullDown>>;
/// The second hal sensor
pub type HS2 = PA1<Input<PullDown>>;
/// The third hal sensor
pub type HS3 = PA2<Input<PullDown>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// The desired phase of a given output
pub enum Phase {
    /// The output should be high
    Positive,
    /// The output should be zero
    Zero,
    /// The output should be negative
    Negative,
}

/// Convert the values read from the hall sensors into the corresponding U, V, and W phases
/// to move the motors
pub fn hall_to_phases(h1: bool, h2: bool, h3: bool, clockwise: bool) -> [Phase; 3] {
    match (h1, h2, h3) {
        (false, false, true) => {
            // Step 1
            if clockwise {
                [Phase::Positive, Phase::Negative, Phase::Zero]
            } else {
                [Phase::Negative, Phase::Zero, Phase::Positive]
            }
        },
        (true, false, true) => {
            // Step 2
            if clockwise {
                [Phase::Positive, Phase::Zero, Phase::Negative]
            } else {
                [Phase::Zero, Phase::Negative, Phase::Positive]
            }
        },
        (true, false, false) => {
            // Step 3
            if clockwise {
                [Phase::Zero, Phase::Positive, Phase::Negative]
            } else {
                [Phase::Positive, Phase::Negative, Phase::Zero]
            }
        },
        (true, true, false) => {
            // Step 4
            if clockwise {
                [Phase::Negative, Phase::Positive, Phase::Zero]
            } else {
                [Phase::Positive, Phase::Zero, Phase::Negative]
            }
        },
        (false, true, false) => {
            // Step 5
            if clockwise {
                [Phase::Negative, Phase::Zero, Phase::Positive]
            } else {
                [Phase::Zero, Phase::Positive, Phase::Negative]
            }
        },
        (false, true, true) => {
            // Step 6
            if clockwise {
                [Phase::Zero, Phase::Negative, Phase::Positive]
            } else {
                [Phase::Negative, Phase::Positive, Phase::Zero]
            }
        },
        _ => [Phase::Zero, Phase::Zero, Phase::Zero]
    }
}

/// Set the value of the positive and negative channels of a pwm output
pub fn set_pwm_output(
    ch: &mut impl PwmPin<Duty=u16>,
    chn: &mut impl PwmPin<Duty=u16>,
    phase: Phase,
    duty: u16
) {
    match phase {
        Phase::Positive => {
            ch.set_duty(duty);
            chn.set_duty(0);
        },
        Phase::Zero => {
            ch.set_duty(0);
            chn.set_duty(0);
        },
        Phase::Negative => {
            ch.set_duty(0);
            chn.set_duty(duty);
        }
    }
}