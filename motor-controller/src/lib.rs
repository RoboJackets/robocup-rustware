//!
//! Library for the motor-controller firmware
//!
//! Note: 1 Mono tick is 1 us
//!

#![no_std]
#![feature(type_alias_impl_trait)]

use core::fmt::Debug;

use defmt::Format;
use embedded_hal::PwmPin;
use stm32f0xx_hal::prelude::*;
use stm32f0xx_hal::{
    gpio::{
        AF1, Alternate, Input, Output, PullDown, PushPull,
        gpioa::{PA0, PA1, PA2, PA11, PA14, PA15},
        gpiob::PB12,
        gpiof::{PF6, PF7},
    },
    pac::{EXTI, SYSCFG, USART1},
    serial::Serial,
};

pub mod encoder;
pub mod pid;

#[cfg(any(
    all(
        not(feature = "motor-2"),
        not(feature = "motor-3"),
        not(feature = "motor-4")
    ),
    feature = "motor-1"
))]
/// The command name of a command for motor one
pub const MOTOR_COMMAND: u8 = 0x11;
#[cfg(feature = "motor-2")]
/// The command name of a command for motor two
pub const MOTOR_COMMAND: u8 = 0x22;
#[cfg(feature = "motor-3")]
/// The command name of a command for motor three
pub const MOTOR_COMMAND: u8 = 0x33;
#[cfg(feature = "motor-4")]
/// The command name of a command for motor four
pub const MOTOR_COMMAND: u8 = 0x44;

/// The frequency of tim2 clock
pub const TIM2_CLOCK_HZ: u32 = 8_000_000;
/// The frequency of tim3 clock
pub const TIM3_CLOCK_HZ: u32 = 8_000_000;
/// There is roughly 187 pwm counts per rotation (i.e. 6200 encoder ticks)
pub const VELOCITY_TO_PWM_MAPING: f32 = 187.0 / 6200.0;
/// The frequency (in hertz) of the motion control loop
pub const MOTION_CONTROL_FREQUENCY: u32 = 1000;

/// The first hal sensor
pub type HS1 = PA0<Input<PullDown>>;
/// The second hal sensor
pub type HS2 = PA1<Input<PullDown>>;
/// The third hal sensor
pub type HS3 = PA2<Input<PullDown>>;

/// The usart TX Pin
pub type UsartTx = PA14<Alternate<AF1>>;
/// The usart RX Pin
pub type UsartRx = PA15<Alternate<AF1>>;
/// The USART Peripheral
pub type SerialInterface = Serial<USART1, UsartTx, UsartRx>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Format)]
/// At what voltage should the overcurrent threshold be tripped
pub enum OvercurrentThreshold {
    /// 90-120 mV maximum
    T1,
    /// 235-275 mV maximum
    T2,
    /// 465-545 mV maximum
    T3,
}

/// Overcurrent comparator
pub struct OvercurrentComparator {
    /// pa11 overcurrent selector
    pa11: PA11<Output<PushPull>>,
    /// pf6 overcurrent selector
    pf6: PF6<Output<PushPull>>,
    /// pf7 overcurrent selector
    pf7: PF7<Output<PushPull>>,
    /// pb12 overcurrent comparator result
    pb12: PB12<Input<PullDown>>,
}

impl OvercurrentComparator {
    /// Create a new overcurrent comparator
    pub fn new(
        pa11: PA11<Output<PushPull>>,
        pf6: PF6<Output<PushPull>>,
        pf7: PF7<Output<PushPull>>,
        pb12: PB12<Input<PullDown>>,
    ) -> Self {
        Self {
            pa11,
            pf6,
            pf7,
            pb12,
        }
    }

    /// Determine if the overcurrent comparator is tripped
    pub fn is_tripped(&mut self) -> bool {
        self.pb12.is_high().unwrap()
    }

    /// Make the overcurrent register automatically turn off the gate driver control logic
    pub fn stop_gate_drivers(&mut self, stop_drivers: bool) {
        if stop_drivers {
            self.pa11.set_high().unwrap();
        } else {
            self.pa11.set_low().unwrap();
        }
    }

    /// Set the overcurrent comparator's overcurrent threshold
    pub fn set_threshold(&mut self, threshold: OvercurrentThreshold) {
        match threshold {
            OvercurrentThreshold::T1 => {
                self.pf6.set_low().unwrap();
                self.pf7.set_high().unwrap();
            }
            OvercurrentThreshold::T2 => {
                self.pf6.set_high().unwrap();
                self.pf7.set_low().unwrap();
            }
            OvercurrentThreshold::T3 => {
                self.pf6.set_high().unwrap();
                self.pf7.set_high().unwrap();
            }
        }
    }

    /// Set an interrupt for the overcurrent comparator
    pub fn set_interrupt(&mut self, syscfg: &SYSCFG, exti: &EXTI) {
        syscfg.exticr1.modify(|_, w| unsafe { w.exti1().bits(12) });
        exti.imr.modify(|_, w| w.mr12().set_bit());
        exti.rtsr.modify(|_, w| w.tr12().set_bit());
    }

    /// Clear a interrupt from the overcurrent comparator
    pub fn clear_interrupt(&mut self, exti: &EXTI) {
        exti.pr.write(|w| w.pr12().set_bit());
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// The desired phase of a given output
pub enum Phase {
    /// The output should be high
    Positive,
    /// The output should be zero
    Zero,
    /// The output should be negative
    Negative,
}

/// Get the current phase number from the hall sensor readings
pub fn phase_number(h1: bool, h2: bool, h3: bool) -> u8 {
    match (h1, h2, h3) {
        (false, false, true) => 0,
        (true, false, true) => 1,
        (true, false, false) => 2,
        (true, true, false) => 3,
        (false, true, false) => 4,
        _ => 5,
    }
}

/// Convert the values read from the hall sensors into the corresponding U, V, and W phases
/// to move the motors
pub fn hall_to_phases(h1: bool, h2: bool, h3: bool, clockwise: bool) -> [Phase; 3] {
    match (h1, h2, h3) {
        (false, false, true) => {
            // Step 1
            if clockwise {
                [Phase::Zero, Phase::Negative, Phase::Positive]
            } else {
                [Phase::Zero, Phase::Positive, Phase::Negative]
            }
        }
        (true, false, true) => {
            // Step 2
            if clockwise {
                [Phase::Positive, Phase::Negative, Phase::Zero]
            } else {
                [Phase::Negative, Phase::Positive, Phase::Zero]
            }
        }
        (true, false, false) => {
            // Step 3
            if clockwise {
                [Phase::Positive, Phase::Zero, Phase::Negative]
            } else {
                [Phase::Negative, Phase::Zero, Phase::Positive]
            }
        }
        (true, true, false) => {
            // Step 4
            if clockwise {
                [Phase::Zero, Phase::Positive, Phase::Negative]
            } else {
                [Phase::Zero, Phase::Negative, Phase::Positive]
            }
        }
        (false, true, false) => {
            // Step 5
            if clockwise {
                [Phase::Negative, Phase::Positive, Phase::Zero]
            } else {
                [Phase::Positive, Phase::Negative, Phase::Zero]
            }
        }
        (false, true, true) => {
            // Step 6
            if clockwise {
                [Phase::Negative, Phase::Zero, Phase::Positive]
            } else {
                [Phase::Positive, Phase::Zero, Phase::Negative]
            }
        }
        _ => [Phase::Zero, Phase::Zero, Phase::Zero],
    }
}

/// Set the value of the positive and negative channels of a pwm output
pub fn set_pwm_output(
    ch: &mut impl PwmPin<Duty = u16>,
    chn: &mut impl PwmPin<Duty = u16>,
    phase: Phase,
    duty: u16,
) {
    match phase {
        Phase::Positive => {
            ch.set_duty(duty);
            chn.set_duty(0);
        }
        Phase::Zero => {
            ch.set_duty(0);
            chn.set_duty(0);
        }
        Phase::Negative => {
            ch.set_duty(0);
            chn.set_duty(duty);
        }
    }
}
