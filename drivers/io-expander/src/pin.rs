//!
//! Definition of the IO-Expander Pins.
//! 

#![allow(unused)]

/// Pins accessible via the io_expander
pub enum Pin {
    PA0 = 0,
    PA1 = 1,
    PA2 = 2,
    PA3 = 3,
    PA4 = 4,
    PA5 = 5,
    PA6 = 6,
    PA7 = 7,
    PB0 = 8,
    PB1 = 9,
    PB2 = 10,
    PB3 = 11,
    PB4 = 12,
    PB5 = 13,
    PB6 = 14,
    PB7 = 15,
}

impl Pin {
    pub fn pin_num(self) -> u8 {
        self as u8
    }
}