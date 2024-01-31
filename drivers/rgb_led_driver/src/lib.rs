#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]

// embedded hal traits
use embedded_hal::digital::v2::OutputPin; 
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::spi::{self, Mode};

enum Colors {
    RED,
    GREEN,
    BLUE,
    PURPLE,
    YELLOW,
    CYAN
}

pub struct RGBDriver<T> {
    pin1: T,
    pin2: T,
    pin3: T,
}

impl<T> RGBDriver<T> where T: OutputPin {
    
}