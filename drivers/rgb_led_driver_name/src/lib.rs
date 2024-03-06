#![no_std]
#![crate_type = "lib"]

extern crate alloc;
use core::fmt::Debug; 


use embedded_hal::digital::v2::OutputPin;

pub struct RgbLedDriver<R:OutputPin<Error = ()>, G:OutputPin<Error = ()>, B:OutputPin<Error = ()>> {
    red_pin: R,
    green_pin: G,
    blue_pin: B,
}   

impl<R: OutputPin<Error = ()>, G: OutputPin<Error = ()>, B: OutputPin<Error = ()> + Debug> RgbLedDriver<R, G, B> {
    pub fn new(red: R, green: G, blue: B) -> Self {

        // sets the output directory for the pins --> not sure if necessary since they are output pins?
        Self { red_pin: red, green_pin: green, blue_pin: blue }
    }

    pub fn set_color(&mut self, red: bool, green: bool, blue: bool) {
        if !red {
            self.red_pin.set_low().unwrap();
        }
        if !green {
            self.green_pin.set_low().unwrap();
        }
        if !blue {
            self.blue_pin.set_low().unwrap();
        }

    }
}