#![no_std]
#![crate_type = "lib"]

extern crate alloc;


use embedded_hal::digital::v2::OutputPin;
// use teensy4_bsp as bsp;
// use bsp::hal::gpio; 
// use teensy4_pins::t41::*;
// use bsp::hal::gpio::Output;

// pub struct RgbLedDriver<R:gpio::Output<P9>, G:gpio::Output<P10>, B:gpio::Output<P11>> {
//     red_pin: R,
//     green_pin: G,
//     blue_pin: B,
// }

pub struct RgbLedDriver<R:OutputPin<Error = GPIOE>, G:OutputPin<Error = GPIOE>, B:OutputPin<Error = GPIOE>, GPIOE> {
    red_pin: R,
    green_pin: G,
    blue_pin: B,
}

impl<R: OutputPin<Error = GPIOE>, G: OutputPin<Error = GPIOE>, B: OutputPin<Error = GPIOE>, GPIOE> RgbLedDriver<R, G, B, GPIOE> {
    pub fn new(red: R, green: G, blue: B) -> Self {

        // sets the output directory for the pins --> not sure if necessary since they are output pins?
        Self { red_pin: red, green_pin: green, blue_pin: blue }
    }

    pub fn set_color(&mut self, red: bool, green: bool, blue: bool) {
        if !red {
            let _ = self.red_pin.set_low();
        }
        if !green {
            let _ = self.green_pin.set_low();
        }
        if !blue {
            let _ = self.blue_pin.set_low();
        }

    }
}