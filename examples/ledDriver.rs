#![no_std]

use rbgLED::rbg_LED_Driver;
use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use teensy4_pins::common::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::gpio::Output;

    use rtic_monotonics::systick::*;

    
}