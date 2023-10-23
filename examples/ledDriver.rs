#![no_std]

use rbgLED::rbg_LED_Driver;
use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use teensy4_pins::common::*;

    use teensy4_bsp as bsp;
    use bsp::board as board;

    use bsp::hal as hal;
    use hal::gpio::Output;

    use rtic_monotonics::systick::*;

    //local because this is currently the only place where this LED is used
    #[local]
    struct local {
        //idk if pin 7 is the right pin or not? Ask Nate
        led: rbg_LED_Driver<Output<P7>> 
    }

    #[shared]
    struct shared {
        //empty because LED currently isn't shared with anything
    }

    #[init]
    fn init (ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio2, //is this gpio correct and if it is/isn't how do I know?
            ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        //if pin 7 isn't the correct PIN then I need to make sure to change it here as well
        let led = gpio2.output(pins.p7);

        blink_led::spawn().ok();

        (
            Shared {},
            Local {
                led,
            },
        )
    }
}