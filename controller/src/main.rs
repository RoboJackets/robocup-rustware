//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

///
/// This is a demo example file that turns on and off the onboard led.
///
/// Please follow this example for future examples and sanity tests
///
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use bsp::board;
    use teensy4_bsp as bsp;
    use teensy4_bsp::hal::gpio::{Input, Output};
    use teensy4_pins::t41::*;

    use rtic_monotonics::systick::*;

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        ledout: Output<P0>,
        button: Input<P1>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb,
            pins,
            mut gpio1,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        //set pin 0 to output
        let ledout = gpio1.output(pins.p0);

        let button = gpio1.input(pins.p1);

        blink_led::spawn().ok();

        (Shared { ledout, button }, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1,shared=[ledout, button])]
    async fn blink_led(mut _ctx: blink_led::Context) {
        let mut toggle = false;
        loop {
            log::info!("On");
            _ctx.shared.ledout.lock(|led| {
                _ctx.shared.button.lock(|button| {
                    if button.is_set() {
                        if toggle {
                            led.set();
                        } else {
                            led.clear();
                        }

                        toggle = !toggle;
                    }
                });
            });

            Systick::delay(100u32.millis()).await;
        }
    }
}
