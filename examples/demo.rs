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
use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use teensy4_pins::common::*;

    use bsp::board;
    use teensy4_bsp as bsp;

    use bsp::hal;
    use hal::gpio::Output;

    use rtic_monotonics::systick::*;

    type Led = Output<P7>;

    #[local]
    struct Local {
        led: Led,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins, mut gpio2, ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        let led = gpio2.output(pins.p7);

        blink_led::spawn().ok();

        (Shared {}, Local { led })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [led], priority = 1)]
    async fn blink_led(ctx: blink_led::Context) {
        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.toggle();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.toggle();
    }
}
