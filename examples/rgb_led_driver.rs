#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;

    use hal::gpio::Output;
    use rtic_monotonics::systick::{ExtU32, Systick};
    use rgb_led_driver::{Color, RgbLedDriver};
    use teensy4_pins::common::*;
    #[local]
    struct Local {
        rgb_led: RgbLedDriver<Output<P4>, Output<P5>, Output<P6>>
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio2,
            mut gpio4,
            ..
        } = board::t41(ctx.device);

        let r = gpio4.output(pins.p4);
        let g = gpio4.output(pins.p5);
        let b = gpio2.output(pins.p6);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        blink_led::spawn().ok();

        (
            Shared {},
            Local {
                rgb_led: RgbLedDriver::new(r, g, b),
            },
        )    }
   #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [rgb_led], priority = 1)]
    async fn blink_led(ctx: blink_led::Context) {
        Systick::delay(1_000u32.millis()).await;

        ctx.local.rgb_led.set_color(Color::Blue);

        Systick::delay(1_000u32.millis()).await;

        ctx.local.rgb_led.set_color(Color::Purple);

        Systick::delay(1_000u32.millis()).await;

        ctx.local.rgb_led.off();
    }
}
