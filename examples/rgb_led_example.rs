#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use teensy4_pins::common::*;

    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::hal::gpio::Output;

    use rtic_monotonics::systick::*;


    use rgb_led_driver_alaric::{RGBLed, Color};

    type Led = RGBLed<Output<P6>, Output<P7>, Output<P8>>;

    #[local]
    struct Local {
        led: Led,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio2,
            ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        let led = RGBLed::new(gpio2.output(pins.p6), gpio2.output(pins.p7), gpio2.output(pins.p8));

        blink_led::spawn().ok();

        (
            Shared {},
            Local {
                led,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [led], priority = 1)]
    async fn blink_led(ctx: blink_led::Context) {
        ctx.local.led.set_color(Color::Red).unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.set_color(Color::Green).unwrap();
        
        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.set_color(Color::Blue).unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.set_color(Color::Yellow).unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.set_color(Color::Purple).unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.set_color(Color::Cyan).unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.clear().unwrap();
    }
}