//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use rgb_led::LedDriver;
use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use rgb_led::LedDriver;
    use teensy4_pins::common::*;

    use bsp::board;
    use teensy4_bsp as bsp;

    use bsp::hal;
    use hal::gpio::Output;

    use rtic_monotonics::systick::*;
    use you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::board::Lpspi4;

    #[local]
    struct Local {
        led_driver: LedDriver<Output<P0>, Output<P8>, Output<P31>>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio1,
            mut gpio2,
            mut gpio3,
            lpspi4,
            ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        let led_driver = LedDriver::new(
            gpio1.output(pins.p0),
            gpio2.output(pins.p8),
            gpio3.output(pins.p31),
        );

        led::spawn().ok();

        (Shared {}, Local { led_driver })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [led_driver], priority = 1)]
    async fn led(ctx: led::Context) {
        Systick::delay(1_000u32.millis()).await;

        ctx.local.led_driver.red().unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led_driver.green().unwrap();
        
        Systick::delay(1_000u32.millis()).await;

        ctx.local.led_driver.blue().unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led_driver.green().unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led_driver.purple().unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led_driver.turquoise().unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led_driver.yellow().unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led_driver.white().unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led_driver.off().unwrap();
    }
}
