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

    type Led = Output<P7>;

    #[local]
    struct Local {
        led: LedDriver<'static, Lpspi4>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio2,
            lpspi4,
            ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        let spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            2_00_000,
        );

		let led = LedDriver::new(spi);

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
