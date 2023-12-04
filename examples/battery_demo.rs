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

    use battery_module::{Battery, BatteryAdc};
    use rtic_monotonics::systick::*;

    #[local]
    struct Local {
        battery: Battery<1>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins, 
            gpio2, 
            adc1,
            usb,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let battery = Battery::new(BatteryAdc::new(adc1));

        (Shared {}, Local { battery })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [battery], priority = 1)]
    async fn blink_led(ctx: blink_led::Context) {
        loop {
            Systick::delay(1_000u32.millis()).await;

            let _ = ctx.local.battery.update();
            let percentage = ctx.local.battery.get_percentage();

            log::info!("Red {}", percentage);
        }
    }
}

