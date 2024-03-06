#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod led_control {
    use teensy4_bsp as bsp;
    use bsp::board;
    use rtic_monotonics::systick::*;

    // use teensy4_pins::t41::*;
    use rgb_led_driver_name::RgbLedDriver;

    #[local]
    struct Local {
        led_driver: RgbLedDriver<OutputPin<Error = ()>, OutputPin<Error = ()>, OutputPin<Error = ()>>,
    }

    // the shared group of resources for everything to use amongst all functions
    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {

        // Grab the board peripherals
        let board::Resources {
            mut pins,
            mut gpio1,
            mut gpio2,
            usb,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let red_pin = gpio2.output(pins.p9);
        let green_pin = gpio2.output(pins.p10);
        let blue_pin = gpio2.output(pins.p9);
        let led_driver = RgbLedDriver::new(red_pin, green_pin, blue_pin);

        // as I understand, this is the initialization calling the first "starting" function?
        blink_rgb::spawn().ok();
        (
            Shared {

            },
            Local { 
                led_driver: led_driver,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [], priority = 1)]
    async fn blink_led(_ctx: blink_led::Context) {
        loop {
            log::info!("On");

            Systick::delay(1_000u32.millis()).await;

            log::info!("Off");

            Systick::delay(1_000u32.millis()).await;
        }
    }

    #[task(local = [led_driver], priority = 1)]
    async fn blink_rgb(_ctx: blink_rgb::Context) {
        let led_driver = _ctx.local.led_driver;
        led_driver.set_color(true, true, true);
    }
}