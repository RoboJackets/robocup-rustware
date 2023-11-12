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

extern crate alloc;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use teensy4_pins::t41::{Pins, *};

    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::board::LPSPI_FREQUENCY;

    use bsp::hal as hal;
    use hal::gpio::Output;

    use bsp::ral as ral;
    use ral::lpspi::{LPSPI1, LPSPI2, LPSPI3, LPSPI4};

    use rtic_monotonics::systick::*;

    #[local]
    struct Local {
        spi_pins: Option<hal::lpspi::Pins<P26, P39, P27, P38>>,
        spi3: Option<ral::Instance<ral::lpspi::RegisterBlock, 3>>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio2,
            usb,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let spi_pins = hal::lpspi::Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39
        };
        let spi3 = unsafe { LPSPI3::instance() };

        blink_led::spawn().ok();
        shot_in_the_dark::spawn().ok();

        (
            Shared {},
            Local {
                spi_pins: Some(spi_pins),
                spi3: Some(spi3),
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
    async fn blink_led(ctx: blink_led::Context) {
        loop {
            log::info!("On");

            Systick::delay(1_000u32.millis()).await;

            log::info!("Off");

            Systick::delay(1_000u32.millis()).await;
        }
    }

    #[task(local = [spi_pins, spi3], priority = 1)]
    async fn shot_in_the_dark(ctx: shot_in_the_dark::Context) {
        Systick::delay(5_000u32.millis()).await;

        log::info!("Initializing the SPI - Hopefully");

        let spi_pins = ctx.local.spi_pins.take().unwrap();
        let spi3 = ctx.local.spi3.take().unwrap();
        let mut test_spi = hal::lpspi::Lpspi::without_pins(spi3);

        // let mut test_spi = hal::lpspi::Lpspi::new(spi3, spi_pins);

        log::info!("No Clue How This Worked");
    }
}