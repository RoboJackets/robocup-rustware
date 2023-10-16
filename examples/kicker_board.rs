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
    use core::convert::Infallible;

    use kicker::KickerBoard;
    use teensy4_bsp::hal::gpio;
    use teensy4_pins::common::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::gpio::Output;

    use rtic_monotonics::systick::*;

    use kicker::KickerBoard;
    use teensy4_pins::tmm::P8;
    use teensy4_pins::tmm::P9;

    type Led = Output<P7>;
    // type Kicker = KickerBoard<Infallible, gpio::Output<P8>, gpio::Output<P9>>,

    #[local]
    struct Local {

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
            lpspi4,
            mut gpt1,
            ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            1_000_000
        );

        let fake_cs = gpio2.output(pins.p8);
        let reset = gpio2.output(pins.p9);

        let kicker_board = KickerBoard::new(fake_cs, reset, false).unwrap();

        (
            Shared {

            },
            Local {
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}