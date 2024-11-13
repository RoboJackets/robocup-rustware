//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

pub mod fsm_menu {
    use teensy4_bsp as bsp;

    use rtic_monotonics::systick::*;

    use embedded_graphics::{
        mono_font::{ascii::FONT_7X13_BOLD, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::*,
        primitives::{PrimitiveStyleBuilder, Rectangle},
        text::{Baseline, Text},
    };
    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
    use teensy4_pins::t41::*;

    type Display = Ssd1306<
        I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
        DisplaySize128x64,
        ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
    >;

    struct MenuItem<'a> {
        label: &'a str,
        goto: i32,
        from: i32,
    }

    struct Menu<'a> {
        label: &'a str,
        entries: [MenuItem<'a>],
    }

    impl<'a> Menu<'a> {
        fn fetch_item(&self, n: usize) -> &MenuItem {
            &self.entries[n]
        }
    }


    
}

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
    use imxrt_hal::gpio::Input;
    use teensy4_bsp as bsp;

    use rtic_monotonics::systick::*;

    use embedded_graphics::{
        mono_font::{ascii::FONT_7X13_BOLD, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::*,
        primitives::{PrimitiveStyleBuilder, Rectangle},
        text::{Baseline, Text},
    };
    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
    use teensy4_pins::t41::*;

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
            DisplaySize128x64,
            ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
        >,
        buttoninc: Input<P5>,
        buttondec: Input<P4>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb,
            pins,
            lpi2c1,
            mut gpio4,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        //setup buttoninc
        let buttoninc = gpio4.input(pins.p5);

        //setup buttondec
        let buttondec = gpio4.input(pins.p4);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        //set i2c
        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);

        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        display_looper::spawn().ok();

        (
            Shared {
                display,
                buttoninc,
                buttondec,
            },
            Local {},
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1,shared=[display,buttoninc,buttondec])]
    async fn display_looper(mut _ctx: display_looper::Context) {
        let mut lastbuttonincState = false;
        let mut lastbuttondecState = false;
        let mut i: i32 = 0;
        loop {
            _ctx.shared.buttoninc.lock(|btn| {
                let currentbuttonincState = btn.is_set();

                if !lastbuttonincState && currentbuttonincState {
                    i += 1;
                    if i > 5 {
                        i = 0
                    };
                }

                lastbuttonincState = currentbuttonincState;
            });

            _ctx.shared.buttondec.lock(|btn| {
                let currentbuttondecState = btn.is_set();

                if !lastbuttondecState && currentbuttondecState {
                    i -= 1;
                    if i < 0 {
                        i = 5
                    };
                }

                lastbuttondecState = currentbuttondecState;
            });

            _ctx.shared.display.lock(|display| {
                display.clear();

                let style = PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .build();

                Rectangle::new(Point::new(0, 10 * i + 1), Size::new(128, 11))
                    .into_styled(style)
                    .draw(display)
                    .unwrap();

                let ts_dark = MonoTextStyleBuilder::new()
                    .font(&FONT_7X13_BOLD)
                    .text_color(BinaryColor::Off)
                    .build();

                let ts_light = MonoTextStyleBuilder::new()
                    .font(&FONT_7X13_BOLD)
                    .text_color(BinaryColor::On)
                    .build();

                Text::with_baseline(
                    "Line 0",
                    Point::new(2, 0),
                    if i == 0 { ts_dark } else { ts_light },
                    Baseline::Top,
                )
                .draw(display)
                .unwrap();

                Text::with_baseline(
                    "Line 1",
                    Point::new(2, 10),
                    if i == 1 { ts_dark } else { ts_light },
                    Baseline::Top,
                )
                .draw(display)
                .unwrap();
                Text::with_baseline(
                    "Line 2",
                    Point::new(2, 20),
                    if i == 2 { ts_dark } else { ts_light },
                    Baseline::Top,
                )
                .draw(display)
                .unwrap();
                Text::with_baseline(
                    "Line 3",
                    Point::new(2, 30),
                    if i == 3 { ts_dark } else { ts_light },
                    Baseline::Top,
                )
                .draw(display)
                .unwrap();
                Text::with_baseline(
                    "Line 4",
                    Point::new(2, 40),
                    if i == 4 { ts_dark } else { ts_light },
                    Baseline::Top,
                )
                .draw(display)
                .unwrap();
                Text::with_baseline(
                    "Line 5",
                    Point::new(2, 50),
                    if i == 5 { ts_dark } else { ts_light },
                    Baseline::Top,
                )
                .draw(display)
                .unwrap();

                display.flush().unwrap();
            });
        }
    }
}
