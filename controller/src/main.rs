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
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use bsp::board;
    use teensy4_bsp as bsp;

    use rtic_monotonics::systick::*;

    use embedded_graphics::{
        mono_font::{ascii::FONT_7X13_BOLD, MonoTextStyleBuilder},
        pixelcolor::{BinaryColor, Rgb888},
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
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb, pins, lpi2c1, ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        //set i2c
        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);

        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        display_looper::spawn().ok();

        (Shared { display }, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1,shared=[display])]
    async fn display_looper(mut _ctx: display_looper::Context) {
        while true {
            for i in 0..6 {
                _ctx.shared.display.lock(|display| {
                    display.clear();

                    let style = PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::On)
                        .build();

                    Rectangle::new(Point::new(0, 10 * i + 1), Size::new(128, 10))
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
                        Point::zero(),
                        if i == 0 { ts_dark } else { ts_light },
                        Baseline::Top,
                    )
                    .draw(display)
                    .unwrap();

                    Text::with_baseline(
                        "Line 1",
                        Point::new(0, 10),
                        if i == 1 { ts_dark } else { ts_light },
                        Baseline::Top,
                    )
                    .draw(display)
                    .unwrap();
                    Text::with_baseline(
                        "Line 2",
                        Point::new(0, 20),
                        if i == 2 { ts_dark } else { ts_light },
                        Baseline::Top,
                    )
                    .draw(display)
                    .unwrap();
                    Text::with_baseline(
                        "Line 3",
                        Point::new(0, 30),
                        if i == 3 { ts_dark } else { ts_light },
                        Baseline::Top,
                    )
                    .draw(display)
                    .unwrap();
                    Text::with_baseline(
                        "Line 4",
                        Point::new(0, 40),
                        if i == 4 { ts_dark } else { ts_light },
                        Baseline::Top,
                    )
                    .draw(display)
                    .unwrap();
                    Text::with_baseline(
                        "Line 5",
                        Point::new(0, 50),
                        if i == 5 { ts_dark } else { ts_light },
                        Baseline::Top,
                    )
                    .draw(display)
                    .unwrap();

                    display.flush().unwrap();
                });

                Systick::delay(1000u32.millis()).await;
            }
        }
    }
}
