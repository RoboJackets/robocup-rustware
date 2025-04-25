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
    use robojackets_robocup_control::Display;
    use teensy4_bsp as bsp;

    use rtic_monotonics::systick::*;

    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::Point,
        text::{Baseline, Text},
        Drawable,
    };

    #[local]
    struct Local {
        toggle: bool,
    }

    #[shared]
    struct Shared {
        display: Display,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb, lpi2c1, pins, ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::MHz1);
        let interface = I2CDisplayInterface::new(i2c);
        let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        let toggle = true;

        delay_tick::spawn().ok();

        (Shared { display }, Local { toggle })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn delay_tick(mut ctx: delay_tick::Context) {
        Systick::delay(1000u32.millis()).await;

        display_update::spawn().ok();
    }

    #[task(priority = 1,shared=[display],local=[toggle])]
    async fn display_update(mut ctx: display_update::Context) {
        (ctx.shared.display).lock(|display| {
            display.clear();

            let default_text_style = MonoTextStyleBuilder::new()
                .font(&FONT_6X10)
                .text_color(BinaryColor::On)
                .background_color(BinaryColor::Off)
                .build();

            Text::with_baseline(
                "Hello, World!",
                Point::new(10, 10),
                default_text_style,
                Baseline::Top,
            )
            .draw(display)
            .unwrap();

            if *ctx.local.toggle {
                Text::with_baseline(
                    "Blink!",
                    Point::new(10, 30),
                    default_text_style,
                    Baseline::Top,
                )
                .draw(display)
                .unwrap();
            }
            *ctx.local.toggle = !*ctx.local.toggle;

            display.flush().unwrap();
        });

        delay_tick::spawn().ok();
    }
}
