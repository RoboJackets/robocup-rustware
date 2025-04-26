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

    use teensy4_pins::t41::*;

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
            usb,
            lpi2c1,
            pins,
            mut gpio1,
            mut gpio2,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        //stop the motorboard from killing itself
        let v3_3_enable = gpio1.output(pins.p23);
        v3_3_enable.set();

        let v5_enable = gpio2.output(pins.p36);
        v5_enable.set();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
        let i2c_bus: &'static _ = shared_bus::new_cortexm!(
            imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1> = i2c
        )
        .unwrap();
        let interface = I2CDisplayInterface::new(i2c_bus.acquire_i2c());
        let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        let toggle = true;

        peripheral_init::spawn().ok();

        (Shared { display }, Local { toggle })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1,shared=[display])]
    async fn peripheral_init(mut ctx: peripheral_init::Context) {
        Systick::delay(4000u32.millis()).await;

        (ctx.shared.display).lock(|display| {
            display.init().unwrap();
            display.flush().unwrap();
        });

        display_update::spawn().ok();
    }

    #[task(priority = 1)]
    async fn delay_tick(mut ctx: delay_tick::Context) {
        Systick::delay(100u32.millis()).await;

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
