#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use rtic_monotonics::systick::*;

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    mono_font::MonoTextStyle,
    mono_font::ascii::FONT_4X6,
    text::Text,
    text::TextStyleBuilder,
    text::Alignment,
    text::Baseline,
};

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use robojackets_robocup_control::peripherals::BatterySenseT;
    use startup_graphic::StartScreen;
    use teensy4_bsp::board::Lpi2c3;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins, lpi2c3, usb, ..
        } = board::t41(cx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 36_000_000, systick_token);

        let mut i2c: Lpi2c3 = board::lpi2c(
            lpi2c3,
            pins.p16,
            pins.p17,
            board::Lpi2cClockSpeed::KHz400,
        );
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(
            interface,
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        ).into_buffered_graphics_mode();

        display.init();

        let text_style = TextStyleBuilder::new()
            .alignment(Alignment::Left)
            .baseline(Baseline::Middle)
            .build();
        let char_style = MonoTextStyle::new(&FONT_4X6, BinaryColor::On);

        Text::with_text_style(
            "Yeet",
            Point::new(0, 0),
            char_style,
            text_style,
        ).draw(&mut display);

        (Shared {}, Local {})

    }
}