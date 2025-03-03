use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::{Baseline, Text},
    Drawable,
};

use crate::module_types::Display;

pub fn encode_btn_state(left: bool, right: bool, up: bool, down: bool) -> u8 {
    let mut btn = 0u8;
    if left {
        btn |= 1 << 0;
    }
    if right {
        btn |= 1 << 1;
    }
    if up {
        btn |= 1 << 2;
    }
    if down {
        btn |= 1 << 3;
    }
    return btn;
}

pub fn render_text(display: Display, text: &str, x: u8, y: u8, highlight: bool) {
    let default_text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .background_color(BinaryColor::Off)
        .build();

    let selected_text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::Off)
        .background_color(BinaryColor::On)
        .build();

    Text::with_baseline(
        text,
        Point::new(x as i32, y as i32),
        if highlight {
            selected_text_style
        } else {
            default_text_style
        },
        Baseline::Top,
    )
    .draw(display)
    .unwrap();
}
