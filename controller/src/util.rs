use core::cmp::min;

use alloc::string::ToString;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::{Baseline, Text},
    Drawable,
};

use crate::types::{Display, RadioState, TEAM_NAME_MAP};

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

pub fn get_successful_ack_count(state: &RadioState) -> u8 {
    let mut ack_count = 0;
    for ack in state.conn_acks_results.iter() {
        if *ack {
            ack_count += 1;
        }
    }
    return ack_count;
}

pub fn render_status_header(display: Display, state: &RadioState) {
    let team_name = TEAM_NAME_MAP[state.team as usize].to_string();
    let team_name = alloc::fmt::format(format_args!("{}{}", team_name, state.robot_id));
    render_text(display, &team_name, 2, 0, false);

    let ack_count = get_successful_ack_count(state);
    let ack_attempts = min(state.conn_acks_attempts, 100);
    let ack_percent = (ack_count as f32 / ack_attempts as f32) * 100.0;
    let ack_percent = alloc::fmt::format(format_args!("{:02}%", ack_percent as u8));
    render_text(display, &ack_percent, 104, 0, false);

    log::info!("{}: {}", ack_count, ack_percent);
}

pub fn render_status_title(display: Display, title: &str) {
    let title_len = title.len() * 6;
    let title_x = (128 - title_len) / 2;
    render_text(display, title, title_x as u8, 0, true);
}
