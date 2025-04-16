#![no_std]

extern crate alloc;
use alloc::format;

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    image::Image,
    text::{Alignment, Baseline, Text, TextStyleBuilder},
    mono_font::{ascii::FONT_7X13, MonoTextStyle},
};

use battery_indicator::BatteryIndicator;

pub struct MainWindow<'a> {
    pub battery_percent: u32,
    pub robot_id: u16,
    pub team: &'a str,
    pub kicker_charged: bool,
    pub ball_sense: bool,
    pub latency: u32,
}

impl<'a> MainWindow<'a> {
    pub fn new(robot_id: u16, team: &'a str) -> Self
    {
        let instance = MainWindow {
            battery_percent: 0,
            robot_id,
            team,
            kicker_charged: false,
            ball_sense: false,
            latency: 0,
        };
        return instance;
    }
}

impl<'a> Drawable for MainWindow<'a> {
    type Output = ();
    type Color = BinaryColor;
    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where D: DrawTarget<Color = BinaryColor>
    {
        let text_style = TextStyleBuilder::new()
        .alignment(Alignment::Left)
        .baseline(Baseline::Middle)
        .build();
        let char_style = MonoTextStyle::new(&FONT_7X13, BinaryColor::On);

        let mut b_i: BatteryIndicator = BatteryIndicator::new(Point::new(108, 1));
        b_i.update(self.battery_percent);
        b_i.draw(target)?;

        let team_s: &str = &format!("Team: {}", self.team);
        Text::with_text_style(team_s, Point::new(1, 1), 
        char_style, text_style).draw(target)?;

        Ok(())
    }
}