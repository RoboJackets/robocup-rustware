extern crate alloc;
use alloc::format;

use crate::battery_indicator::BatteryIndicator;
use embedded_graphics::{
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X9, FONT_7X13},
    },
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};

pub struct MainWindow<'a> {
    pub battery_percent: u32,
    pub robot_id: u16,
    pub team: &'a str,
    pub kicker_charged: bool,
    pub ball_sense: bool,
    pub latency: u32,
}

impl<'a> MainWindow<'a> {
    // MainWindow object can be updated by manually changing its members.
    // Might need an update() function in the future.
    pub fn new(robot_id: u16, team: &'a str) -> Self {
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
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        let text_style = TextStyleBuilder::new()
            .alignment(Alignment::Left)
            .baseline(Baseline::Middle)
            .build();
        let char_style = MonoTextStyle::new(&FONT_6X9, BinaryColor::On);
        let header_style = MonoTextStyle::new(&FONT_7X13, BinaryColor::On);

        let mut b_i: BatteryIndicator = BatteryIndicator::new(Point::new(108, 1));
        b_i.update(self.battery_percent);
        b_i.draw(target)?;

        let robot_id_s: &str = &format!("Robot {}", self.robot_id);
        Text::with_text_style(robot_id_s, Point::new(1, 5), header_style, text_style)
            .draw(target)?;

        let team_s: &str = &format!("Team: {}", self.team);
        Text::with_text_style(team_s, Point::new(1, 20), char_style, text_style).draw(target)?;

        let kicker_s = &format!(
            "Kicker charged: {}",
            if self.kicker_charged { "yes" } else { "no" }
        );
        Text::with_text_style(kicker_s, Point::new(1, 30), char_style, text_style).draw(target)?;

        let ball_sense_s = &format!(
            "Ball sensed: {}",
            if self.ball_sense { "yes" } else { "no" }
        );
        Text::with_text_style(ball_sense_s, Point::new(1, 40), char_style, text_style)
            .draw(target)?;

        let latency_s = &format!("Latency: {}", self.latency);
        Text::with_text_style(latency_s, Point::new(1, 50), char_style, text_style).draw(target)?;

        Ok(())
    }
}
