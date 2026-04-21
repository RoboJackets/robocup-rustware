extern crate alloc;
use alloc::format;

use embedded_graphics::{
    image::Image,
    mono_font::{MonoTextStyle, ascii::FONT_6X9},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};

use tinybmp::Bmp;

pub struct StartScreen<'a> {
    logo: Bmp<'a, BinaryColor>,
    text: Bmp<'a, BinaryColor>,
    logo_pos: Point,
    text_pos: Point,
    blue_team: bool,
    robot_id: u8,
}

impl StartScreen<'_> {
    pub fn new(logo_pos: Point, text_pos: Point, blue_team: bool, robot_id: u8) -> Self {
        let logo_data = include_bytes!("robobuzz.bmp");
        let text_data = include_bytes!("robojackets.bmp");
        let logo_bmp = Bmp::from_slice(logo_data).unwrap();
        let text_bmp = Bmp::from_slice(text_data).unwrap();
        let instance = StartScreen {
            logo: logo_bmp,
            text: text_bmp,
            logo_pos: logo_pos,
            text_pos: text_pos,
            blue_team,
            robot_id,
        };
        return instance;
    }
}

impl Drawable for StartScreen<'_> {
    type Color = BinaryColor;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color>,
    {
        Image::new(&self.logo, self.logo_pos).draw(target)?;
        Image::new(&self.text, self.text_pos).draw(target)?;

        let text_style = TextStyleBuilder::new()
            .alignment(Alignment::Left)
            .baseline(Baseline::Middle)
            .build();
        let char_style = MonoTextStyle::new(&FONT_6X9, BinaryColor::On);
        Text::with_text_style(
            if self.blue_team {
                "Blue Team"
            } else {
                "Yellow Team"
            },
            Point::new(45, 30),
            char_style,
            text_style,
        )
        .draw(target)?;
        Text::with_text_style(
            &format!("Robot {}", self.robot_id),
            Point::new(45, 50),
            char_style,
            text_style,
        )
        .draw(target)?;
        Ok(())
    }
}
