extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

use embedded_graphics::{
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_5X8, FONT_7X13},
    },
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};

pub struct ErrorScreen<'a> {
    heading: &'a str,
    message: &'a str,
}

impl<'a> ErrorScreen<'a> {
    pub fn new(heading: &'a str, message: &'a str) -> Self {
        let instance = ErrorScreen { heading, message };
        return instance;
    }

    /**
     * Wraps text around at 24 column intervals.
     * Does not account for word endings.
     * @return Vector of String, one element for each line on the display
     */
    pub fn wrap(&self, text: &str) -> Vec<String> {
        let mut strs: Vec<String> = Vec::new();
        let iters = text.len() / 24;
        if iters == 0 {
            strs.push(text.into());
            return strs;
        } else {
            for i in 0..iters {
                let temp = text[(i * 24)..((i + 1) * 24)].into();
                strs.push(temp);
            }
            strs.push(text[(iters * 24)..].into());
            return strs;
        }
    }

    pub fn update(&mut self, heading: &'a str, message: &'a str) {
        self.heading = heading;
        self.message = message;
    }
}

impl<'a> Drawable for ErrorScreen<'a> {
    type Output = ();
    type Color = BinaryColor;

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color>,
    {
        let text_style = TextStyleBuilder::new()
            .alignment(Alignment::Left)
            .baseline(Baseline::Middle)
            .build();
        let char_style = MonoTextStyle::new(&FONT_5X8, BinaryColor::On);
        let header_style = MonoTextStyle::new(&FONT_7X13, BinaryColor::On);

        Text::with_text_style(self.heading, Point::new(1, 8), header_style, text_style)
            .draw(target)?;
        let wrapped = self.wrap(self.message);
        for (i, line) in wrapped.iter().enumerate() {
            Text::with_text_style(
                line,
                Point::new(1, (20 + 7 * i) as i32),
                char_style,
                text_style,
            )
            .draw(target)?;
        }
        Ok(())
    }
}
