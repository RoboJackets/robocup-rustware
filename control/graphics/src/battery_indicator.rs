extern crate alloc;
use alloc::format;
use embedded_graphics::{
    mono_font::{ascii::FONT_7X13, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Baseline, Text, TextStyleBuilder}
};


/**
 * Battery indicator as a battery icon and percentage value.
 * Uses Drawable trait.
 */
pub struct BatteryIndicator {
    pub top_left: Point,
    pub percent: u32,
}

impl BatteryIndicator {
    pub fn new(top_left: Point) -> Self {
        let instance = BatteryIndicator {
            top_left: top_left,
            percent: 0,
        };
        return instance;
    }

    pub fn update(&mut self, percent: u32) {
        if percent > 100 {
            return;
        }
        self.percent = percent;
    }
}

impl Drawable for BatteryIndicator {
    
    type Color = BinaryColor;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error> 
    where D: DrawTarget<Color = BinaryColor>,
    {
        let fill_style = PrimitiveStyle::with_fill(BinaryColor::On);
        let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

        let width_fill = ((self.percent as f32 / 100.0) * 12.0) as u32;

        let text = &format!("{}%", self.percent);
        
        let text_style = TextStyleBuilder::new()
            .alignment(Alignment::Right)
            .baseline(Baseline::Middle)
            .build();
        let char_style = MonoTextStyle::new(&FONT_7X13, BinaryColor::On);

        Rectangle::new(self.top_left, Size::new(15, 8))
            .into_styled(line_style)
            .draw(target)?;
        Rectangle::new(Point::new(self.top_left.x + 15, self.top_left.y + 2), Size::new(2, 4))
            .into_styled(fill_style)
            .draw(target)?;
        Rectangle::new(self.top_left, Size::new(width_fill, 6))
            .into_styled(fill_style)
            .draw(target)?;
        Text::with_text_style(
            text,
            Point::new(self.top_left.x + - 3, self.top_left.y + 3),
            char_style,
            text_style,
        ).draw(target)?;
        Ok(())
    }
}
