#![no_std]

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    image::Image,
};

use tinybmp::Bmp;

pub struct StartScreen<'a> {
    logo: Bmp<'a, BinaryColor>,
    text: Bmp<'a, BinaryColor>,
    logo_pos: Point,
    text_pos: Point,
}

impl StartScreen<'_> {
    pub fn new(logo_pos: Point, text_pos: Point)
    -> Self {
        let logo_data = include_bytes!("robobuzz.bmp");
        let text_data = include_bytes!("robojackets.bmp");
        let logo_bmp = Bmp::from_slice(logo_data).unwrap();
        let text_bmp = Bmp::from_slice(text_data).unwrap();
        let instance = StartScreen {
            logo: logo_bmp,
            text: text_bmp,
            logo_pos: logo_pos,
            text_pos: text_pos,
        };
        return instance;
    }
}

impl Drawable for StartScreen<'_> {
    type Color = BinaryColor;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where D: DrawTarget<Color = Self::Color>
    {
        Image::new(&self.logo, self.logo_pos).draw(target)?;
        Image::new(&self.text, self.text_pos).draw(target)?;
        Ok(())
    }
}