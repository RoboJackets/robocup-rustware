#![no_std]

use embedded_hal::prelude::{_embedded_hal_blocking_spi_Write, _embedded_hal_serial_Write};
use embedded_hal::serial::Write;
use crate::RgbLedError::{BluePinError, GreenPinError, RedPinError};

pub struct RgbLedDriver<R: Write<u8>, G: Write<u8>, B: Write<u8>> {
    r: R,
    g: G,
    b: B,
}

pub enum RgbLedError<R: Write<u8>, G: Write<u8>, B: Write<u8>> {
    RedPinError(nb::Error<R::Error>),
    GreenPinError(nb::Error<G::Error>),
    BluePinError(nb::Error<B::Error>),
}

impl<R: Write<u8>, G: Write<u8>, B: Write<u8>> RgbLedDriver<R, G, B> {
    pub fn new(r: R, g: G, b: B) -> Self {
        Self {r, g, b}
    }

    pub fn set_color(&mut self, color: Color) -> Result<(), RgbLedError<R, G, B>> {
        let rgb = color.get_rgb();
        self.r.write(rgb.r)
            .map_err(|e| RedPinError(e))?;
        self.g.write(rgb.g)
            .map_err(GreenPinError)?;
        self.b.write(rgb.b)
            .map_err(BluePinError)
    }

    pub fn off(&mut self) -> Result<(), RgbLedError<R, G, B>> {
        self.r.write(0).map_err(RedPinError)?;
        self.g.write(0).map_err(GreenPinError)?;
        self.b.write(0).map_err(BluePinError)
    }
}

pub enum Color {
    Red,
    Green,
    Blue,
    Purple,
    Yellow,
    Cyan,
    Turquoise,
    Custom { r: u8, g: u8, b: u8 }
}

impl Color {
    fn get_rgb(&self) -> RgbColor {
        match self {
            Color::Red => RgbColor::red(),
            Color::Green => RgbColor::green(),
            Color::Blue => RgbColor::blue(),
            Color::Purple => RgbColor::purple(),
            Color::Yellow => RgbColor::yellow(),
            Color::Cyan => RgbColor::cyan(),
            Color::Turquoise => RgbColor::turquoise(),
            Color::Custom {r, g, b} => RgbColor::new(*r, *g, *b)
        }
    }
}

struct RgbColor {
    r: u8,
    g: u8,
    b: u8
}

impl RgbColor {

    fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b}
    }

    fn red() -> Self {
        Self::new(255, 0, 0)
    }
    fn green() -> Self {
        Self::new(0, 225, 0)
    }
    fn blue() -> Self {
        Self::new(0, 0, 255)
    }
    fn purple() -> Self {
        Self::new(255,0,255)
    }
    fn yellow() -> Self {
        Self::new(255, 225, 0)
    }
    fn cyan() -> Self {
        Self::new(0, 255, 255)
    }
    fn turquoise() -> Self {
        Self::new(64,224,208)
    }
}