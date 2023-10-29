#![no_std]

use embedded_hal::digital::v2::{OutputPin, PinState};

use crate::RgbLedError::{BluePinError, GreenPinError, RedPinError};

pub struct RgbLedDriver<R: OutputPin, G: OutputPin, B: OutputPin> {
    r: R,
    g: G,
    b: B,
}

pub enum RgbLedError<R: OutputPin, G: OutputPin, B: OutputPin> {
    RedPinError(R::Error),
    GreenPinError(G::Error),
    BluePinError(B::Error),
}

impl<R: OutputPin, G: OutputPin, B: OutputPin> RgbLedDriver<R, G, B> {
    pub fn new(r: R, g: G, b: B) -> Self {
        Self { r, g, b }
    }

    pub fn set_color(&mut self, color: Color) -> Result<(), RgbLedError<R, G, B>> {
        let rgb = color.get_rgb();
        self.r.set_state(rgb.r).map_err(RedPinError)?;
        self.g.set_state(rgb.g).map_err(GreenPinError)?;
        self.b.set_state(rgb.b).map_err(BluePinError)?;
        Ok(())
    }

    pub fn off(&mut self) -> Result<(), RgbLedError<R, G, B>> {
        self.r.set_low().map_err(RedPinError)?;
        self.g.set_low().map_err(GreenPinError)?;
        self.b.set_low().map_err(BluePinError)?;
        Ok(())
    }
}

pub enum Color {
    Red,
    Green,
    Blue,
    Purple,
    Yellow,
    Cyan,
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
        }
    }
}

struct RgbColor {
    r: PinState,
    g: PinState,
    b: PinState,
}

impl RgbColor {
    fn new(r: PinState, g: PinState, b: PinState) -> Self {
        Self { r, g, b }
    }

    fn red() -> Self {
        Self::new(PinState::High, PinState::Low, PinState::Low)
    }
    fn green() -> Self {
        Self::new(PinState::Low, PinState::High, PinState::Low)
    }
    fn blue() -> Self {
        Self::new(PinState::Low, PinState::Low, PinState::High)
    }
    fn purple() -> Self {
        Self::new(PinState::High, PinState::Low, PinState::High)
    }
    fn yellow() -> Self {
        Self::new(PinState::High, PinState::High, PinState::Low)
    }
    fn cyan() -> Self {
        Self::new(PinState::Low, PinState::High, PinState::High)
    }
}

