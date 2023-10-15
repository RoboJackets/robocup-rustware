#![no_std]

use embedded_hal::digital::v2::OutputPin;

pub struct LedDriver<R, G, B> {
    r_pin: R,
    g_pin: G,
    b_pin: B,
}

impl<R: OutputPin<Error = E>, G: OutputPin<Error = E>, B: OutputPin<Error = E>, E>
    LedDriver<R, G, B>
{
    pub fn new(r_pin: R, g_pin: G, b_pin: B) -> Self {
        Self {
            r_pin,
            g_pin,
            b_pin,
        }
    }

    pub fn off(&mut self) -> Result<(), E> {
        self.r_pin.set_low()?;
        self.g_pin.set_low()?;
        self.b_pin.set_low()
    }

    pub fn red(&mut self) -> Result<(), E> {
        self.off()?;
        self.r_pin.set_high()
    }

    pub fn green(&mut self) -> Result<(), E> {
        self.off()?;
        self.g_pin.set_high()
    }

    pub fn blue(&mut self) -> Result<(), E> {
        self.off()?;
        self.b_pin.set_high()
    }

    pub fn yellow(&mut self) -> Result<(), E> {
        self.red()?;
        self.g_pin.set_high()
    }

    pub fn turquoise(&mut self) -> Result<(), E> {
        self.green()?;
        self.b_pin.set_high()
    }

    pub fn purple(&mut self) -> Result<(), E> {
        self.red()?;
        self.b_pin.set_high()
    }

    pub fn white(&mut self) -> Result<(), E> {
        self.red()?;
        self.g_pin.set_high()?;
        self.b_pin.set_high()
    }
}
