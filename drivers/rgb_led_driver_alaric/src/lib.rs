#![no_std]

use embedded_hal::digital::v2::OutputPin;

pub enum Color {
    Red,
    Green,
    Blue,
    Yellow,
    Purple,
    Cyan,
}



pub struct RGBLed<P1, P2, P3>
where
    P1: OutputPin,
    P2: OutputPin,
    P3: OutputPin,
{
    r: P1,
    g: P2,
    b: P3,
}

impl<P1, P2, P3> RGBLed<P1, P2, P3>
where
    P1: OutputPin,
    P2: OutputPin<Error=P1::Error>,
    P3: OutputPin<Error=P1::Error>,
{
    pub fn new(r_pin: P1, g_pin: P2, b_pin: P3) -> RGBLed<P1, P2, P3> {
        RGBLed {
            r: r_pin,
            g: g_pin,
            b: b_pin,
        }
    }

    pub fn set_color(&mut self, color: Color) -> Result<(), P1::Error> {
        match color {
            Color::Red => {
                self.r.set_high()?;
                self.g.set_low()?;
                self.b.set_low()?;
            }
            Color::Green => {
                self.r.set_low()?;
                self.g.set_high()?;
                self.b.set_low()?;
            }
            Color::Blue => {
                self.r.set_low()?;
                self.g.set_low()?;
                self.b.set_high()?;
            }
            Color::Yellow => {
                self.r.set_high()?;
                self.g.set_high()?;
                self.b.set_low()?;
            }
            Color::Purple => {
                self.r.set_high()?;
                self.g.set_low()?;
                self.b.set_high()?;
            }
            Color::Cyan => {
                self.r.set_low()?;
                self.g.set_high()?;
                self.b.set_high()?;
            }
        }
        Ok(())
    }

    pub fn clear(&mut self) -> Result<(), P1::Error> {
        self.r.set_low()?;
        self.g.set_low()?;
        self.b.set_low()?;
        Ok(())
    }
}