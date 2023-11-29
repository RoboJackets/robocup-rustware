#![no_std]
#![allow(non_camel_case_types)]

//use teensy4_panic as _;

// This is the trait that we are going to use to drive output to the LED
use embedded_hal::digital::v2::OutputPin;

// Struct to represent the LED, generic T will be the OutputPin Trait from embedded HAL
pub struct rbg_LED_Driver<R, G, B> {
    r: R,
    g: G,
    b: B,
}

impl<R, G, B> rbg_LED_Driver<R, G, B> 
where 
    R: OutputPin,
    G: OutputPin<Error=R::Error>,
    B: OutputPin<Error=R::Error>,
{
    pub fn new(r: R, g: G, b: B) -> Self {
        Self {
            r,
            g,
            b,
        }
    }

    pub fn off (&mut self) -> Result<(), R::Error> {
        self.r.set_low()?;
        self.g.set_low()?;
        self.b.set_low()?;

        Ok(())
    }

    pub fn red(&mut self) -> Result<(), R::Error> {
        self.r.set_high()?;
        self.g.set_low()?;
        self.b.set_low()?;

        Ok(())
    }

    pub fn green(&mut self) -> Result<(), R::Error> {
        self.r.set_low()?;
        self.g.set_high()?;
        self.b.set_low()?;

        Ok(())
    }

    pub fn blue(&mut self) -> Result<(), R::Error> {
        self.r.set_low()?;
        self.g.set_low()?;
        self.b.set_high()?;

        Ok(())
    }

    pub fn yellow(&mut self) -> Result<(), R::Error> {
        self.r.set_high()?;
        self.g.set_high()?;
        self.b.set_low()?;

        Ok(())
    }

    pub fn purple(&mut self) -> Result<(), R::Error> {
        self.r.set_high()?;
        self.g.set_low()?;
        self.b.set_high()?;
        
        Ok(())
    }

    pub fn cyan(&mut self) -> Result<(), R::Error> {
        self.r.set_low()?;
        self.g.set_high()?;
        self.b.set_high()?;

        Ok(())
    }
}