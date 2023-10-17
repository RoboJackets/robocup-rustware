#![no_std]

use core::fmt::Debug;

pub trait ConfigurableSpi {
    type Error: Debug;

    fn set_frequency(&mut self, frequency: fugit::HertzU32) -> Result<(), Self::Error>;
}
