#![no_std]

pub trait ConfigurableSpi {
    type Error;

    fn set_frequency(&mut self, frequency: fugit::HertzU32) -> Result<(), Self::Error>;
}
