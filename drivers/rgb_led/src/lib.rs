#![no_std]

use core::marker::PhantomData;

use embedded_hal::spi::FullDuplex;

pub struct LedDriver<'a, T> {
    spi: T,
    _lifetime: PhantomData<&'a ()>
}

impl<'a, T: FullDuplex<u8> + 'a> LedDriver<'a, T> {
    pub fn new(spi: T) -> Self {
        Self {
            spi,
            _lifetime: PhantomData,
        }
    }

    fn write(&mut self) -> nb::Result<(), T::Error> {
        self.spi.send(0)
    }

    fn read(&mut self) -> nb::Result<u8, T::Error> {
        self.spi.read()
    }
} 
