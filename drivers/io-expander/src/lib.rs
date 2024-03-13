//!
//! Driver for the MCP23017 io-expander chip.
//! 

#![no_std]

extern crate alloc;
use alloc::rc::Rc;

mod register;

pub mod error;

pub use error::IOExpanderError;

pub mod pin;
pub use pin::Pin;

use core::cell::RefCell;
use embedded_hal::digital::v2::{InputPin, OutputPin, IoPin, PinState};
use embedded_hal::blocking::i2c::{Write, Read};
use register::Register;

pub const IO_EXPANDER_ADDRESS: u8 = 0x00;

#[inline(always)]
pub const fn buffer_to_u16(buffer: [u8; 2]) -> u16 {
    u16::from_le_bytes(buffer)
}

#[inline(always)]
pub const fn u16_to_buffer(value: u16) -> (u8, u8) {
    let result = value.to_le_bytes();
    (result[0], result[1])
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinMode {
    Input,
    Output,
}

pub enum PinError<E> {
    BusBusy,
    IncorrectPinMode,
    I2CError(E),
}

pub struct IOExpanderPin<I2C> {
    pin: Pin,
    cached_gpio: Rc<RefCell<u16>>,
    cached_pin_mode: Rc<RefCell<u16>>,
    i2c: Rc<RefCell<I2C>>,
}

pub struct IOExpanderInput<I2C> {
    pin: IOExpanderPin<I2C>,
}

pub struct IOExpanderOutput<I2C> {
    pin: IOExpanderPin<I2C>,
}

pub struct IOExpander<I2C> {
    pa0: Option<IOExpanderPin<I2C>>,
    pa1: Option<IOExpanderPin<I2C>>,
    pa2: Option<IOExpanderPin<I2C>>,
    pa3: Option<IOExpanderPin<I2C>>,
    pa4: Option<IOExpanderPin<I2C>>,
    pa5: Option<IOExpanderPin<I2C>>,
    pa6: Option<IOExpanderPin<I2C>>,
    pa7: Option<IOExpanderPin<I2C>>,
    pb0: Option<IOExpanderPin<I2C>>,
    pb1: Option<IOExpanderPin<I2C>>,
    pb2: Option<IOExpanderPin<I2C>>,
    pb3: Option<IOExpanderPin<I2C>>,
    pb4: Option<IOExpanderPin<I2C>>,
    pb5: Option<IOExpanderPin<I2C>>,
    pb6: Option<IOExpanderPin<I2C>>,
    pb7: Option<IOExpanderPin<I2C>>,

    cached_polarity_mask: u16,
    cached_pullup_mask: u16,
}

impl<I2C, E> IOExpander<I2C> where I2C: Write<Error=E> + Read<Error=E> {
    pub fn new(i2c: I2C) -> Self {
        let i2c = Rc::new(RefCell::new(i2c));
        let cached_gpio = Rc::new(RefCell::new(0));
        let cached_pin_mode = Rc::new(RefCell::new(0));

        Self {
            pa0: Some(IOExpanderPin { pin: Pin::PA0, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pa1: Some(IOExpanderPin { pin: Pin::PA1, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pa2: Some(IOExpanderPin { pin: Pin::PA2, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pa3: Some(IOExpanderPin { pin: Pin::PA3, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pa4: Some(IOExpanderPin { pin: Pin::PA4, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pa5: Some(IOExpanderPin { pin: Pin::PA5, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pa6: Some(IOExpanderPin { pin: Pin::PA6, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pa7: Some(IOExpanderPin { pin: Pin::PA7, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pb0: Some(IOExpanderPin { pin: Pin::PB0, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pb1: Some(IOExpanderPin { pin: Pin::PB1, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pb2: Some(IOExpanderPin { pin: Pin::PB2, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pb3: Some(IOExpanderPin { pin: Pin::PB3, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pb4: Some(IOExpanderPin { pin: Pin::PB4, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pb5: Some(IOExpanderPin { pin: Pin::PB5, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pb6: Some(IOExpanderPin { pin: Pin::PB6, cached_gpio: cached_gpio.clone(), cached_pin_mode: cached_pin_mode.clone(), i2c: i2c.clone() }),
            pb7: Some(IOExpanderPin { pin: Pin::PB7, cached_gpio, cached_pin_mode, i2c }),
            cached_polarity_mask: 0,
            cached_pullup_mask: 0,
        }
    }

    pub fn take(&mut self, pin: Pin) -> Option<IOExpanderPin<I2C>> {
        match pin {
            Pin::PA0 => self.pa0.take(),
            Pin::PA1 => self.pa1.take(),
            Pin::PA2 => self.pa2.take(),
            Pin::PA3 => self.pa3.take(),
            Pin::PA4 => self.pa4.take(),
            Pin::PA5 => self.pa5.take(),
            Pin::PA6 => self.pa6.take(),
            Pin::PA7 => self.pa7.take(),
            Pin::PB0 => self.pb0.take(),
            Pin::PB1 => self.pb1.take(),
            Pin::PB2 => self.pb2.take(),
            Pin::PB3 => self.pb3.take(),
            Pin::PB4 => self.pb4.take(),
            Pin::PB5 => self.pb5.take(),
            Pin::PB6 => self.pb6.take(),
            Pin::PB7 => self.pb7.take(),
        }
    }

    pub fn set_polarity_mask(&mut self, mask: u16) -> Result<(), PinError<E>> {
        todo!()
    }

    pub fn set_pin_polarity_mask(&mut self, pin: IOExpanderInput<E>, masked: bool) -> Result<(), PinError<E>> {
        todo!()
    }

    pub fn set_pullup_mask(&mut self, mask: u16) -> Result<(), PinError<E>> {
        todo!()
    }

    pub fn set_pin_pullup_mask(&mut self, pin: IOExpanderInput<E>, pullup: bool) -> Result<(), PinError<E>> {
        todo!()
    }
}

impl<I2C, E> IOExpanderPin<I2C> 
    where I2C: Write<Error=E> + Read<Error=E> {
    fn read_gpio(&self) -> Result<u16, PinError<E>> {
        let gpio_value = self.read_register(Register::GPIO)?;
        *(self.cached_gpio.borrow_mut()) = gpio_value;
        Ok(gpio_value)
    }

    fn write_gpio(&self, state: PinState) -> Result<(), PinError<E>> {
        let mut cached_value = self.cached_gpio.borrow_mut();
        match state {
            PinState::High => {
                *cached_value |= 1 << self.pin.pin_num();
                self.write_register(Register::GPIO, *cached_value)
            },
            PinState::Low => {
                *cached_value &= !(1 << self.pin.pin_num());
                self.write_register(Register::GPIO, *cached_value)
            },
        }
    }

    fn read_register(&self, register: Register) -> Result<u16, PinError<E>> {
        if let Ok(mut i2c) = self.i2c.try_borrow_mut() {
            let mut buffer = [0u8; 2];
            i2c.write(IO_EXPANDER_ADDRESS, &[register.address()]).map_err(PinError::I2CError)?;
            i2c.read(IO_EXPANDER_ADDRESS, &mut buffer).map_err(PinError::I2CError)?;
            Ok(buffer_to_u16(buffer))
        } else {
            Err(PinError::BusBusy)
        }
    }

    fn write_register(&self, register: Register, data: u16) -> Result<(), PinError<E>> {
        if let Ok(mut i2c) = self.i2c.try_borrow_mut() {
            let (high, low) = u16_to_buffer(data);
            i2c.write(IO_EXPANDER_ADDRESS, &[register.address(), high, low]).map_err(PinError::I2CError)
        } else {
            Err(PinError::BusBusy)
        }
    }
}

impl<I2C, E> InputPin for IOExpanderInput<I2C>
    where I2C: Write<Error=E> + Read<Error=E> {
    type Error = PinError<E>;

    fn is_high(&self) -> Result<bool, Self::Error> {
        let value = self.pin.read_gpio()?;
        Ok((value & (1 << self.pin.pin.pin_num())) != 0)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        let value = self.pin.read_gpio()?;
        Ok((value & (1 << self.pin.pin.pin_num())) == 0)
    }
}

impl<I2C, E> OutputPin for IOExpanderOutput<I2C>
    where I2C: Write<Error=E> + Read<Error=E> {
    type Error = PinError<E>;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.write_gpio(PinState::High)
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.write_gpio(PinState::Low)
    }

    fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> {
        self.pin.write_gpio(state)
    }
}

impl<I2C, E> IoPin<IOExpanderInput<I2C>, IOExpanderOutput<I2C>> for IOExpanderInput<I2C>
    where I2C: Write<Error=E> + Read<Error=E> {
    type Error = PinError<E>;

    fn into_input_pin(self) -> Result<IOExpanderInput<I2C>, Self::Error> {
        Ok(self)
    }

    fn into_output_pin(self, state: PinState) -> Result<IOExpanderOutput<I2C>, Self::Error> {
        todo!()
    }
}

impl<I2C, E> IoPin<IOExpanderInput<I2C>, IOExpanderOutput<I2C>> for IOExpanderOutput<I2C>
    where I2C: Write<Error=E> + Read<Error=E> {
    type Error = PinError<E>;

    fn into_input_pin(self) -> Result<IOExpanderInput<I2C>, Self::Error> {
        todo!()
    }

    fn into_output_pin(self, state: PinState) -> Result<IOExpanderOutput<I2C>, Self::Error> {
        self.pin.write_gpio(state)?;
        Ok(self)
    }
}

impl<I2C, E> IoPin<IOExpanderInput<I2C>, IOExpanderOutput<I2C>> for IOExpanderPin<E>
    where I2C: Write<Error=E> + Read<Error=E> {
    type Error = PinError<E>;

    fn into_input_pin(self) -> Result<IOExpanderInput<I2C>, Self::Error> {
        todo!()
    }

    fn into_output_pin(self, state: PinState) -> Result<IOExpanderOutput<I2C>, Self::Error> {
        todo!()
    }
}
