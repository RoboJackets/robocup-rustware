#![no_std]
#![crate_type = "lib"]

use embedded_hal::digital::v2::InputPin;

/// A generic rotary pin driver that reads values from multiple digital input pins.
pub struct RotaryPinDriver<ONE:InputPin<Error = GPIOE>, TWO:InputPin<Error = GPIOE>, THREE:InputPin<Error = GPIOE>, 
    FOUR:InputPin<Error = GPIOE>, GPIOE> {
    _one: ONE,
    _two: TWO,
    _three: THREE,
    _four: FOUR,
    _gpio: GPIOE,
}

impl<const NUM_PINS: usize> RotaryPinDriver<NUM_PINS> {
    /// Creates a new instance of `RotaryPinDriver`.
    pub fn new(one: _one, two:_two, three:_three, four:_four) -> Self {
        Self { _one: one, _two: two, _three: three, _four: four, _gpio: GPIOE }
    }

    /// Reads the value from the rotary pins and returns the result.
    pub fn read<T>(&self, pins: [T; NUM_PINS]) -> u8
    where
        T: InputPin<Error = ()>,
    {
        pins.iter()
            .enumerate()
            .fold(0, |reading, (i, pin)| {
                let is_high = pin.is_high().unwrap_or(false);
                reading | ((is_high as u8) << i)
            })
    }
}