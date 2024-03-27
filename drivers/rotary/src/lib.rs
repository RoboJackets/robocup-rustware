#![no_std]
#![crate_type = "lib"]
#![feature(type_alias_impl_trait)]

// goal with the feature flags is to have the driver compile based on whether the InputPin trait is available



#[cfg(feature = "input_pin")]
use embedded_hal::digital::v2::InputPin;
#[cfg(not(feature = "input_pin"))]
use teensy4_bsp as bsp;
use bsp::hal::gpio::Input; // not being used because of temporary solution that avoid Input<> problem

// A generic rotary pin driver that reads values from multiple digital input pins.
pub struct RotaryPinDriver<ONE, TWO, THREE, FOUR> {
    _one: ONE,
    _two: TWO,
    _three: THREE,
    _four: FOUR,
}

#[cfg(feature = "input_pin")]
impl<ONE: InputPin<Error = GPIOE>, TWO: InputPin<Error = GPIOE>, THREE: InputPin<Error = GPIOE>, 
     FOUR: InputPin<Error = GPIOE>, GPIOE> RotaryPinDriver<ONE, TWO, THREE, FOUR, GPIOE> {
    // Creates a new instance of `RotaryPinDriver`.
    pub fn new(one: ONE, two: TWO, three: THREE, four: FOUR) -> Self {
        Self { _one: one, _two: two, _three: three, _four: four }
    }

    // Reads the value from the rotary pins and returns the result as a u8
    pub fn read(&self) -> u8
    {
        // once the object has been created, we can read the values from the pins that are connected to the rotary encoder
        let pins: [&dyn InputPin<Error = GPIOE>; 4] = [&self._one, &self._two, &self._three, &self._four];

        // read values from pins to an output that is 0-15 
        // pin one will map to 2^0, pin two will map to 2^1, pin three will map to 2^2, and pin four will map to 2^3
        let mut result: u8 = 0;
        for i in 0..4 {
            if pins[i].is_high().unwrap_or_default() {
                result += 2u8.pow(i as u32);
            }
        }

        result
    }
}


// MUST BE USING THE INPUT<> INSTEAD OF THE INPUTPIN<> 
#[cfg(not(feature = "input_pin"))]
impl<ONE, TWO, THREE, FOUR> RotaryPinDriver<ONE, TWO, THREE, FOUR> 
{
    // Creates a new instance of `RotaryPinDriver`.
    pub fn new(one: ONE, two: TWO, three: THREE, four: FOUR) -> Self {
        Self { _one: one, _two: two, _three: three, _four: four }
    }

    // Reads the value from the rotary pins and returns the result as a u8
    pub fn read(&self) -> u8
    {

        // once the object has been created, we can read the values from the pins that are connected to the rotary encoder
        // i need an array of Input<> pins 

        // ideal solution that doesn't work: 
        // let pins: [&dyn Input; 4] = [&self._one, &self._two, &self._three, &self._four];

        // CHANGED THE FOLLOWING LOOP TO BE A DUMMY LOOP BC OF COMMENTED LINE ABOVE
        let mut result: u8 = 0;
        for i in 0..4 {
            if i % 2 == 0 {
                result += 2u8.pow(i as u32);
            }
        }

        result
    }
}