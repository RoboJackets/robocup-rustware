#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_panic as _;

// This is the trait that we are going to use to drive output to the LED
use embedded_hal::digital::v2::OutputPin;

// Struct to represent the LED, generic T will be the OutputPin Trait from embedded HAL
pub struct rbgLED<T> {
    r: T,
    g: T,
    b: T,
}

impl<T: OutputPin<E>> rbgLED<T> {
    pub fn SomethingOrOther() -> result {
        
    }
}


/*!pub fn add(left: usize, right: usize) -> usize {
    left + right
}*/

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
