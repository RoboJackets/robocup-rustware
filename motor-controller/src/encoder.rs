//!
//! Encoder Abstraction to handle encoder counting
//! 
//! new	new	old	old
//!	pin2	pin1	pin2	pin1	Result
//!	----	----	----	----	------
//!	0	0	0	0	no movement
//!	0	0	0	1	+1
//!	0	0	1	0	-1
//!	0	0	1	1	+2  (assume pin1 edges only)
//!	0	1	0	0	-1
//!	0	1	0	1	no movement
//!	0	1	1	0	-2  (assume pin1 edges only)
//!	0	1	1	1	+1
//!	1	0	0	0	+1
//!	1	0	0	1	-2  (assume pin1 edges only)
//!	1	0	1	0	no movement
//!	1	0	1	1	-1
//!	1	1	0	0	+2  (assume pin1 edges only)
//!	1	1	0	1	-1
//!	1	1	1	0	+1
//!	1	1	1	1	no movement
//! 

use core::fmt::Debug;
use embedded_hal::digital::v2::InputPin;

/// Encoder abstraction
pub struct Encoder<E1, E2> {
    /// The last encoder 1 value
    last_e1: bool,
    /// The last encoder 2 value
    last_e2: bool,

    /// The current encoder count
    pub count: i64,

    /// The first encoder input
    e1: E1,
    /// The second encoder input
    e2: E2,
}

impl<E1, E2, GPIOE> Encoder<E1, E2> where 
    E1: InputPin<Error=GPIOE>,
    E2: InputPin<Error=GPIOE>,
    GPIOE: Debug,
{
    /// Create a new encoder abstraction
    pub fn new(e1: E1, e2: E2) -> Self {
        let last_e1 = e1.is_high().unwrap();
        let last_e2 = e2.is_high().unwrap();

        Self {
            last_e1,
            last_e2,
            count: 0,
            e1,
            e2
        }
    }

    /// Update the encoder count value
    pub fn update(&mut self) -> Result<(), GPIOE> {
        let new_e1 = self.e1.is_high()?;
        let new_e2 = self.e2.is_high()?;

        match (new_e2, new_e1, self.last_e2, self.last_e1) {
            (false, false, false, false) |
            (false, true, false, true) |
            (true, false, true, false) |
            (true, true, true, true) => (),
            (false, false, false, true) |
            (false, true, true, true) |
            (true, false, false, false) |
            (true, true, true, false) => self.count += 1,
            (false, false, true, true) |
            (true, true, false, false) => self.count += 2,
            (false, true, true, false) |
            (true, false, false, true) => self.count -= 2,
            (false, false, true, false) |
            (false, true, false, false) |
            (true, false, true, true) |
            (true, true, false, true) => self.count -= 1,
        }
        
        Ok(())
    }
}