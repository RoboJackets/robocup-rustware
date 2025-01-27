//!
//! Errors that can occur with the IO Expander
//! 


use core::fmt::Debug;


// having the generic error type allows us to pass in the actual error type in lib.rs
#[derive(Debug)]
// an error from using the IO Expander
pub enum IOExpanderError<I2CError: Debug> {
    // figure out what the i2c error is called
    // seems like this isn't a type, just a label
    I2C(I2CError),
    // An Error received from the expander
    BadResponse(u8, u8) // address, expected
}