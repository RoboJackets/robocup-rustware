//!
//! Errors that can occur with the IO Expander
//! 


use core::fmt::Debug;


// having the generic error type allows us to pass in the actual error type in lib.rs
#[derive(Debug)]
pub enum RotaryError<IOExpanderError: Debug> {

    IOExpanderError(IOExpanderError),

    Other
}