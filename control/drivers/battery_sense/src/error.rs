//!
//! Errors that can occur with the  Battery Driver
//! 


use core::fmt::Debug;


// having the generic error type allows us to pass in the actual error type in lib.rs
#[derive(Debug)]
/// an error from using the IO Expander
pub enum BatteryError {
    
    /// error directly from the I2C line
    ADC
}
