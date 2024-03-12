//!
//! Error for the io-expander chip
//! 

/// Error from the IO Expander
pub enum IOExpanderError<E> {
    I2CError(E),
}