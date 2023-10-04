//!
//! Errors for the Kicker Module
//! 

use core::fmt::Debug;

use defmt::Format;

pub fn convert_gpio_error<E: Debug>(err: E) -> KickerBoardError<E> { KickerBoardError::Gpio(err) }
pub fn convert_spi_error<E: Debug>(err: E) -> KickerBoardError<E> { KickerBoardError::Spi(err) }
pub fn convert_error<E: Debug>(err: E) -> KickerBoardError<E> { KickerBoardError::Other(err) }

#[derive(Debug, Clone, Copy, Format)]
pub enum KickerBoardError<E> {
    UnableToEnableProgramming,
    Gpio(E),
    Spi(E),
    Other(E),
}