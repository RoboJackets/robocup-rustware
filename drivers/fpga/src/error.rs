//!
//! Errors for Operating the FPGA Module
//! 

use core::fmt::Debug;
use defmt::Format;

pub fn convert_gpio_error<E: Debug>(err: E) -> FpgaError<E> { FpgaError::GpioError(err) }
pub fn convert_spi_error<E: Debug>(err: E) -> FpgaError<E> { FpgaError::SpiError(err) }
pub fn convert_error<E: Debug>(err: E) -> FpgaError<E> { FpgaError::Other(err) }

#[derive(Debug, Clone, Copy, Format)]
pub enum FpgaError<E> {
    NoDelay,
    InitTimeout,
    UnableToSendConfiguration,
    UnableToSetConfiguration,
    InvalidGitHash,
    InvalidDutyCycle,
    WrongPinMode(E),
    GpioError(E),
    SpiError(E),
    Other(E),
}