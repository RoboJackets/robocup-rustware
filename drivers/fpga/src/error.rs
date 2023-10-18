//!
//! Errors for Operating the FPGA Module
//! 

use core::fmt::Debug;

use defmt_macros::Format;

pub fn convert_gpio_error<GpioE: Debug, SpiE: Debug>(err: GpioE) -> FpgaError<GpioE, SpiE> { FpgaError::GpioError(err) }
pub fn convert_spi_error<GpioE: Debug, SpiE: Debug>(err: SpiE) -> FpgaError<GpioE, SpiE> { FpgaError::SpiError(err) }
pub fn convert_error<GpioE: Debug, SpiE: Debug>(err: SpiE) -> FpgaError<GpioE, SpiE> { FpgaError::Other(err) }

#[derive(Debug, Clone, Copy, Format)]
pub enum FpgaError<GpioE: Debug, SpiE: Debug> {
    NoDelay,
    InitTimeout,
    UnableToSendConfiguration,
    UnableToSetConfiguration,
    InvalidGitHash,
    InvalidDutyCycle,
    WrongPinMode(GpioE),
    GpioError(GpioE),
    SpiError(SpiE),
    Other(SpiE),
}