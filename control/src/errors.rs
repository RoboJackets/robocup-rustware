//!
//! Definitions of the common errors present in Robocup-Rustware to
//! make error handling more smooth.
//!

use core::convert::Infallible;

use teensy4_bsp::hal::{lpi2c::ControllerStatus, lpspi::LpspiError};

use icm42605_driver::ImuError;

use fpga_rs::error::FpgaError;

use rtic_nrf24l01::error::RadioError;

use kicker_programmer::KickerProgrammerError;

use kicker_controller::KickerServiceError;

/// Error for initializing the IMU
pub type ImuInitError = ImuError<ControllerStatus>;
/// Error for Programming the FPGA
pub type FPGAProgError = FpgaError<LpspiError, Infallible>;
/// Error for initializing the FPGA
pub type FPGAInitError = FpgaError<LpspiError, Infallible>;
/// Error for initializing the Radio
pub type RadioInitError = RadioError;
/// Error for programming the Kicker
pub type KickerProgramError = KickerProgrammerError<Infallible, Infallible>;
/// Error for servicing the kicker
pub type KickerServicingError = KickerServiceError<Infallible, Infallible>;
