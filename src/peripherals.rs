//!
//! Peripheral Type and Wiring Definitions to ensure
//! all examples are correctly wired at compile
//! time.
//! 

use core::convert::Infallible;

use teensy4_pins::t41::*;

use teensy4_bsp::board::{self, Lpi2c1, Lpi2c3, PERCLK_FREQUENCY};
use teensy4_bsp::hal::{
    lpspi::{LpspiError, Lpspi},
    gpio::{Output, Input, Port},
    gpt::{Gpt1, Gpt2},
    timer::Blocking,
    pit::Pit2,
};

use fpga_rs::FPGA;
use rotary_switch_rs::RotarySwitch;
use battery_sense_rs::BatterySense;
use io_expander_rs::IoExpander;
use icm42605_driver::IMU;
use imxrt_hal::adc::Adc;

use super::GPT_FREQUENCY;

/// SPI that is used for the FPGA
pub type FpgaSpi = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
/// The FPGA
pub type Fpga = FPGA<FpgaSpi, Output<P9>, P29, Output<P28>, P30, Delay1, LpspiError, Infallible>;
// i2c for the io expander
pub type IoI2C = Lpi2c3;
// io expander
pub type Expander = IoExpander<IoI2C, imxrt_hal::lpi2c::ControllerStatus>;
// rotary switch
pub type Rotary = RotarySwitch<IoI2C, imxrt_hal::lpi2c::ControllerStatus>;

/// Shared Spi
pub type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
/// The Chip Enable for the Radio
pub type RadioCE = Output<P20>;
/// The Chip Select for the Radio
pub type RadioCSN = Output<P14>;
/// The Interrupt for the Radio
pub type RadioInterrupt = Input<P15>;
/// The Radio
pub type RFRadio = rtic_nrf24l01::Radio<RadioCE, RadioCSN, SharedSPI, Delay2, Infallible, LpspiError>;
/// The Delay used by the FPGA
pub type Delay1 = Blocking<Gpt1, GPT_FREQUENCY>;
/// The general-purpose delay shared by different peripherals
pub type Delay2 = Blocking<Gpt2, GPT_FREQUENCY>;
/// The PIT-defined delay for initializing the IMU.
pub type PitDelay = Blocking<Pit2, PERCLK_FREQUENCY>;
/// The first GPIO port
pub type Gpio1 = Port<1>;
/// The second GPIO port
pub type Gpio2 = Port<2>;
/// The third GPIO port
pub type Gpio3 = Port<3>;
/// The fourth GPIO port
pub type Gpio4 = Port<4>;
/// The IMU
pub type Imu = IMU<Lpi2c1>;

pub type AdcP = P41;

/// One of two ADCs defined under Teensy 4.1 docs
pub type Adc1 = Adc<1>;

pub type BatterySenseT = BatterySense<Adc1, u16, AdcP, Infallible>;