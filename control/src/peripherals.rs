//!
//! Peripheral Type and Wiring Definitions to ensure
//! all examples are correctly wired at compile
//! time.
//!

use core::convert::Infallible;

use imxrt_hal::lpuart::{self, Lpuart};
use teensy4_pins::t41::*;

use teensy4_bsp::board::{self, Lpi2c1, Lpuart4, Lpuart6, Lpuart8, PERCLK_FREQUENCY};
use teensy4_bsp::hal::{
    gpio::{Input, Output, Port},
    gpt::{Gpt1, Gpt2},
    lpspi::{Lpspi, LpspiError},
    pit::Pit2,
    timer::Blocking,
};

use fpga_rs::FPGA;
use icm42605_driver::IMU;
use kicker_programmer::KickerProgrammer;

use super::GPT_FREQUENCY;

/// SPI that is used for the FPGA
pub type FpgaSpi = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
/// The FPGA
pub type Fpga = FPGA<FpgaSpi, Output<P9>, P29, Output<P28>, P30, LpspiError, Infallible>;
/// Shared Spi
pub type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
/// The Chip Enable for the Radio
pub type RadioCE = Output<P20>;
/// The Chip Select for the Radio
pub type RadioCSN = Output<P14>;
/// The Interrupt for the Radio
pub type RadioInterrupt = Input<P15>;
/// The Radio
pub type RFRadio = rtic_nrf24l01::Radio<RadioCE, RadioCSN, SharedSPI, Infallible, LpspiError>;
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
/// The Kicker RESET pin
pub type KickerReset = Output<P6>;
/// The Kicker Chip Select
pub type KickerCSn = Output<P5>;
/// The Kicker Programmer
pub type KickerProg = KickerProgrammer<KickerCSn, KickerReset>;

/// The enable 3v3 pin for the motor_board
pub type MotorEn = Output<P23>;
/// The Motor killn pin
pub type Killn = Output<P36>;
/// The uart interface connected to the first motor
pub type MotorOneUart = Lpuart6;
/// The prog pin connected to the first motor
pub type MotorOneProg = Output<P2>;
/// The uart interface connected to the second motor
pub type MotorTwoUart = Lpuart4;
/// The prog pin connected to the second motor
pub type MotorTwoProg = Output<P6>;
/// The uart interface connected to the third motor
pub type MotorThreeUart = Lpuart<lpuart::Pins<P24, P25>, 1>;
/// The prog pin connected to the third motor
pub type MotorThreeProg = Output<P31>;
/// The uart interface connected to the fourth motor
pub type MotorFourUart = Lpuart<lpuart::Pins<P29, P28>, 7>;
/// The prog pin connected to the fourth motor
pub type MotorFourProg = Output<P30>;
/// The uart interface connnected to the dribbler motor
pub type DribblerUart = Lpuart8;
/// The prog pin connected to the dribbler
pub type DribblerProg = Output<P22>;
