//!
//! Peripheral Type and Wiring Definitions to ensure
//! all examples are correctly wired at compile
//! time.
//!

use core::convert::Infallible;

use teensy4_pins::t41::*;

use teensy4_bsp::board::{
    self, Lpi2c1, Lpi2c3, Lpuart1, Lpuart4, Lpuart6, Lpuart7, Lpuart8, PERCLK_FREQUENCY,
};
use teensy4_bsp::hal::{
    adc::AnalogInput,
    gpio::{Input, Output, Port},
    gpt::{Gpt1, Gpt2},
    lpspi::{Lpspi, LpspiError},
    pit::Pit2,
    timer::Blocking,
};

use battery_sense_rs::BatterySense;
use core::cell::RefCell;
use icm42605_driver::IMU;
use imxrt_hal::adc::Adc;
use io_expander_rs::IoExpander;
use kicker_programmer::KickerProgrammer;
use rotary_switch_rs::RotarySwitch;
use shared_bus::I2cProxy;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::I2CInterface;
use ssd1306::size::DisplaySize128x64;
use ssd1306::Ssd1306;

use super::GPT_FREQUENCY;
//See spi.rs for the fake SPI for kicker
/// Radio Spi
pub type RadioSPI = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
// i2c for the io expander
pub type IoI2C = Lpi2c3;
// io expander
pub type Expander = IoExpander<IoI2C, imxrt_hal::lpi2c::ControllerStatus>;
// rotary switch
pub type Rotary = RotarySwitch<IoI2C, imxrt_hal::lpi2c::ControllerStatus>;
/// The Chip Enable for the Radio
pub type RadioCE = Output<P41>; //Changed from P20
/// The Chip Select for the Radio
pub type RadioCSN = Output<P14>; //Changed from P14
/// The Interrupt for the Radio
pub type RadioInterrupt = Input<P9>; //Changed from P15
/// The Radio
pub type RFRadio = rtic_nrf24l01::Radio<RadioCE, RadioCSN, RadioSPI, Infallible, LpspiError>;
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
pub type Imu = IMU<I2cProxy<'static, shared_bus::cortex_m::interrupt::Mutex<RefCell<Lpi2c1>>>>;
/// The Kicker RESET pin
pub type KickerReset = Output<P37>; //Changed from P6
/// The Kicker Chip Select
pub type KickerCSn = Output<P38>; //Changed from P5
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
pub type MotorThreeUart = Lpuart1;
/// The prog pin connected to the third motor
pub type MotorThreeProg = Output<P31>;
/// The uart interface connected to the fourth motor
pub type MotorFourUart = Lpuart7;
/// The prog pin connected to the fourth motor
pub type MotorFourProg = Output<P30>;
/// The uart interface connnected to the dribbler motor
pub type DribblerUart = Lpuart8;
/// The prog pin connected to the dribbler
pub type DribblerProg = Output<P22>;
/// The display
pub type Display = Ssd1306<
    I2CInterface<
        shared_bus::I2cProxy<'static, cortex_m::interrupt::Mutex<core::cell::RefCell<Lpi2c1>>>,
    >,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;
// Adc Port used by BatterySense
pub type AdcP = AnalogInput<P41, 1>;
/// One of two ADCs defined under Teensy 4.1 docs
pub type Adc1 = Adc<1>;
// Alias of BatterySense
pub type BatterySenseT = BatterySense<Adc1, u16, AdcP, Infallible>;
