//! 
//! Driver for reading the battery capacity
//! 


#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]
//#![deny(missing_docs)]



use core::fmt::Debug;
use imxrt_hal as hal;
use hal::adc::{Adc,AnalogInput};
use hal::adc::*;
use embedded_hal::adc::{OneShot, Channel};
use core::convert::Infallible;

// import fpga error type
pub mod error;
use error::BatteryError;

/// Taken from the C++ firmware
/// 5-cell lipo * 4.2 v per cell = 21 battery vols
/// 68k and 10k ohm voltage divider, analog in read voltage 2.756
const MAX_SAFE_BATT_VOLTAGE_READ: f32 = 2.692;

/// 5-cell lip * 3 v per cell = 15 battery volts
/// 68k and 10k ohm voltage divider, analog in read voltage 1.923
const MIN_SAFE_BATT_VOLTAGE_READ: f32 = 1.923;

/// total range for good batter voltage
const BATT_VOLTAGE_READ_RANGE: f32 = (MAX_SAFE_BATT_VOLTAGE_READ - MIN_SAFE_BATT_VOLTAGE_READ);

pub struct BatterySense <AdcT, WordT, PinT, AdcE> where 
        PinT: Channel<AdcT>,
        WordT: Into<u16> + From<u16>,
        AdcT: OneShot<AdcT, WordT, PinT>,
        AdcE: Debug
{
    adc: AdcT,
    pin: PinT,
    percent_capacity: f32,
    raw_voltage: WordT
}


impl<AdcT, WordT, PinT, AdcE> BatterySense <AdcT, WordT, PinT, AdcE> where 
    PinT: Channel<AdcT>,
    WordT: Into<u16> + From<u16>,
    AdcT: OneShot<AdcT, WordT, PinT>,
    AdcE: Debug
     {
    pub fn new(&mut self, adc: AdcT, pin: PinT) -> Self {

        let instance = Self {
            adc: adc,
            pin: pin,
            percent_capacity: 0.,
            raw_voltage: 0.
        };
        return instance;
    }

    pub fn read_voltage(&mut self) -> Result<WordT,BatteryError> {
        let voltage_in = self.adc.read(&mut self.pin);
        match voltage_in {
            Ok(word) => Ok(WordT::from(word.into() * 78/10)),
            Err(_) => Err(BatteryError::ADC)
            }
    }

    pub fn get_percent_capacity(&mut self) -> Result<u16, BatteryError> {
        let voltage = self.read_voltage()?;
        Ok(WordT::from(voltage.into() * 26 - 450))
    }

}