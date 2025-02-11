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

/// Taken from the C++ firmware
/// 5-cell lipo * 4.2 v per cell = 21 battery vols
/// 68k and 10k ohm voltage divider, analog in read voltage 2.756
const MAX_SAFE_BATT_VOLTAGE_READ: f32 = 2.692;

/// 5-cell lip * 3 v per cell = 15 battery volts
/// 68k and 10k ohm voltage divider, analog in read voltage 1.923
const MIN_SAFE_BATT_VOLTAGE_READ: f32 = 1.923;

/// total range for good batter voltage
const BATT_VOLTAGE_READ_RANGE: f32 = (MAX_SAFE_BATT_VOLTAGE_READ - MIN_SAFE_BATT_VOLTAGE_READ);

pub struct BatterySense <AnalogInputT, AdcT, WordT, PinT> where 
        PinT: Channel<AdcT>,
        AnalogInputT: OneShot<AdcT, WordT, PinT>
{
    adc: AnalogInputT,
    percent_capacity: f32,
    raw_voltage: f32
}


impl<P, AdcE> BatterySense <P, AdcE> where 
    PinT: Channel<AdcT>,
    AnalogInputT: OneShot<AdcT, WordT, PinT>
     {
    pub fn new(&mut self, analog_input: AnalogInput<P, 0>, adc: Adc<0>) -> Self {
        let instance = BatterySense <P, Adc> {
            pin: analog_input,
            adc: adc,
            percent_capacity: 0.,
            raw_voltage: 0.
        };
        return Ok(instance);
    }

    pub fn read_voltage(&mut self) -> Result<f32, AdcE> {
        let voltage_in = self.adc.read_blocking(&mut self.pin);
        Ok(f32::from(voltage_in) * 78. / 10.)
    }

    pub fn get_percent_capacity(&mut self) -> Result<u8, AdcE> {
        let voltage = self.read_voltage()?;
        Ok(voltage * 26 - 450)
    }

}