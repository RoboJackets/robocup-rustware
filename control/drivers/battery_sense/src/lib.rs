//!
//! Driver for reading the battery capacity
//!

#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]
#![deny(missing_docs)]

use core::fmt::Debug;
use embedded_hal::adc::{Channel, OneShot};

// import fpga error type
pub mod error;
use error::BatteryError;

/// Driver for reading Rattery Capacity, returns capacity as a u16
pub struct BatterySense<AdcT, WordT, PinT, AdcE>
where
    PinT: Channel<AdcT>,
    WordT: Into<u16> + From<u16>,
    AdcT: OneShot<AdcT, WordT, PinT, Error = AdcE>,
    AdcE: Debug,
{
    adc: AdcT,
    pin: PinT,
    percent_capacity: u16,
    _word: WordT,
}

impl<AdcT, WordT, PinT, AdcE> BatterySense<AdcT, WordT, PinT, AdcE>
where
    PinT: Channel<AdcT>,
    WordT: Into<u16> + From<u16>,
    AdcT: OneShot<AdcT, WordT, PinT, Error = AdcE>,
    AdcE: Debug,
{
    /// creates a new instance of the driver
    pub fn new(adc: AdcT, pin: PinT) -> Self {
        let instance = Self {
            adc: adc,
            pin: pin,
            percent_capacity: 0,
            _word: 0.into(),
        };
        return instance;
    }

    /// Reads the voltage straight from the ADC, converts into real voltage of battery
    fn read_voltage(&mut self) -> Result<f32, BatteryError> {
        let raw_in = self.adc.read(&mut self.pin);
        match raw_in {
            // Voltage is given as a range from 0 to 1024, representing 0 to 3V3
            Ok(word) => Ok(((word.into() as f32) * 78. / 10.) / 1024. * 3.3),
            Err(_) => Err(BatteryError::ADC),
        }
    }

    /// Gets the percent capacity using a model we created
    pub fn get_percent_capacity(&mut self) -> Result<u16, BatteryError> {
        let voltage = self.read_voltage()?;
        self.percent_capacity = (voltage * 16.37124 - 450.39) as u16;
        Ok(self.percent_capacity)
    }
}
