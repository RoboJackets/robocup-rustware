//! 
//! Driver for reading the battery capacity
//! 


#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]
#![deny(missing_docs)]



use core::fmt::Debug;
use embedded_hal::adc;

/// Taken from the C++ firmware
/// 5-cell lipo * 4.2 v per cell = 21 battery vols
/// 68k and 10k ohm voltage divider, analog in read voltage 2.756
const MAX_SAFE_BATT_VOLTAGE_READ: f32 = 2.692;

/// 5-cell lip * 3 v per cell = 15 battery volts
/// 68k and 10k ohm voltage divider, analog in read voltage 1.923
const MIN_SAFE_BATT_VOLTAGE_READ: f32 = 1.923;

/// total range for good batter voltage
const BATT_VOLTAGE_READ_RANGE: f32 = (MAX_SAFE_BATT_VOLTAGE_READ - MIN_SAFE_BATT_VOLTAGE_READ);

pub struct Battery_Sense {

}