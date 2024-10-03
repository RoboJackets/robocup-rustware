//!
//! The duty cycles are sent to the Fpga in 10-bit signed magnitude.
//!
//! Note: These are not velocities and encoder + IMU data is needed to
//! determine the correct duty cycles to the wheels (per velocity)
//!

///
/// Helper structure that wraps a duty cycle value for the FPGA
///
/// Duty cycles are sent and stored as a 10-bit signed magnitude value
/// | bit_9 | bit_8 ~ bit_0 |
/// | SIGN  |   MAGNITUDE   |
///
/// Where SIGN = 0 is positive and SIGN = 1 is negative
///
/// For reference check the following link:
/// https://www.electronics-tutorials.ws/binary/signed-binary-numbers.html
///
/// The available range of duty cycles is: [-511, 511]
///
/// Use the from function to generate a DutyCycle from an i16 value. Ensure that
/// the passed in i16 value is within the provided range
///
/// Example:
/// let duty_cycle = DutyCycle::from(-487 as i16)
/// let value_in_duty_cycle = i16::from(duty_cycle)
///
#[derive(core::fmt::Debug)]
pub struct DutyCycle(u16);

impl DutyCycle {
    /// constructor used on read duty cycle function
    pub fn new(value: u16) -> Self {
        DutyCycle(value)
    }

    /// shifts and masks the upper 8 bits
    pub fn msb(&mut self) -> u8 {
        ((self.0 >> 8) & 0xFF) as u8
    }

    /// masks the lower 8 bits
    pub fn lsb(&mut self) -> u8 {
        (self.0 & 0xFF) as u8
    }
}

impl From<i16> for DutyCycle {
    fn from(value: i16) -> Self {
        // check that value is not outside of the allowed range
        if !(-511..=511).contains(&value) {
            return DutyCycle(0);
        }

        // check if the value is negative and assign accordingly
        let duty_cycle: u16 = match value < 0 {
            true => ((-value) | (1 << 9)) as u16,
            false => value as u16,
        };

        // returns struct
        DutyCycle(duty_cycle)
    }
}

impl From<DutyCycle> for i16 {
    fn from(dc: DutyCycle) -> Self {
        let mut result: i16 = 0;
        if (dc.0 >> 9) & 1 == 1 {
            result = -((dc.0 & 0x1FF) as i16);
        } else {
            result = dc.0 as i16;
        }

        result
    }
}
