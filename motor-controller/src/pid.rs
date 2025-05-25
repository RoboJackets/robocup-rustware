//!
//! PID Controller Implementation in rust
//! 

use core::cmp::max;

use defmt::Format;

#[derive(Clone, Copy, PartialEq, Format, Debug)]
/// PID Controller implementation.
pub struct Pid {
    /// The maximum output pwm to send to the robot
    maximum_output: u16,
    /// The proportional gain of the controller
    kp: f32,
    /// The integral gain of the controller
    ki: f32,
    /// The derivative gain of the controller
    kd: f32,
    /// Limiter for the proportional gain `-p_limit <= P <= p_limit`
    p_limit: Option<f32>,
    /// Limiter for the integral gain `-i_limit <= I <= i_limit`
    i_limit: Option<f32>,
    /// Limiter for the derivative gain `g_limit <= G <= g_limit`
    d_limit: Option<f32>,
    /// Last calculated integral value
    integral_term: f32,
    /// The last measurement in ticks per second
    previous_measurement: Option<i32>,
    /// A measurement of the error from the last update
    last_error: f32,
}

impl Pid {
    /// Create a new pid controller
    pub fn new(
        maximum_output: u16,
        kp: f32,
        ki: f32,
        kd: f32,
    ) -> Self {
        Self {
            maximum_output,
            kp,
            ki,
            kd,
            p_limit: None,
            i_limit: None,
            d_limit: None,
            integral_term: 0.0,
            previous_measurement: None,
            last_error: 0.0,
        }
    }
    
    /// Set the maximum p value
    pub fn set_p_limit(&mut self, p_limit: f32) {
        self.p_limit = Some(p_limit);
    }

    /// Set the maximum i value
    pub fn set_i_limit(&mut self, i_limit: f32) {
        self.i_limit = Some(i_limit);
    }

    /// Set the maximum d value
    pub fn set_d_limit(&mut self, d_limit: f32) {
        self.d_limit = Some(d_limit);
    }

    /// Get the last error value
    pub fn last_error(&self) -> f32 {
        self.last_error
    }

    /// Apply the pid controller to determine the next control output
    /// 
    /// Returns: (pwm, clockwise)
    pub fn update(&mut self, setpoint: i32, measurement: i32) -> (u16, bool) {
        let error = (setpoint - measurement) as f32;
        self.last_error = error;

        // Calculate the proportional term
        let mut p = error * self.kp;
        if let Some(p_limit) = self.p_limit {
            if p.abs() > p_limit {
                if p > 0.0 { p = p_limit } else { p = -p_limit };
            }
        }

        // Calculate the i value
        self.integral_term = self.integral_term + error * self.ki;
        if let Some(i_limit) = self.i_limit {
            if self.integral_term.abs() > i_limit {
                if self.integral_term > 0.0 { self.integral_term = i_limit } else { self.integral_term = -i_limit };
            }
        }

        // Calculate the d value
        let mut d = if let Some(previous_measurement) = self.previous_measurement {
            ((measurement - previous_measurement) as f32) * self.kd
        } else {
            0.0
        };
        self.previous_measurement = Some(measurement);
        if let Some(d_limit) = self.d_limit {
            if d.abs() > d_limit {
                if d > 0.0 { d = d_limit } else { d = -d_limit };
            }
        }

        let output: i32 = unsafe { (p + self.integral_term + d).to_int_unchecked() };

        if output > 0 {
            (
                max(output as u16, self.maximum_output),
                true,
            )
        } else {
            (
                max(output.abs() as u16, self.maximum_output),
                false,
            )
        }
    }
}