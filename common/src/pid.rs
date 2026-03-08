//!
//! PID Implementation in Rust
//! 

use defmt::Format;

#[derive(Format, Debug)]
/// PID Controller Implementation for shared use across our codebase
pub struct Pid {
    /// The maximum output from the pid controller
    pub max_output: Option<f32>,
    /// The proportional gain for the controller
    pub kp: f32,
    /// The integral gain for the controller
    pub ki: f32,
    /// The derivative gain for the controller
    pub kd: f32,

    /// The limiter for the proportional gain contribution `-p_limit <= P <= p_limit`
    pub p_limit: Option<f32>,
    /// The limiter for the integral gain contribution `-i_limit <= I <= i_limit`
    pub i_limit: Option<f32>,
    /// The limiter for the derivative gain contribution `-d_limit <= D <= d_limit`
    pub d_limit: Option<f32>,

    /// The last measurement from the controller
    last_measurement: f32,
    /// The integrated error from the controller
    integrated_error: f32,
}

impl Pid {
    /// Create a new pid controller
    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        max_output: Option<f32>,
    ) -> Self {
        Self {
            max_output,
            kp,
            ki,
            kd,
            p_limit: None,
            i_limit: None,
            d_limit: None,
            last_measurement: 0.0,
            integrated_error: 0.0,
        }
    }

    /// Set the proportional limit for the controller
    pub fn set_p_limit(&mut self, p_limit: f32) {
        self.p_limit = Some(p_limit);
    }

    /// Set the integral limit for the controller
    pub fn set_i_limit(&mut self, i_limit: f32) {
        self.i_limit = Some(i_limit);
    }

    /// Set the derivative limit for the controller
    pub fn set_d_limit(&mut self, d_limit: f32) {
        self.d_limit = Some(d_limit);
    }

    pub fn update(&mut self, measurement: f32, setpoint: f32) -> f32 {
        let error = setpoint - measurement;

        let mut p = error * self.kp;
        if let Some(p_limit) = self.p_limit {
            if p > p_limit {
                p = p_limit;
            } else if p < -p_limit {
                p = -p_limit;
            }
        }

        self.integrated_error += error;
        let mut i = self.integrated_error * self.ki;
        if let Some(i_limit) = self.i_limit {
            if i > i_limit {
                i = i_limit;
            } else if i < -i_limit {
                i = -i_limit;
            }
        }

        let mut d = (measurement - self.last_measurement) * self.kd;
        self.last_measurement = measurement;
        if let Some(d_limit) = self.d_limit {
            if d > d_limit {
                d = d_limit;
            } else if d < -d_limit {
                d = -d_limit;
            }
        }

        let output = p + i + d;
        if let Some(max_output) = self.max_output {
            if output > max_output {
                max_output
            } else if output < -max_output {
                -max_output
            } else {
                output
            }
        } else {
            output
        }
    }
}