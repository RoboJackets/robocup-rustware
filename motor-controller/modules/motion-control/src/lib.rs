//!
//! The motion control module is just an extraction of the motion control that runs on the
//! motor controllers.  The only reason its in its own module is it is a bit easier to write unit
//! tests for crates that don't have a set compilation target
//! 

#![no_std]

use defmt::Format;

use core::cmp::min;

/// LERP up or down to the desired frequency moving my LERP_VELOCITY each tick
/// until we are within the LERP_CUTOFF of the setpoint
const LERP_VELOCITY: i32 = 1;
/// The window of samples used to calculate the current velocity
const VELOCITY_WINDOW: usize = 10;
/// The convesion between velocity to pwm (I've calculated there's roughly 187 pwm  per rotation (6200 ticks))
const VELOCITY_TO_PWM_MAPPING: f32 = 3.0 * 187.0 / 6200.0;

/// Calculate the current speed of the encoders in ticks per second
fn calculate_ticks_per_second(encoders: u16, last_encoders: u16, motion_control_frequency: u32) -> i32 {
    let elapsed_encoders = if encoders < 10_000 && last_encoders > u16::MAX - 10_000 {
        // Overflow
        (u16::MAX as i32) - (last_encoders as i32) + (encoders as i32)
    } else if last_encoders < 10_000 && encoders > u16::MAX - 10_000 {
        // Underflow
        -((last_encoders  as i32) + (u16::MAX as i32) - (encoders as i32))
    } else {
        // Normal
        (encoders as i32) - (last_encoders as i32)
    };
    elapsed_encoders * motion_control_frequency as i32
}

/// The motion controller is responsible for maintaining any information necessary for controlling
/// the motion of the motor controller
pub struct MotionController {
    /// The last measured encoder count
    last_encoders_value: u16,
    /// The last setpoint (ticks / second)
    last_setpoint: i32,
    /// The current velocity measurement
    pub current_velocity: i32,
    /// A buffer containing the most recent 50 velocities
    velocity_buffer: [i32; VELOCITY_WINDOW],
    /// The current index in the circular velocity buffer
    velocity_idx: usize,
    /// The number of iterations we have been at the current setpoint
    pub iterations: u32,
    /// The frequency (hz) of motion control
    motion_control_frequency: u32,
    /// The last lerp target
    lerp_target: i32,
}

impl MotionController {
    /// Create a new motion control module
    pub fn new(motion_control_frequency: u32) -> Self {
        Self {
            last_encoders_value: 0,
            last_setpoint: 0,
            current_velocity: 0,
            velocity_buffer: [0i32; VELOCITY_WINDOW],
            velocity_idx: 0,
            iterations: 0,
            motion_control_frequency: motion_control_frequency,
            lerp_target: 0,
        }
    }

    /// Perform a motion control update returning the pwm to move the motors at and
    /// the direction to move the motors (true = clockwise, false = counter-clockwise)
    pub fn update(&mut self, encoder_count: u16, setpoint: i32) -> (u16, bool) {
        // Check if this is a new setpoint
        if setpoint != self.last_setpoint {
            self.iterations = 0;
            self.last_setpoint = setpoint;
            self.lerp_target = self.current_velocity;
        }

        let elapsed_encoders = if encoder_count < 10_000 && self.last_encoders_value > u16::MAX - 10_000 {
            // Overflow
            (u16::MAX as i32) - (self.last_encoders_value as i32) + (encoder_count as i32)
        } else if self.last_encoders_value < 10_000 && encoder_count > u16::MAX - 10_000 {
            // Underflow
            -((self.last_encoders_value  as i32) + (u16::MAX as i32) - (encoder_count as i32))
        } else {
            // Normal
            (encoder_count as i32) - (self.last_encoders_value as i32)
        };
        let ticks_per_second = elapsed_encoders * self.motion_control_frequency as i32;
        self.velocity_buffer[self.velocity_idx] = ticks_per_second;
        self.velocity_idx += 1;
        if self.velocity_idx == self.velocity_buffer.len() {
            self.velocity_idx = 0;
        }
        self.last_encoders_value = encoder_count;

        // Calculate current velocity
        self.current_velocity = self.velocity_buffer.iter().sum::<i32>() / self.velocity_buffer.len() as i32;



        // defmt::info!("TPS: {}, target: {}, Setpoint: {}", ticks_per_second, target_velocity, self.last_setpoint);

        // Use a proportional controller to attempt to achieve the setpoint
        self.lerp_target = if self.current_velocity > self.last_setpoint {
            ticks_per_second - LERP_VELOCITY
            // defmt::info!("DOWN");
            // core::cmp::min(ticks_per_second, self.lerp_target) - LERP_VELOCITY
        } else {
            ticks_per_second + LERP_VELOCITY
            // defmt::info!("UP");
            // core::cmp::max(ticks_per_second, self.lerp_target) + LERP_VELOCITY
        };
        let target_velocity = self.lerp_target;

        self.iterations += 1;

        if self.last_setpoint == 0 {
            (
                0,
                true
            )
        // } else if target_velocity.abs() < MIN_VELOCITY {
            // Set the velocity to the minimum velocity
            // (
            //     unsafe { (MIN_VELOCITY as f32 * VELOCITY_TO_PWM_MAPPING).to_int_unchecked() },
            //     target_velocity < 0
            // )
        } else if target_velocity < 0 {
            (
                unsafe { (target_velocity.abs() as f32 * VELOCITY_TO_PWM_MAPPING).to_int_unchecked() },
                false
            )
        } else {
            (
                unsafe { (target_velocity as f32 * VELOCITY_TO_PWM_MAPPING).to_int_unchecked() },
                true
            )
        }
    }
}

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
    pub last_error: f32,
    /// THe last encoders measurements
    last_encoders: u16,
    // THe motion control frequency
    motion_control_frequency: u32,

    /// The current velocity measurement
    pub current_velocity: i32,
    /// A buffer containing the most recent 50 velocities
    velocity_buffer: [i32; VELOCITY_WINDOW],
    /// The current index in the circular velocity buffer
    velocity_idx: usize,
}

impl Pid {
    /// Create a new pid controller
    pub fn new(
        maximum_output: u16,
        kp: f32,
        ki: f32,
        kd: f32,
        motion_control_frequency: u32,
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
            last_encoders: 0,
            motion_control_frequency,
            current_velocity: 0,
            velocity_buffer: [0i32; VELOCITY_WINDOW],
            velocity_idx: 0,
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
    pub fn update(&mut self, setpoint: i32, encoders_count: u16) -> (u16, bool) {
        let measurement = calculate_ticks_per_second(encoders_count, self.last_encoders, self.motion_control_frequency);
        self.velocity_buffer[self.velocity_idx] = measurement;
        self.velocity_idx += 1;
        if self.velocity_idx == self.velocity_buffer.len() {
            self.velocity_idx = 0;
        }
        self.current_velocity = self.velocity_buffer.iter().sum::<i32>() / self.velocity_buffer.len() as i32;

        self.last_encoders = encoders_count;

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

        defmt::info!("P: {}; I: {}; D: {}", p, self.integral_term, d);

        let intended_velocity = p + self.integral_term + d;
        let output: i32 = unsafe { (intended_velocity * VELOCITY_TO_PWM_MAPPING).to_int_unchecked() };

        if output > 0 {
            (
                min(output as u16, self.maximum_output),
                true,
            )
        } else {
            (
                min(output.abs() as u16, self.maximum_output),
                false,
            )
        }
    }
}