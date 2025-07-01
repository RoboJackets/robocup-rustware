//!
//! The motion control module is just an extraction of the motion control that runs on the
//! motor controllers.  The only reason its in its own module is it is a bit easier to write unit
//! tests for crates that don't have a set compilation target
//! 

#![no_std]

/// The minimum velocity the robot will move at
const MIN_VELOCITY: i32 = 3000;
/// LERP up or down to the desired frequency moving my LERP_VELOCITY each tick
/// until we are within the LERP_CUTOFF of the setpoint
const LERP_VELOCITY: i32 = 20;
/// The window of samples used to calculate the current velocity
const VELOCITY_WINDOW: usize = 10;
/// The convesion between velocity to pwm (I've calculated there's roughly 187 pwm  per rotation (6200 ticks))
const VELOCITY_TO_PWM_MAPPING: f32 = 3.0 * 187.0 / 6200.0;

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

        // Use a proportional controller to attempt to achieve the setpoint
        self.lerp_target = if self.current_velocity > self.last_setpoint {
            core::cmp::min(ticks_per_second, self.lerp_target) - LERP_VELOCITY
        } else {
            core::cmp::max(ticks_per_second, self.lerp_target) + LERP_VELOCITY
        };
        let target_velocity = self.lerp_target;

        self.iterations += 1;

        if self.last_setpoint == 0 {
            (
                0,
                true
            )
        } else if target_velocity.abs() < MIN_VELOCITY {
            // Set the velocity to the minimum velocity
            (
                unsafe { (MIN_VELOCITY as f32 * VELOCITY_TO_PWM_MAPPING).to_int_unchecked() },
                target_velocity < 0
            )
        } else if target_velocity < 0 {
            (
                unsafe { (target_velocity.abs() as f32 * VELOCITY_TO_PWM_MAPPING).to_int_unchecked() },
                true
            )
        } else {
            (
                unsafe { (target_velocity as f32 * VELOCITY_TO_PWM_MAPPING).to_int_unchecked() },
                false
            )
        }
    }
}

