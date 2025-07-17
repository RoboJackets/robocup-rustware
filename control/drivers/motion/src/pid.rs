//!
//! On Robot PID Implementation
//! 

use core::cmp::min;

use super::{WHEEL_RADIUS, REAR_WHEEL_DIST, FRONT_WHEEL_DIST, FRONT_ANGLE, BACK_ANGLE, TICKS_PER_ROTATION};

/// The length of the window used to report the current velocity of the robot
pub const VELOCITY_WINDOW: usize = 5;

#[derive(Clone, Copy, PartialEq, Debug)]
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
    /// The last measurement
    previous_measurement: Option<f32>,
    /// A measurement of the error from the last update
    pub last_error: f32,

    /// The current measurement
    pub current_value: f32,
    /// A buffer containign the most recent values
    value_buffer: [f32; VELOCITY_WINDOW],
    /// The current index in the circular value buffer
    value_idx: usize,
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
            current_value: 0.0,
            value_buffer: [0.0; VELOCITY_WINDOW],
            value_idx: 0,
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
    /// Returns: The new setpoint
    pub fn update(&mut self, setpoint: f32, measurement: f32) -> f32 {
        let error = (setpoint - measurement) as f32;
        self.last_error = error;

        let mut p = error * self.kp;
        if let Some(p_limit) = self.p_limit {
            if p.abs() > p_limit {
                if p > 0.0 { p = p_limit } else { p = -p_limit };
            }
        }

        self.integral_term = self.integral_term + error * self.ki;
        if let Some(i_limit) = self.i_limit {
            if self.integral_term.abs() > i_limit {
                if self.integral_term > 0.0 { self.integral_term = i_limit } else { self.integral_term = -i_limit };
            }
        }

        let mut d = if let Some(previous_measurement) = self.previous_measurement {
            (measurement - previous_measurement) * self.kd
        } else {
            0.0
        };

        self.previous_measurement = Some(measurement);
        if let Some(d_limit) = self.d_limit {
            if d.abs() > d_limit {
                if d.abs() > d_limit {
                    if d > 0.0 { d = d_limit } else { d = -d_limit };
                }
            }
        }


        0.0
    }

    // /// Apply the pid controller to determine the next control output
    // /// 
    // /// Returns: (pwm, clockwise)
    // pub fn update(&mut self, setpoint: i32, encoders_count: u16) -> (u16, bool) {

    //     let intended_velocity = p + self.integral_term + d;
    //     let output: i32 = unsafe { (intended_velocity * VELOCITY_TO_PWM_MAPPING).to_int_unchecked() };

    //     if output > 0 {
    //         (
    //             min(output as u16, self.maximum_output),
    //             true,
    //         )
    //     } else {
    //         (
    //             min(output.abs() as u16, self.maximum_output),
    //             false,
    //         )
    //     }
    // }

    // /// Calculate the Current State Estimate Based on IMU Measurements
    // ///
    // /// Params:
    // ///     imu_measurements: <a_xt,    a_yt,    w_t>
    // ///                      ((m/s^2), (m/s^2), (rad/s))
    // ///     delta: elapsed_time (us)
    // ///
    // /// Returns:
    // ///     Velocity Estimate (v_xt,  y_xt,  w_t)
    // ///                      ((m/s), (m/s), (rad/s))
    // fn imu_estimate(&mut self, imu_measurements: Vector3<f32>, delta: u32) -> Vector3<f32> {
    //     let delta = (delta as f32) / 1_000_000.0;
    //     Vector3::new(
    //         delta * (imu_measurements[0] - self.last_imu[0]) + self.last_state[0],
    //         delta * (imu_measurements[1] - self.last_imu[1]) + self.last_state[1],
    //         imu_measurements[2],
    //     )
    // }

    // /// Calculate the Current State Estimate Based on Encoder Velocities
    // ///
    // /// Params:
    // ///     encoder_velocities: < v1_t,  v2_t,  v3_t,  v4_t> (ticks/s)
    // ///
    // /// Returns Velocity Estimate (v_xt,  v_yt,  w_t) (m/s)
    // fn encoder_estimate(&mut self, encoder_velocities: Vector4<i32>) -> Vector3<f32> {
    //     self.wheel_to_bot * encoder_velocities.map(|v| MotionControl::ticks_to_meters(v))
    // }

    #[inline(always)]
    /// Convert ticks / second to meters / second
    pub fn ticks_to_meters(value: i32) -> f32 {
        (value as f32 / TICKS_PER_ROTATION) * 4.0 * core::f32::consts::PI * WHEEL_RADIUS
    }

    #[inline(always)]
    /// Convert meters / second to ticks / second
    pub fn meters_to_ticks(value: f32) -> i32 {
        unsafe {
            (value / (4.0 * core::f32::consts::PI * WHEEL_RADIUS) * TICKS_PER_ROTATION)
                .to_int_unchecked::<i32>()
        }
    }
}