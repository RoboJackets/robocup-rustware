//!
//! Motion Control Module for controlling the commands sent to move the motors on the
//! robot.
//!

use core::fmt::Debug;

use libm::{cosf, sinf};

use nalgebra::base::*;

use crate::{BACK_ANGLE, FRONT_ANGLE, WHEEL_DIST};

/// Wheel Radius (m)
pub const WHEEL_RADIUS: f32 = 0.02786;
/// The number of encoder ticks per rotation
pub const TICKS_PER_ROTATION: f32 = 6200.0;

pub const SCALE_FACTOR: f32 = 0.75;
/// Weighting for IMU Sensor Readings
pub const ALPHA: f32 = 0.15;
/// Weighting for the IMU Sensor Readings on High Acceleration
pub const HIGH_ALPHA: f32 = 0.80;
/// High Acceleration Cutoff (squared) (We should weight the accelerometer higher if there
/// is a high acceleration)
pub const HIGH_ACCELERATION_CUTOFF: f32 = 0.5 * 0.5;
/// Factor with which to correct headings my
pub const CORRECT_FACTOR: f32 = 0.5;
/// Number of timesteps to wait for measurements to stabilize
pub const STABLIZE_TIME: u32 = 2;

#[derive(Debug)]
pub struct MotionControl {
    // Conversion Matrix from Body Coordinates to Wheel Coordinates
    bot_to_wheel: Matrix4x3<f32>,
    // Conversion from Wheel Coordinates to Body Coordinates
    wheel_to_bot: Matrix3x4<f32>,
    // Last State Estimate <v_x(t-1), v_y(t-1), w_(t-1)> ((m/s), (m/s), (rad/s))
    last_state: Vector3<f32>,
    // Last IMU Measurement <a_x(t-1), a_y(t-1), w_(t-1)> ((m/s^2), (m/s^2), (rad/s))
    last_imu: Vector3<f32>,
    // Last Target Velocity <v_x(t-1), v_y(t-1), w_(t-1)>
    //                      ((m/2),   (m/s),    (m/s))
    last_target_velocity: Vector3<f32>,
    // Correction Value per each wheel (m/s)
    wheel_correction: Vector4<f32>,
    // Number of timesteps at the current target velocity
    timesteps: u32,
}

impl Default for MotionControl {
    fn default() -> Self {
        Self::new()
    }
}

impl MotionControl {
    /// Initialize a new motion control module
    pub fn new() -> Self {
        let wheel_angles = [
            (180.0 - FRONT_ANGLE).to_radians(),
            (180.0 + BACK_ANGLE).to_radians(),
            (360.0 - BACK_ANGLE).to_radians(),
            (0.0 + FRONT_ANGLE).to_radians(),
        ];

        let bot_to_wheel = Matrix4x3::new(
            sinf(wheel_angles[0]),
            -cosf(wheel_angles[0]),
            -WHEEL_DIST,
            sinf(wheel_angles[1]),
            -cosf(wheel_angles[1]),
            -WHEEL_DIST,
            sinf(wheel_angles[2]),
            -cosf(wheel_angles[2]),
            -WHEEL_DIST,
            sinf(wheel_angles[3]),
            -cosf(wheel_angles[3]),
            -WHEEL_DIST,
        );

        let bot_to_wheel_t = bot_to_wheel.transpose();

        let wheel_to_bot = (bot_to_wheel_t * bot_to_wheel).try_inverse().unwrap() * bot_to_wheel_t;

        Self {
            bot_to_wheel,
            wheel_to_bot,
            last_state: Vector3::zeros(),
            last_imu: Vector3::zeros(),
            last_target_velocity: Vector3::zeros(),
            wheel_correction: Vector4::zeros(),
            timesteps: STABLIZE_TIME,
        }
    }

    /// Convert a body velocity into duty cycles for the robot
    pub fn body_to_wheels(&self, body_velocity: Vector3<f32>) -> Vector4<f32> {
        self.bot_to_wheel * body_velocity
    }

    /// Converts a wheel's duty cycles into velocity for the robot
    pub fn wheels_to_body(&self, wheel_velocity: Vector4<f32>) -> Vector3<f32> {
        self.wheel_to_bot * wheel_velocity
    }

    /// Control Update
    ///
    /// Parameters:
    ///     imu_measurements: (accel_x (m/s^2), accel_y (m/s^2), w (rad/s))
    ///     encoder_velocities: (motor_1, motor_2, motor_3, motor_4) (ticks/s)
    ///     target_velocity: (x (m/s), y (m/s), w (rad/s))
    ///     delta: elapsed time (us)
    ///
    /// Returns:
    ///     (motor_1, motor_2, motor_3, motor_4) (ticks / s)
    pub fn control_update(
        &mut self,
        imu_measurements: Vector3<f32>,
        encoder_velocities: Vector4<i32>,
        target_velocity: Vector3<f32>,
        delta: u32,
    ) -> Vector4<i32> {
        // Make Sure 0 is 0
        if target_velocity == Vector3::zeros() {
            return Vector4::zeros();
        }

        // Reset New State Timesteps
        if target_velocity != self.last_target_velocity {
            self.timesteps = 0;
        }

        // Find State Estimates
        let imu_estimate = self.imu_estimate(imu_measurements, delta);
        let encoder_estimate = self.encoder_estimate(encoder_velocities);

        // let state_estimate: Vector3<f32> = imu_estimate.map(|v| v * ALPHA) + encoder_estimate.map(|v| v * (1.0 - ALPHA));
        let state_estimate = if imu_measurements[0] * imu_measurements[0]
            + imu_measurements[1] * imu_measurements[1]
            > HIGH_ACCELERATION_CUTOFF
        {
            imu_estimate.map(|v| v * ALPHA) + encoder_estimate.map(|v| v * (1.0 - ALPHA))
        } else {
            imu_estimate.map(|v| v * HIGH_ALPHA) + encoder_estimate.map(|v| v * (1.0 - HIGH_ALPHA))
        };

        let difference = (target_velocity - state_estimate).map(|v| v * CORRECT_FACTOR);

        // Find the Update Velocity
        let update_velocity = self.bot_to_wheel * difference;

        // Update Last Measurements
        self.last_state = state_estimate;
        self.last_imu = imu_measurements;

        if self.timesteps >= STABLIZE_TIME {
            self.wheel_correction += update_velocity;
        }

        // Return the New Wheel Velocities
        (self.bot_to_wheel * target_velocity + self.wheel_correction).map(|v| MotionControl::meters_to_ticks(v))
    }

    /// Calculate the Current State Estimate Based on IMU Measurements
    ///
    /// Params:
    ///     imu_measurements: <a_xt,    a_yt,    w_t>
    ///                      ((m/s^2), (m/s^2), (rad/s))
    ///     delta: elapsed_time (us)
    ///
    /// Returns:
    ///     Velocity Estimate (v_xt,  y_xt,  w_t)
    ///                      ((m/s), (m/s), (rad/s))
    fn imu_estimate(&mut self, imu_measurements: Vector3<f32>, delta: u32) -> Vector3<f32> {
        let delta = (delta as f32) / 1_000_000.0;
        Vector3::new(
            delta * (imu_measurements[0] - self.last_imu[0]) + self.last_state[0],
            delta * (imu_measurements[1] - self.last_imu[1]) + self.last_state[1],
            imu_measurements[2],
        )
    }

    /// Calculate the Current State Estimate Based on Encoder Velocities
    ///
    /// Params:
    ///     encoder_velocities: < v1_t,  v2_t,  v3_t,  v4_t> (ticks/s)
    ///
    /// Returns Velocity Estimate (v_xt,  v_yt,  w_t) (m/s)
    fn encoder_estimate(&mut self, encoder_velocities: Vector4<i32>) -> Vector3<f32> {
        self.wheel_to_bot * encoder_velocities.map(|v| MotionControl::ticks_to_meters(v))
    }

    #[inline(always)]
    /// Convert ticks / second to meters / second
    pub fn ticks_to_meters(value: i32) -> f32 {
        (value as f32 / TICKS_PER_ROTATION) * 4.0 * core::f32::consts::PI * WHEEL_RADIUS
    }

    #[inline(always)]
    /// Convert meters / second to ticks / second
    pub fn meters_to_ticks(value: f32) -> i32 {
        unsafe { (value / (4.0 * core::f32::consts::PI * WHEEL_RADIUS) * TICKS_PER_ROTATION).to_int_unchecked::<i32>() }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    const TOLERANCE: f32 = 0.00001;

    fn within_tolerance(value: f32, target: f32) -> bool {
        if target > value - TOLERANCE && target < value + TOLERANCE {
            true
        } else {
            false
        }
    }

    #[test]
    fn test_bot_to_wheel() {
        let motion_control = MotionControl::new();

        let expected_matrix = Matrix4x3::new(
            0.50000006,
            0.8660254,
            -0.0779815,
            -0.7071067,
            0.7071068,
            -0.0779815,
            -0.7071069,
            -0.70710665,
            -0.0779815,
            0.5,
            -0.8660254,
            -0.0779815,
        );

        assert_eq!(motion_control.bot_to_wheel, expected_matrix);
    }

    #[test]
    fn test_wheel_to_bot() {
        let motion_control = MotionControl::new();

        let expected_wheel_to_bot = Matrix3x4::new(
            0.41421354,
            -0.41421348,
            -0.4142136,
            0.41421357,
            0.3464102,
            0.2828428,
            -0.2828427,
            -0.3464102,
            -3.755932,
            -2.6558452,
            -2.6558454,
            -3.7559323,
        );

        assert_eq!(motion_control.wheel_to_bot, expected_wheel_to_bot);
    }

    #[test]
    fn test_ticks_to_meters() {
        assert_eq!(MotionControl::ticks_to_meters(6200), 1.0 * core::f32::consts::PI * 2.0 * 2.0 * WHEEL_RADIUS);
    }

    #[test]
    fn test_meters_to_ticks() {
        assert_eq!(MotionControl::meters_to_ticks(core::f32::consts::PI * 2.0 * 2.0 * WHEEL_RADIUS), 6200);
    }

    #[test]
    fn test_conversions() {
        assert_eq!(MotionControl::meters_to_ticks(MotionControl::ticks_to_meters(6200)), 6200);
    }

    #[test]
    fn test_body_to_wheels_up_and_down() {
        let motion_control = MotionControl::new();

        let up = Vector3::new(0.0, 1.0, 0.0);

        let wheel_velocities = motion_control.body_to_wheels(up);

        assert!(within_tolerance(wheel_velocities[0], -wheel_velocities[3]));
        assert!(within_tolerance(wheel_velocities[1], -wheel_velocities[2]));

        let down = Vector3::new(0.0, -1.0, 0.0);

        let wheel_velocities = motion_control.body_to_wheels(down);

        assert!(within_tolerance(wheel_velocities[0], -wheel_velocities[3]));
        assert!(within_tolerance(wheel_velocities[1], -wheel_velocities[2]));
    }

    #[test]
    fn test_body_to_wheels_left_and_right() {
        let motion_control = MotionControl::new();

        let left = Vector3::new(-1.0, 0.0, 0.0);

        let wheel_velocities = motion_control.body_to_wheels(left);

        assert!(within_tolerance(wheel_velocities[0], wheel_velocities[3]));
        assert!(within_tolerance(wheel_velocities[1], wheel_velocities[2]));

        let right = Vector3::new(1.0, 0.0, 0.0);

        let wheel_velocities = motion_control.body_to_wheels(right);

        assert!(within_tolerance(wheel_velocities[0], wheel_velocities[3]));
        assert!(within_tolerance(wheel_velocities[1], wheel_velocities[2]));
    }

    #[test]
    fn test_body_to_wheels_spin() {
        let motion_control = MotionControl::new();

        let counterclockwise = Vector3::new(0.0, 0.0, -1.0);

        let wheel_velocities = motion_control.body_to_wheels(counterclockwise);

        assert!(within_tolerance(wheel_velocities[0], wheel_velocities[1]));
        assert!(within_tolerance(wheel_velocities[1], wheel_velocities[2]));
        assert!(within_tolerance(wheel_velocities[2], wheel_velocities[3]));

        let clockwise = Vector3::new(0.0, 0.0, 1.0);

        let wheel_velocities = motion_control.body_to_wheels(clockwise);

        assert!(within_tolerance(wheel_velocities[0], wheel_velocities[1]));
        assert!(within_tolerance(wheel_velocities[1], wheel_velocities[2]));
        assert!(within_tolerance(wheel_velocities[2], wheel_velocities[3]));
    }

    /// Wheel Velocities need to be the following format to move up/down
    /// √(3)/2 * a + √(2)/2 * a - √(2)/2 * a - √(3)/2 * a
    #[test]
    fn test_wheel_to_body_vertical() {
        let motion_control = MotionControl::new();

        let up = Vector4::new(0.8660254038, 0.7071067812, -0.7071067812, -0.8660254038);

        let body_velocity = motion_control.wheels_to_body(up);

        assert!(within_tolerance(body_velocity[0], 0.0));
        assert!(body_velocity[1] > 0.0);
        assert!(within_tolerance(body_velocity[2], 0.0));

        let down = Vector4::new(-0.8660254038, -0.7071067812, 0.7071067812, 0.8660254038);

        let body_velocity = motion_control.wheels_to_body(down);

        assert!(within_tolerance(body_velocity[0], 0.0));
        assert!(body_velocity[1] < 0.0);
        assert!(within_tolerance(body_velocity[2], 0.0));
    }

    /// Wheel Velocities must be as follows to move horizontally:
    /// 1/2 * a - √(2)/2 * a - √(2)/2 * a + 1/2 * a
    #[test]
    fn test_wheel_to_body_horizontal() {
        let motion_control = MotionControl::new();

        let left = Vector4::new(0.5, -0.7071067812, -0.7071067812, 0.5);

        let body_velocity = motion_control.wheels_to_body(left);

        assert!(body_velocity[0] > 0.0);
        assert!(within_tolerance(body_velocity[1], 0.0));
        assert!(within_tolerance(body_velocity[2], 0.0));

        let right = Vector4::new(-0.5, 0.7071067812, 0.7071067812, -0.5);

        let body_velocity = motion_control.wheels_to_body(right);

        assert!(body_velocity[0] < 0.0);
        assert!(within_tolerance(body_velocity[1], 0.0));
        assert!(within_tolerance(body_velocity[2], 0.0));
    }

    #[test]
    fn test_wheel_to_body_spin() {
        let motion_control = MotionControl::new();

        let counterclockwise = Vector4::new(1.0, 1.0, 1.0, 1.0);

        let body_velocity = motion_control.wheels_to_body(counterclockwise);

        assert!(within_tolerance(body_velocity[0], 0.0));
        assert!(within_tolerance(body_velocity[1], 0.0));
        assert!(body_velocity[2] < 0.0);

        let clockwise = Vector4::new(-1.0, -1.0, -1.0, -1.0);

        let body_velocity = motion_control.wheels_to_body(clockwise);

        assert!(within_tolerance(body_velocity[0], 0.0));
        assert!(within_tolerance(body_velocity[1], 0.0));
        assert!(body_velocity[2] > 0.0);
    }

    #[test]
    fn test_wheels() {
        let motion_control = MotionControl::new();

        let motion = Vector3::new(0.0, 0.1, 0.0);

        let wheel_velocity = motion_control.body_to_wheels(motion);

        assert_eq!(
            wheel_velocity,
            Vector4::new(0.08660254, 0.07071068, -0.07071067, -0.08660254)
        );
    }

    #[test]
    fn test_wheels_two() {
        let motion_control = MotionControl::new();

        let motion = Vector3::new(0.0, 0.05, 0.0);

        let wheel_velocity = motion_control.body_to_wheels(motion);

        assert_eq!(
            wheel_velocity,
            Vector4::new(0.04330127, 0.03535534, -0.035355333, -0.04330127)
        );
    }
}
