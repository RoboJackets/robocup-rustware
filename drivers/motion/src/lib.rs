#![no_std]

extern crate alloc;

use alloc::{format, string::String, vec::Vec};
use libm::{sinf, cosf};

use nalgebra::base::*;

const WHEEL_RADIUS: f32 = 0.02786;
const REAR_WHEEL_DIST: f32 = 0.077874;
const FRONT_WHEEL_DIST: f32 = 0.078089;
const FRONT_ANGLE: f32 = 30.0;
const BACK_ANGLE: f32 = 45.0;

const WHEEL_DIST: f32 = (FRONT_WHEEL_DIST + REAR_WHEEL_DIST) / 2.0;

// Constants for Test Fake Motion Control
pub const LEFT: Vector3<f32> = Vector3::new(-1.0, 0.0, 0.0);
pub const RIGHT: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
pub const UP: Vector3<f32> = Vector3::new(0.0, 0.1, 0.0);
pub const DOWN: Vector3<f32> = Vector3::new(0.0, -1.0, 0.0);
pub const COUNTERCLOCKWISE: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
pub const CLOCKWISE: Vector3<f32> = Vector3::new(0.0, 0.0, -1.0);

pub struct MotionControl {
    bot_to_wheel: Matrix4x3<f32>,
    wheel_to_bot: Matrix3x4<f32>,
    // Conversion from duty cycles -> rad/s
    duty_cycle_to_speed: f32,
    // Conversion from speed (rad/s) -> duty cycles
    speed_to_duty_cycle: f32,
    // Cumulative Encoder Values
    cumulative_encoders: Vector4<i16>,
    // Cumulative Time
    start_time: u32,
    // Velocities
    velocities: Vec<Vector4<f32>>,
    counter: usize,
}

impl MotionControl {
    pub fn new() -> Self {
        let wheel_angles = [
            (180.0 - FRONT_ANGLE).to_radians(),
            (180.0 + BACK_ANGLE).to_radians(),
            (360.0 - BACK_ANGLE).to_radians(),
            (0.0 + FRONT_ANGLE).to_radians(),
        ];

        let bot_to_wheel = Matrix4x3::new(
            -sinf(wheel_angles[0]), cosf(wheel_angles[0]), WHEEL_DIST,
            -sinf(wheel_angles[1]), cosf(wheel_angles[1]), WHEEL_DIST,
            -sinf(wheel_angles[2]), cosf(wheel_angles[2]), WHEEL_DIST,
            -sinf(wheel_angles[3]), cosf(wheel_angles[3]), WHEEL_DIST,
        );
        let bot_to_wheel = bot_to_wheel.map(|v| (v * -1.0) / WHEEL_RADIUS);

        let bot_to_wheel_t = bot_to_wheel.transpose();

        let wheel_to_bot = (&bot_to_wheel_t * &bot_to_wheel).try_inverse().unwrap() * &bot_to_wheel_t;

        Self {
            bot_to_wheel,
            wheel_to_bot,
            duty_cycle_to_speed: 125.0,
            speed_to_duty_cycle: 1.0 / 125.0,
            cumulative_encoders: Vector4::zeros(),
            counter: 0,
            start_time: 0,
            velocities: Vec::with_capacity(5),
        }
    }
    
    /// Convert a body velocity into duty cycles for the robot
    pub fn body_to_wheels(&self, body_velocity: Vector3<f32>) -> Vector4<f32> {
        let velocity = self.bot_to_wheel * body_velocity;
        velocity.map(|v| v * self.speed_to_duty_cycle)
    }
    
    /// Converts a wheel's duty cycles into velocity for the robot
    pub fn wheels_to_body(&self, wheel_velocity: Vector4<f32>) -> Vector3<f32> {
        let body_duty_cycles = self.wheel_to_bot * wheel_velocity;
        body_duty_cycles.map(|v| v * self.duty_cycle_to_speed)
    }

    /// Add the timestamp and encoder values from the last iteration
    pub fn add_encoders(&mut self, encoder_values: [i16; 5], timestamp: u32) {
        if self.start_time == 0 {
            self.start_time = timestamp;
        }
        let encoder_deltas = Vector4::new(encoder_values[0], encoder_values[1], encoder_values[2], encoder_values[3]);
        self.cumulative_encoders += encoder_deltas;

        if timestamp - self.start_time > 500_000 {
            let cumulative_time = (timestamp - self.start_time) as f32;
            let velocity = self.cumulative_encoders.map(|v| (v as f32) / cumulative_time);
            self.velocities.push(velocity);

            self.cumulative_encoders = Vector4::zeros();
            self.start_time = timestamp;

            if self.velocities.len() == 5 {
                self.velocities.clear();
            }
        }
    }

    pub fn get_stats(&self) -> String {
        format!("Velocities: {:?}", self.velocities)
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
            17.946878, 31.084902, -2.799049,
            -25.380714, 25.380718, -2.799049,
            -25.38072, -25.380713, -2.799049,
            17.946877, -31.084902, -2.799049,
        );

        assert_eq!(motion_control.bot_to_wheel, expected_matrix);
    }

    #[test]
    fn test_wheel_to_bot() {
        let motion_control = MotionControl::new();

        let expected_wheel_to_bot = Matrix3x4::new(
            0.011539989, -0.011539988, -0.011539991, 0.011539988,
            0.009650988, 0.007879999, -0.007879999, -0.009650989,
            -0.10464025, -0.073991835, -0.073991835, -0.10464027
        );

        assert_eq!(motion_control.wheel_to_bot, expected_wheel_to_bot);
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

        assert_eq!(wheel_velocity, Vector4::zeros());
    }

    #[test]
    fn test_wheels_two() {
        let motion_control = MotionControl::new();

        let motion = Vector3::new(0.0, 0.05, 0.0);

        let wheel_velocity = motion_control.body_to_wheels(motion);

        assert_eq!(wheel_velocity, Vector4::zeros());
    }
}
