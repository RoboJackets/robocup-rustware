//!
//! 
//!

use core::fmt::Debug;

use alloc::boxed::Box;
use libm::{sinf, cosf};

use nalgebra::base::*;

use crate::{WHEEL_DIST, WHEEL_RADIUS, FRONT_ANGLE, BACK_ANGLE};
use crate::motion_control_clock::MotionControlClock;

pub const SCALE_FACTOR: f32 = 0.5;

pub struct MotionControl {
    bot_to_wheel: Matrix4x3<f32>,
    wheel_to_bot: Matrix3x4<f32>,
    last_target_velocity: Vector3<f32>,
    scale_factor: f32,
    rotational_correction: f32,
    // Conversion from duty cycles -> rad/s
    duty_cycle_to_speed: f32,
    // Conversion from speed (rad/s) -> duty cycles
    speed_to_duty_cycle: f32,
    counter: u32,
}

impl MotionControl {
    pub fn new(clock: Box<dyn MotionControlClock>) -> Self {
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
            scale_factor: SCALE_FACTOR,
            last_target_velocity: Vector3::zeros(),
            rotational_correction: 0.0,
            counter: 0,
        }
    }

    pub fn without_clock() -> Self {
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
            scale_factor: SCALE_FACTOR,
            last_target_velocity: Vector3::zeros(),
            rotational_correction: 0.0,
            counter: 0,
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

    /// Convert target velocity to wheel velocity compensating for the rotation over time.
    pub fn control_update(&mut self, mut target_velocity: Vector3<f32>, rotational_measurement: f32) -> Vector4<f32> {
        let rotational_difference = self.last_target_velocity[2] - rotational_measurement;
        self.rotational_correction += rotational_difference * self.scale_factor;
        target_velocity[2] += self.rotational_correction;

        self.counter += 1;
        if self.counter == 100 {
            self.counter = 0;
            self.scale_factor *= SCALE_FACTOR;
        }

        self.body_to_wheels(target_velocity)
    }
}

impl Debug for MotionControl {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Motion Control")
            .field("Correction", &self.rotational_correction)
            .field("Last Target", &self.last_target_velocity)
            .finish()
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
        let motion_control = MotionControl::without_clock();

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
        let motion_control = MotionControl::without_clock();

        let expected_wheel_to_bot = Matrix3x4::new(
            0.011539989, -0.011539988, -0.011539991, 0.011539988,
            0.009650988, 0.007879999, -0.007879999, -0.009650989,
            -0.10464025, -0.073991835, -0.073991835, -0.10464027
        );

        assert_eq!(motion_control.wheel_to_bot, expected_wheel_to_bot);
    }

    #[test]
    fn test_body_to_wheels_up_and_down() {
        let motion_control = MotionControl::without_clock();

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
        let motion_control = MotionControl::without_clock();

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
        let motion_control = MotionControl::without_clock();

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
        let motion_control = MotionControl::without_clock();

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
        let motion_control = MotionControl::without_clock();

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
        let motion_control = MotionControl::without_clock();

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
        let motion_control = MotionControl::without_clock();

        let motion = Vector3::new(0.0, 0.1, 0.0);

        let wheel_velocity = motion_control.body_to_wheels(motion);

        assert_eq!(wheel_velocity, Vector4::new(0.024867924, 0.020304576, -0.020304572, -0.024867924));
    }

    #[test]
    fn test_wheels_two() {
        let motion_control = MotionControl::without_clock();

        let motion = Vector3::new(0.0, 0.05, 0.0);

        let wheel_velocity = motion_control.body_to_wheels(motion);

        assert_eq!(wheel_velocity, Vector4::new(0.012433962, 0.010152288, -0.010152286, -0.012433962));
    }

    #[test]
    fn test_control_update_with_angular_rotation() {
        let mut motion_control = MotionControl::without_clock();

        let target_motion = Vector3::new(1.0, 0.0, 0.0);
        let angular_measurement = -0.1;

        let wheel_velocities = motion_control.control_update(target_motion, angular_measurement);

        assert_eq!(wheel_velocities, motion_control.body_to_wheels(Vector3::new(1.0, 0.0, 0.05)));

        let target_motion = Vector3::new(1.0, 0.0, 0.0);
        let angular_measurement = -0.1;

        let wheel_velocities = motion_control.control_update(target_motion, angular_measurement);

        assert_eq!(wheel_velocities, motion_control.body_to_wheels(Vector3::new(1.0, 0.0, 0.1)));
    }
}