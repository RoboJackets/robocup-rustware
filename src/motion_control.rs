use libm::{sinf, cosf};

use nalgebra::base::*;

const PI: f32 = 3.14159;

const WHEEL_RADIUS: f32 = 0.02786;
const READ_WHEEL_DIST: f32 = 0.077874;
const FRONT_WHEEL_DIST: f32 = 0.078089;
const FRONT_ANGLE: f32 = 30.0;
const BACK_ANGLE: f32 = 45.0;

const WHEEL_ANGLES: [f32; 4] = [
    (180.0 - FRONT_ANGLE) * PI / 180.0,
    (180.0 + BACK_ANGLE) * PI / 180.0,
    (360.0 - BACK_ANGLE) * PI / 180.0,
    (0.0 + FRONT_ANGLE) * PI / 180.0,
];

const WHEEL_DIST: f32 = (FRONT_WHEEL_DIST + READ_WHEEL_DIST) / 2.0;

const BODY_KP: Vector3<f32> = Vector3::new(0.8, 0.8, 1.5);

const MAX_ACCELERATION: Vector3<f32> = Vector3::new(4.0, 3.0, 30.0);

pub struct MotionControl {
    bot_to_wheel: Matrix4x3<f32>,
    wheel_to_bot: Matrix3x4<f32>,
}

impl MotionControl {
    pub fn new() -> Self {
        let bot_to_wheel = Matrix4x3::new(
            -sinf(WHEEL_ANGLES[0]), cosf(WHEEL_ANGLES[1]), WHEEL_DIST,
            -sinf(WHEEL_ANGLES[1]), cosf(WHEEL_ANGLES[1]), WHEEL_DIST,
            -sinf(WHEEL_ANGLES[2]), cosf(WHEEL_ANGLES[2]), WHEEL_DIST,
            -sinf(WHEEL_ANGLES[3]), cosf(WHEEL_ANGLES[3]), WHEEL_DIST,
        );
        // Invert because the wheels spin opposite of paper
        let bot_to_wheel = bot_to_wheel.map(|a| a * -1.0 / WHEEL_RADIUS);
        
        let wheel_to_bot = bot_to_wheel.transpose() * bot_to_wheel;
        let wheel_to_bot = wheel_to_bot.try_inverse().unwrap();
        let wheel_to_bot = wheel_to_bot * bot_to_wheel.transpose();

        Self {
            bot_to_wheel,
            wheel_to_bot,
        }
    }

    pub fn body_to_wheel(&self, body_velocity: &[f32; 3]) -> Vector4<f32> {
        let body_velocity = Vector3::from_row_slice(body_velocity);

        return self.bot_to_wheel * body_velocity;
    }
}