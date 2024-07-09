#![allow(unused)]

use libm::{sinf, cosf};

use nalgebra::base::*;

use fixed_queue::VecDeque;

const PI: f32 = 3.14159;

const WHEEL_RADIUS: f32 = 0.02786;
const READ_WHEEL_DIST: f32 = 0.077874;
const FRONT_WHEEL_DIST: f32 = 0.078089;
const FRONT_ANGLE: f32 = 30.0;
const BACK_ANGLE: f32 = 45.0;
const ENCODER_COUNTS_PER_TURN: f32 = 512.0;

/// We think the gear ratio is 1 / 18.0 because encoder readings seem to be
/// roughly 18x what we expect
const GEAR_RATIO: f32 = 1.0 / 18.0;

const WHEEL_DIST: f32 = (FRONT_WHEEL_DIST + READ_WHEEL_DIST) / 2.0;

const BODY_KP: Vector3<f32> = Vector3::new(0.8, 0.8, 1.5);

const MAX_ACCELERATION: Vector3<f32> = Vector3::new(4.0, 3.0, 30.0);

/// Alpha weighting for acceleration
pub const ALPHA: f32 = 0.15;

// There are 512 Encoder Counts Per Revolution

pub struct MotionControl {
    bot_to_wheel: Matrix4x3<f32>,
    _wheel_to_bot: Matrix3x4<f32>,
    // Last 10 desired wheel velocities (most_recent, ..., least_recent)
    wheel_velocities: Matrix<f32, U4, U10, ArrayStorage<f32, 4, 10>>,
    // Last 10 encoder velocities (most_recent, ..., least_recent)
    encoder_velocities: Matrix<f32, U4, U10, ArrayStorage<f32, 4, 10>>,
    // The measurement timestamp we are at
    measurement: usize,
    // A linear value used to compensate for motors being off
    linear_compensator: Vector4<f32>,
    // Multiplier to convert form encoder counts to meters.
    encoder_multiplier: f32,
    // Delay (in us) between motion control commands
    motion_control_delay_s: f32,
}

impl MotionControl {
    pub fn new(motion_control_delay_us: u32) -> Self {
        let wheel_angles: [f32; 4] = [
            f32::to_radians(180.0 - FRONT_ANGLE),
            f32::to_radians(180.0 + BACK_ANGLE),
            f32::to_radians(360.0 - BACK_ANGLE),
            f32::to_radians(0.0 + FRONT_ANGLE),
        ];

        let bot_to_wheel = Matrix4x3::new(
            -sinf(wheel_angles[0]), cosf(wheel_angles[0]), WHEEL_DIST,
            -sinf(wheel_angles[1]), cosf(wheel_angles[1]), WHEEL_DIST,
            -sinf(wheel_angles[2]), cosf(wheel_angles[2]), WHEEL_DIST,
            -sinf(wheel_angles[3]), cosf(wheel_angles[3]), WHEEL_DIST,
        );

        // Invert because the wheels spin opposite of paper
        let bot_to_wheel = bot_to_wheel.map(|a| a * -1.0 / WHEEL_RADIUS);
        
        let wheel_to_bot = bot_to_wheel.transpose() * bot_to_wheel;
        let wheel_to_bot = wheel_to_bot.try_inverse().unwrap();
        let wheel_to_bot = wheel_to_bot * bot_to_wheel.transpose();

        let encoder_multiplier = WHEEL_RADIUS * 2.0 * PI / ENCODER_COUNTS_PER_TURN;

        Self {
            bot_to_wheel,
            _wheel_to_bot: wheel_to_bot,
            wheel_velocities: Matrix::<f32, U4, U10, ArrayStorage<f32, 4, 10>>::zeros(),
            encoder_velocities: Matrix::<f32, U4, U10, ArrayStorage<f32, 4, 10>>::zeros(),
            measurement: 0,
            linear_compensator: Vector4::zeros(),
            encoder_multiplier,
            motion_control_delay_s: (motion_control_delay_us as f32) / 1_000_000.0,
        }
    }

    pub fn body_to_wheels(&self, body_velocity: Vector3<f32>) -> Vector4<f32> {
        self.bot_to_wheel * body_velocity
    }

    /// Use encoder values to make motion control better.
    /// 
    /// 1. Store the past 10 encoder and target velocities.
    /// 2. Compute the differences and find the average difference between the encoder velocities and the
    /// target velocities.
    /// 3. Store this average as a linear offset to the given target velocity
    /// 4. Offset the target velocity by the linear offset
    pub fn body_with_encoders(
        &mut self,
        body_velocity: Vector3<f32>,
        encoder_counts: &[i16; 5],
    ) -> Vector4<f32> {
        let dt = (encoder_counts[4] as f32) * (1.0 / 18.432e6) * 2.0 * 128.0;
        let encoder_values = Vector4::new(
            (encoder_counts[0] as f32) / self.motion_control_delay_s,
            (encoder_counts[1] as f32) / self.motion_control_delay_s,
            (encoder_counts[2] as f32) / self.motion_control_delay_s,
            (encoder_counts[3] as f32) / self.motion_control_delay_s,
        );
        self.encoder_velocities.set_column((self.measurement - 1) % 10, &encoder_values);

        let target_velocity = self.bot_to_wheel * body_velocity;
        self.wheel_velocities.set_column(self.measurement % 10, &target_velocity);

        if self.measurement == 10 {
            let differences = self.wheel_velocities - self.encoder_velocities;
            self.linear_compensator = differences.column_mean();

            self.wheel_velocities = Matrix::<f32, U4, U10, ArrayStorage<f32, 4, 10>>::zeros();
            self.encoder_velocities = Matrix::<f32, U4, U10, ArrayStorage<f32, 4, 10>>::zeros();
            self.measurement = 0;
        } else {
            self.measurement += 1;
        }

        target_velocity + self.linear_compensator
    }
}