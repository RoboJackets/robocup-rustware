//!
//! Controller for the robot
//! 

use common::motor::MotorCommand;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::{Publisher, Subscriber, WaitResult}};
use nalgebra::{Matrix4x3, Vector3};
use libm::{sinf, cosf};

use crate::radio::control_command::ControlMessage;

/// The proportional, integral, and derivative gains for the PID controller
pub const LINEAR_KP: f32 = 1.0;
pub const LINEAR_KI: f32 = 0.0;
pub const LINEAR_KD: f32 = 0.0;
pub const ANGULAR_KP: f32 = 1.0;
pub const ANGULAR_KI: f32 = 0.0;
pub const ANGULAR_KD: f32 = 0.0;

/// The wheel radius in meters
const WHEEL_RADIUS: f32 = 0.02786;
/// The number of encoder ticks per rotation
const ENCODER_TICKS_PER_ROTATION: f32 = 6144.0;
/// The rear wheel distance
const REAR_WHEEL_DISTANCE: f32 = 0.077874;
/// The front wheel distance
const FRONT_WHEEL_DISTANCE: f32 = 0.078089;
/// The front wheel angle
const FRONT_ANGLE: f32 = 30.0;
/// The back wheel angle
const BACK_ANGLE: f32 = 45.0;
/// The distance between wheels
const WHEEL_DIST: f32 = (FRONT_WHEEL_DISTANCE + REAR_WHEEL_DISTANCE) / 2.0;

/// The angles of the wheels in radians
const WHEEL_ANGLES: [f32; 4] = [
    (180.0 - FRONT_ANGLE).to_radians(),
    (180.0 + FRONT_ANGLE).to_radians(),
    (180.0 - BACK_ANGLE).to_radians(),
    (180.0 + BACK_ANGLE).to_radians(),
];

/// Our robot's idea of it's current state
pub struct State {
    /// The x-direction velocity of the robot (m/s)
    pub vx: f32,
    /// The y-direction velocity of the robot (m/s)
    pub vy: f32,
    /// The angular velocity of the robot (rad/s)
    pub omega: f32,
}

#[inline(always)]
pub fn ticks_to_meters(ticks: i32) -> f32 {
    (ticks as f32 / ENCODER_TICKS_PER_ROTATION) * 2.0 * core::f32::consts::PI * WHEEL_RADIUS
}

#[inline(always)]
pub fn meters_to_ticks(meters: f32) -> i32 {
    (meters / (2.0 * core::f32::consts::PI * WHEEL_RADIUS) * ENCODER_TICKS_PER_ROTATION) as i32
}

#[embassy_executor::task]
pub async fn control_task(
    mut command_subscriber: Subscriber<'static, NoopRawMutex, ControlMessage, 4, 2, 1>,
    motor_command_publishers: [Publisher<'static, NoopRawMutex, MotorCommand, 4, 1, 1>; 4],
    mut power_off_subscriber: Subscriber<'static, NoopRawMutex, (), 4, 2, 2>,
) {
    // The conversion matrix from body velocities to wheel velocities
    let body_to_wheel = Matrix4x3::new(
        sinf(WHEEL_ANGLES[0]), -cosf(WHEEL_ANGLES[0]), -WHEEL_DIST,
        sinf(WHEEL_ANGLES[1]), -cosf(WHEEL_ANGLES[1]), -WHEEL_DIST,
        sinf(WHEEL_ANGLES[2]), -cosf(WHEEL_ANGLES[2]), -WHEEL_DIST,
        sinf(WHEEL_ANGLES[3]), -cosf(WHEEL_ANGLES[3]), -WHEEL_DIST,
    );

    let mut desired_state = State { vx: 0.0, vy: 0.0, omega: 0.0 };
    let mut turn_off = false;

    loop {
        // Wait for an updated control message and update our desired state accordingly
        if let WaitResult::Message(command) = command_subscriber.next_message().await {
            if !turn_off {
                desired_state.vx = (command.body_x as f32) * 1000.0;
                desired_state.vy = (command.body_y as f32) * 1000.0;
                desired_state.omega = (command.body_w as f32) * 1000.0;
            }
        }

        // Check if we have low battery and should stop the robot
        if let Some(WaitResult::Message(())) = power_off_subscriber.try_next_message() {
            turn_off = true;
            desired_state.vx = 0.0;
            desired_state.vy = 0.0;
            desired_state.omega = 0.0;
        }

        let wheel_velocities = body_to_wheel * Vector3::new(desired_state.vx, desired_state.vy, desired_state.omega);
        for i in 0..4 {
            motor_command_publishers[i].publish_immediate(MotorCommand::Move { ticks_per_second: meters_to_ticks(wheel_velocities[i]) });
        }
    }
}