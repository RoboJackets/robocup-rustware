#![no_std]

extern crate alloc;

use nalgebra::base::*;

pub mod motion_control;
pub use motion_control::MotionControl;

pub const WHEEL_RADIUS: f32 = 0.02786;
pub const REAR_WHEEL_DIST: f32 = 0.077874;
pub const FRONT_WHEEL_DIST: f32 = 0.078089;
pub const FRONT_ANGLE: f32 = 30.0;
pub const BACK_ANGLE: f32 = 45.0;

pub const WHEEL_DIST: f32 = (FRONT_WHEEL_DIST + REAR_WHEEL_DIST) / 2.0;

// Constants for Test Fake Motion Control
pub const LEFT: Vector3<f32> = Vector3::new(-1.0, 0.0, 0.0);
pub const RIGHT: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
pub const UP: Vector3<f32> = Vector3::new(0.0, 0.1, 0.0);
pub const DOWN: Vector3<f32> = Vector3::new(0.0, -1.0, 0.0);
pub const COUNTERCLOCKWISE: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
pub const CLOCKWISE: Vector3<f32> = Vector3::new(0.0, 0.0, -1.0);

/// Weighting for measurements
pub const ALPHA: f32 = 0.15;
