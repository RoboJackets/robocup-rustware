//!
//! The Controller used to test manual control and consists of a bunch of buttons
//! on a breadboard (currently) to trigger specific fpga actions.
//!

#![no_std]

use teensy4_bsp::hal::gpio::Input;

use nalgebra::base::*;

pub const LEFT: Vector3<f32> = Vector3::new(-10.0, 0.0, 0.0);
pub const RIGHT: Vector3<f32> = Vector3::new(10.0, 0.0, 0.0);
pub const UP: Vector3<f32> = Vector3::new(0.0, 10.0, 0.0);
pub const DOWN: Vector3<f32> = Vector3::new(0.0, -10.0, 0.0);
pub const CLOCKWISE: Vector3<f32> = Vector3::new(0.0, 0.0, -10.0);
pub const COUNTERCLOCKWISE: Vector3<f32> = Vector3::new(0.0, 0.0, 10.0);

pub struct Controller<L, R, U, D, CCWS, CWS> {
    left: Input<L>,
    right: Input<R>,
    up: Input<U>,
    down: Input<D>,
    clockwise: Input<CWS>,
    counterclockwise: Input<CCWS>,
}

impl<L, R, U, D, CCWS, CWS> Controller<L, R, U, D, CCWS, CWS> {
    pub fn new(
        left: Input<L>,
        right: Input<R>,
        up: Input<U>,
        down: Input<D>,
        counterclockwise: Input<CCWS>,
        clockwise: Input<CWS>,
    ) -> Self {
        Self {
            left,
            right,
            up,
            down,
            clockwise,
            counterclockwise,
        }
    }

    pub fn calculate_movement(&self) -> Vector3<f32> {
        let mut movement = Vector3::zeros();

        if !self.left.is_triggered() {
            movement += LEFT;
        }

        if !self.right.is_triggered() {
            movement += RIGHT;
        }

        if !self.up.is_triggered() {
            movement += UP;
        }

        if !self.down.is_triggered() {
            movement += DOWN;
        }

        if !self.clockwise.is_triggered() {
            movement += CLOCKWISE;
        }

        if !self.counterclockwise.is_triggered() {
            movement += COUNTERCLOCKWISE;
        }

        movement
    }
}