//!
//! Linear Kalman Filter Implementation for our robots
//! 

use nalgebra::{Vector3, Matrix3};

pub struct KalmanState {
    /// The filter's representation of our state
    state: Vector3<f32>,
    /// The filter's representation of our covariance
    covariance: Matrix3<f32>,
}

impl KalmanState {
    /// Create a new Kalman state to estimate our robot's velocity
    pub fn new() -> Self {
        Self {
            state: Vector3::zeros(),
            covariance: Matrix3::identity(),
        }
    }

    /// Propogate a prediction step through the filter using the IMU data
    pub fn predict_imu(&mut self, gyro_z: f32, accel_x: f32, accel_y: f32) {
        
    }
}