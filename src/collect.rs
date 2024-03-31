//!
//! Types necessary for collecting and recording motion control
//! data
//! 

#[derive(Debug, Clone, Copy)]
pub struct MotionControlReading {
    pub valid: bool,
    pub accel_x: i32,
    pub accel_y: i32,
    pub gyro_z: i32,
    pub encoder_values: [u8; 10],
    pub delta_t: u64,
}

impl MotionControlReading {
    pub fn to_bytes(self) -> [u8; 31] {
        let mut buffer = [0u8; 31];

        buffer[..4].copy_from_slice(&self.accel_x.to_le_bytes());
        buffer[4..8].copy_from_slice(&self.accel_y.to_le_bytes());
        buffer[8..12].copy_from_slice(&self.gyro_z.to_le_bytes());
        buffer[12..22].copy_from_slice(&self.encoder_values);
        buffer[23..31].copy_from_slice(&self.delta_t.to_le_bytes());

        if self.valid {
            buffer[22] = 0;
        } else {
            buffer[22] = 255;
        }

        buffer
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        let accel_x = i32::from_le_bytes(bytes[0..4].try_into().unwrap());
        let accel_y = i32::from_le_bytes(bytes[4..8].try_into().unwrap());
        let gyro_z = i32::from_le_bytes(bytes[8..12].try_into().unwrap());
        let encoder_values: [u8; 10] = bytes[12..22].try_into().unwrap();
        let valid = bytes[22] == 0;
        let delta_t = u64::from_le_bytes(bytes[23..31].try_into().unwrap());

        Self {
            accel_x,
            accel_y,
            gyro_z,
            encoder_values,
            valid,
            delta_t,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MotionControlHeader {
    pub target_velocity: [f32; 3],
    pub valid: bool,
}

impl MotionControlHeader {
    pub fn to_bytes(self) -> [u8; 13] {
        let mut buffer = [0u8; 13];

        buffer[..4].copy_from_slice(&self.target_velocity[0].to_le_bytes());
        buffer[4..8].copy_from_slice(&self.target_velocity[1].to_le_bytes());
        buffer[8..12].copy_from_slice(&self.target_velocity[2].to_le_bytes());
        
        if self.valid {
            buffer[12] = 0;
        } else {
            buffer[12] = 255;
        }

        buffer
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        let x = f32::from_le_bytes(bytes[0..4].try_into().unwrap());
        let y = f32::from_le_bytes(bytes[4..8].try_into().unwrap());
        let w = f32::from_le_bytes(bytes[8..12].try_into().unwrap());
        let valid = bytes[12] == 0;

        Self {
            target_velocity: [x, y, w],
            valid,
        }
    }
}