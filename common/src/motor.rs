//!
//! Common interfaces for the motor board
//! 

use core::fmt::Debug;
use defmt::Format;
use ncomm_utils::packing::{Packable, PackingError};

/// The size (in bytes) of a motor command.
pub const MOTOR_COMMAND_SIZE: usize = 13;
/// The size (in bytes) of a motor move response
pub const MOTOR_MOVE_RESPONSE_SIZE: usize = 5;
/// The size (in bytes) of a pid response
pub const PID_RESPONSE_SIZE: usize = 4000;

#[derive(Format, Debug, PartialEq, Clone, Copy)]
/// Commands that can be sent to the motor board
pub enum MotorCommand {
    /// Move the motor at `ticks_per_second` ticks per second, where a tick
    /// is a singular encoder tick
    Move { ticks_per_second: i32 },
    /// Set the PID Values for the motor driver
    SetPid {
        // the kp value for the motor driver
        kp: f32,
        // the ki value for the motor driver
        ki: f32,
        // the kd value for the motor driver
        kd: f32,
    },
    /// Unknown command
    Unknown,
}

impl Default for MotorCommand {
    fn default() -> Self {
        Self::Move { ticks_per_second: 0 }
    }
}

impl Packable for MotorCommand {
    fn len() -> usize {
        MOTOR_COMMAND_SIZE
    }

    fn pack(self, buffer: &mut [u8]) -> Result<(), PackingError> {
        if buffer.len() < Self::len() {
            return Err(PackingError::InvalidBufferSize);
        }

        match self {
            Self::Move { ticks_per_second} => {
                buffer[0] = 0x01;
                buffer[1..5].copy_from_slice(&ticks_per_second.to_le_bytes());
            },
            Self::SetPid { kp, ki, kd } => {
                buffer[0] = 0x02;
                buffer[1..5].copy_from_slice(&kp.to_le_bytes());
                buffer[5..9].copy_from_slice(&ki.to_le_bytes());
                buffer[9..13].copy_from_slice(&kd.to_le_bytes());
            },
            Self::Unknown => (),
        }

        Ok(())
    }

    fn unpack(data: &[u8]) -> Result<Self, PackingError> {
        if data.len() < Self::len() {
            return Err(PackingError::InvalidBufferSize);
        }

        match data[0] {
            0x01 => Ok(Self::Move {
                ticks_per_second: i32::from_le_bytes(data[1..5].try_into().unwrap()),
            }),
            0x02 => Ok(Self::SetPid {
                kp: f32::from_le_bytes(data[1..5].try_into().unwrap()),
                ki: f32::from_le_bytes(data[5..9].try_into().unwrap()),
                kd: f32::from_le_bytes(data[9..13].try_into().unwrap()),
            }),
            _ => Ok(Self::Unknown),
        }
    }
}

#[derive(Format, Debug, Clone, Copy, PartialEq, Eq)]
/// The response returned from the motor board when it is instructed to move
pub struct MotorMoveResponse {
    // TODO: Add status byte to tell the status of the motor board

    /// The number of ticks per second the motor is actually moving at
    pub ticks_per_second: i32,
}

impl Packable for MotorMoveResponse {
    fn len() -> usize {
        MOTOR_MOVE_RESPONSE_SIZE
    }

    fn pack(self, buffer: &mut [u8]) -> Result<(), PackingError> {
        if buffer.len() < Self::len() {
            return Err(PackingError::InvalidBufferSize);
        }

        buffer[1..5].copy_from_slice(&self.ticks_per_second.to_le_bytes());

        Ok(())
    }

    fn unpack(data: &[u8]) -> Result<Self, PackingError> {
        if data.len() < Self::len() {
            return Err(PackingError::InvalidBufferSize);
        }
        
        
        Ok(Self {
            ticks_per_second: i32::from_le_bytes(data[1..5].try_into().unwrap()),
        })
    }
}

#[derive(Format, Debug, Clone, PartialEq)]
/// The response returned from the motor board when it is tuning PID
pub struct PidResponse {
    /// An array of the various errors from attempting to move the motor at the given
    /// speed
    pub errors: [f32; 1000],
}

impl Packable for PidResponse {
    fn len() -> usize {
        PID_RESPONSE_SIZE
    }

    fn pack(self, buffer: &mut [u8]) -> Result<(), PackingError> {
        if buffer.len() < Self::len() {
            return Err(PackingError::InvalidBufferSize);
        }

        for i in 0..1000 {
            buffer[i*4..(i+1)*4].copy_from_slice(&self.errors[i].to_le_bytes());
        }

        Ok(())
    }

    fn unpack(data: &[u8]) -> Result<Self, PackingError> {
        let mut errors = [0f32; 1000];

        for i in 0..1000 {
            errors[i] = f32::from_le_bytes(data[i*4..(i+1)*4].try_into().unwrap());
        }

        Ok(Self {
            errors
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_pack_unpack_move_command() {
        let command = MotorCommand::Move { ticks_per_second: 32 };
        let mut buffer = [0u8; 13];
        command.clone().pack(&mut buffer).unwrap();
        assert_eq!(
            command,
            MotorCommand::unpack(&buffer).unwrap()
        );
    }

    #[test]
    fn test_pack_unpack_pid_command() {
        let command = MotorCommand::SetPid { kp: 32.0, ki: 0.0, kd: 1.0 };
        let mut buffer = [0u8; 13];
        command.clone().pack(&mut buffer).unwrap();
        assert_eq!(
            command,
            MotorCommand::unpack(&buffer).unwrap(),
        );
    }

    #[test]
    fn test_pack_unpack_move_respones() {
        let move_response = MotorMoveResponse {
            ticks_per_second: 60,
        };
        let mut buffer = [0u8; 5];
        move_response.clone().pack(&mut buffer).unwrap();
        assert_eq!(
            move_response,
            MotorMoveResponse::unpack(&buffer).unwrap(),
        )
    }

    #[test]
    fn test_pack_unpack_pid_response() {
        let mut errors = [0f32; 1000];
        for i in 0..1000 {
            errors[i] = i as f32;
        }
        let response = PidResponse {
            errors
        };
        let mut buffer = [0u8; 4000];
        response.clone().pack(&mut buffer).unwrap();
        assert_eq!(
            response,
            PidResponse::unpack(&buffer).unwrap(),
        );
    }
}