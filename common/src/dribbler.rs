//!
//! Common interfaces for the dribbler
//! 

use core::fmt::Debug;
use defmt::Format;
use ncomm_utils::packing::{Packable, PackingError};

/// The size (in bytes) of a dribbler command
pub const DRIBBLER_COMMAND_SIZE: usize = 2;
/// The size (in bytes) of a dribbler response
pub const DRIBBLER_RESPONSE_SIZE: usize = 1;

#[derive(Format, Debug, PartialEq, Clone, Copy)]
/// Commands that can be sent to the dribber
pub enum DribblerCommand {
    /// Move the motor at some percent of maximum speed
    Move { percent: u8 },
    /// Unknown command
    Unknown,
}

impl Default for DribblerCommand {
    fn default() -> Self {
        Self::Move { percent: 0 }
    }
}

impl Packable for DribblerCommand {
    fn len() -> usize {
        DRIBBLER_COMMAND_SIZE
    }

    fn pack(self, buffer: &mut [u8]) -> Result<(), PackingError> {
        if buffer.len() < Self::len() {
            return Err(PackingError::InvalidBufferSize);
        }

        match self {
            Self::Move { percent } => {
                buffer[0] = 0x01;
                buffer[1] = percent;
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
            0x01 => Ok(Self::Move { percent: data[1] }),
            _ => Ok(Self::Unknown)
        }
    }
}

#[derive(Format, Debug, Clone, Copy, PartialEq, Eq)]
/// The response returnd from the dribbler telling its status
pub struct DribblerResponse {
    pub healthy: bool,
}

impl Packable for DribblerResponse {
    fn len() -> usize {
        DRIBBLER_RESPONSE_SIZE
    }

    fn pack(self, buffer: &mut [u8]) -> Result<(), PackingError> {
        if buffer.len() < Self::len() {
            return Err(PackingError::InvalidBufferSize);
        }

        buffer[0] = if self.healthy { 0xFF } else { 0x00 };

        Ok(())
    }

    fn unpack(data: &[u8]) -> Result<Self, PackingError> {
        if data.len() < Self::len() {
            return Err(PackingError::InvalidBufferSize);
        }

        Ok(Self {
            healthy: data[0] == 0xFF,
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_pack_unpack_dribbler_command() {
        let command = DribblerCommand::Move { percent: 50 };
        let mut buffer = [0u8; 2];
        command.clone().pack(&mut buffer).unwrap();
        assert_eq!(
            command,
            DribblerCommand::unpack(&buffer).unwrap(),
        )
    }

    #[test]
    fn test_pack_unpack_dribbler_response() {
        let response = DribblerResponse { healthy: true };
        let mut buffer = [0u8];
        response.clone().pack(&mut buffer).unwrap();
        assert_eq!(
            response,
            DribblerResponse::unpack(&buffer).unwrap(),
        )
    }
}