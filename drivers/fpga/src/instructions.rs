// Helper file that contains definitions used in the fpga
// Extracted from the FPGA.cpp driver

#[allow(dead_code, non_camel_case_types)]

use core::fmt::Debug;

/// Instructions to be sent to the FPGA for performing various operations.
/// 
/// In general, these instructions should never be used alone and instead, the
/// methods present in the library files should be used to generate these instructions.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Instruction {
    EnableMotors = 0x30,
    DisableMotors = 0xB0,
    ReadEncodersWriteVelocity = 0x80,
    ReadEncoders = 0x91,
    ReadHalls = 0x92,
    ReadDuties = 0x93,
    ReadHash1 = 0x94,
    ReadHash2 = 0x95,
    CheckDrive = 0x96,
}

impl Instruction {
    pub(crate) fn opcode(&self) -> u8 {
        *self as u8
    }
}