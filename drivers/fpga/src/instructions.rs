// Helper file that contains definitions used in the fpga
// Extracted from the FPGA.cpp driver

#[allow(dead_code, non_camel_case_types)]
#[allow(dead_code, non_camel_case_types)]

/// Instruction to Send to the Robot
#[derive(Clone, Copy)]
pub(crate) enum Instruction {
    EN_MOTORS = 0x30,
    DIS_MOTORS = 0xB0,
    R_ENC_W_VEL = 0x80,
    R_ENC = 0x91,
    R_HALLS = 0x92,
    R_DUTY = 0x93,
    R_HASH_1 = 0x94,
    R_HASH_2 = 0x95,
    CHECK_DRV = 0x96,
}

impl Instruction {
    pub(crate) fn opcode(&self) -> u8 {
        *self as u8
    }
}
