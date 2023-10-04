//!
//! List of Instructions / Commands for the Kicker Board
//! 

/// Enum containing the various commands to be sent over SPI to the Kicker Board
pub(crate) enum Command {
    /// Command Issued to the Kicker Board to Indicate a Write is Coming
    Write = 0b0100_1100,
    /// TODO: 0b01001000
    WriteHighByte = 0b0100_1000,
    /// TODO:
    WriteLowByte = 0x0100_0000,
    /// TODO: 0b0010_1000
    ReadHighByte = 0b0010_1000,
    /// TODO:
    ReadLowByte = 0b0010_0000,
    /// TODO: 
    WriteHighFlashByte = 0b0110_1000,
    /// TODO:
    WriteLowFlashByte = 0b0110_0000,
}