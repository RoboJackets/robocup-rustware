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

/// Kicker packet definition
/// |---------------------------------------|
/// | (7) | (6) (5) | (4) | (3) (2) (1) (0) |
/// |---------------------------------------| 
///
/// Bits 0-3
///  Power of kick
///      0 - 15
///      0 is min power
///      15 is max power
///
/// Bits 4
///  Charge Allowed
///      Whether the kicker can start charging the caps
///      1 Charge allowed
///      0 Charge not allowed
///
/// Bits 5-6
///  Type of kick activation
///      0b01 Kick on breakbeam
///      0b10 Kick immediately
///      0b11 Cancel all current kick commands
///
/// Bits 7
///  Type of kick
///      1 Chip
///      0 Kick
pub(crate) enum KickerCommand {
    NormalKick = 1 << 7,
    ChipKick = 0 << 7,
    KickImmediate = 1 << 6,
    KickOnBreakbeam = 1 << 5,
    CancelKick = 0b11 << 5,
    ChargeAllowed = 1 << 4,
}