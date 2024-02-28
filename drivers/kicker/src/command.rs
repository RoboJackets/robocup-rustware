#![allow(unused)]

/// The kick type for the kicker
#[derive(Debug, Clone, Copy)]
pub enum KickType {
    Chip = 1 << 7,
    Kick = 0 << 7,
}

impl From<bool> for KickType {
    fn from(value: bool) -> Self {
        match value {
            true => KickType::Chip,
            false => KickType::Kick,
        }
    }
}

/// The activation type of the kicker
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum KickActivation {
    Charge = 0 << 5,
    Immediate = 1 << 6,
    Breakbeam = 1 << 5,
    Cancel = 0b11 << 5,
}

impl From<u8> for KickActivation {
    fn from(value: u8) -> Self {
        match value {
            0 => KickActivation::Charge,
            1 => KickActivation::Immediate,
            2 => KickActivation::Breakbeam,
            _ => KickActivation::Cancel,
        }
    }
}

/// Builder for a command to be sent to the kicker
/// If not in cancel, the kicker is allowed to charge
pub(crate) struct KickCommand {
    kick_type: Option<KickType>,
    activation: Option<KickActivation>,
    kick_power: Option<u8>,
}

impl KickCommand {
    /// Create a New Empty Kick Command
    pub(crate) fn new() -> Self {
        Self {
            kick_type: None,
            activation: None,
            kick_power: None,
        }
    }

    /// Set the kick type of the kick command
    pub(crate) fn kick_type(mut self, kick_type: KickType) -> Self {
        self.kick_type = Some(kick_type);
        self
    }

    /// Set the activation type for the kick command
    pub(crate) fn activation(mut self, activation: KickActivation) -> Self {
        self.activation = Some(activation);
        self
    }

    /// Set the power for the next kick
    pub(crate) fn kick_power(mut self, kick_power: u8) -> Self {
        self.kick_power = Some(kick_power);
        self
    }

    /// Kicker Packet Definition
    ///|---------------------------------------|
    ///| (7) | (6) (5) | (4) | (3) (2) (1) (0) |
    ///|---------------------------------------|
    ///
    /// Bits 0-3 - Kick Power - 0 (min) -> 15 (max)
    ///
    /// Bit 4 - Charge Allowed
    ///
    /// Bits 5-6 - Kick activation
    ///
    /// Bit 7 - Kick Type
    ///
    pub(crate) fn build(self) -> [u8; 1] {
        let mut command = 0x00;

        if let Some(kick_type) = self.kick_type {
            command |= kick_type as u8;
        } else {
            command |= KickType::Kick as u8;
        }

        if let Some(activation) = self.activation {
            // Set Charge Allowed
            if activation != KickActivation::Cancel {
                command |= 1 << 4;
            }

            // Set Activation
            command |= activation as u8;
        } else {
            command |= KickActivation::Cancel as u8;
        }

        if let Some(kick_power) = self.kick_power {
            command |= (((kick_power as f32) / 255.0) * 15.0) as u8 & 0x0F;
        }

        [command]
    }
}
