#![allow(unused)]

/// The kick type for the kicker
#[derive(Debug, Clone, Copy)]
pub enum KickType {
    Chip = 1 << 7,
    Kick = 0 << 7,
}

/// The activation type of the kicker
#[derive(Debug, Clone, Copy)]
pub(crate) enum KickActivation {
    Immediate = 1 << 6,
    Breakbeam = 1 << 5,
    Cancel = 0b11 << 5,
}

/// Builder for a command to be sent to the kicker
pub(crate) struct KickCommand {
    kick_type: Option<KickType>,
    activation: Option<KickActivation>,
    charge_allowed: Option<bool>,
    kick_power: Option<u8>,
}

impl KickCommand {
    /// Create a New Empty Kick Command
    pub(crate) fn new() -> Self {
        Self {
            kick_type: None,
            activation: None,
            charge_allowed: None,
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

    /// Set whether the kicker is allowed to begin charging
    pub(crate) fn charge_allowed(mut self, charge_allowed: bool) -> Self {
        self.charge_allowed = Some(charge_allowed);
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
            command |= activation as u8;
        } else {
            command |= KickActivation::Cancel as u8;
        }

        if let Some(charge_allowed) = self.charge_allowed { 
            command |= (charge_allowed as u8) << 4;
        }

        if let Some(kick_power) = self.kick_power {
            command |= (((kick_power as f32) / 255.0) * 15.0) as u8 & 0x0F;
        }

        [command]
    }
}
