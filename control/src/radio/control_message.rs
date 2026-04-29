//!
//! Control Message Sent from the Base Station to the Robots to command
//! them to move
//!

use kicker_controller::KickerCommand;
use nalgebra::base::*;

use crate::radio::{PackingError, Team};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// The Trigger Mode Kicker
pub enum TriggerMode {
    /// Slowly expel the charge in the Kicker
    StandDown = 0,
    /// Immediately activate the Kicker
    Immediate = 1,
    /// Activate the kicker on the next break beam
    OnBreakBeam = 2,
}

impl Into<u8> for TriggerMode {
    fn into(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// How the robot should kick the ball
pub enum ShootMode {
    /// The robot should kick the ball
    Kick = 0,
    /// The robot should chip the ball
    Chip = 1,
}

impl Into<bool> for ShootMode {
    fn into(self) -> bool {
        match self {
            ShootMode::Kick => false,
            ShootMode::Chip => true,
        }
    }
}

/// The Control Message is Sent from the Base Station to the Robots.
///
/// The Packed Format of this message is as follows:
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// |    7    |    6    |    5    |    4    |    3    |    2    |    1    |    0    |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | team    | robot id                              | shoot_m | trigger_mode      |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | body_x (lsb)                                                                  |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | body_x (msb)                                                                  |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | body_y (lsb)                                                                  |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | body_y (msb)                                                                  |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | body_w (lsb)                                                                  |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | body_w (msb)                                                                  |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | dribbler_speed                                                                |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | kick_strength                                                                 |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | role              | mode                                                      |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
///
/// Size = 80 Bits = 10 Bytes
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlMessage {
    /// Team of the Robot (0: Blue) (1: Yellow)
    pub team: Team,
    /// Id of the Robot
    pub robot_id: u8,
    /// Mode of kicking for the robot
    pub shoot_mode: ShootMode,
    /// Trigger Mode for the Robot
    pub trigger_mode: TriggerMode,
    /// Desired X Velocity of the Robot in mm/s
    pub body_x: i16,
    /// Desired Y velocity of the Robot in mm/s
    pub body_y: i16,
    /// Desired Angular speed of the robot in mrad/s
    pub body_w: i16,
    /// Speed of the dribbler (in percent of being on)
    pub dribbler_speed: i8,
    /// Strength of the kicker on kick
    pub kick_strength: u8,
    /// Role of the robot
    pub role: u8,
    /// The mode of the robot
    pub mode: u8,
}

impl ControlMessage {
    pub const fn velocity_scale_factor() -> f32 {
        1000.0
    }

    pub const fn size() -> usize {
        10
    }

    pub fn get_velocity(&self) -> Vector3<f32> {
        Vector3::new(
            (self.body_x as f32) / Self::velocity_scale_factor(),
            (self.body_y as f32) / Self::velocity_scale_factor(),
            (self.body_w as f32) / Self::velocity_scale_factor(),
        )
    }

    pub fn pack(self, buffer: &mut [u8]) -> Result<(), PackingError> {
        if buffer.len() < Self::size() {
            return Err(PackingError::InvalidBufferSize);
        }

        buffer[0] = ((self.team as u8) & 0b1) << 7
            | (self.robot_id & 0b1111) << 3
            | ((self.shoot_mode as u8) & 0b1) << 2
            | (self.trigger_mode as u8) & 0b11;
        let bytes = self.body_x.to_le_bytes();
        buffer[1] = bytes[0];
        buffer[2] = bytes[1];
        let bytes = self.body_y.to_le_bytes();
        buffer[3] = bytes[0];
        buffer[4] = bytes[1];
        let bytes = self.body_w.to_le_bytes();
        buffer[5] = bytes[0];
        buffer[6] = bytes[1];
        buffer[7] = self.dribbler_speed.to_le_bytes()[0];
        buffer[8] = self.kick_strength;
        buffer[9] = (self.role & 0b11) << 6 | (self.mode as u8);
        Ok(())
    }

    pub fn unpack(data: &[u8]) -> Result<Self, PackingError> {
        if data.len() < Self::size() {
            return Err(PackingError::InvalidBufferSize);
        }

        Ok(Self {
            team: if data[0] & (0b1 << 7) == 0 {
                Team::Blue
            } else {
                Team::Yellow
            },
            robot_id: (data[0] & (0b1111 << 3)) >> 3,
            shoot_mode: if data[0] & (0b1 << 2) != 0 {
                ShootMode::Chip
            } else {
                ShootMode::Kick
            },
            trigger_mode: match data[0] & 0b11 {
                1 => TriggerMode::Immediate,
                2 => TriggerMode::OnBreakBeam,
                _ => TriggerMode::StandDown,
            },
            body_x: i16::from_le_bytes(data[1..=2].try_into().unwrap()),
            body_y: i16::from_le_bytes(data[3..=4].try_into().unwrap()),
            body_w: i16::from_le_bytes(data[5..=6].try_into().unwrap()),
            dribbler_speed: i8::from_le_bytes([data[7]]),
            kick_strength: data[8],
            role: (data[9] & (0b11 << 6)) >> 6,
            mode: (data[9] & 0b0011_1111).into(),
        })
    }
}

impl Into<KickerCommand> for ControlMessage {
    fn into(self) -> KickerCommand {
        let kick_type = match self.shoot_mode {
            ShootMode::Kick => kicker_controller::KickType::Kick,
            ShootMode::Chip => kicker_controller::KickType::Chip,
        };
        let kick_trigger = match self.trigger_mode {
            TriggerMode::StandDown => kicker_controller::KickTrigger::Disabled,
            TriggerMode::Immediate => kicker_controller::KickTrigger::Immediate,
            TriggerMode::OnBreakBeam => kicker_controller::KickTrigger::Breakbeam,
        };
        KickerCommand {
            kick_type,
            kick_trigger,
            kick_strength: self.kick_strength,
            charge_allowed: self.trigger_mode != TriggerMode::StandDown,
        }
    }
}

/// Builder for a Control Message
pub struct ControlMessageBuilder {
    /// The message's team
    pub team: Option<Team>,
    /// The message's robot_id
    pub robot_id: Option<u8>,
    /// The message's shoot mode
    pub shoot_mode: Option<ShootMode>,
    /// The message's trigger mode
    pub trigger_mode: Option<TriggerMode>,
    /// The message's body velocity in the x direction
    pub body_x: Option<i16>,
    /// The message's body velocity in the y direction
    pub body_y: Option<i16>,
    /// The message's body velocity in the w direction
    pub body_w: Option<i16>,
    /// The speed of the dribbler
    pub dribbler_speed: Option<i8>,
    /// The strength of the kicker (used to charge the kicker)
    pub kick_strength: Option<u8>,
    /// The role the robot is playing
    pub role: Option<u8>,
    /// The mode the robot is in
    pub mode: Option<u8>,
}

impl ControlMessageBuilder {
    /// Start building a new control message
    pub fn new() -> Self {
        Self {
            team: None,
            robot_id: None,
            shoot_mode: None,
            trigger_mode: None,
            body_x: None,
            body_y: None,
            body_w: None,
            dribbler_speed: None,
            kick_strength: None,
            role: None,
            mode: None,
        }
    }

    /// Assign the team for the control message
    pub fn team(mut self, team: Team) -> Self {
        self.team = Some(team);
        self
    }

    /// Assign the robot_id for the control message
    pub fn robot_id(mut self, robot_id: u8) -> Self {
        self.robot_id = Some(robot_id);
        self
    }

    /// Assign the shoot mode for the control message
    pub fn shoot_mode(mut self, shoot_mode: ShootMode) -> Self {
        self.shoot_mode = Some(shoot_mode);
        self
    }

    /// Assign the trigger mode for the control message
    pub fn trigger_mode(mut self, trigger_mode: TriggerMode) -> Self {
        self.trigger_mode = Some(trigger_mode);
        self
    }

    /// Assign the x-direction body velocity for the control message
    pub fn body_x(mut self, body_x: f32) -> Self {
        self.body_x = Some((body_x * ControlMessage::velocity_scale_factor()) as i16);
        self
    }

    /// Assign the y-direction body velocity for the control message
    pub fn body_y(mut self, body_y: f32) -> Self {
        self.body_y = Some((body_y * ControlMessage::velocity_scale_factor()) as i16);
        self
    }

    /// Assign the w-direction body velocity for the control message
    pub fn body_w(mut self, body_w: f32) -> Self {
        self.body_w = Some((body_w * ControlMessage::velocity_scale_factor()) as i16);
        self
    }

    /// Assign the dribbler velocity for the control message
    pub fn dribbler_speed(mut self, dribbler_speed: i8) -> Self {
        self.dribbler_speed = Some(dribbler_speed);
        self
    }

    /// Assign the kick strength for the control message
    pub fn kick_strength(mut self, kick_strength: u8) -> Self {
        self.kick_strength = Some(kick_strength);
        self
    }

    /// Assign the role for the control message
    pub fn role(mut self, role: u8) -> Self {
        self.role = Some(role);
        self
    }

    /// Assign a specific mode for the robot in the control message
    pub fn mode(mut self, mode: u8) -> Self {
        self.mode = Some(mode);
        self
    }

    /// Build the control message from the assigned fields.
    pub fn build(self) -> ControlMessage {
        let team = match self.team {
            Some(team) => team,
            None => Team::Blue,
        };

        let robot_id = match self.robot_id {
            Some(robot_id) => robot_id,
            None => 0,
        };

        let shoot_mode = match self.shoot_mode {
            Some(shoot_mode) => shoot_mode.into(),
            None => ShootMode::Kick,
        };

        let trigger_mode = match self.trigger_mode {
            Some(trigger_mode) => trigger_mode.into(),
            None => TriggerMode::StandDown,
        };

        let body_x = match self.body_x {
            Some(body_x) => body_x,
            None => 0,
        };

        let body_y = match self.body_y {
            Some(body_y) => body_y,
            None => 0,
        };

        let body_w = match self.body_w {
            Some(body_w) => body_w,
            None => 0,
        };

        let dribbler_speed = match self.dribbler_speed {
            Some(dribbler_speed) => dribbler_speed,
            None => 0,
        };

        let kick_strength = match self.kick_strength {
            Some(kick_strength) => kick_strength,
            None => 0,
        };

        let role = match self.role {
            Some(role) => role,
            None => 0,
        };

        let mode = self.mode.unwrap_or_default();

        ControlMessage {
            team,
            robot_id,
            shoot_mode,
            trigger_mode,
            body_x,
            body_y,
            body_w,
            dribbler_speed,
            kick_strength,
            role,
            mode,
        }
    }
}
