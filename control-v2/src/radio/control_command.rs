//!
//! The Control Message is sent to robots over radio to inform them of what actions to take.
//!

#![allow(dead_code)]

use nalgebra::base::*;
use common::packing::{Packable, PackingError};
use kicker_controller::{KickerCommand, KickType, KickTrigger};

use crate::Team;

/// The body{X, Y, W} are multiplied (upon sending) by the VELOCITY_SCALE_FACTOR and divided
/// (upon receiving) to preserve at least 3 decimals of floating point precision.
pub const VELOCITY_SCALE_FACTOR: f32 = 1000.0;

/// The size of a ControlMessage in Bytes as a constant.
/// Note: This is tested in the tests so it can be trusted
pub const CONTROL_MESSAGE_SIZE: usize = 10;

/// The Trigger Mode Kicking
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriggerMode {
    /// Slowly expel the charge in the kicker
    StandDown = 0,
    /// Immediately activate the kicker
    Immediate = 1,
    /// Activate the kicker on the next break beam trip
    OnBreakBeam = 2,
}

impl Into<u8> for TriggerMode {
    fn into(self) -> u8 {
        self as u8
    }
}

impl Into<KickTrigger> for TriggerMode {
    fn into(self) -> KickTrigger {
        match self {
            TriggerMode::StandDown => KickTrigger::Disabled,
            TriggerMode::Immediate => KickTrigger::Immediate,
            TriggerMode::OnBreakBeam => KickTrigger::Breakbeam,
        }
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

impl Into<KickType> for ShootMode {
    fn into(self) -> KickType {
        match self {
            ShootMode::Kick => kicker_controller::KickType::Kick,
            ShootMode::Chip => kicker_controller::KickType::Chip,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// The `mode` the robot should be in
/// 
/// In general, this field should pretty much always be set to Default.  However, I created
/// this field so we can tell the robots to conduct specific testing procedures (or perhaps
/// special moves) based on the flags sent from the base station
pub enum Mode {
    /// Default execution mode.  In default execution mode, the robot continually
    /// runs the normal motion control update loop and should behave as one would expect
    /// our robots to perform
    Default = 0,
    /// Test the IMU on the robot
    ImuTest = 1,
    /// Benchmark the radio's receive functionality
    ReceiveBenchmark = 2,
    /// Benchmark the radio's send functionality
    SendBenchmark = 3,
    /// Program the kicker with kick-on-breakbeam
    ProgramKickOnBreakbeam = 4,
    /// Program the kicker with normal-operation
    ProgramKicker = 5,
    /// Test the kicker
    KickerTest = 6,
    /// Test the FPGA Movement
    FpgaTest = 7,
}

impl Default for Mode {
    fn default() -> Self {
        Self::Default
    }
}

impl From<u8> for Mode {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Default,
            1 => Self::ImuTest,
            2 => Self::ReceiveBenchmark,
            3 => Self::SendBenchmark,
            4 => Self::ProgramKickOnBreakbeam,
            5 => Self::ProgramKicker,
            6 => Self::KickerTest,
            7 => Self::FpgaTest,
            _ => Self::Default,
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
    /// Trigger Mode for the Robot (TODO: Finish Docs)
    pub trigger_mode: TriggerMode,
    /// X Coordinate of the Robot's Body Frame (multiplied by VELOCITY_SCALE_FACTOR
    /// and truncated)
    pub body_x: i16,
    /// Y Coordinate of the Robot's Body Frame (multiplied by VELOCITY_SCALE_FACTOR
    /// and truncated)
    pub body_y: i16,
    /// W Coordinate of the Robot's Body Frame (multiplied by VELOCITY_SCALE_FACTOR
    /// and truncated))
    pub body_w: i16,
    /// Speed of the dribbler (TODO: Determine Units)
    pub dribbler_speed: i8,
    /// Strength of the kicker on kick (TODO: Determine Units)
    pub kick_strength: u8,
    /// Role of This Robot (TODO: Finish Docs)
    pub role: u8,
    /// The mode of the robot
    pub mode: Mode,
}

impl ControlMessage {
    /// Get the velocity (x, y, w) from the control message in a vector
    pub fn get_velocity(&self) -> Vector3<f32> {
        Vector3::new(
            (self.body_x as f32) / VELOCITY_SCALE_FACTOR,
            (self.body_y as f32) / VELOCITY_SCALE_FACTOR,
            (self.body_w as f32) / VELOCITY_SCALE_FACTOR,
        )
    }
}

impl Packable for ControlMessage {
    fn len() -> usize {
        CONTROL_MESSAGE_SIZE
    }

    fn pack(self, buffer: &mut [u8]) -> Result<(), PackingError> {
        if buffer.len() < CONTROL_MESSAGE_SIZE {
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

    fn unpack(data: &[u8]) -> Result<Self, PackingError> {
        if data.len() < CONTROL_MESSAGE_SIZE {
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
        KickerCommand {
            kick_type: self.shoot_mode.into(),
            kick_trigger: self.trigger_mode.into(),
            kick_strength: self.kick_strength as f32 * 255.0 / 15.0,
            charge_allowed: true,
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
    pub mode: Option<Mode>,
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
        self.body_x = Some((body_x * VELOCITY_SCALE_FACTOR) as i16);
        self
    }

    /// Assign the y-direction body velocity for the control message
    pub fn body_y(mut self, body_y: f32) -> Self {
        self.body_y = Some((body_y * VELOCITY_SCALE_FACTOR) as i16);
        self
    }
    
    /// Assign the w-direction body velocity for the control message
    pub fn body_w(mut self, body_w: f32) -> Self {
        self.body_w = Some((body_w * VELOCITY_SCALE_FACTOR) as i16);
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
    pub fn mode(mut self, mode: Mode) -> Self {
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Test that ControlMessageBuilder uses the correct default fields when
    /// they are not provided.
    #[test]
    fn test_empty_control_message_builder() {
        let control_message = ControlMessageBuilder::new().build();

        let expected = ControlMessage {
            team: Team::Blue,
            robot_id: 0,
            shoot_mode: ShootMode::Kick,
            trigger_mode: TriggerMode::StandDown,
            body_x: 0,
            body_y: 0,
            body_w: 0,
            dribbler_speed: 0,
            kick_strength: 0,
            role: 0,
            mode: Mode::default(),
        };

        assert_eq!(expected, control_message);
    }

    /// Test that the ControlMessageBuilder uses the correct fields when they
    /// are provided
    #[test]
    fn test_complete_control_message_builder() {
        let control_message = ControlMessageBuilder::new()
            .team(Team::Yellow)
            .robot_id(3)
            .shoot_mode(ShootMode::Chip)
            .trigger_mode(TriggerMode::OnBreakBeam)
            .body_x(20.0)
            .body_y(10.0)
            .body_w(45.0)
            .dribbler_speed(-5)
            .kick_strength(3)
            .role(1)
            .build();

        let expected = ControlMessage {
            team: Team::Yellow,
            robot_id: 3,
            shoot_mode: ShootMode::Chip,
            trigger_mode: TriggerMode::OnBreakBeam,
            body_x: 20_000,
            body_y: 10_000,
            body_w: 32_767,
            dribbler_speed: -5,
            kick_strength: 3,
            role: 1,
            mode: Mode::default(),
        };

        assert_eq!(expected, control_message);
    }

    /// The Control Message for
    /// ControlMessage {
    ///     team: Yellow (false),
    ///     robot_id: 3,
    ///     shoot_mode: Chip (1),
    ///     trigger_mode: OnBreakBeam (2),
    ///     body_x: 20.0 (20_000),
    ///     body_y: 10.0 (10_000),
    ///     body_w: 45.0 (32_767),
    ///     dribbler_speed: -5,
    ///     role: 1,
    /// }
    ///
    /// is as follows:
    ///     1_0011_1_10 | 00100000 | 01001110 | 00010000 | 00100111 | ...
    ///     ^   ^  ^  ^       ^          ^         ^          ^
    ///     |   |  |  |       |          |         |          |
    /// team-   |  |  |       |          |         |          |
    /// robot_id-  |  |       |          |         |          |
    /// shoot_mode--  |       |          |         |          |
    /// trigger_mode---       |          |         |          |
    /// body_x (lsb)-----------          |         |          |
    /// body_x (msb)----------------------         |          |
    /// body_y (lsb)--------------------------------          |
    /// body_y (msb)-------------------------------------------
    ///
    ///     11111111 | 01111111 | 11111011 | 00000011 | 01_000111
    ///         ^          ^          ^          ^       ^    ^
    ///         |          |          |          |       |    |
    /// body_w (lsb)       |          |          |       |    |
    /// body_w (msb)--------          |          |       |    |
    /// dribbler_speed (2s Comp)-------          |       |    |
    /// kick_strength-----------------------------       |    |
    /// role----------------------------------------------    |
    /// mode---------------------------------------------------
    #[test]
    fn test_pack() {
        let control_message = ControlMessageBuilder::new()
            .team(Team::Yellow)
            .robot_id(3)
            .shoot_mode(ShootMode::Chip)
            .trigger_mode(TriggerMode::OnBreakBeam)
            .body_x(20.0)
            .body_y(10.0)
            .body_w(45.0)
            .dribbler_speed(-5)
            .kick_strength(3)
            .role(1)
            .mode(Mode::FpgaTest)
            .build();

        let mut packed_data = [0u8; CONTROL_MESSAGE_SIZE];
        control_message.pack(&mut packed_data).unwrap();

        assert_eq!(packed_data.len(), CONTROL_MESSAGE_SIZE);
        assert_eq!(packed_data[0], 0b1_0011_1_10);
        assert_eq!(packed_data[1], 0b00100000);
        assert_eq!(packed_data[2], 0b01001110);
        assert_eq!(packed_data[3], 0b00010000);
        assert_eq!(packed_data[4], 0b00100111);
        assert_eq!(packed_data[5], 0b11111111);
        assert_eq!(packed_data[6], 0b01111111);
        assert_eq!(packed_data[7], 0b11111011);
        assert_eq!(packed_data[8], 0b00000011);
        assert_eq!(packed_data[9], 0b01_000111);
    }

    /// The Control Message from:
    ///     1_0011_1_10 | 00100000 | 01001110 | 00010000 | 00100111 | ...
    ///     ^   ^  ^  ^       ^          ^         ^          ^
    ///     |   |  |  |       |          |         |          |
    /// team-   |  |  |       |          |         |          |
    /// robot_id-  |  |       |          |         |          |
    /// shoot_mode--  |       |          |         |          |
    /// trigger_mode---       |          |         |          |
    /// body_x (lsb)-----------          |         |          |
    /// body_x (msb)----------------------         |          |
    /// body_y (lsb)--------------------------------          |
    /// body_y (msb)-------------------------------------------
    ///
    ///     11111111 | 01111111 | 11111011 | 00000011 | 01_000010
    ///         ^          ^          ^          ^       ^    ^
    ///         |          |          |          |       |    |
    /// body_w (lsb)       |          |          |       |    |
    /// body_w (msb)--------          |          |       |    |
    /// dribbler_speed (2s Comp)-------          |       |    |
    /// kick_strength-----------------------------       |    |
    /// role----------------------------------------------    |
    /// unused-------------------------------------------------
    ///
    /// is as follows:
    /// ControlMessage {
    ///     team: Yellow (false),
    ///     robot_id: 3,
    ///     shoot_mode: Chip (1),
    ///     trigger_mode: OnBreakBeam (2),
    ///     body_x: 20.0 (20_000),
    ///     body_y: 10.0 (10_000),
    ///     body_w: 45.0 (32_767),
    ///     dribbler_speed: -5,
    ///     role: 1,
    ///     mode: Mode::ReceiveBenchmark,
    /// }
    #[test]
    fn test_unpack() {
        let data = [
            0b1_0011_1_10,
            0b00100000,
            0b01001110,
            0b00010000,
            0b00100111,
            0b11111111,
            0b01111111,
            0b11111011,
            0b00000011,
            0b01_000010,
        ];

        let control_message = ControlMessage::unpack(&data).unwrap();

        let expected = ControlMessage {
            team: Team::Yellow,
            robot_id: 3,
            shoot_mode: ShootMode::Chip,
            trigger_mode: TriggerMode::OnBreakBeam,
            body_x: 20_000,
            body_y: 10_000,
            body_w: 32_767,
            dribbler_speed: -5,
            kick_strength: 3,
            role: 1,
            mode: Mode::ReceiveBenchmark,
        };

        assert_eq!(expected, control_message);
    }
}