//!
//! The Robot Status Message is sent from the robots to the base station and finally to the computer
//!

#![allow(dead_code)]

use common::packing::{Packable, PackingError};

use crate::Team;

/// The size of a RobotStatusMessage in Bytes as a constant.
/// Note: This is tested in the tests so it can be trusted
pub const ROBOT_STATUS_SIZE: usize = 3;

/// The Robot Status Message is sent back from the robot's whenever they receive communication
/// to let software know that they are doing good.
///
/// The RobotStatusMessage has the following format:
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// |    7    |    6    |    5    |    4    |    3    |    2    |    1    |    0    |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | team    | robot_id                              | b_sense | k_status| k_health|
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | battery_voltage                                                               |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
/// | motor_errors                                    | fpga_s  | unused            |
/// +---------+---------+---------+---------+---------+---------+---------+---------+
///
/// Size = 3 Bytes
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RobotStatusMessage {
    /// Team of the RObot (0: Blue) (1: Yellow)
    pub team: Team,
    /// Id of the Robot
    pub robot_id: u8,
    /// True if the robot currently has ball sense
    pub ball_sense_status: bool,
    /// Status of the kicker (TODO: Confirm this)
    pub kick_status: bool,
    /// Health of the kicker
    pub kick_healthy: bool,
    /// Voltage measured by the ADC of the Microcontroller
    pub battery_voltage: u8,
    /// Errors experienced by the motor (TODO: Doc this better)
    pub motor_errors: u8,
    /// Status of the FPGA
    pub fpga_status: bool,
}

impl Packable for RobotStatusMessage {
    fn len() -> usize {
       ROBOT_STATUS_SIZE 
    }

    fn pack(self, buffer: &mut [u8]) -> Result<(), PackingError> {
        if buffer.len() < ROBOT_STATUS_SIZE {
            return Err(PackingError::InvalidBufferSize);
        }

        buffer[0] = (self.team as u8) << 7
            | ((self.robot_id) & 0b1111) << 3
            | (self.ball_sense_status as u8) << 2
            | (self.kick_status as u8) << 1
            | self.kick_healthy as u8;
        buffer[1] = self.battery_voltage;
        buffer[2] = (self.motor_errors & 0b11111) << 3 | (self.fpga_status as u8) << 2;
        Ok(())
    }

    fn unpack(data: &[u8]) -> Result<Self, PackingError> {
        if data.len() < ROBOT_STATUS_SIZE {
            return Err(PackingError::InvalidBufferSize);
        }

        Ok(Self {
            team: if data[0] & (0b1 << 7) == 0 {
                Team::Blue
            } else {
                Team::Yellow
            },
            robot_id: (data[0] & (0b1111 << 3)) >> 3,
            ball_sense_status: data[0] & (0b1 << 2) != 0,
            kick_status: data[0] & (0b1 << 1) != 0,
            kick_healthy: data[0] & 0b1 != 0,
            battery_voltage: data[1],
            motor_errors: (data[2] & (0b11111 << 3)) >> 3,
            fpga_status: data[2] & (0b1 << 2) != 0,
        })
    }
}

/// Builder helper to create a robot status message
pub struct RobotStatusMessageBuilder {
    /// The team of the robot status message
    pub team: Option<Team>,
    /// The robot id of the robot status message
    pub robot_id: Option<u8>,
    /// Whether or not the robot status message has ball sense
    pub ball_sense_status: Option<bool>,
    /// Whether or not the robot status message is kicking
    pub kick_status: Option<bool>,
    /// Whether or not the robot status message has a healthy kicker
    pub kick_healthy: Option<bool>,
    /// The battery voltage of the robot status message
    pub battery_voltage: Option<u8>,
    /// Any errors associated with the robot status message
    pub motor_errors: Option<u8>,
    /// The status of the fpga in the robot status message
    pub fpga_status: Option<bool>,
}

impl RobotStatusMessageBuilder {
    /// Instantiate a new RobotStatusMessageBuilder to allow for the creation of
    /// a new RobotStatusMessage
    pub fn new() -> Self {
        Self {
            team: None,
            robot_id: None,
            ball_sense_status: None,
            kick_status: None,
            kick_healthy: None,
            battery_voltage: None,
            motor_errors: None,
            fpga_status: None,
        }
    }

    /// Assign the team for the robot status message
    pub fn team(mut self, team: Team) -> Self {
        self.team = Some(team);
        self
    }

    /// Assign the robot id for the robot status message
    pub fn robot_id(mut self, robot_id: u8) -> Self {
        self.robot_id = Some(robot_id);
        self
    }

    /// Assign the ball sense status for the robot status message
    pub fn ball_sense_status(mut self, ball_sense_status: bool) -> Self {
        self.ball_sense_status = Some(ball_sense_status);
        self
    }

    /// Assign the kick status for the robot status message
    pub fn kick_status(mut self, kick_status: bool) -> Self {
        self.kick_status = Some(kick_status);
        self
    }

    /// Assign whether the kicker is healthy for the robot status message
    pub fn kick_healthy(mut self, kick_healthy: bool) -> Self {
        self.kick_healthy = Some(kick_healthy);
        self
    }

    /// Assign the battery voltage for the robot status message
    pub fn battery_voltage(mut self, battery_voltage: u8) -> Self {
        self.battery_voltage = Some(battery_voltage);
        self
    }

    /// Assign the motor errors for the robot status message
    pub fn motor_errors(mut self, motor_errors: u8) -> Self {
        self.motor_errors = Some(motor_errors);
        self
    }

    /// Assign the fpga status for the robot status message
    pub fn fpga_status(mut self, fpga_status: bool) -> Self {
        self.fpga_status = Some(fpga_status);
        self
    }

    /// Build a new RobotStatusMessage from the assigned fields on the builder
    pub fn build(self) -> RobotStatusMessage {
        let team = match self.team {
            Some(team) => team,
            None => Team::Blue,
        };

        let robot_id = match self.robot_id {
            Some(robot_id) => robot_id,
            None => 0,
        };

        let ball_sense_status = match self.ball_sense_status {
            Some(ball_sense_status) => ball_sense_status,
            None => false,
        };

        let kick_status = match self.kick_status {
            Some(kick_status) => kick_status,
            None => false,
        };

        let kick_healthy = match self.kick_healthy {
            Some(kick_healthy) => kick_healthy,
            None => false,
        };

        let battery_voltage = match self.battery_voltage {
            Some(battery_voltage) => battery_voltage,
            None => 0,
        };

        let motor_errors = match self.motor_errors {
            Some(motor_errors) => motor_errors,
            None => 0,
        };

        let fpga_status = match self.fpga_status {
            Some(fpga_status) => fpga_status,
            None => false,
        };

        RobotStatusMessage {
            team,
            robot_id,
            ball_sense_status,
            kick_status,
            kick_healthy,
            battery_voltage,
            motor_errors,
            fpga_status,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test that the RobotStatusMessageBuilder uses the correct default fields when
    /// they are not provided.
    #[test]
    fn test_empty_robot_status_message_builder() {
        let robot_status = RobotStatusMessageBuilder::new().build();

        let expected = RobotStatusMessage {
            team: Team::Blue,
            robot_id: 0.into(),
            ball_sense_status: false,
            kick_status: false,
            kick_healthy: false,
            battery_voltage: 0,
            motor_errors: 0,
            fpga_status: false,
        };

        assert_eq!(expected, robot_status);
    }

    /// Test that the RobotStatusMessageBuilder uses the filled in fields to create
    /// a RobotStatusMessage.
    #[test]
    fn test_complete_robot_status_message_builder() {
        let robot_status = RobotStatusMessageBuilder::new()
            .team(Team::Yellow)
            .robot_id(1)
            .ball_sense_status(true)
            .kick_status(true)
            .kick_healthy(true)
            .battery_voltage(10)
            .motor_errors(2)
            .fpga_status(true)
            .build();

        let expected: RobotStatusMessage = RobotStatusMessage {
            team: Team::Yellow,
            robot_id: 1.into(),
            ball_sense_status: true,
            kick_status: true,
            kick_healthy: true,
            battery_voltage: 10,
            motor_errors: 2,
            fpga_status: true,
        };

        assert_eq!(expected, robot_status);
    }

    /// The Robot Status for
    /// RobotStatusMessage {
    ///     team: Yellow (true),
    ///     robot_id: 1,
    ///     ball_sense_status: true,
    ///     kick_status: true,
    ///     kick_healthy: false,
    ///     battery_voltage: 10,
    ///     motor_errors: 0,
    ///     fpga_status: true,
    /// }
    ///
    /// is as follows:
    ///     1_0001_1_1_0 | 00001010 | 00000_1_00
    ///     ^   ^  ^ ^ ^       ^        ^   ^  ^
    ///     |   |  | | |       |        |   |  |
    /// team-   |  | | |       |        |   |  |
    /// robot_id-  | | |       |        |   |  |
    /// ball_sense-- | |       |        |   |  |
    /// kick_status--- |       |        |   |  |
    /// kick_healthy----       |        |   |  |
    /// battery_voltage---------        |   |  |
    /// motor_errors---------------------   |  |
    /// fpga_status--------------------------  |
    /// unused----------------------------------
    ///
    #[test]
    fn test_pack() {
        let robot_status = RobotStatusMessageBuilder::new()
            .team(Team::Yellow)
            .robot_id(1)
            .ball_sense_status(true)
            .kick_status(true)
            .battery_voltage(10)
            .motor_errors(0)
            .fpga_status(true)
            .build();

        let mut packed_data = [0u8; ROBOT_STATUS_SIZE];
        robot_status.pack(&mut packed_data).unwrap();

        assert_eq!(packed_data.len(), ROBOT_STATUS_SIZE);
        assert_eq!(packed_data[0], 0b1_0001_1_1_0);
        assert_eq!(packed_data[1], 0b00001010);
        assert_eq!(packed_data[2], 0b00000_1_00);
    }

    /// The Robot Status for the slice:
    ///     1_0001_1_1_0 | 00001010 | 00000_1_00
    ///     ^   ^  ^ ^ ^       ^        ^   ^  ^
    ///     |   |  | | |       |        |   |  |
    /// team-   |  | | |       |        |   |  |
    /// robot_id-  | | |       |        |   |  |
    /// ball_sense-- | |       |        |   |  |
    /// kick_status--- |       |        |   |  |
    /// kick_healthy----       |        |   |  |
    /// battery_voltage---------        |   |  |
    /// motor_errors---------------------   |  |
    /// fpga_status--------------------------  |
    /// unused----------------------------------
    ///
    /// is as follows:
    /// RobotStatusMessage {
    ///     team: Yellow (true),
    ///     robot_id: 1,
    ///     ball_sense_status: true,
    ///     kick_status: true,
    ///     kick_healthy: false,
    ///     battery_voltage: 10,
    ///     fpga_status: true,
    /// }
    #[test]
    fn test_unpack() {
        let status_slice: [u8; 3] = [0b1_0001_1_1_0, 0b00001010, 0b00000_1_00];
        let robot_status = RobotStatusMessage::unpack(&status_slice).unwrap();

        let expected = RobotStatusMessage {
            team: Team::Yellow,
            robot_id: 1,
            ball_sense_status: true,
            kick_status: true,
            kick_healthy: false,
            battery_voltage: 10,
            fpga_status: true,
            motor_errors: 0,
        };

        assert_eq!(expected, robot_status);
    }
}