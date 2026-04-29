//!
//! Robot Status Message Sent from the Robots to the Base Station
//! relaying their statuses
//!

use crate::radio::Team;

use super::PackingError;

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
    /// Team of the Robot (0: Blue) (1: Yellow)
    pub team: Team,
    /// Id of the Robot
    pub robot_id: u8,
    /// True if the robot currently has ball sense
    pub ball_sense_status: bool,
    /// Status of the kicker
    pub kick_status: bool,
    /// Health of the kicker
    pub kick_healthy: bool,
    /// Voltage measured by the ADC of the Microcontroller
    pub battery_voltage: u8,
    /// Errors experienced by the motor
    pub motor_errors: u8,
    /// Status of the FPGA
    pub fpga_status: bool,
}

impl RobotStatusMessage {
    pub const fn size() -> usize {
        3
    }

    pub fn pack(self, buffer: &mut [u8]) -> Result<(), PackingError> {
        if buffer.len() < Self::size() {
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
