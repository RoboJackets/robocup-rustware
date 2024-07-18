//!
//! Robot Constants and Configuration Information
//! 

#[cfg(any(
    not(any(
        feature = "robot-0",
        feature = "robot-1",
        feature = "robot-2",
        feature = "robot-3",
        feature = "robot-4",
        feature = "robot-5",
    )),
    feature = "robot-0"
))]
pub mod robot_config {
    use robojackets_robocup_rtp::ROBOT_RADIO_ADDRESSES;

    /// The Robot ID for Robot 0
    pub const ROBOT_ID: u8 = 0;
    /// The Error Correction Equation (form y = mx + b) for each
    /// wheel of this robot
    pub const CORRECTION_FACTORS: [(f32, f32); 4] = [
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
    ];
    /// The Radio address of this robot
    pub const RADIO_ADDRESS: [u8; 5] = ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize];
}

#[cfg(feature = "robot-1")]
pub mod robot_config {
    use robojackets_robocup_rtp::ROBOT_RADIO_ADDRESSES;

    /// The Robot ID for Robot 1
    pub const ROBOT_ID: u8 = 1;
    /// The Error Correction Equation (form y = mx + b) for each
    /// wheel of this robot
    pub const CORRECTION_FACTORS: [(f32, f32); 4] = [
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
    ];
    /// The Radio address of this robot
    pub const RADIO_ADDRESS: [u8; 5] = ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize];
}

#[cfg(feature = "robot-2")]
pub mod robot_config {
    use robojackets_robocup_rtp::ROBOT_RADIO_ADDRESSES;

    /// The Robot ID for Robot 2
    pub const ROBOT_ID: u8 = 2;
    /// The Error Correction Equation (form y = mx + b) for each
    /// wheel of this robot
    pub const CORRECTION_FACTORS: [(f32, f32); 4] = [
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
    ];
    /// The Radio address of this robot
    pub const RADIO_ADDRESS: [u8; 5] = ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize];
}

#[cfg(feature = "robot-3")]
pub mod robot_config {
    use robojackets_robocup_rtp::ROBOT_RADIO_ADDRESSES;

    /// The Robot ID for Robot 3
    pub const ROBOT_ID: u8 = 3;
    /// The Error Correction Equation (form y = mx + b) for each
    /// wheel of this robot
    pub const CORRECTION_FACTORS: [(f32, f32); 4] = [
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
    ];
    /// The Radio address of this robot
    pub const RADIO_ADDRESS: [u8; 5] = ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize];
}

#[cfg(feature = "robot-4")]
pub mod robot_config {
    use robojackets_robocup_rtp::ROBOT_RADIO_ADDRESSES;

    /// The Robot ID for Robot 4
    pub const ROBOT_ID: u8 = 4;
    /// The Error Correction Equation (form y = mx + b) for each
    /// wheel of this robot
    pub const CORRECTION_FACTORS: [(f32, f32); 4] = [
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
    ];
    /// The Radio address of this robot
    pub const RADIO_ADDRESS: [u8; 5] = ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize];
}

#[cfg(feature = "robot-5")]
pub mod robot_config {
    use robojackets_robocup_rtp::ROBOT_RADIO_ADDRESSES;
    
    /// The Robot ID for Robot 5
    pub const ROBOT_ID: u8 = 5;
    /// The Error Correction Equation (form y = mx + b) for each
    /// wheel of this robot
    pub const CORRECTION_FACTORS: [(f32, f32); 4] = [
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
    ];
    /// The Radio address of this robot
    pub const RADIO_ADDRESS: [u8; 5] = ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize];
}