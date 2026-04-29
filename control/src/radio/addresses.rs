//!
//! Addresses for each of the robots in the system.
//!
//! These are used to uniquely identify the base station and each robot
//!

/// Addresses for the base station
pub const BASE_STATION_ADDRESSES: [[u8; 5]; 2] = [
    [0xE7, 0xE7, 0xE7, 0xE7, 0xE7], // Base Station Address for the Blue Base Station
    [0xA4, 0xA4, 0xA4, 0xA4, 0xA4], // Base Station Address for the Yellow Base Station
];

/// Addresses for each of the robots
pub const ROBOT_ADDRESSES: [[[u8; 5]; 6]; 2] = [
    [
        [0xC3, 0xC3, 0xC3, 0xC3, 0xC1], // Robot 1 Address for the Blue Base Station
        [0xC3, 0xC3, 0xC3, 0xC3, 0xC2], // Robot 2 Address for the Blue Base Station
        [0xC3, 0xC3, 0xC3, 0xC3, 0xC3], // Robot 3 Address for the Blue Base Station
        [0xC3, 0xC3, 0xC3, 0xC3, 0xC4], // Robot 4 Address for the Blue Base Station
        [0xC3, 0xC3, 0xC3, 0xC3, 0xC5], // Robot 5 Address for the Blue Base Station
        [0xC3, 0xC3, 0xC3, 0xC3, 0xC6], // Robot 6 Address for the Blue Base Station
    ],
    [
        [0xD5, 0xD5, 0xD5, 0xD5, 0xD1], // Robot 1 Address for the Yellow Base Station
        [0xD5, 0xD5, 0xD5, 0xD5, 0xD2], // Robot 2 Address for the Yellow Base Station
        [0xD5, 0xD5, 0xD5, 0xD5, 0xD3], // Robot 3 Address for the Yellow Base Station
        [0xD5, 0xD5, 0xD5, 0xD5, 0xD4], // Robot 4 Address for the Yellow Base Station
        [0xD5, 0xD5, 0xD5, 0xD5, 0xD5], // Robot 5 Address for the Yellow Base Station
        [0xD5, 0xD5, 0xD5, 0xD5, 0xD6], // Robot 6 Address for the Yellow Base Station
    ],
];
