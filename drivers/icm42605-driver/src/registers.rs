//!
//! Register definitions for the various registers of the ICM42605 IMU.
//!

/// The register number of the register containing the high byte of the
/// x direction acceleration
pub(crate) const ACCEL_DATA_X1: u8 = 0x1F;
/// The register number of the register containing the low byte of the
/// x direction acceleration
pub(crate) const ACCEL_DATA_X0: u8 = 0x20;
/// The register number of the register containing the high byte of the
/// y direction acceleration
pub(crate) const ACCEL_DATA_Y1: u8 = 0x21;
/// The register number of the register containing the low byte of the
/// y direction acceleration
pub(crate) const ACCEL_DATA_Y0: u8 = 0x22;
/// The register number of the register containing the high byte of the
/// z direction velocity
pub(crate) const GYRO_DATA_Z1: u8 = 0x29;
/// The register number of the register containing the low byte of the
/// z direction velocity
pub(crate) const GYRO_DATA_Z0: u8 = 0x2A;
/// The expected return value from checking the device identification from
/// the IMU.
pub(crate) const WHO_AM_I: u8 = 0x75;

#[allow(missing_docs)]
pub mod flags {
    use bitflags::bitflags;

    bitflags! {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        /// A bitflag to enable the selection of the various IMU register
        /// banks
        pub struct BankSelect: u8 {
            const BANK_0 = 0;
            const BANK_1 = 1;
            const BANK_2 = 2;
            const BANK_3 = 3;
            const BANK_4 = 4;
        }

        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        /// A bitfag containing the device configuration options for the IMU.
        pub struct DeviceConfig: u8 {
            const SPI_MODE_0_AND_3 = 0b0 << 4;
            const SPI_MODE_1_AND_2 = 0b1 << 4;

            const SOFT_RESET_CONFIG = 0b1;
        }

        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        /// A bitflag containing the power management options for the IMU
        pub struct PowerManagement: u8 {
            const TEMP_DISABLE = 0b1 << 5;

            const IDLE = 0b1 << 4;

            const GYRO_OFF = 0b00 << 2;
            const GYRO_STANDBY = 0b01 << 2;
            const GYRO_LOW_NOISE = 0b11 << 2;

            const ACCEL_OFF = 0b00;
            const ACCEL_LOW_POWER = 0b10;
            const ACCEL_LOW_NOISE = 0b11;
        }

        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        /// A bitflag containing the gyrometer accuracy options for the IMU
        pub struct GyroConfig: u8 {
            const GYRO_FS_2000 = 0b000 << 5;
            const GYRO_FS_1000 = 0b001 << 5;
            const GYRO_FS_500 = 0b010 << 5;
            const GYRO_FS_250 = 0b011 << 5;
            const GYRO_FS_125 = 0b100 << 5;
            const GYRO_FS_62_5 = 0b101 << 5;
            const GYRO_FS_31_25 = 0b110 << 5;
            const GYRO_FS_15_625 = 0b111 << 5;

            const GYRO_ODR_8kHz = 0b0011;
            const GYRO_ODR_4kHz = 0b0100;
            const GYRO_ODR_2kHz = 0b0101;
            const GYRO_ODR_1kHz = 0b0110;
            const GYRO_ODR_200Hz = 0b0111;
            const GYRO_ODR_100Hz = 0b1000;
            const GYRO_ODR_50Hz = 0b1001;
            const GYRO_ODR_25Hz = 0b1010;
            const GYRO_ODR_12_5Hz = 0b1011;
            const GYRO_ODR_500Hz = 0b1111;
        }

        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        /// A bitflag containig the accelerometer accuracy options for the IMU
        pub struct AccelConfig: u8 {
            const ACCEL_FS_16 = 0b000 << 5;
            const ACCEL_FS_8 = 0b001 << 5;
            const ACCEL_FS_4 = 0b010 << 5;
            const ACCEL_FS_2 = 0b011 << 5;

            const ACCEL_ODR_8kHz = 0b0011;
            const ACCEL_ODR_4kHz = 0b0100;
            const ACCEL_ODR_2kHz = 0b0101;
            const ACCEL_ODR_1kHz = 0b0110;
            const ACCEL_ODR_200Hz = 0b0111;
            const ACCEL_ODR_100Hz = 0b1000;
            const ACCEL_ODR_50Hz = 0b1001;
            const ACCEL_ODR_25Hz = 0b1010;
            const ACCEL_ODR_12_5Hz = 0b1011;
            const ACCEL_ODR_6_25Hz = 0b1100;
            const ACCEL_ODR_3_125Hz = 0b1101;
            const ACCEL_ODR_1_5625Hz = 0b1110;
            const ACCEL_ODR_500Hz = 0b1111;
        }
    }
}

pub use flags::*;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
/// The various register banks in the IMU
pub enum Bank {
    /// The first bank (i.e. the default bank)
    #[default]
    Bank0,
    /// The second register bank
    Bank1,
    /// The third register bank
    Bank2,
    /// The fourth register bank
    Bank3,
    /// The fifth register bank
    Bank4,
}

impl Bank {
    /// Convert the register bank enum to an ordinal u8.
    pub fn value(self) -> u8 {
        match self {
            Bank::Bank0 => 0,
            Bank::Bank1 => 1,
            Bank::Bank2 => 2,
            Bank::Bank3 => 3,
            Bank::Bank4 => 4,
        }
    }
}

#[allow(dead_code)]
impl BankSelect {
    /// The register address of the bank select
    pub const ADDR: u8 = 0x76;
    /// The bank the bank select register is in
    pub const BANK: Bank = Bank::Bank0;
}

impl DeviceConfig {
    /// The register address of the device configuration
    pub const ADDR: u8 = 0x11;
    /// The bank the device configuration register is in
    pub const BANK: Bank = Bank::Bank0;
}

impl PowerManagement {
    /// The register address of the power management configuration
    pub const ADDR: u8 = 0x4E;
    /// The bank the power management register is in
    pub const BANK: Bank = Bank::Bank0;
}

impl GyroConfig {
    /// The register address of the gyro configuration
    pub const ADDR: u8 = 0x4F;
    /// The bank the gyro configuration is in
    pub const BANK: Bank = Bank::Bank0;
}

impl AccelConfig {
    /// The register address of the accelerometer configuration
    pub const ADDR: u8 = 0x50;
    /// The bank the accelerometer configuration is in
    pub const BANK: Bank = Bank::Bank0;
}
