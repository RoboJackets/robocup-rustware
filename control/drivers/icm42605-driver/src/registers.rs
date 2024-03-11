use bitflags::bitflags;

pub const ACCEL_DATA_X1: u8 = 0x1F;
pub const ACCEL_DATA_X0: u8 = 0x20;
pub const ACCEL_DATA_Y1: u8 = 0x21;
pub const ACCEL_DATA_Y0: u8 = 0x22;
pub const GYRO_DATA_Z1: u8 = 0x29;
pub const GYRO_DATA_Z0: u8 = 0x2A;
pub const WHO_AM_I: u8 = 0x75;

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct BankSelect: u8 {
        const BANK_0 = 0;
        const BANK_1 = 1;
        const BANK_2 = 2;
        const BANK_3 = 3;
        const BANK_4 = 4;
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct DeviceConfig: u8 {
        const SPI_MODE_0_AND_3 = 0b0 << 4;
        const SPI_MODE_1_AND_2 = 0b1 << 4;

        const SOFT_RESET_CONFIG = 0b1;
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
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

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Bank {
	#[default]
    Bank0,
    Bank1,
    Bank2,
    Bank3,
    Bank4,
}

impl Bank {
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

impl BankSelect {
    pub const ADDR: u8 = 0x76;
    pub const BANK: Bank = Bank::Bank0;
}

impl DeviceConfig {
    pub const ADDR: u8 = 0x11;
    pub const BANK: Bank = Bank::Bank0;
}

impl PowerManagement {
    pub const ADDR: u8 = 0x4E;
    pub const BANK: Bank = Bank::Bank0;
}

impl GyroConfig {
    pub const ADDR: u8 = 0x4F;
	pub const BANK: Bank = Bank::Bank0;
}

impl AccelConfig {
    pub const ADDR: u8 = 0x50;
	pub const BANK: Bank = Bank::Bank0;
}
