pub enum Command {
    EnableDisableMotors = 0x30,
    ReadStatusWriteDuty = 0x80,
    ReadEncoders = 0x91,
    ReadHalls = 0x92,
    ReadDuty = 0x93,
    ReadHash1 = 0x94,
    ReadHash2 = 0x95,
    CheckDrive = 0x96,
}