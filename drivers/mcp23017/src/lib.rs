use embedded_hal;

pub struct MCP23017<T>
where
    T: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::Read,
{
    i2c: T,
}

impl<T> MCP23017<T>
where
    T: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::Read,
{
    
}
