#![no_std]

use embedded_hal::blocking::i2c;

pub struct Mcp23017<T>
where
    T: i2c::Write + i2c::WriteRead,
{
    i2c: T,
    addr: u8,
    cached_gpio: u16,
    cached_dir: u16,
}

impl<T> Mcp23017<T>
where
    T: i2c::Write + i2c::WriteRead,
{
    pub fn new(i2c: T, addr: u8) -> Result<Mcp23017<T>, <T as i2c::Write>::Error> {
        let mut res = Mcp23017 {
            i2c,
            addr,
            cached_gpio: 0,
            cached_dir: 0xFFFF,
        };

        res.reset()?;
        
        Ok(res)
    }

    pub fn write(&mut self, reg: Register, data: u16) -> Result<(), <T as i2c::Write>::Error> {
        let buff = [reg as u8, (data & 0xFF) as u8, (data >> 8) as u8];
        self.i2c.write(self.addr, &buff)
    }

    pub fn read(&mut self, reg: Register) -> Result<u16, <T as i2c::WriteRead>::Error> {
        let write_buff = [reg as u8];
        let mut read_buff: [u8; 2] = [0; 2];

        self.i2c.write_read(self.addr, &write_buff, &mut read_buff)?;

        Ok((read_buff[0] as u16) | ((read_buff[1] as u16) << 8))
    }

    pub fn reset(&mut self) -> Result<(), <T as i2c::Write>::Error> {
        self.write(Register::IODIR, 0xFFFF)?;  // sets everything to input

        // sets every register besides the IODIR to zero
        let mut buff: [u8; 21] = [0; 21];
        buff[0] = Register::IPOL as u8;
        self.i2c.write(self.addr, &buff)?;

        self.cached_gpio = 0;
        self.cached_dir = 0xFFFF;

        Ok(())
    }

    pub fn set_pin(&mut self, pin: GPIOPin) -> Result<(), <T as i2c::Write>::Error> {
        self.cached_gpio |= 0x1 << (pin as u8);

        self.write(Register::GPIO, self.cached_gpio)
    }

    pub fn clear_pin(&mut self, pin: GPIOPin) -> Result<(), <T as i2c::Write>::Error> {
        self.cached_gpio &= !(0x1 << (pin as u8));

        self.write(Register::GPIO, self.cached_gpio)
    }

    pub fn toggle_pin(&mut self, pin: GPIOPin) -> Result<(), <T as i2c::Write>::Error> {
        self.cached_gpio ^= 0x1 << (pin as u8);

        self.write(Register::GPIO, self.cached_gpio)
    }

    pub fn write_pin(&mut self, pin: GPIOPin, value: bool) -> Result<(), <T as i2c::Write>::Error> {
        if value {
            self.set_pin(pin)
        } else {
            self.clear_pin(pin)
        }
    }

    pub fn read_pin(&mut self, pin: GPIOPin) -> Result<bool, <T as i2c::WriteRead>::Error> {
        self.cached_gpio = self.read(Register::GPIO)?;
        
        Ok(((self.cached_gpio >> (pin as u8)) & 0x1) != 0)
    }

    pub fn pin_mode(&mut self, pin: GPIOPin, mode: PinMode) -> Result<(), <T as i2c::Write>::Error> {
        match mode {
            PinMode::Input => self.cached_dir |= 0x1 << (pin as u8),
            PinMode::Output => self.cached_dir &= 0x1 << (pin as u8),
        }

        self.write(Register::IODIR, self.cached_dir)
    }

    pub fn write_mask(&mut self, data: u16, mask: u16) -> Result<(), <T as i2c::Write>::Error> {
        self.cached_gpio = (self.cached_gpio & !mask) | data;

        self.write(Register::GPIO, self.cached_gpio)
    }

    pub fn config(&mut self, dir_config: u16, pullup_config: u16, polarity_config: u16) -> Result<(), <T as i2c::Write>::Error> {
        self.write(Register::IODIR, dir_config)?;
        self.write(Register::GPPU, pullup_config)?;
        self.write(Register::IPOL, polarity_config)?;

        Ok(())
    }

    pub fn enable_pullup(&mut self, pin: GPIOPin) -> Result<(), <T as i2c::Write>::Error> {
        self.write(Register::GPPU, 0x1 << (pin as u8))
    }
}

pub enum Register {
    IODIR = 0x00,
    IPOL = 0x02,
    GPINTEN = 0x04,
    DEFVAL = 0x06,
    INTCON = 0x08,
    IOCON = 0x0A,
    GPPU = 0x0C,
    INTF = 0x0E,
    INTCAP = 0x10,
    GPIO = 0x12,
    OLAT = 0x14,
}

pub enum GPIOPin {
    A0 = 0,
    A1 = 1,
    A2 = 2,
    A3 = 3,
    A4 = 4,
    A5 = 5,
    A6 = 6,
    A7 = 7,
    B0 = 8,
    B1 = 9,
    B2 = 10,
    B3 = 11,
    B4 = 12,
    B5 = 13,
    B6 = 14,
    B7 = 15,
}

pub enum PinMode {
    Input,
    Output,
}
