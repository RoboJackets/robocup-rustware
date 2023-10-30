#![no_std]

use embedded_hal::blocking::i2c;

pub struct Mcp23017<T>
where
    T: i2c::Write + i2c::WriteRead,
{
    i2c: T,
    addr: u8,
    cached_gpio: u16,
}

impl<T> Mcp23017<T>
where
    T: i2c::Write + i2c::WriteRead,
{
    pub fn new(i2c: T, addr: u8) -> Result<Mcp23017<T>, <T as i2c::Write>::Error> {
        let mut res = Mcp23017 {
            i2c,
            addr,
            cached_gpio: 0
        };

        res.reset()?;
        
        Ok(res)
    }

    pub fn write(&mut self, reg: Register, data: u16) -> Result<(), <T as i2c::Write>::Error> {
        let buff = [reg.get_addr(), (data & 0xFF) as u8, (data >> 8) as u8];
        self.i2c.write(self.addr, &buff)
    }

    pub fn read(&mut self, reg: Register) -> Result<u16, <T as i2c::WriteRead>::Error> {
        let write_buff = [reg.get_addr()];
        let mut read_buff: [u8; 2] = [0; 2];

        self.i2c.write_read(self.addr, &write_buff, &mut read_buff)?;

        Ok((read_buff[0] as u16) | ((read_buff[1] as u16) << 8))
    }

    pub fn reset(&mut self) -> Result<(), <T as i2c::Write>::Error> {
        self.write(Register::IODIR, 0xFFFF)?;  // sets everything to input

        // sets every register besides the IODIR to zero
        let mut buff: [u8; 21] = [0; 21];
        buff[0] = Register::IPOL.get_addr();
        self.i2c.write(self.addr, &buff)?;

        self.cached_gpio = 0;

        Ok(())
    }

    pub fn set_pin(&mut self, pin: GPIOPin) -> Result<(), <T as i2c::Write>::Error> {
        self.cached_gpio |= 0x1 << pin.get_shift();

        self.write(Register::GPIO, self.cached_gpio)
    }

    pub fn clear_pin(&mut self, pin: GPIOPin) -> Result<(), <T as i2c::Write>::Error> {
        self.cached_gpio &= !(0x1 << pin.get_shift());

        self.write(Register::GPIO, self.cached_gpio)
    }

    pub fn toggle_pin(&mut self, pin: GPIOPin) -> Result<(), <T as i2c::Write>::Error> {
        self.cached_gpio ^= 0x1 << pin.get_shift();

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
        
        Ok(((self.cached_gpio >> pin.get_shift()) & 0x1) != 0)
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
        self.write(Register::GPPU, 0x1 << pin.get_shift())
    }
}

pub enum Register {
    IODIR,
    IPOL,
    GPINTEN,
    DEFVAL,
    INTCON,
    IOCON,
    GPPU,
    INTF,
    INTCAP,
    GPIO,
    OLAT,
}

impl Register {
    pub fn get_addr(&self) -> u8 {
        match *self {
            Self::IODIR => 0x00,
            Self::IPOL => 0x02,
            Self::GPINTEN => 0x04,
            Self::DEFVAL => 0x06,
            Self::INTCON => 0x08,
            Self::IOCON => 0x0A,
            Self::GPPU => 0x0C,
            Self::INTF => 0x0E,
            Self::INTCAP => 0x10,
            Self::GPIO => 0x12,
            Self::OLAT => 0x14,
        }
    }
}

pub enum GPIOPin {
    A0,
    A1,
    A2,
    A3,
    A4,
    A5,
    A6,
    A7,
    B0,
    B1,
    B2,
    B3,
    B4,
    B5,
    B6,
    B7,
}

impl GPIOPin {
    pub fn get_shift(&self) -> u8 {
        match *self {
            Self::A0 => 0,
            Self::A1 => 1,
            Self::A2 => 2,
            Self::A3 => 3,
            Self::A4 => 4,
            Self::A5 => 5,
            Self::A6 => 6,
            Self::A7 => 7,
            Self::B0 => 8,
            Self::B1 => 9,
            Self::B2 => 10,
            Self::B3 => 11,
            Self::B4 => 12,
            Self::B5 => 13,
            Self::B6 => 14,
            Self::B7 => 15,
        }
    }
}

pub enum PinMode {
    Input,
    Output,
}
