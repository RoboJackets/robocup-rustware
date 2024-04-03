#![no_std]
#![crate_type = "lib"]

pub struct MCP23017Driver<I2C> {
    _i2c: I2C,
    _i2c_address: u8,
    _cached_gpio: u16,
    _cached_iodir: u16,
    _cached_gppu: u16,
    _cached_ipol: u16,
}

impl<I2C> MCP23017Driver<I2C> {

    pub fn new(i2c: I2C, i2c_address: u8) -> Self {
        Self {
            _i2c: i2c,
            _i2c_address: i2c_address,
            _cached_gpio: 0,
            _cached_iodir: 0,
            _cached_gppu: 0,
            _cached_ipol: 0,
        }
    }

    // Initialize the device.
    pub fn init(&self) {}

    // Reset MCP23017 device to its power-on state
    pub fn reset(&self) {
        let mut i2c_lock = self._i2c;

        // Set all pins to input mode (via IODIR register)
        self.input_output_mask(0xFFFF, i2c_lock);

        // set all other registers to zero (last of 10 registers is OLAT)
        for reg_addr in (2..=10).step_by(2) {
            self.write_register(reg_addr as u8, 0x0000, i2c_lock);
        }

        // reset cached values
        self._cached_gpio = 0;
        self._cached_gppu = 0;
        self._cached_ipol = 0;
    }

    // Write a 0/1 value to an output bit
    pub fn write_pin(&mut self, value: u8, pin: u8) {
        if value == 0 {
            self._cached_gpio &= !(1 << pin);
        } else {
            self._cached_gpio |= 1 << pin;
        }

        self.digital_word_write(self._cached_gpio);
    }

    pub fn write_mask(&mut self, data: u16, mask: u16) {
        self._cached_gpio = (self._cached_gpio & !mask) | data;
        self.digital_word_write(self._cached_gpio);
    }

    pub fn read_pin(&mut self, pin: u8) -> u8 {
        self._cached_gpio = self.digital_word_read();
        (self._cached_gpio >> pin) as u8 & 0x01
    }

    // Configure an MCP23017 device
    pub fn config(&mut self, dir_config: u16, pullup_config: u16, polarity_config: u16) {
        self.input_output_mask(dir_config,  self._i2c);
        self.internal_pullup_mask(pullup_config, self._i2c);
        self.input_polarity_mask(polarity_config,self._i2c);
    }

    pub fn write_register(&mut self, reg_address: u8, data: u16, i2c_lock: I2C) {
        let buffer: [u8; 2] = [(data & 0xFF) as u8, (data >> 8) as u8];
        let _ = i2c_lock.write(self._i2c_address, &[reg_address, buffer[0], buffer[1]]);
    }

    pub fn read_register(&mut self, reg_address: u8, i2c_lock: &mut I2C) -> u16 {
        let mut buffer = [0; 2];
        let _ = i2c_lock.write_read(self._i2c_address, &[reg_address], &mut buffer);
        (buffer[0] as u16) | ((buffer[1] as u16) << 8)
    }

    // Set pin mode
    pub fn pin_mode(&mut self, pin: u8, mode: u8) {
        if mode == 1 {
            self._cached_iodir |= 1 << pin;
        } else {
            self._cached_iodir &= !(1 << pin);
        }

        self.input_output_mask(self._cached_iodir,  self._i2c);
    }

    pub fn digital_read(&mut self, pin: u8) -> u8 {
        self._cached_gpio = self.read_register(0x12, &mut self._i2c);
        (self._cached_gpio >> pin) as u8 & 0x01
    }

    pub fn digital_write(&mut self, pin: u8, val: u8) {
        let is_output = (self._cached_iodir & (1 << pin)) == 0;

        if is_output {
            if val == 1 {
                self._cached_gpio |= 1 << pin;
            } else {
                self._cached_gpio &= !(1 << pin);
            }

            self.digital_word_write(self._cached_gpio);
        } else {
            if val == 1 {
                self._cached_gppu |= 1 << pin;
            } else {
                self._cached_gppu &= !(1 << pin);
            }

            self.internal_pullup_mask(self._cached_gppu, self._i2c);
        }
    }

    // Read word from digital pins - no idea what this is supposed to be 
    pub fn digital_word_read(&mut self) -> u16 {
        self.read_register(0x12, &mut self._i2c)
    }

    pub fn digital_word_write(&mut self, w: u16) {
        self._cached_gpio = w;
        self.write_register(0x12, w, self._i2c);
    }

    pub fn input_polarity_mask(&mut self, mask: u16, i2c_lock: I2C) {
        self._cached_ipol = mask;
        self.write_register(0x02, mask, i2c_lock);
    }

    pub fn input_output_mask(&mut self, mask: u16, i2c_lock: I2C) {
        self._cached_iodir = mask;
        self.write_register(0x00, mask, i2c_lock);
    }

    pub fn internal_pullup_mask(&mut self, mask: u16, i2c_lock: I2C) {
        self._cached_gppu = mask;
        self.write_register(0x06, mask, i2c_lock);
    }
}