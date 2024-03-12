//!
//! Definition of the registers of the io-expander chip
//! 

#![allow(unused)]

/// The various registers of the io-expander
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

impl Register {
    pub fn address(self) -> u8 {
        self as u8
    }
}