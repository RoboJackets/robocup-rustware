//!
//! Driver for the Rotary Switch on the current Robojackets Robocup Control Board
//! 



#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]
#![deny(missing_docs)] // DO DOCS



use core::fmt::Debug;

// embedded hal traits
//use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::i2c;


//imxrt gpio Input struct definition
// needed bc embedded_hal 0.2 default features do not
// support InputPin trait. This trait is part of hte "unproven"
// features of embedded_hal 0.2
//use teensy4_bsp as bsp;
//use bsp::hal::gpio::Input;

// import Rotary error type
//pub mod error;
//use error::RotaryError;


use io_expander_rs as io_expander;
use io_expander::IoExpander;
use io_expander::*; // Imports all of the enums and everything else
//use io_expander::GpioStatus;
//use io_expander::GpioDir;
use io_expander::error::IOExpanderError;

use teensy4_bsp::hal::gpt::Gpt2;
use teensy4_bsp as bsp;
use bsp::hal::timer::Blocking;




// pins on the IO expander:
//     IO pin  ||    connection
//     ------------------------
//     28/GPA7 ||     Hex 0     GPA7 IS OUTPUT ONLY.... need to set to 0 immediately so it never will short with ground through the encoder
//     25/GPA4 ||     Hex 1
//     27/GPA6 ||     Hex 2
//     26/GPA5 ||     Hex 3
//
//     22/GPA1 ||     Dip 1
//     23/GPA2 ||     Dip 2
//     24/GPA3 ||     Dip 3
//
//     01/GPB0 ||     LED-KICK
//     03/GPB2 ||     RST-I2C_n
//     04/GPB3 ||     LED-M4
//     05/GPB4 ||     LED-M3
//     06/GPB5 ||     LED-M2
//     07/GPB6 ||     LED-M1
//     08/GPB8 ||     LED-DD
//
//      INTA   ||     P3


const HEX0_PIN : GpioPin = GpioPin::GPIOA7;
const HEX1_PIN : GpioPin = GpioPin::GPIOA4;
const HEX2_PIN : GpioPin = GpioPin::GPIOA6;
const HEX3_PIN : GpioPin = GpioPin::GPIOA5;

const DIP1_PIN : GpioPin = GpioPin::GPIOA1;
const DIP2_PIN : GpioPin = GpioPin::GPIOA2;
const DIP3_PIN : GpioPin = GpioPin::GPIOA3;

const LED_KICK_PIN : GpioPin = GpioPin::GPIOB0;
//const RS_I2C_n_PIN : GpioPin = GpioPin::GPIOB2;
//const LED_M4_PIN : GpioPin = GpioPin::GPIOB3;
//const LED_M3_PIN : GpioPin = GpioPin::GPIOB4;
//const LED_M2_PIN : GpioPin = GpioPin::GPIOB5;
//const LED_M1_PIN : GpioPin = GpioPin::GPIOB6;
//const LED_DD_PIN : GpioPin = GpioPin::GPIOB7; // Was previously B8, but was probably a mistake. double check


/// Driver for the Rotary Switch
pub struct RotarySwitch <I2C, I2CE> where
    I2C: i2c::Write<Error=I2CE> + i2c::Read<Error=I2CE>
    {
    /// Takes an IO expander object
    io_expander : IoExpander<I2C, I2CE>,
    /// Holds the current value of the rotary switch
    value : u8
}

impl <I2C, I2CE> RotarySwitch<I2C, I2CE>
    where
        I2C: i2c::Write<Error=I2CE> + i2c::Read<Error=I2CE> + i2c::WriteRead<Error=I2CE>,
        I2CE: Debug,
    {
        
    /// Creates a new rotary switch interface, taking in an io expander
    pub fn new (io_expander: IoExpander<I2C, I2CE>) -> Result<Self, IOExpanderError<I2CE>> {
        let rotary_switch = RotarySwitch {
            io_expander: io_expander,
            value: 0
        };
        Ok(rotary_switch)
    }

    /// initializes the io expander
    /// needs to take in a gpt for delays. Can't do multiple transactions too quickly
    pub fn init(&mut self, gpt: &mut Blocking<Gpt2, 1_000>) -> Result<u8, IOExpanderError<I2CE>> {
        // set the directions of the pins

        self.io_expander.set_dir(HEX0_PIN, GpioDir::INPUT)?; // GPIOA7 CAN ONLY BE AN OUTPUT, SO THIS MIGHT NOT WORK*****
        gpt.block_ms(10);
        self.io_expander.set_dir(HEX1_PIN, GpioDir::INPUT)?;
        gpt.block_ms(10);
        self.io_expander.set_dir(HEX2_PIN, GpioDir::INPUT)?;
        gpt.block_ms(10);
        self.io_expander.set_dir(HEX3_PIN, GpioDir::INPUT)?;
        gpt.block_ms(10);
        self.io_expander.set_dir(DIP1_PIN, GpioDir::INPUT)?;
        gpt.block_ms(10);
        self.io_expander.set_dir(DIP2_PIN, GpioDir::INPUT)?;
        gpt.block_ms(10);
        self.io_expander.set_dir(DIP3_PIN, GpioDir::INPUT)?;
        gpt.block_ms(10);
        self.io_expander.set_iocon_reg(0b00000010)?; // set Open drain to off, turns int to active high
        gpt.block_ms(10);
        self.io_expander.set_bank_a_ints(0xFF)?;
        //gpt.block_ms(10);
        
        // use this to check if it is working with the LEDS
        self.io_expander.set_bank_b_dirs(0xFF)?; // THIS WORKS
        gpt.block_ms(10);
        self.io_expander.set_dir(LED_KICK_PIN, GpioDir::INPUT)?; // THIS WORKS
        gpt.block_ms(10);
        self.io_expander.write_single_output(LED_KICK_PIN, GpioStatus::HIGH)?; // THIS ISN'T WORKING

        // set pullups for bank A input pins
        // THIS IS NEEDED
        self.io_expander.set_bank_a_pus(0xFF)?;

        self.value = self.read()?;
        
    

        // gets the initial return value
        Ok(self.value)
    }

    /// Clears the interrupts by reading the interrupt register
    /// returns the pins that caused the interrupt
    pub fn clear_interrupts(&mut self) -> Result<u8, IOExpanderError<I2CE>> {
        return self.io_expander.read_intf_a();
    }

    /// read the rotary switch. Reads GPIO bank A and decodes it
    pub fn read(&mut self) -> Result<u8, IOExpanderError<I2CE>> {
        let gpio_a = self.io_expander.read_gpio_a()?;

        let (bit0, _) = HEX0_PIN.get_addr();
        let (bit1, _) = HEX1_PIN.get_addr();
        let (bit2, _) = HEX2_PIN.get_addr();
        let (bit3, _) = HEX3_PIN.get_addr();

        let hex0 = ((gpio_a >> bit0) & 1) ^ 1;
        let hex1 = ((gpio_a >> bit1) & 1) ^ 1;
        let hex2 = ((gpio_a >> bit2) & 1) ^ 1;
        let hex3 = ((gpio_a >> bit3) & 1) ^ 1;

        self.value = (hex3 << 3) | (hex2 << 2) | (hex1 << 1) | (hex0);
        return Ok(self.value);
    }

    /// Returns the most recent value of the rotary switch
    pub fn get_value(&mut self) -> u8 {
        self.value
    }

    
}


