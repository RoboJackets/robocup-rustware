//!
//! Driver for the MCP23017 IO Expander on the current Robojackets Robocup Control Board
//! NEED TO WAIT IN BETWEEEN EACH USE
//! 



#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]

// FIGURE OUT HOW THIS WORKS
#![deny(missing_docs)]



use core::fmt::Debug;

// embedded hal traits
//use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::i2c;
use embedded_hal::blocking::i2c::*;//{I2C, I2CError, SevenBitAddress};
//use imxrt_hal::lpi2c::*;


//imxrt gpio Input struct definition
// needed bc embedded_hal 0.2 default features do not
// support InputPin trait. This trait is part of hte "unproven"
// features of embedded_hal 0.2
//use teensy4_bsp as bsp;
//use bsp::hal::gpio::Input;

// import fpga error type
pub mod error;
use error::IOExpanderError;


const GPPUA_ADDR : u8 = 0x0C;
const GPPUB_ADDR : u8 = 0x0D;
const GPIOA_ADDR : u8 = 0x12;
const GPIOB_ADDR : u8 = 0x13;
const IODIRA_ADDR : u8 = 0x00;
const IODIRB_ADDR : u8 = 0x01;
const GPINTENA_ADDR : u8 = 0x04;
const GPINTENB_ADDR : u8 = 0x05;
const IOCON_ADDR : u8 = 0x0A;
const INTRFA_ADDR : u8 = 0x0E;
//const INTRFB_ADDR : u8 = 0x0F;


/// enum for the output pins on the io expander
pub enum GpioPin {
    /// Bank A, pin 0
    GPIOA0,
    /// Bank A, pin 1
    GPIOA1,
    /// Bank A, pin 2
    GPIOA2,
    /// Bank A, pin 3
    GPIOA3,
    /// Bank A, pin 4
    GPIOA4,
    /// Bank A, pin 5
    GPIOA5,
    /// Bank A, pin 6
    GPIOA6,
    /// Bank A, pin 7
    GPIOA7,
    /// Bank B, pin 0
    GPIOB0,
    /// Bank B, pin 1
    GPIOB1,
    /// Bank B, pin 2
    GPIOB2,
    /// Bank B, pin 3
    GPIOB3,
    /// Bank B, pin 4
    GPIOB4,
    /// Bank B, pin 5
    GPIOB5,
    /// Bank B, pin 6
    GPIOB6,
    /// Bank B, pin 7
    GPIOB7,
}


// A0 = 1
// A1 = 0
// A2 = 0
const DEVICE_ADDR: u8 = 0b00100001; // 0 & 0100 & A2A1A0


/// High or low status for GPIO logic 
pub enum GpioStatus {
    /// High logic
    HIGH,
    /// Low logic
    LOW
}

/// Enum for the direciton of a GPIO pin
pub enum GpioDir {
    /// Takes inputs
    INPUT,
    /// Gives an output
    OUTPUT
}

impl GpioPin {

    /// gets the address and bank of the IO pin
    pub fn get_addr(self) -> (u8, u8) { // returns (bit, bank_num)
        match self {
            GpioPin::GPIOA0 => return (0, 0),
            GpioPin::GPIOA1 => return (1, 0),
            GpioPin::GPIOA2 => return (2, 0),
            GpioPin::GPIOA3 => return (3, 0),
            GpioPin::GPIOA4 => return (4, 0),
            GpioPin::GPIOA5 => return (5, 0),
            GpioPin::GPIOA6 => return (6, 0),
            GpioPin::GPIOA7 => return (7, 0),
            GpioPin::GPIOB0 => return (0, 1),
            GpioPin::GPIOB1 => return (1, 1),
            GpioPin::GPIOB2 => return (2, 1),
            GpioPin::GPIOB3 => return (3, 1),
            GpioPin::GPIOB4 => return (4, 1),
            GpioPin::GPIOB5 => return (5, 1),
            GpioPin::GPIOB6 => return (6, 1),
            GpioPin::GPIOB7 => return (7, 1)
        }
    }
}

///  Read operations are done to specific registers in the io expander by first doing a valid write with the address,
///  then doing a read with a restart condition. This is done using i2c::write_read
///   
///  Write operations are done by putting the register address as the first thing written in the buffer
pub enum OperationAddr<'a> {
    /// Contains the read buffer and the internal target address
    Read(&'a mut [u8], u8),    // read operation along with address
    /// Contains the write buffer, which should have the address as the first byte
    Write(&'a [u8])            // write operation. No address extra, because the address should be included in the buffer
}


/// takes ownership of the I2C line
/// 
/// Also holds the statuses of io expander registers to allow writes for individual bits
pub struct IoExpander<I2C, I2CE> where
    I2C: i2c::Write<Error=I2CE> + i2c::Read<Error = I2CE>
{
    /// The I2C line for the io expander. Can't bus share if we take ownership like this
    i2c : I2C,
    /// Bank A GPIO output register
    bank_a_out : u8, // keeping track of the most recently written data output allows for a function to write one bit without changing the rest
    /// Bank B GPIO output register
    bank_b_out : u8,
    /// Bank A GPIO direction register (0: output, 1: input)
    bank_a_dir : u8,
    /// Bank B GPIO direction register (0: output, 1: input)
    bank_b_dir : u8
}




impl<I2C, I2CE> IoExpander<I2C, I2CE>
    where
        I2C: Write<Error=I2CE> + Read<Error=I2CE> + WriteRead<Error=I2CE>,
        I2CE: Debug,

{
    
    // need to give it access to I2C for the io expander
    // This allows us to actually use it to communicate with the io expander
    /// Takes in i2c line and returns an IoExpander instance
    pub fn new(i2c: I2C) -> Result<IoExpander<I2C, I2CE>, IOExpanderError<I2CE>> {
        let io_expander_instance = Self {  
            i2c:i2c,
            bank_a_out: 0,
            bank_b_out: 0,
            bank_a_dir: 0,
            bank_b_dir: 0
        };

        //IoExpanderInstance.i2c.write(8, &[8]);
        Ok(io_expander_instance)
    }


    /// Carries out an operation
    /// 
    /// Always writes the DEVICE_ADDR first, then the internal address, then data is transmitted
    pub fn transaction(&mut self, operation: OperationAddr) -> Result<(), IOExpanderError<I2CE>> {
        let res;

        match operation {

            OperationAddr::Read(buffer, addr) => {
                // this if let block allows us to do the operation and handle an error easily in one step
                // need to first do a write of the address we want, then do the read
                res = self.i2c.write_read(DEVICE_ADDR, &[addr], buffer);
            },

            OperationAddr::Write(buffer) => {
                res = self.i2c.write(DEVICE_ADDR, buffer);
            }
        }

        match res {
            Err(error) => {
                return Err(IOExpanderError::I2C(error));
            },
            Ok(()) => {
                return Ok(());
            }
        }
    }



    /// initializes the the io expander
    /// Isn't actually necessary, initialization can be done with whatever is using the device
    pub fn init(&mut self) -> Result<(), IOExpanderError<I2CE>> {
        // Nothing is really necessary to initialize in this driver
        // unless we want the default to be something different
        /*
        if let Err(initialized) = transaction(&mut self, 0) {
            return (Err(initialized));
        } else {
            log::info!("successfully initialized the IO expander");
        }
        */
        return Ok(());
    }


    /// Sets the GPIO directions for bank A
    /// 
    /// 0 means output
    /// 1 means input
    pub fn set_bank_a_dirs(&mut self, dirs: u8) -> Result<(), IOExpanderError<I2CE>> {
        let arr_dirs = [IODIRA_ADDR, dirs];
        let operation = OperationAddr::Write(&arr_dirs);
        self.bank_a_dir = dirs;
        return self.transaction(operation);
    }

    /// Sets the GPIO directions for bank B
    /// 
    /// 0 means output
    /// 1 means input
    pub fn set_bank_b_dirs(&mut self, dirs: u8) -> Result<(), IOExpanderError<I2CE>> {
        let arr_dirs = [IODIRB_ADDR, dirs];
        let operations = OperationAddr::Write(&arr_dirs);
        self.bank_b_dir = dirs;
        return self.transaction(operations);
    }

    /// Sets the directions for both banks
    /// 
    /// Does not work as of right now since it is done in two separate transactions, which is too fast
    /// Need to enable address increment, then do one transaction sending both bytes
    pub fn set_all_dirs(&mut self, dirs: u16) -> Result<(), IOExpanderError<I2CE>> {
        let bank_a_dirs = (dirs & 0xFF00 >> 8) as u8;
        let bank_b_dirs = (dirs & 0xFF) as u8;
        
        self.bank_a_dir = bank_a_dirs;
        self.bank_b_dir = bank_b_dirs;

        // if there's an error with the first transaction, return that error
        // otherwise continue with the second transaction
        match self.set_bank_a_dirs(bank_a_dirs) {
            Err(error) => return Err(error),
            Ok(()) => {
                return self.set_bank_b_dirs(bank_b_dirs);
            }
        }
        
    }


    /// set the direction of a single pin, keeping the rest the same
    /// 
    /// takes a GpioPin enum, which allows us to get the bank number and offset
    pub fn set_dir(&mut self, pin: GpioPin, value: GpioDir) -> Result<(), IOExpanderError<I2CE>> {
        let (bit, banknum) = pin.get_addr(); // holds (addr, bit, banknum)
        let mask: u8;
        let operation: OperationAddr;

        match value {

            // 1 means input
            GpioDir::INPUT => {

                mask = 1 << bit; // 1 in the position that we're writing high to

                // this means it is bank A
                if banknum == 0 {
                    self.bank_a_dir |= mask; // bitwise or this with bank_a_out to write just that one
                    let dir = [IODIRA_ADDR, self.bank_a_dir];
                    operation = OperationAddr::Write(&dir); // create the operation for the transaction
                    return self.transaction(operation); // do the transaction
                } else {
                    // then it is bank B
                    self.bank_b_dir |= mask;
                    let dir = [IODIRB_ADDR, self.bank_b_dir];
                    operation = OperationAddr::Write(&dir);
                    return self.transaction(operation);
                }
                
            },

            // 0 means output
            GpioDir::OUTPUT  => {

                mask = 0xFF ^ (1<< bit);  // creates all ones except for a 0 where we write low

                // bank A
                if banknum == 0 {
                    self.bank_a_dir &= mask;
                    let dir = [IODIRA_ADDR, self.bank_a_dir];
                    operation = OperationAddr::Write(&dir);
                    return self.transaction(operation);

                // bank B
                } else {
                    self.bank_b_dir &= mask;
                    let dir = [IODIRB_ADDR, self.bank_b_dir];
                    operation = OperationAddr::Write(&dir);
                    return self.transaction(operation);
                }
            }
        }
    }    

    /// Writes to the IOCON register
    /// bit:
    /// 7  BANK: address config;
    /// 6  MIRROR: connect int pins;
    /// 5  SEQOP:  Address increment;
    /// 4  DISSLW: disable slew;
    /// 3  HAEN: en address pins;
    /// 2  ODR: open drain;
    /// 1  IOPOL: 1(active high int), 0(active low int)  ;
    /// 0  Nothing;
    pub fn set_iocon_reg(&mut self, value: u8) -> Result<(), IOExpanderError<I2CE>> {
        let bytes = [IOCON_ADDR, value];
        let operation = OperationAddr::Write(&bytes);
        return self.transaction(operation);
    }

    /// reads the bank a interrupt register, clearing the interrupt.
    /// Need to do this after an interrupts to prevent retriggering
    pub fn read_intf_a(&mut self) -> Result<u8, IOExpanderError<I2CE>> {
        let mut read = [0];
        let operation = OperationAddr::Read(&mut read, INTRFA_ADDR);
        self.transaction(operation)?;
        return Ok(read[0]);
    }

    /// Set the interrupts for bank A
    pub fn set_bank_a_ints(&mut self, ints: u8) -> Result<(), IOExpanderError<I2CE>> {
        let ints = [GPINTENA_ADDR, ints];
        let operation = OperationAddr::Write(&ints);
        return self.transaction(operation);
    }

    /// set the interrupts for bank b
    pub fn set_bank_b_ints(&mut self, ints: u8) -> Result<(), IOExpanderError<I2CE>> {
        let ints = [GPINTENB_ADDR, ints];
        let operation = OperationAddr::Write(&ints);
        return self.transaction(operation);
    }
    
    /// set the pullups for bank A
    pub fn set_bank_a_pus(&mut self, pus: u8) -> Result<(), IOExpanderError<I2CE>> {
        let pus = [GPPUA_ADDR, pus];
        let operations = OperationAddr::Write(&pus);
        return self.transaction(operations);
    }

    /// Set the pullups for bank B
    pub fn set_bank_b_pus(&mut self, pus: u8) -> Result<(), IOExpanderError<I2CE>> {
        let pus = [GPPUB_ADDR, pus];
        let operations = OperationAddr::Write(&pus);
        return self.transaction(operations);
    }

    /// Set all pullups.doesn't work right now because it is in two transactions
    /// Need to turn on SEQOP and do in one transaction
    pub fn set_all_pus(&mut self, pus: u16) -> Result<(), IOExpanderError<I2CE>> {
        let bank_a = [GPPUA_ADDR, (pus >> 8) as u8];
        let bank_b = [GPPUB_ADDR, (pus & 0xFF) as u8];
        let operation0 = OperationAddr::Write(&bank_a);
        let operation1 = OperationAddr::Write(&bank_b);
        
        match self.transaction(operation0) {
            Err(error) => return Err(error),
            Ok(()) => {
                return self.transaction(operation1);
            }
        }
    }

    /// Write the outputs for bank a
   pub fn write_gpio_a(&mut self, byte: u8) -> Result<(), IOExpanderError<I2CE>> {
        self.bank_a_out = byte;
        let byte = [GPIOA_ADDR, byte];
        let operation = OperationAddr::Write(&byte);
        return self.transaction(operation);
   }

   /// Write the outputs for bank b
   pub fn write_gpio_b(&mut self, byte: u8) -> Result<(), IOExpanderError<I2CE>> {
        self.bank_b_out = byte;
        let byte = [GPIOB_ADDR, byte];
        let operations = OperationAddr::Write(&byte);
        return self.transaction(operations);
    }

    /// Write to both GPIO bank outputs. Doesn't work because two transactions
    /// need to turn on seqop and do one transaction
    pub fn write_both_gpio(&mut self, bytes: [u8; 2]) -> Result<(), IOExpanderError<I2CE>> {
        self.bank_a_out = bytes[0];
        self.bank_b_out = bytes[1];
        
        match self.write_gpio_a(bytes[0]) {
            Err(error) => return Err(error),
            Ok(()) => {
                return self.write_gpio_b(bytes[1]);
            }
        }
    }

    /// read from gpio bank a
    pub fn read_gpio_a(&mut self) -> Result<u8, IOExpanderError<I2CE>> {
        let mut received : [u8; 1] = [0];
        let operation = OperationAddr::Read(&mut received, GPIOA_ADDR);

        match self.transaction(operation) {
            Err(error) => return Err(error),
            Ok(()) => return Ok(received[0])
        }
    }

    /// read from gpio bank b
    pub fn read_gpio_b(&mut self) -> Result<u8, IOExpanderError<I2CE>> {
        let mut received : [u8; 1] = [0];
        let operation = OperationAddr::Read(&mut received, GPIOB_ADDR);

        match self.transaction(operation) {
            Err(error) => return Err(error),
            Ok(()) => return Ok(received[0])
        }
    }

    /// read a single input from a single bank
    pub fn read_single_input(&mut self, pin: GpioPin) -> Result<u8, IOExpanderError<I2CE>> {
        let (bit, banknum) = pin.get_addr();
        let res;

        //means bank A
        if banknum == 0 {
            res = self.read_gpio_a()?;
        } else {
            res = self.read_gpio_b()?;
        }


        Ok((res >> bit) & 1) // extracts only the specified bit
        
        
    }

    /// read from both banks. Need to enable seqop and do one transaction
    pub fn read_both_banks(&mut self) -> Result<[u8; 2], IOExpanderError<I2CE>> {
        let bank_a = self.read_gpio_a()?;
        let bank_b = self.read_gpio_b()?;
        
        let mut res = [0;2];
        res[0] = bank_a;
        res[1] = bank_b;
        
        return Ok(res);
    }

    /// writes to only one IO pin, either a high or a low.
    /// won't change the other outputs in the bank, as we kept track of them before
    pub fn write_single_output(&mut self, pin: GpioPin, value: GpioStatus) -> Result<(), IOExpanderError<I2CE>> {
        let (bit, banknum) = pin.get_addr(); // holds (addr, bit, banknum)
        let mask: u8;
        let operation: OperationAddr;

        match value {
            GpioStatus::HIGH => {

                mask = 1 << bit; // 1 in the position that we're writing high to

                // this means it is bank A
                if banknum == 0 {
                    self.bank_a_out |= mask; // bitwise or this with bank_a_out to write just that one
                    let out = [GPIOA_ADDR, self.bank_a_out];
                    operation = OperationAddr::Write(&out); // create the operation for the transaction
                    return self.transaction(operation); // do the transaction
                } else {
                    // then it is bank B
                    self.bank_b_out |= mask;
                    let out = [GPIOB_ADDR, self.bank_b_out];
                    operation = OperationAddr::Write(&out);
                    return self.transaction(operation);
                }
                
            },

            GpioStatus::LOW  => {

                mask = 0xFF ^ (1<< bit);  // all ones xored with 1 in position we're writing low to

                // bank A
                if banknum == 0 {
                    self.bank_a_out &= mask;
                    let out = [GPIOA_ADDR, self.bank_a_out];
                    operation = OperationAddr::Write(&out);
                    return self.transaction(operation);

                // bank B
                } else {
                    self.bank_b_out &= mask;
                    let out = [GPIOB_ADDR, self.bank_b_out];
                    operation = OperationAddr::Write(&out);
                    return self.transaction(operation);
                }
            }
        }
    }    
}