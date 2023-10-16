#![allow(unused_assignments, arithmetic_overflow)]
#![no_std]
#![crate_type = "lib"]

// import instructions
pub mod instructions;
use instructions::Instruction;
use instructions::DutyCycle;

// import fpga configuration array
pub mod config_bin;
use config_bin::FPGA_BYTES;

// import fpga error type
pub mod error;
use error::FpgaError;

// embedded hal traits
use embedded_hal::digital::v2::OutputPin; 
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::spi::{self, Mode};

use teensy4_bsp as bsp;

use bsp::hal::gpio::Input; // change made for the input pin compatibility


/// enum for fpga status :)
// maybe move into its own module?
// TODO: maybe wrap state info relaated to the FPGA into the enum varaints??
#[derive(Clone, Copy)]
pub enum FpgaStatus {
    NotReady, // haven't been succesfully init
    Standby, // init was done succesfully
}

/// use this constants when configuring the spi :)
pub const FPGA_SPI_FREQUENCY: u32 = 400_000;
pub const FPGA_SPI_MODE: Mode = spi::Mode{
    polarity: spi::Polarity::IdleLow,
    phase: spi::Phase::CaptureOnFirstTransition,
};

pub struct FPGA<SPI, CS, InitP, PROG, DoneP> {
    spi: SPI,
    cs: CS,
    init_b: Input<InitP>,
    prog_b: PROG,   // prog_b output pin "MUST" BE on OPEN_DRAIN configuration!!
    done: Input<DoneP>,
    status: FpgaStatus,
    /* TODO: 
        [] check if vTaskDelay() == delay_ms()
        [] finish implementing reading functions :)
        [] Add watchdog timer feature to stop motors when timeout occurs
    */
}

/**
    NOTE: Notice the name used in this impl for the Error types. I intentionally
    used different names from our custom FpgaError enum defined in the error.rs
    to outline how we are using our custom error to wrap embedded trait error types
*/
impl<SPI, CS, InitP, PROG, DoneP, SpiE, PinE> FPGA<SPI, CS, InitP, PROG, DoneP>
    where 
        SPI: Write<u8, Error = SpiE> + Transfer<u8, Error = SpiE>,
        PROG: OutputPin<Error = PinE>,
        CS: OutputPin<Error = PinE>,
{
    
    /// Builds and returns a new instance of the FPGA controller. Only one
    /// instance of this FPGA should exist at a time
    pub fn new(spi: SPI, cs: CS, init_b: Input<InitP>, prog_b: PROG, done: Input<DoneP>) 
            -> Result<Self, FpgaError<SpiE, PinE>> {
        
        // create new instance of FPGA and pass in appropriate pins
        let mut fpga = FPGA {
            spi,
            cs,
            init_b,
            prog_b,
            done,
            status: FpgaStatus::NotReady,
        };

        // initialize cs pin to be high (idle state)
        fpga.cs.set_high().map_err(FpgaError::CSPin)?;

        // returnt the instance
        Ok(fpga)
    }

    /// Configures the FPGA as follows:
    ///    1. Toggles the prog_b pin to clear out anything prior
    ///    2. Awaits for the FPGA init_b pin
    ///    3. Sends config
    ///    4. Awaits for the FPGA done pin
    ///    5. Returns Ok if no errors or timeout
    ///
    /// Parameters:
    ///    delay: An instance of a blocking delay that implements the DelayUs and
    ///           DelayMs embedded traits
    pub fn configure<D>(&mut self, delay: &mut D) 
        -> Result<(), FpgaError<SpiE, PinE>> 
        where 
            D: DelayMs<u8> + DelayUs<u8>
    {
        // First we toggle the prog_b pin
        /*  
            For this map_err call I'll demonstrate what's happening behind the scenes

            1. We're passing in a closure whose parameter is the error from set_low
               which we named "e" in this case
            2. This closure is "remapping" the error "e" into our custom error enum
            3. The Enum::<Generic Type Arguments>::Variant syntax is used when assigning
               struct or enums that take in generic parameters
            4. We pass in the pin error from set_low as the argument of ProgPin
            5. We're also returning this same enum 

            NOTE: This syntax can be shortened as follows:
            
            ```
                self.prog_b.set_low().map_err(FpgaError::ProgPin)?;

            ```
            
            This simplified syntax will be used throughout the remaining of the driver file
        */
        self.prog_b.set_low().map_err(|e| {
            let new_error = FpgaError::<SpiE, PinE>::ProgPin(e);
            new_error
        })?;
        delay.delay_ms(1);
        self.prog_b.set_high().map_err(FpgaError::ProgPin)?;

        // delay until init_b is ready
        let mut timeout = 100;
        while timeout > 0 {

            // wait for 10 ms on each cycle
            delay.delay_ms(10);

            // if the init_b pin is high => FPGA is ready to be programmed
            if self.init_b.is_set() {
                break;
            }            

            // decrement remaining timeout counter
            timeout -= 1;
            
            // check whether timeout has been reached (i.e. 100 wait cycles)
            if timeout == 0 {
                // return a timeout error with code 0x1
                return Err(FpgaError::<SpiE, PinE>::FPGATimeout(0x1));
            }
        }

        // send configuration array after init_b pin is asserted
        self.cs.set_low().map_err(FpgaError::CSPin)?;
        self.spi.write(&FPGA_BYTES).map_err(FpgaError::SPI)?;
        self.cs.set_high().map_err(FpgaError::CSPin)?;
        
        timeout = 100;
        // delay until done pin is ready
        while timeout > 0 {
            
            // wait for 100 ms on each cycle
            delay.delay_ms(100);

            // if done pin is high => FPGA is done configuring
            // update status to Standby
            if self.done.is_set() {
                self.status = FpgaStatus::Standby;
                break;
            }

            // decrement remaining timeout counter
            timeout -= 1;

            // check for timeout
            if timeout == 0 {
                // not sure why done pin is not being asserted as expected will bypass this for now and
                // test writing duty cycle into the fpga
                return Err(FpgaError::FPGATimeout(0x2));
            }
        }

        // Returns Ok if no timeout or errors occurred
        Ok(())
    }

    /// Reads the status field of the FPGA instance
    pub fn status(&mut self) -> FpgaStatus {
        self.status
    }

    /// 
    pub fn read_halls(&mut self, halls: &mut [u8; 5])
        -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // send READ HALLS instruction through an SPI transfer transaction
        let mut status: [u8; 1] = [Instruction::R_HALLS.opcode()];
        self.spi.transfer(&mut status).map_err(FpgaError::SPI)?;

        // Send 0x00 accordingly to read each motor's hall
        for i in 0..5 {
            // the result of each hall will be stored in a temp buffer
            let mut temp: [u8;1] = [0x00];
            self.spi.transfer(&mut temp).map_err(FpgaError::SPI)?;

            // store the hall value in the corresponding hall buffer
            halls[i] = temp[0];
        }

        // return status code
        Ok(status[0])
    }

    ///
    pub fn read_encs(&mut self, encs: &mut [i16; 5]) 
        -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // send READ ENCODERS instruction through an SPI transfer transaction
        let mut status: [u8; 1] = [Instruction::R_ENC.opcode()];
        self.spi.transfer(&mut status).map_err(FpgaError::SPI)?;
        
        /*
         * Send 0x00 accordingly to read each motor's encoder values
         * 
         * Each ecnoder value is obtained from two transfer transactions 
         * 
         * Example:
         * ENCODER[0] = [Response_0, Response_1]
         * Response_0 is stored in msByte and Response_1 is stored in lsByte
         */
        for i in 0..5 {
            // a buffer used to store each byte for the two spi transfers
            let mut ls_byte: [u8;1] = [0x00];
            let mut ms_byte: [u8;1] = [0x00];

            // read MSByte first
            self.spi.transfer(&mut ms_byte).map_err(FpgaError::SPI)?;
            // read the LSByte second
            self.spi.transfer(&mut ls_byte).map_err(FpgaError::SPI)?;

            // combine the two bytes obtained into a single i16 value
            encs[i] = (((ms_byte[0] as u16) << 8) | ls_byte[0] as u16) as i16; 
        }
        
        // return status code
        Ok(status[0])
    }

    ///
    pub fn read_duty_cycles(&mut self, duty_cycles: &mut [i16; 5]) 
        -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // send READ DUTY CYCLES instruction through SPI transfer transaction
        let mut status:[u8;1] = [Instruction::R_DUTY.opcode()];
        self.spi.transfer(&mut status).map_err(FpgaError::SPI)?;
        
        /*
         * Each duty cycle is obtained frorm two transfer transactions
         * The first transaction returns the MS Byte and the second transanction
         * returns the LS Byte
         *  
         * Duty cycles are encoded as a 10-bit signed magnitude
         * 
         * We're using the DutyCycle wrapping struct to hide the bit length conversion
        */
        for i in 0..5 {
            // a buffer used to store each byte for the two spi transfers
            let mut ls_byte: [u8;1] = [0x00];
            let mut ms_byte: [u8;1] = [0x00];

            // read MSByte first
            self.spi.transfer(&mut ms_byte).map_err(FpgaError::SPI)?;
            // read the LSByte second
            self.spi.transfer(&mut ls_byte).map_err(FpgaError::SPI)?;

            // stored returned bytes into a duty_cycle wrapper
            let dc = DutyCycle::new(((ms_byte[0] << 8) | ls_byte[0]) as u16);

            // convert the duty_cycle into an i16 and store in passed in buffer
            duty_cycles[i] = i16::from(dc);
        }
        
        // return status code
        Ok(status[0])
    }

    ///
    pub fn set_duty_cycles(&mut self, duty_cycles: &mut [DutyCycle; 5]) 
        -> Result<u8, FpgaError<SpiE, PinE>> 
        {

        // init write buffer
        let mut write_buffer: [u8; 11] = [0;11];
        
        // send READ ENC WRITE VEL instruction
        // we'll only write vel and not read any enc information for this function
        write_buffer[0] = Instruction::R_ENC_W_VEL.opcode();

        write_buffer[1] = 0x0F; // this is for motor 1
        write_buffer[3] = 0x0F; // this is for motor 2
        write_buffer[5] = 0x0F; // this is for motor 3
        write_buffer[7] = 0x0F; // this is for motor 4
        write_buffer[9] = 0x0F; // this is for dribbler :)
    
        //let mut counter = 1;

        // for i in 0..5 {
        //     // break down each duty cycle into two bytes
        //     write_buffer[counter] = duty_cycles[i].lsb();
        //     counter += 1;
        //     write_buffer[counter] = duty_cycles[i].msb();
        //     counter += 1;
        // }

        // send the ls byte first and then the ms byte
        self.cs.set_low().map_err(FpgaError::CSPin)?;
        self.spi.transfer(&mut write_buffer).map_err(FpgaError::SPI)?;
        self.cs.set_high().map_err(FpgaError::CSPin)?;

        // return status code
        Ok( write_buffer[0] )
    }

    ///
    pub fn set_duty_get_encs(&mut self, duty_cycles: &mut [DutyCycle; 5], encs: &mut [i16; 5]) 
            -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // send READ ENC WRITE VEL instruction
        let mut status: [u8;1] = [Instruction::R_ENC_W_VEL.opcode()];
        self.spi.transfer(&mut status).map_err(FpgaError::SPI)?;

        // send dutycycle bytes and receive encoder bytes
        // NOTE: that duty cycle bytes are send as Lower bytes first and then Upper bytes
        // while, the received encoder values are Upper bytes first, and then Lower bytes 
        for i in 0..5 {
            // break down each duty cycle into two bytes
            let mut first_sent = [duty_cycles[i].lsb()];
            let mut second_sent = [duty_cycles[i].msb()];

            // send the lower duty cycle bytes first and receive the upper encoder bytes
            // send the upper duty cycle bytes second and receive the lower encoder bytes
            self.spi.transfer(&mut first_sent).map_err(FpgaError::SPI)?;
            self.spi.transfer(&mut second_sent).map_err(FpgaError::SPI)?;
            
            // combined the two received bytes into a single i16 encoder value
            encs[i] = ((first_sent[0] << 8) | second_sent[0]) as i16; 
        }

        // convert 
        Ok(status[0])
    }

    ///
    pub fn get_git_hash<D>(&mut self, hash: &mut [u8; 20], delay: &mut D) 
        -> Result<bool, FpgaError<SpiE, PinE>> 
        where D: DelayMs<u8> + DelayUs<u8> 
        {
        
        // the entire git hash is obtained from two different instruction opcodes
        let mut hash_1: [u8;11] = [0;11];
        let mut hash_2: [u8;12] = [0;12];

        hash_1[0] = Instruction::R_HASH_1.opcode();
        hash_2[0] = Instruction::R_HASH_2.opcode();
        
        // Let's execute the first one and store the lower 10 bytes
        // iterate through indices 19 ~ 10
        self.cs.set_low().map_err(FpgaError::CSPin)?;
        self.spi.transfer(&mut hash_1).map_err(FpgaError::SPI)?;
        self.cs.set_high().map_err(FpgaError::CSPin)?;
        
        // git hash 2
        self.cs.set_low().map_err(FpgaError::CSPin)?;
        self.spi.transfer(&mut hash_2).map_err(FpgaError::SPI)?;
        self.cs.set_high().map_err(FpgaError::CSPin)?;       
        // for i in 19..=10 {
        //     // create buffer for byte
        //     let mut temp: [u8;1] = [0x00];

        //     // receive git hash byte
        //     self.cs.set_low().map_err(FpgaError::CSPin)?;
        //     self.spi.transfer(&mut temp).map_err(FpgaError::SPI)?;
        //     self.cs.set_high().map_err(FpgaError::CSPin)?;
        //     delay.delay_us(1);

        //     // store in corresponding index
        //     hash[i] = temp[0];
        // }

        // // Let's receive the second set of bytes corresponding to the upper 10 bytes
        // // iterate through indices 9 ~ 0
        // self.cs.set_low().map_err(FpgaError::CSPin)?;
        // self.spi.write(&hash_2).map_err(FpgaError::SPI)?;
        // self.cs.set_high().map_err(FpgaError::CSPin)?;
        // delay.delay_us(1);

        // for i in 9..=0 {
        //     // create buffer for byte
        //     let mut temp: [u8;1] = [0x00];

        //     // receive git hash byte
        //     self.cs.set_low().map_err(FpgaError::CSPin)?;
        //     self.spi.transfer(&mut temp).map_err(FpgaError::SPI)?;
        //     self.cs.set_high().map_err(FpgaError::CSPin)?;
        //     delay.delay_us(1);

        //     // store in corresponding index
        //     hash[i] = temp[0];
        // }

        // // the last byte contains the fpga firmware dirty bit
        // let mut dirty: [u8;1] = [0x00];
        // self.spi.transfer(&mut dirty).map_err(FpgaError::SPI)?;

        // return whether there's a dirty bit
        Ok(hash_2[11] == 0x01)
    }

    ///
    pub fn gate_drivers(&mut self, gate_status: &mut [u32; 5]) 
        -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // send READ DRV instruction
        let mut status: [u8; 1] = [Instruction::CHECK_DRV.opcode()];
        self.spi.transfer(&mut status).map_err(FpgaError::SPI)?;

        // Each DRV value contains 3 "Nibbles"
        for i in 0..5 {
            // buffer used to store all 3 "Nibbles" and the empty "extra" byte
            let mut empty: [u8;1] = [0x00];
            let mut nib_0:[u8; 1] = [0x00];
            let mut nib_1:[u8; 1] = [0x00];
            let mut nib_2:[u8; 1] = [0x00];

            self.spi.transfer(&mut nib_1).map_err(FpgaError::SPI)?;
            self.spi.transfer(&mut nib_0).map_err(FpgaError::SPI)?;
            self.spi.transfer(&mut empty).map_err(FpgaError::SPI)?;
            self.spi.transfer(&mut nib_2).map_err(FpgaError::SPI)?;

            // combine all 4 bytes to form a single DRV encoded value
            gate_status[i] = ((empty[0] << 24) | (nib_2[0] << 16) | (nib_1[0] << 8) | nib_0[0]) as u32;
        }
        
        Ok(status[0])

    }

    ///
    pub fn motors_en(&mut self, state: bool) 
        -> Result<u8, FpgaError<SpiE, PinE>> {
        
        let mut status: [u8;1] = [0];

        // send either the EN MOTORS or DISABLE MOTORS command based on the passed in state
        match state {
            true => {
                //status[0] = Instruction::EN_MOTORS.opcode();
                status[0] = 0x30;
            } 
            false => {
                //status[0] = Instruction::DIS_MOTORS.opcode();
                status[0] = 0xB0;
            }
        }
        
        // spi transfer
        self.cs.set_low().map_err(FpgaError::CSPin)?;
        self.spi.transfer(&mut status).map_err(FpgaError::SPI)?;
        self.cs.set_high().map_err(FpgaError::CSPin)?;

        // return status code
        Ok(status[0])
    }

    /// 
    pub fn watchdog_reset<D>(&mut self, delay: &mut D) 
        -> Result<(), FpgaError<SpiE, PinE>> 
        where D: DelayMs<u8> + DelayUs<u8> 
        {
        
        // the easiest way to perform a watchdog reset is to toggle the motors
        self.motors_en(false)?;
        delay.delay_us(1);
        self.motors_en(true)?;
        delay.delay_us(1);

        Ok(())
    }

}
