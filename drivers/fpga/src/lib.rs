#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]

// import instructions & helper/wrapper structs
pub mod structs;
use structs::Instruction;
use structs::DutyCycle;

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

/* 
* imxrt gpio Input struct definition
* This is needed because embedded_hal 0.2 default faetures do not
* support the InputPin trait. This trait is part of the "unproven"
* features of embedded_hal 0.2
*/
use teensy4_bsp as bsp;
use bsp::hal::gpio::Input;


/// enum for fpga status :)
// (IMPORTANT): maybe move into the structs module?
// TODO: do we need this?...could be useful as we can do a quick check before 
// every fpga call to ensure that it is still working
#[derive(Clone, Copy)]
pub enum FpgaStatus {
    NotReady, // haven't been succesfully init
    Standby, // init was done succesfully
}

/// use this constants when configuring the spi :)
/// (IMPORTANT): move into the structs module?
pub const FPGA_SPI_FREQUENCY: u32 = 400_000;
pub const FPGA_SPI_MODE: Mode = spi::Mode{
    polarity: spi::Polarity::IdleLow,
    phase: spi::Phase::CaptureOnFirstTransition,
};

/**
 * Structure that represents an instance on the FPGA hardware.
 * 
 * Holds all of the necessary hardware instances to enable motor control commands.
 * 
 * NOTES: 
 *  - The FPGA takes ownership of the SPI hardware instance i.e. cannot share the SPI
 *  - InputPin is not supported in the default features of embedded-hal 0.2, so we 
 *    define it using the Input struct and the Pin Number instead -> Input<PinNum>
 */
pub struct FPGA<SPI, CS, InitP, PROG, DoneP> {
    spi: SPI,
    cs: CS,
    init_b: Input<InitP>,
    prog_b: PROG,   // prog_b output pin "MUST" BE on OPEN_DRAIN configuration!!
    done: Input<DoneP>,
    status: FpgaStatus,

    /* TODO: 
        [x] check if vTaskDelay() == delay_ms()
        [x] finish implementing reading functions :)
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
    /// instance of this FPGA should exist at any time
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

        // initialize prob_b pin to be high (idle state)
        fpga.prog_b.set_high().map_err(FpgaError::ProgPin)?;

        // initialize cs pin to be high (idle state)
        fpga.cs.set_high().map_err(FpgaError::CSPin)?;

        // returnt the instance
        Ok(fpga)
    }

    /**
    * Configures the FPGA as follows:
    *    1. Toggles the prog_b pin to clear out anything prior
    *    2. Awaits for the FPGA init_b pin
    *    3. Sends config
    *    4. Awaits for the FPGA done pin
    *    5. Returns Ok if no errors or timeout
    *
    * Parameters:
    *    delay: An instance of a blocking delay that implements the DelayUs and
    *           DelayMs embedded traits
    */
    pub fn configure<D>(&mut self, delay: &mut D) 
        -> Result<(), FpgaError<SpiE, PinE>> 
        where 
            D: DelayMs<u8> + DelayUs<u8>
    {
        /* TODO: maybe move this into some Rust resources compilation file??? 
        =================================
        === MAP_ERR BEHIND THE SCENES ===
        =================================

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

        // First we toggle the prog_b pin
        self.prog_b.set_low().map_err(|e| {
            let new_error = FpgaError::<SpiE, PinE>::ProgPin(e);
            new_error
        })?;
        delay.delay_ms(1); // allow for hardware to latch properly
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
        self.spi_write(&FPGA_BYTES)?;
        
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

        // spi transfer buffer
        let mut write_buffer: [u8; 7] = [0x00; 7];
        
        // send READ HALLS instruction through an SPI transfer transaction
        write_buffer[0] = Instruction::R_HALLS.opcode();
        write_buffer[1] = 0x00; // all instructions are appended with a 0x00

        // read the halls into the write_buffer
        self.spi_transfer(&mut write_buffer)?;

        // read write_buffer for each hall
        for i in 0..5 {
            // store the hall value in the corresponding hall buffer
            halls[i] = write_buffer[i + 2];
        }

        // return status code
        Ok(write_buffer[1])
    }

    ///
    pub fn read_encs(&mut self, encs: &mut [i16; 5]) 
        -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // spi transaction buffer
        let mut write_buffer: [u8; 12] = [0x00; 12];

        // send READ ENCODERS instruction through an SPI transfer transaction
        write_buffer[0] = Instruction::R_ENC.opcode();
        write_buffer[1] = 0x00; // always append a 0x00 after the instruction
        
        /*
        * Send 0x00 accordingly to read each motor's encoder values
        * 
        * Each ecnoder value is obtained from two transfer transactions 
        *
        * [opcode | status | enc_1 msByte | enc_1 lsByte | ... | enc_5 msByte | enc_5 lsByte]
        *
        * NOTE: the 5th encoder value encs[4] is the delta time and it has a crazy conversion
        * which is explained below.
        *
        * -----------------------------------------------------------------------
        * -----            DELTA -> DT Explanation                        -------
        * -----------------------------------------------------------------------
        * The time since the last update is derived with the value of
        * WATCHDOG_TIMER_CLK_WIDTH in robocup.v
        *
        * The last encoder reading (5th one) from the FPGA is the watchdog
        * timer's tick since the last SPI transfer.
        *
        * Multiply the received tick count by:
        *     (1/18.432) * 2 * (2^WATCHDOG_TIMER_CLK_WIDTH)
        *
        * This will give you the duration since the last SPI transfer in
        * microseconds (us).
        *
        * For example, if WATCHDOG_TIMER_CLK_WIDTH = 6, here's how you would
        * convert into time assuming the fpga returned a reading of 1265 ticks:
        *     time_in_us = [ 1265 * (1/18.432) * 2 * (2^6) ] = 8784.7us
        *
        * The precision would be in increments of the multiplier. For
        * this example, that is:
        *     time_precision = 6.94us
        *
        * TODO: UPDATE THIS IN RUST FORMAT
        * float dt = static_cast<float>(encDeltas[4]) * (1 / 18.432e6) * 2 * 128;
        */
        self.spi_transfer(&mut write_buffer)?;
        
        // store each encoder value accordingly
        for i in 0..5 {
            // store each byte and convert them into u16 to avoid bit truncation
            let ms_byte: u16 = write_buffer[2 * (i + 1)] as u16;
            let ls_byte: u16 = write_buffer[2 * (i + 1) + 1] as u16;

            // combine the two bytes into a single i16 value
            encs[i] = (ms_byte << 8 | ls_byte) as i16; 
        }
        
        // return status code
        Ok(write_buffer[1])
    }

    ///
    pub fn read_duty_cycles(&mut self, duty_cycles: &mut [i16; 5]) 
        -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // spi trasaction buffer
        let mut write_buffer: [u8; 12] = [0x00; 12];

        // send READ DUTY CYCLES instruction through SPI transfer transaction
        write_buffer[0] = Instruction::R_DUTY.opcode();
        write_buffer[1] = 0x00; // always append 0x00 after instruction

        /*
         * Each duty cycle is obtained from two transfer transactions
         * The first transaction returns the MS Byte and the second transanction
         * returns the LS Byte
         * 
         * [opcode | status | duty_1 msByte | duty_1 lsByte | ... | duty_5 msByte | duty_5 lsByte]
         *  
         * Duty cycles are encoded as a 10-bit signed magnitude
         * 
         * We're using the DutyCycle wrapping struct to hide the bit length conversion
        */
        self.spi_transfer(&mut write_buffer)?;

        // extract the duty cycles accordingly
        for i in 0..5 {
            // a buffer used to store each byte for the two spi transfers
            let ms_byte: u16 = write_buffer[2 * (i + 1)] as u16;
            let ls_byte: u16 = write_buffer[2 * (i + 1) + 1] as u16;

            // stored returned bytes into a duty_cycle wrapper
            let dc = DutyCycle::new((ms_byte << 8) | ls_byte);

            // convert the duty_cycle into an i16 and store in passed in buffer
            duty_cycles[i] = i16::from(dc);
        }
        
        // return status code
        Ok(write_buffer[1])
    }

    ///
    pub fn set_duty_cycles(&mut self, duty_cycles: &mut [DutyCycle; 5]) 
        -> Result<u8, FpgaError<SpiE, PinE>> 
        {

        // init write buffer
        let mut write_buffer: [u8; 12] = [0;12];
        
        // send READ ENC WRITE VEL instruction
        // we'll only write vel and not read any enc information for this function
        write_buffer[0] = Instruction::R_ENC_W_VEL.opcode();
        write_buffer[1] = 0x00; // all instructions are appended a 0x00 byte

        // for each duty cycle in duty_cycle...
        for i in 0..5 {
            // first the lower 8 bits first (lsByte)
            write_buffer[2 * (i + 1)] = duty_cycles[i].lsb();
            // then we send the upper 8 bits (msByte)
            write_buffer[(2 * (i + 1)) + 1] = duty_cycles[i].msb();
        }

        // send the ls byte first and then the ms byte
        self.spi_transfer(&mut write_buffer)?;

        // return status code
        Ok( write_buffer[1] )
    }

    ///
    pub fn set_duty_get_encs(&mut self, duty_cycles: &mut [DutyCycle; 5], encs: &mut [i16; 5]) 
            -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // spi transaction write buffer
        let mut write_buffer: [u8; 12] = [0x00; 12];

        // send READ ENC WRITE VEL instruction
        write_buffer[0] = Instruction::R_ENC_W_VEL.opcode();
        write_buffer[1] = 0x00;

        // for each duty cycle in duty_cycle...
        for i in 0..5 {
            // first the lower 8 bits first (lsByte)
            write_buffer[2 * (i + 1)] = duty_cycles[i].lsb();
            // then we send the upper 8 bits (msByte)
            write_buffer[(2 * (i + 1)) + 1] = duty_cycles[i].msb();
        }

        // NOTE: that duty cycle bytes are send as Lower bytes first and then Upper bytes
        // while, the received encoder values are Upper bytes first, and then Lower bytes 
        self.spi_transfer(&mut write_buffer)?;

        // store each encoder value accordingly
        for i in 0..5 {
            // store each byte and convert them into u16 to avoid bit truncation
            let ms_byte: u16 = write_buffer[2 * (i + 1)] as u16;
            let ls_byte: u16 = write_buffer[2 * (i + 1) + 1] as u16;

            // combine the two bytes into a single i16 value
            encs[i] = (ms_byte << 8 | ls_byte) as i16; 
        }

        // return status code 
        Ok(write_buffer[1])
    }

    /// TODO: Fix get git_hash function
    // pub fn get_git_hash<D>(&mut self, hash: &mut [u8; 20], delay: &mut D) 
    //     -> Result<bool, FpgaError<SpiE, PinE>> 
    //     where D: DelayMs<u8> + DelayUs<u8> 
    //     {
        
    //     // the entire git hash is obtained from two different instruction opcodes
    //     let mut hash_1: [u8;11] = [0;11];
    //     let mut hash_2: [u8;12] = [0;12];

    //     hash_1[0] = Instruction::R_HASH_1.opcode();
    //     hash_2[0] = Instruction::R_HASH_2.opcode();

    //     // return whether there's a dirty bit
    //     Ok(hash_2[11] == 0x01)
    // }

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

    // Private helper method to abstract the SPI write transaction
    fn spi_write(&mut self, buffer: &[u8])
        -> Result<(), FpgaError<SpiE, PinE>> {
            
        // pull cs pin low
        self.cs.set_low().map_err(FpgaError::CSPin)?;    
        // write buffer contents
        self.spi.write(buffer).map_err(FpgaError::SPI)?;
        // pull cs pin back to high (default satate)
        self.cs.set_high().map_err(FpgaError::CSPin)?;

        // return () if spi_write was succesfull
        Ok(())
    }

    // Private helper method to abstract the SPI transfer transaction
    // NOTE: SPI transfer function uses a Full-Duplex protocol
    fn spi_transfer(&mut self, buffer: &mut [u8])
        -> Result<(), FpgaError<SpiE, PinE>> {
            
        // TODO: decide between manually controlled cs and teensy's auto cs

        // pull cs pin low
        self.cs.set_low().map_err(FpgaError::CSPin)?;    
        // write  buffer contents and read from spi
        self.spi.transfer(buffer).map_err(FpgaError::SPI)?;
        // pull cs pin back to high (default satate)
        self.cs.set_high().map_err(FpgaError::CSPin)?;

        // return () if spi_write was succesfull
        Ok(())
    }
}
