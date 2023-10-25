#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]

pub mod instructions;
use instructions::Instruction;

pub mod duty_cycle;
use duty_cycle::DutyCycle;

pub mod config_bin;
use config_bin::FPGA_BYTES;

pub mod error;
use error::FpgaError;

pub mod status;
use status::FpgaStatus;

// embedded hal traits
use embedded_hal::digital::v2::OutputPin; 
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::spi::{self, Mode};

/// Input is an unproven feature in the embedded_hal
use teensy4_bsp as bsp;
use bsp::hal::gpio::Input;

/// This Driver is meant to be used with RTIC (Systick Montonics Enabled)
use rtic_monotonics::systick::{Systick, ExtU32};

/// FPGA SPI Configuration Options
pub const FPGA_SPI_FREQUENCY: u32 = 400_000;
pub const FPGA_SPI_MODE: Mode = spi::Mode{
    polarity: spi::Polarity::IdleLow,
    phase: spi::Phase::CaptureOnFirstTransition,
};

/// Driver for the FPGA
/// 
/// Notes:
/// * The FPGA takes ownership of the SPI hardware instance and cannot share the SPI
/// * InputPin is unproven in the embedded-hal so we need to use the device specific hal
///     with Input<PinNum>
/// * prog_b must be OPEN_DRAIN configured
pub struct FPGA<SPI, CS, InitP, PROG, DoneP> {
    spi: SPI,
    cs: CS,
    init_b: Input<InitP>,
    prog_b: PROG,
    done: Input<DoneP>,
    status: FpgaStatus,
}

impl<SPI, CS, InitP, PROG, DoneP, SpiE, PinE> FPGA<SPI, CS, InitP, PROG, DoneP>
    where SPI: Write<u8, Error = SpiE> + Transfer<u8, Error = SpiE>,
          PROG: OutputPin<Error = PinE>,
          CS: OutputPin<Error = PinE>, {
    /// Builds and returns a new instance of the FPGA controller. Only one
    /// instance of this FPGA should exist at any time
    pub fn new(spi: SPI, cs: CS, init_b: Input<InitP>, prog_b: PROG, done: Input<DoneP>) -> Result<Self, FpgaError<SpiE, PinE>> {
        
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

        // return the instance
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
    pub async fn configure(&mut self) -> Result<(), FpgaError<SpiE, PinE>> {
        // First we toggle the prog_b pin
        self.prog_b.set_low().map_err(FpgaError::ProgPin)?;
        Systick::delay(1u32.millis()).await;
        self.prog_b.set_high().map_err(FpgaError::ProgPin)?;

        // delay until init_b is ready
        let mut timeout = 100;
        while timeout > 0 {

            // wait for 10 ms on each cycle
            Systick::delay(10u32.millis()).await;

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
            Systick::delay(100u32.millis()).await;

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

    /// TODO: Debug This
    pub fn read_halls(&mut self, halls: &mut [u8; 5]) -> Result<u8, FpgaError<SpiE, PinE>> {

        // spi transfer buffer
        let mut write_buffer: [u8; 7] = [0x00; 7];
        
        // send READ HALLS instruction through an SPI transfer transaction
        write_buffer[0] = Instruction::ReadHalls.opcode();
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

    /// Read the encoder values from the FPGA
    /// 
    /// Format:
    /// [opcode | status | enc_1 msByte | enc_1 lsByte | ... | enc_5 msByte | enc_5 lsByte ]
    /// 
    /// Note: the 5th encoder value encs[4] is the delta time and has an interesting conversion:
    /// enc_5 * (1/18.432) * 2 * (2^WATCHDOG_TIMER_CLK_WIDTH) = duration since last transmission (ms)
    pub fn read_encoders(&mut self, encs: &mut [i16; 5]) -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // spi transaction buffer
        let mut write_buffer: [u8; 12] = [0x00; 12];

        // send READ ENCODERS instruction through an SPI transfer transaction
        write_buffer[0] = Instruction::ReadEncoders.opcode();

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

    /// Read the Duty Cycles from the FPGA
    /// 
    /// Format:
    /// [opcode | status | duty_1 msByte | duty_1 lsByte | ... | duty_5 msByte | duty_5 lsByte ]
    /// 
    /// Duty cycles are encoded as 10-bit signed magnitude
    pub fn read_duty_cycles(&mut self, duty_cycles: &mut [i16; 5]) -> Result<u8, FpgaError<SpiE, PinE>> {
        // spi transaction buffer
        let mut write_buffer: [u8; 12] = [0x00; 12];

        // send READ DUTY CYCLES instruction through SPI transfer transaction
        write_buffer[0] = Instruction::ReadDuties.opcode();
        write_buffer[1] = 0x00; // always append 0x00 after instruction

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

    /// Set the duty cycles (i.e. velocities of the motors)
    /// 
    /// Note: The transmission must be 12 bytes long
    pub fn set_duty_cycles(&mut self, duty_cycles: &mut [DutyCycle; 5]) -> Result<u8, FpgaError<SpiE, PinE>> {
        // init write buffer
        let mut write_buffer: [u8; 12] = [0x0;12];
        
        //send READ ENC WRITE VEL instruction
        //we'll only write vel and not read any enc information for this function
        write_buffer[0] = Instruction::ReadEncodersWriteVelocity.opcode();
        
        // This loop iterates through each duty_cycle and sets the correspondng bytes of the write buffer
        // However, because of the weird behavior of Dribbler we are currently hardcoding a duty_cycle for it
        // so we don't manually set dribbler's duty_cycle
        for i in 0..4 {
                // first the lower 8 bits first (lsByte)
                write_buffer[(2 * i) + 1] = duty_cycles[i].lsb();
                // then we send the upper 8 bits (msByte)
                write_buffer[(2 * i) + 2] = duty_cycles[i].msb();
            }
            
        //
        // Dribbler?
        // For some unkown reason we need to at least assert the lsb of the higher byte write_buffer[10] = 0x01
        // for any motors to move... No idea why. It might be because of the mechanism where if an invalid duty_cycle is sent         
        // it automatically triggers the watchdog or something like that
        // 
        // I'll dive deeper into the FPGA verilog code as I am very suspiscious about the fact that our verilog sucks and
        // the weird motion might be due to a shitty FPGA verilog lol
        // 
        write_buffer[9] = 0x00;
        write_buffer[10] = 0x01;
            
        // I have NO IDEA why we need this here. Setting duties doesn't work unless we append a 0x00 at the end :)
        write_buffer[11] = 0x00;

        self.spi_transfer(&mut write_buffer)?;

        // return status code
        Ok( write_buffer[0] )
    }

    /// Set the duty values of the FPGA and get the encoder values
    pub fn set_duty_get_encoders(&mut self, duty_cycles: &mut [DutyCycle; 5], encs: &mut [i16; 5]) 
            -> Result<u8, FpgaError<SpiE, PinE>> {
        
        // spi transaction write buffer
        let mut write_buffer: [u8; 12] = [0x00; 12];

        // send READ ENC WRITE VEL instruction
        write_buffer[0] = Instruction::ReadEncodersWriteVelocity.opcode();

        // for each duty cycle in duty_cycle...
        for i in 0..4 {
            // first the lower 8 bits first (lsByte)
            write_buffer[2 * (i + 1)] = duty_cycles[i].lsb();
            // then we send the upper 8 bits (msByte)
            write_buffer[(2 * (i + 1)) + 1] = duty_cycles[i].msb();
        }

        write_buffer[10] = 0x01;

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

    /// TODO: Fix the gate_drivers function
    // pub fn gate_drivers(&mut self, gate_status: &mut [u32; 5]) 
    //     -> Result<u8, FpgaError<SpiE, PinE>> {
        
    //     // send READ DRV instruction
    //     let mut status: [u8; 1] = [Instruction::CHECK_DRV.opcode()];
    //     self.spi.transfer(&mut status).map_err(FpgaError::SPI)?;

    //     // Each DRV value contains 3 "Nibbles"
    //     for i in 0..5 {
    //         // buffer used to store all 3 "Nibbles" and the empty "extra" byte
    //         let mut empty: [u8;1] = [0x00];
    //         let mut nib_0:[u8; 1] = [0x00];
    //         let mut nib_1:[u8; 1] = [0x00];
    //         let mut nib_2:[u8; 1] = [0x00];

    //         self.spi.transfer(&mut nib_1).map_err(FpgaError::SPI)?;
    //         self.spi.transfer(&mut nib_0).map_err(FpgaError::SPI)?;
    //         self.spi.transfer(&mut empty).map_err(FpgaError::SPI)?;
    //         self.spi.transfer(&mut nib_2).map_err(FpgaError::SPI)?;

    //         // combine all 4 bytes to form a single DRV encoded value
    //         gate_status[i] = ((empty[0] << 24) | (nib_2[0] << 16) | (nib_1[0] << 8) | nib_0[0]) as u32;
    //     }
        
    //     Ok(status[0])

    // }

    /// Enable or disable the motors
    pub fn enable_motors(&mut self, on: bool) -> Result<u8, FpgaError<SpiE, PinE>> {
        
        let mut write_buffer: [u8; 1] = [0x00];

        // send either the EN MOTORS or DISABLE MOTORS command based on the passed in state
        match on {
            true => {
                write_buffer[0] = Instruction::ReadEncoders.opcode();
            } 
            false => {
                write_buffer[0] = Instruction::DisableMotors.opcode();
            }
        }
        
        // spi transfer
        self.spi_transfer(&mut write_buffer)?;

        // return status code
        Ok(write_buffer[0])
    }

    /// Reset the Watchdog Timer
    pub async fn watchdog_reset(&mut self) -> Result<(), FpgaError<SpiE, PinE>> {
        // the easiest way to perform a watchdog reset is to toggle the motors
        self.enable_motors(false)?;
        Systick::delay(1u32.millis()).await;
        self.enable_motors(true)?;
        Systick::delay(1u32.millis()).await;

        Ok(())
    }

    // Private helper method to abstract the SPI write transaction
    pub(crate) fn spi_write(&mut self, buffer: &[u8]) -> Result<(), FpgaError<SpiE, PinE>> {
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
    pub(crate) fn spi_transfer(&mut self, buffer: &mut [u8]) -> Result<(), FpgaError<SpiE, PinE>> {
            
        // TODO: decide between manually controlled cs and teensy's auto cs

        // pull cs pin high
        self.cs.set_low().map_err(FpgaError::CSPin)?;    
        // write  buffer contents and read from spi
        self.spi.transfer(buffer).map_err(FpgaError::SPI)?;
        // pull cs pin back to low (default satate)
        self.cs.set_high().map_err(FpgaError::CSPin)?;

        // return () if spi_write was succesfull
        Ok(())
    }
}
