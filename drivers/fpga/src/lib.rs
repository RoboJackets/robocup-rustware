#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]

// import instructions & helper/wrapper structs
pub mod instructions;

use instructions::Instruction;

// import helper/wrapper for DutyCycles
pub mod duty_cycle;
pub use duty_cycle::DutyCycle;

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

/// imxrt gpio Input struct definition
/// This is needed because embedded_hal 0.2 default features do not
/// support the InputPin trait. This trait is part of the "unproven"
/// features of embedded_hal 0.2
use teensy4_bsp as bsp;
use bsp::hal::gpio::Input;

/// use this constants when configuring the spi :)
/// (IMPORTANT): move into the structs module?
pub const FPGA_SPI_FREQUENCY: u32 = 400_000;
pub const FPGA_SPI_MODE: Mode = spi::Mode{
    polarity: spi::Polarity::IdleLow,
    phase: spi::Phase::CaptureOnFirstTransition,
};
pub const GEAR_RATIO: f32 = 20.0;

/// The maximum duty cycle
pub const MAX_DUTY_CYCLE: f32 = 511.0;

/// Wheel Radius (m)
pub const WHEEL_RADIUS: f32 = 0.02786;
/// Duty Cycle to Velocity (m/s) (Tested.  See duty-to-wheel-data <https://docs.google.com/spreadsheets/d/1Y931pXyfOq7iaclSkPCwzVSvX_aOvcgqkb2LaQmUZGI/edit?usp=sharing>)
pub const DUTY_CYCLE_TO_VELOCITY: f32 = 7.37;
/// Velocity (m/s) to Duty Cycle
pub const VELOCITY_TO_DUTY_CYCLES: f32 = 1.0 / DUTY_CYCLE_TO_VELOCITY;

#[inline]
/// To convert a duty cycle to fpga it is multiplied by the max duty cycle and divided by 2, then written to
/// the fpga in the format [sign, (value & 0xFF)].
///
/// I.E.
/// [signof(duty_cycle * MAX_DUTY_CYCLE / 2.0), (duty_cycle * MAX_DUTY_CYCLE / 2.0) & 0xFF]
///
/// Return (low_byte, high_byte)
pub fn duty_cycle_to_fpga(duty_cycle: f32) -> (u8, u8) {
    let mut duty = (duty_cycle * MAX_DUTY_CYCLE) as i16;
    
    let mut negative = false;
    if duty < 0 {
        negative = true;
        duty *= -1;
    }

    if duty > 511 {
        duty = 511;
    }

    let high = ((duty >> 8) & 1) as u8;
    let low = (duty & 0xFF) as u8;

    match negative {
        true => (low, 0b10 | high),
        false => (low, high),
    }
}

#[inline]
pub fn duty_cycles_to_fpga(duty_cycles: [f32; 4], write_buffer: &mut [u8]) {
    for (i, duty_cycle) in duty_cycles.iter().enumerate() {
        (write_buffer[2*i], write_buffer[(2*i)+1]) = duty_cycle_to_fpga(*duty_cycle);
    }
}

#[inline]
pub fn buffer_to_i16s(buffer: &[u8]) -> [i16; 5] {
    let mut encoder_buffer = [0i16; 5];

    for i in 0..5 {
        encoder_buffer[i] = i16::from_be_bytes([buffer[2*i], buffer[(2*i)+1]]);
    }

    encoder_buffer
}


 /// 
 /// Structure that represents an instance on the FPGA hardware.
 /// 
 /// Holds all of the necessary hardware instances to enable motor control commands.
 /// 
 /// NOTES: 
 ///  - The FPGA takes ownership of the SPI hardware instance i.e. cannot share the SPI
 ///  - InputPin is not supported in the default features of embedded-hal 0.2, so we 
 ///    define it using the Input struct and the Pin Number instead -> Input<PinNum>
 /// 
pub struct FPGA<SPI, CS, INIT, PROG, DONE, DELAY, SPIE, GPIOE> where
    SPI: Write<u8, Error=SPIE> + Transfer<u8, Error=SPIE>,
    PROG: OutputPin<Error=GPIOE>,
    CS: OutputPin<Error=GPIOE>,
    DELAY: DelayMs<u32> + DelayUs<u32>
{
    spi: SPI,
    cs: CS,
    init_b: Input<INIT>,
    prog_b: PROG,   // prog_b output pin "MUST" BE on OPEN_DRAIN configuration!!
    done: Input<DONE>,
    delay: DELAY,
    // The status of the FPGA
    pub status: u8,
    // When Done is pulled true, this is set true
    initialized: bool,
}

/// 
/// NOTE: Notice the name used in this impl for the Error types. I intentionally
/// used different names from our custom FpgaError enum defined in the error.rs
/// to outline how we are using our custom error to wrap embedded trait error types
///
impl<SPI, CS, INIT, PROG, DONE, DELAY, SPIE, GPIOE> FPGA<SPI, CS, INIT, PROG, DONE, DELAY, SPIE, GPIOE>
    where 
        SPI: Write<u8, Error = SPIE> + Transfer<u8, Error = SPIE>,
        PROG: OutputPin<Error = GPIOE>,
        CS: OutputPin<Error = GPIOE>,
        DELAY: DelayMs<u32> + DelayUs<u32>,
{
    /// Create an uninitialized FPGA driver.
    pub fn new(spi: SPI, cs: CS, init_b: Input<INIT>, prog_b: PROG, done: Input<DONE>, delay: DELAY) 
        -> Result<Self, FpgaError<SPIE, GPIOE>> {
        // create new instance of FPGA and pass in appropriate pins
        let mut fpga = FPGA {
            spi,
            cs,
            init_b,
            prog_b,
            done,
            delay,
            status: 0u8,
            initialized: false,
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
    pub fn configure(&mut self) -> Result<(), FpgaError<SPIE, GPIOE>>  {
        // toggle the prog_b pin
        self.prog_b.set_low().map_err(|e| {
            let new_error = FpgaError::<SPIE, GPIOE>::ProgPin(e);
            new_error
        })?;
        self.delay.delay_ms(1); // allow for hardware to latch properly
        self.prog_b.set_high().map_err(FpgaError::ProgPin)?;

        // delay until init_b is ready
        let mut timeout = 100;
        while timeout > 0 {

            // wait for 10 ms on each cycle
            self.delay.delay_ms(10);

            // if the init_b pin is high => FPGA is ready to be programmed
            if self.init_b.is_set() {
                break;
            }            

            // decrement remaining timeout counter
            timeout -= 1;
            
            // check whether timeout has been reached (i.e. 100 wait cycles)
            if timeout == 0 {
                // return a timeout error with code 0x1
                return Err(FpgaError::<SPIE, GPIOE>::FPGATimeout(0x1));
            }
        }

        // send configuration array after init_b pin is asserted
        self.spi_write(&FPGA_BYTES)?;
        
        timeout = 100;
        // delay until done pin is ready
        while timeout > 0 {
            
            // wait for 100 ms on each cycle
            self.delay.delay_ms(100);

            // if done pin is high => FPGA is done configuring
            // update status to Standby
            if self.done.is_set() {
                self.initialized = true;
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

    /// True if the fpga has been initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Reads the status field of the FPGA instance
    pub fn status(&mut self) -> u8 {
        self.status
    }

    /// Read Hall Sensor values from the FPGA
    pub fn read_halls(&mut self) -> Result<[u8; 5], FpgaError<SPIE, GPIOE>> {
        let mut write_buffer = [0u8; 7];
        write_buffer[0] = Instruction::R_HALLS.opcode();

        self.spi_transfer(&mut write_buffer)?;

        let mut halls = [0u8; 5];
        halls[..].copy_from_slice(&write_buffer[1..6]);

        Ok(halls)
    }

    /// Read the encoder values for the motors connected to the FPGA.
    pub fn read_encoders(&mut self) -> Result<[i16; 5], FpgaError<SPIE, GPIOE>> {
        let mut write_buffer = [0u8; 12];
        write_buffer[0] = Instruction::R_ENC.opcode();

        self.spi_transfer(&mut write_buffer)?;

        Ok(buffer_to_i16s(&write_buffer[1..11]))
    }

    // Send 0x00 accordingly to read each motor's encoder values
    // 
    // Each ecnoder value is obtained from two transfer transactions 
    // 
    // [opcode | status | enc_1 msByte | enc_1 lsByte | ... | enc_5 msByte | enc_5 lsByte]
    // 
    // NOTE: the 5th encoder value encs[4] is the delta time and it has a crazy conversion
    // which is explained below.
    // 
    // -----------------------------------------------------------------------
    // -----            DELTA -> DT Explanation                        -------
    // -----------------------------------------------------------------------
    // The time since the last update is derived with the value of
    // WATCHDOG_TIMER_CLK_WIDTH in robocup.v
    // 
    // The last encoder reading (5th one) from the FPGA is the watchdog
    // timer's tick since the last SPI transfer.
    // 
    // Multiply the received tick count by:
    //     (1/18.432) * 2 * (2^WATCHDOG_TIMER_CLK_WIDTH)
    // 
    // This will give you the duration since the last SPI transfer in
    // microseconds (us).
    // 
    // For example, if WATCHDOG_TIMER_CLK_WIDTH = 6, here's how you would
    // convert into time assuming the fpga returned a reading of 1265 ticks:
    //     time_in_us = [ 1265 * (1/18.432) * 2 * (2^6) ] = 8784.7us
    // 
    // The precision would be in increments of the multiplier. For
    // this example, that is:
    //     time_precision = 6.94us
    // 
    // TODO: UPDATE THIS IN RUST FORMAT
    // float dt = static_cast<float>(encDeltas[4]) * (1 / 18.432e6) * 2 * 128;

    /// Read the duty cycles the FPGA is currently running at
    pub fn read_duty_cycles(&mut self) -> Result<[i16; 5], FpgaError<SPIE, GPIOE>> {
        let mut write_buffer = [0u8; 12];
        write_buffer[0] = Instruction::R_DUTY.opcode();

        self.spi_transfer(&mut write_buffer)?;

        Ok(buffer_to_i16s(&write_buffer[1..11]))
    }

    /// Set the duty cycles for the FPGA motors
    pub fn set_duty_cycles(
        &mut self,
        duty_cycles: [f32; 4],
        _dribbler_duty_cycle: f32,
    ) -> Result<(), FpgaError<SPIE, GPIOE>> {
        let mut write_buffer = [0u8; 12];
        write_buffer[0] = Instruction::R_ENC_W_VEL.opcode();

        duty_cycles_to_fpga(duty_cycles, &mut write_buffer[1..9]);
        
        // TODO: Set Dribblers
        // duty_cycle_to_fpga(dribbler_duty_cycle, &mut write_buffer[9..11])
        write_buffer[10] = 0x00;
        write_buffer[11] = 0x01;

        self.spi_transfer(&mut write_buffer)?;

        Ok(())
    }

    /// Set the duty cycles for the various motors of the FPGA.
    // pub fn set_duty_cycles(&mut self, duty_cycles: &mut [DutyCycle; 5]) 
    //     -> Result<u8, FpgaError<SpiE, PinE>> 
    //     {

    //     // init write buffer
    //     let mut write_buffer = [0x0u8; 11];

        
    //     //send READ ENC WRITE VEL instruction
    //     //we'll only write vel and not read any enc information for this function
    //     write_buffer[0] = Instruction::R_ENC_W_VEL.opcode();
        
    //     // This loop iterates through each duty_cycle and sets the correspondng bytes of the write buffer
    //     // However, because of the weird behavior of Dribbler we are currently hardcoding a duty_cycle for it
    //     // so we don't manually set dribbler's duty_cycle
    //     for i in 0..4 {
    //         // first the lower 8 bits first (lsByte)
    //         write_buffer[(2 * i) + 1] = duty_cycles[i].lsb();
    //         // then we send the upper 8 bits (msByte)
    //         write_buffer[(2 * i) + 2] = duty_cycles[i].msb();
    //     }

    //     //
    //     // Dribbler?
    //     // For some unkown reason we need to at least assert the lsb of the higher byte write_buffer[10] = 0x01
    //     // for any motors to move... No idea why. It might be because of the mechanism where if an invalid duty_cycle is sent         
    //     // it automatically triggers the watchdog or something like that
    //     // 
    //     // I'll dive deeper into the FPGA verilog code as I am very suspiscious about the fact that our verilog sucks and
    //     // the weird motion might be due to a shitty FPGA verilog lol
    //     // 
    //     write_buffer[9] = 0x00;
    //     write_buffer[10] = 0x01;
            
    //     // I have NO IDEA why we need this here. Setting duties doesn't work unless we append a 0x00 at the end :)
    //     // write_buffer[11] = 0x00;
    //     self.spi_transfer(&mut [write_buffer[0]])?;
    //     self.spi_transfer(&mut write_buffer[1..])?;

    //     // this is the actual SPI transfer what writes the duty_cycles to the FPGA
    //     // self.spi_transfer(&mut write_buffer)?;        

    //     // return status code
    //     Ok( write_buffer[0] )
    // }

    /// Set the velocity of the motors and get the velocities the motors
    /// were spinning at (based on the encoder deltas)
    /// 
    /// Params:
    ///     wheel_velocities: The Velocity to spin each motor at
    ///         ( v1,    v2,    v3,    4)
    ///         ((m/s), (m/s), (m/s), (m/s))
    ///     dribbler_velocity: The velocity to spin the dribbler at (m/s)
    /// 
    /// Return:
    ///     Wheel Velocities: The Velocity (according to the encoders) the motors
    ///         are spinning at:
    ///         ( v1,    v2,    v3,    v4)
    ///         ((m/s), (m/s), (m/s), (m/s))
    pub fn set_velocities(
        &mut self,
        mut wheel_velocities: [f32; 4],
        dribble: bool,
    ) -> Result<[f32; 4], FpgaError<SPIE, GPIOE>> {
        let mut write_buffer = [0u8; 12];
        write_buffer[0] = Instruction::R_ENC_W_VEL.opcode();

        for i in 0..wheel_velocities.len() {
            wheel_velocities[i] *= VELOCITY_TO_DUTY_CYCLES;
        }

        duty_cycles_to_fpga(wheel_velocities, &mut write_buffer[1..9]);

        // If the last bit is 1 the dribbler goes (however this byte
        // seems to need to be not zero)
        if dribble {
            write_buffer[10] = 0b1111_1111;
        } else {
            write_buffer[10] = 0b1111_1110;
        }
        // write_buffer[10] = 0b1111_1111;
        write_buffer[11] = 0x00;

        self.spi_transfer(&mut write_buffer[..])?;

        let delta = (i16::from_be_bytes(write_buffer[9..11].try_into().unwrap()) as f32) * (1.0 / 18.432) * 256.0;
        let mut delta_encoders = [
            (i16::from_be_bytes(write_buffer[1..3].try_into().unwrap()) as f32),
            (i16::from_be_bytes(write_buffer[3..5].try_into().unwrap()) as f32),
            (i16::from_be_bytes(write_buffer[5..7].try_into().unwrap()) as f32),
            (i16::from_be_bytes(write_buffer[7..9].try_into().unwrap()) as f32),
        ];
        for i in 0..4 {
            delta_encoders[i] *= 1e6 * 2.0 * core::f32::consts::PI * WHEEL_RADIUS / (6144.0 * delta);
        }
        self.status = write_buffer[0];
        Ok(delta_encoders)
    }

    pub fn set_duty_get_encoders(
        &mut self,
        wheel_duty_cycles: [f32; 4],
        _dribbler_duty_cycle: f32,
    ) -> Result<([f32; 4], f32), FpgaError<SPIE, GPIOE>> {
        let mut write_buffer = [0u8; 12];
        write_buffer[0] = Instruction::R_ENC_W_VEL.opcode();

        duty_cycles_to_fpga(wheel_duty_cycles, &mut write_buffer[1..9]);

        // TODO: Write dribbler to write_buffer[9] and write_buffer[10]
        // duty_cycle_to_fpga(dribbler_duty_cycle, &mut write_buffer[9..11]);
        write_buffer[10] = 0x01;
        write_buffer[11] = 0x00;

        self.spi_transfer(&mut write_buffer[..])?;

        let delta = (i16::from_be_bytes(write_buffer[9..11].try_into().unwrap()) as f32) * (1.0 / 18.432) * 256.0;
        let delta_encoders = [
            (i16::from_be_bytes(write_buffer[1..3].try_into().unwrap()) as f32),
            (i16::from_be_bytes(write_buffer[3..5].try_into().unwrap()) as f32),
            (i16::from_be_bytes(write_buffer[5..7].try_into().unwrap()) as f32),
            (i16::from_be_bytes(write_buffer[7..9].try_into().unwrap()) as f32),
        ];
        Ok((delta_encoders, delta))
    }

    pub fn check_drv(&mut self) -> Result<[u8; 3], FpgaError<SPIE, GPIOE>> {
        let mut write_buffer = [0x96, 0x00, 0x00, 0x00, 0x00];
        self.spi_transfer(&mut write_buffer)?;

        self.status = write_buffer[0];

        Ok([write_buffer[1], write_buffer[2], write_buffer[4]])
    }

    /// Set the duty cycles and read the current encoder values from the FPGA
    // pub fn set_duty_get_encs(
    //     &mut self,
    //     duty_cycles: &mut [DutyCycle; 5],
    //     encoders: &mut [i16; 5]
    // ) -> Result<u8, FpgaError<SpiE, PinE>> {

    //     let mut write_buffer = [0u8; 12];

    //     write_buffer[0] = Instruction::R_ENC_W_VEL.opcode();

    //     for i in 0..4 {
    //         write_buffer[(2 * i) + 1] = duty_cycles[i].lsb();
    //         write_buffer[(2 * i) + 2] = duty_cycles[i].msb();
    //     }

    //     write_buffer[9] = 0x00;
    //     write_buffer[10] = 0x01;

    //     self.spi_transfer(&mut write_buffer[..])?;

    //     for i in 0..4 {
    //         // Encoders are in form (msb, lsb)
    //         encoders[i] = (((write_buffer[(2 * i) + 1]) as i16) << 8) | (write_buffer[(2 * i) + 2] as i16);
    //     }

    //     Ok(write_buffer[0])
    // }

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

    /// Enable the motors connected to the FPGA
    pub fn motors_en(&mut self, enable: bool) -> Result<u8, FpgaError<SPIE, GPIOE>> {
        let mut write_buffer: [u8; 1] = [0x00];
        write_buffer[0] = if enable { Instruction::EN_MOTORS.opcode() }
                            else { Instruction::DIS_MOTORS.opcode() };
        
        self.spi_transfer(&mut write_buffer)?;
        Ok(self.status)
    }

    pub fn reset_motors(&mut self) -> Result<u8, FpgaError<SPIE, GPIOE>> {
        // Set Motors High -> Low -> High
        let mut buffer = [0x30 | 1 << 7];
        self.spi_write(&buffer)?;
        buffer[0] = 0x30;
        self.spi_write(&buffer)?;
        buffer[0] = 0x30 | 1 << 7;
        self.spi_transfer(&mut buffer)?;
        Ok(buffer[0])
    }

    /// Reset the watchdog on the FPGA.
    pub fn watchdog_reset(&mut self) -> Result<(), FpgaError<SPIE, GPIOE>> {
        let _ = self.motors_en(true)?;
        self.delay.delay_us(1);
        let _ = self.motors_en(true)?;
        self.delay.delay_us(1);

        Ok(())
    }

    // Private helper method to abstract the SPI write transaction
    fn spi_write(&mut self, buffer: &[u8]) -> Result<(), FpgaError<SPIE, GPIOE>> {
        // pull cs pin low
        self.cs.set_low().map_err(FpgaError::CSPin)?;    
        // write buffer contents
        self.spi.write(buffer).map_err(FpgaError::SPI)?;
        // pull cs pin back to high (default satate)
        self.cs.set_high().map_err(FpgaError::CSPin)?;

        self.delay.delay_us(1);
        Ok(())
    }

    // Private helper method to abstract the SPI transfer transaction
    // NOTE: SPI transfer function uses a Full-Duplex protocol
    fn spi_transfer(&mut self, buffer: &mut [u8]) -> Result<(), FpgaError<SPIE, GPIOE>> {
        // pull cs pin high
        self.cs.set_low().map_err(FpgaError::CSPin)?;    
        // write buffer contents and read from spi
        self.spi.transfer(buffer).map_err(FpgaError::SPI)?;
        // pull cs pin back to low (default state)
        self.cs.set_high().map_err(FpgaError::CSPin)?;

        self.delay.delay_us(1);

        // Set status from buffer[0]
        self.status = buffer[0];
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn test_duty_to_fpga() {
        let (low, high) = duty_cycle_to_fpga(0.247);

        assert_eq!(low, 126);
        assert_eq!(high, 0b00);
    }

    #[test]
    fn test_duty_to_fpga_negative() {
        let (low, high) = duty_cycle_to_fpga(-0.247);

        assert_eq!(high, 0b10);
        assert_eq!(low, 126);
    }

    #[test]
    fn test_duty_to_fpga_over_one() {
        let (low, high) = duty_cycle_to_fpga(1.5);

        assert_eq!(high, 0b01);
        assert_eq!(low, 255);
    }

    #[test]
    fn test_duty_to_fpga_less_than_negative_one() {
        let (low, high) = duty_cycle_to_fpga(-1.5);

        assert_eq!(high, 0b11);
        assert_eq!(low, 255);
    }

    #[test]
    fn test_duties_to_fpga() {
        let duty_cycles = [0.247, 0.247, 0.247, 0.247];
        let mut buffer = [0u8; 8];
        duty_cycles_to_fpga(duty_cycles, &mut buffer);

        for i in 0..4 {
            assert_eq!(buffer[2*i], 126);
            assert_eq!(buffer[(2*i)+1], 0);
        }
    }

    #[test]
    fn test_duties_to_fpga_negative() {
        let duty_cycles = [-0.247, -0.247, -0.247, -0.247];
        let mut buffer = [0u8; 8];
        duty_cycles_to_fpga(duty_cycles, &mut buffer);

        for i in 0..4 {
            assert_eq!(buffer[2*i], 126);
            assert_eq!(buffer[(2*i)+1], 0b10);
        }
    }

    #[test]
    fn test_buffer_to_i16s_positive() {
        let buffer = [0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF];
        let i16s = buffer_to_i16s(&buffer);
        let expected = [0x0AFF, 0x0AFF, 0x0AFF, 0x0AFF, 0x0AFF];
        assert_eq!(i16s, expected);
    }

    #[test]
    fn test_buffer_to_i16s_negative() {
        let buffer = [0xFA, 0x0A, 0xFA, 0x0A, 0xFA, 0x0A, 0xFA, 0x0A, 0xFA, 0x0A];
        let i16s = buffer_to_i16s(&buffer);
        let expected = [-1526, -1526, -1526, -1526, -1526];
        assert_eq!(i16s, expected);
    }
    
    #[test]
    fn test_create_buffer() {
        let duty_cycles = [
            0.012433962,
            0.010152288,
            -0.010152288,
            -0.012433962,
        ];
        let mut buffer = [0u8; 8];
        duty_cycles_to_fpga(duty_cycles, &mut buffer);

        assert_eq!(buffer, [6, 0, 5, 0, 5, 2, 6, 2]);
    }

    #[test]
    fn test_create_buffer_2() {
        let duty_cycles = [
            0.024867924,
            0.020402576,
            -0.020402576,
            -0.024867924,
        ];
        let mut buffer = [0u8; 8];
        duty_cycles_to_fpga(duty_cycles, &mut buffer);

        assert_eq!(buffer, [12, 0, 10, 0, 10, 2, 12, 2]);
    }
}