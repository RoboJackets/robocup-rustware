#![no_std]

use core::fmt::Debug;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::DelayUs;

use spi_config::ConfigurableSpi;

mod init_command;
use init_command::FPGA_BYTES;

mod command;
use command::Command;

const FPGA_SPI_FREQUENCY_HZ: u32 = 100_000;
const MAX_DUTY_CYCLES: u16 = 511;

pub struct FPGA<E, CSN, INITB, PROGB, DONE, SPI, DELAY>
    where E: Debug,
          CSN: OutputPin<Error = E>,
          INITB: OutputPin<Error = E>,
          PROGB: OutputPin<Error = E>,
          DONE: OutputPin<Error = E>,
          SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
          DELAY: DelayUs<u32> {
    csn: CSN,
    init_b: INITB,
    prog_b: PROGB,
    done: DONE,
    spi: SPI,
    delay: DELAY,
    is_init: bool,
}

impl<E, CSN, INITB, PROGB, DONE, SPI, DELAY> FPGA<E, CSN, INITB, PROGB, DONE, SPI, DELAY>
    where E: Debug,
    CSN: OutputPin<Error = E>,
    INITB: OutputPin<Error = E>,
    PROGB: OutputPin<Error = E>,
    DONE: OutputPin<Error = E>,
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    DELAY: DelayUs<u32> {
    pub fn new(spi: SPI, csn: CSN, init_b: INITB, prog_b: PROGB, done: DONE, delay: DELAY) -> Self {
        // TODO: Configure SPI To Use FPGA_SPI_FREQ_HZ frequency
        Self {
            csn,
            init_b,
            prog_b,
            done,
            spi,
            delay,
            is_init: false,
        }
    }

    /// Configure FPGA with the binary.  Must be called to initialize the fpga
    /// 
    /// Returns true on successful FPGA Initialization
    pub fn configure(&mut self) -> bool {
        todo!()
    }

    /// Return true if initialized and configured
    pub fn is_ready(&mut self) -> bool {
        todo!()
    }

    /// Sets the duty cycles and reads the encoders for all motors (plus resets the fpga watchdog)
    /// 
    /// duty_cycles (5 element array specifying the duty cycle for the specific motor (1-4 are drive motors and 5 is dribbler))
    /// encoder_deltas (5 element array specifying the encoder counts for the 4 drive motors and delta ticks)
    ///     1-4 => drive motors 1-4
    ///     5 => delta tick
    /// 
    /// Returns The FPGA Status
    ///     FPGA_READY [7] 1 if fpga ready to run (no errors)
    ///     WATCHDOG_TRIGGER [6] 0 if watchdog has not triggered yet
    ///     MOTORS_ENABLED [5] 1 if motors are enabled
    ///     MOTOR_HAS_ERROR [4:0] 1 to indicate error on specific motor
    ///         [4] - Dribbler
    ///         [3] - Drive motor 4
    ///         [2] - Drive motor 3
    ///         [1] - Drive motor 2
    ///         [0] - Drive motor 1
    /// 
    /// Can also return 0x7F if the MAX_DUTY_CYCLE magnitude is exceeded
    pub fn set_duty_get_enc(&mut self, duty_cycles: &[u16; 5], encoder_deltas: &[u16; 5]) -> u8 {
        todo!()
    }

    /// Sets the duty cycles for all motors (plus resets the fpga watchdog)
    /// 
    /// duty_cycles (5 element array specifying the duty cycle for the specific motor)
    ///     1-4 are drive motors
    ///     5 is dribbler motor
    /// 
    /// Returns The FPGA Status
    ///     FPGA_READY [7] 1 if fpga ready to run (no errors)
    ///     WATCHDOG_TRIGGER [6] 0 if watchdog has not triggered yet
    ///     MOTORS_ENABLED [5] 1 if motors are enabled
    ///     MOTOR_HAS_ERROR [4:0] 1 to indicate error on specific motor
    ///         [4] - Dribbler
    ///         [3] - Drive motor 4
    ///         [2] - Drive motor 3
    ///         [1] - Drive motor 2
    ///         [0] - Drive motor 1
    /// 
    /// Can also return 0x7F if the MAX_DUTY_CYCLE magnitude is exceeded
    pub fn set_duty_cycles(&mut self, duty_cycles: &[u16; 5]) -> u8 {
        todo!()
    }

    /// Reads the duty cycles for all motors into the duty_cycles array
    /// 
    /// duty_cycles - array that will hold the duty cycles for the motors
    ///     1-4 drive motors 1-4
    ///     5 dribbler motor
    /// 
    /// Returns The FPGA Status
    ///     FPGA_READY [7] 1 if fpga ready to run (no errors)
    ///     WATCHDOG_TRIGGER [6] 0 if watchdog has not triggered yet
    ///     MOTORS_ENABLED [5] 1 if motors are enabled
    ///     MOTOR_HAS_ERROR [4:0] 1 to indicate error on specific motor
    ///         [4] - Dribbler
    ///         [3] - Drive motor 4
    ///         [2] - Drive motor 3
    ///         [1] - Drive motor 2
    ///         [0] - Drive motor 1
    pub fn read_duty_cycles(&mut self, duty_cycles: &mut [u16; 5]) -> u8 {
        todo!()
    }

    /// Reads the encoders for all motors
    /// 
    /// encoder_counts - 5 element array to write the encoder values to
    ///     1-4 = drive motors 1-4
    ///     5 = dribbler motor
    /// 
    /// Returns The FPGA Status
    ///     FPGA_READY [7] 1 if fpga ready to run (no errors)
    ///     WATCHDOG_TRIGGER [6] 0 if watchdog has not triggered yet
    ///     MOTORS_ENABLED [5] 1 if motors are enabled
    ///     MOTOR_HAS_ERROR [4:0] 1 to indicate error on specific motor
    ///         [4] - Dribbler
    ///         [3] - Drive motor 4
    ///         [2] - Drive motor 3
    ///         [1] - Drive motor 2
    ///         [0] - Drive motor 1
    pub fn read_encoders(&mut self, encoder_counts: &mut [u16; 5]) -> u8 {
        todo!()
    }

    /// Reads the hall count for all motors (similar to encoders)
    /// 
    /// halls - 5 element array to write hall counts to
    ///     1-4 = drive motors 1-4
    ///     5 = dribbler motor
    /// 
    /// Returns The FPGA Status
    ///     FPGA_READY [7] 1 if fpga ready to run (no errors)
    ///     WATCHDOG_TRIGGER [6] 0 if watchdog has not triggered yet
    ///     MOTORS_ENABLED [5] 1 if motors are enabled
    ///     MOTOR_HAS_ERROR [4:0] 1 to indicate error on specific motor
    ///         [4] - Dribbler
    ///         [3] - Drive motor 4
    ///         [2] - Drive motor 3
    ///         [1] - Drive motor 2
    ///         [0] - Drive motor 1
    pub fn read_halls(&mut self, halls: &mut [u16; 5]) -> u8 {
        todo!()
    }

    /// Enables ore disables the motors on the fpga
    /// on->off or off->on resets the watchdog
    /// 
    /// Returns The FPGA Status
    ///     FPGA_READY [7] 1 if fpga ready to run (no errors)
    ///     WATCHDOG_TRIGGER [6] 0 if watchdog has not triggered yet
    ///     MOTORS_ENABLED [5] 1 if motors are enabled
    ///     MOTOR_HAS_ERROR [4:0] 1 to indicate error on specific motor
    ///         [4] - Dribbler
    ///         [3] - Drive motor 4
    ///         [2] - Drive motor 3
    ///         [1] - Drive motor 2
    ///         [0] - Drive motor 1
    pub fn set_motors_enabled(&mut self, on: bool) -> u8 {
        todo!()
    }

    /// Resets the watchdog on the fpga.
    /// If the watchdog is not reset, the motors stop after some time
    /// 
    /// Returns The FPGA Status
    ///     FPGA_READY [7] 1 if fpga ready to run (no errors)
    ///     WATCHDOG_TRIGGER [6] 0 if watchdog has not triggered yet
    ///     MOTORS_ENABLED [5] 1 if motors are enabled
    ///     MOTOR_HAS_ERROR [4:0] 1 to indicate error on specific motor
    ///         [4] - Dribbler
    ///         [3] - Drive motor 4
    ///         [2] - Drive motor 3
    ///         [1] - Drive motor 2
    ///         [0] - Drive motor 1
    pub fn watchdog_reset(&mut self) -> u8 {
        todo!()
    }

    /// Gets the git hash of the current fpga firmware
    /// 
    /// hash will be overwritten to store the hash into (21 characters)
    /// 
    /// Returns true if the current fpga firmware has been modified and not committed
    pub fn git_hash(&mut self, hash: &mut [u8; 21]) -> bool {
        todo!()
    }

    /// Get information from each of the DRV8303's
    /// 
    /// Status Structures is overwritten with 10 16bit status structures that structured as follows
    /// 
    ///          | nibble 3: | 0        | 0        | 0        | 0        |
    ///          | nibble 2: | GVDD_OV  | FAULT    | GVDD_UV  | PVDD_UV  |
    ///          | nibble 1: | OTSD     | OTW      | FETHA_OC | FETLA_OC |
    ///          | nibble 0: | FETHB_OC | FETLB_OC | FETHC_OC | FETLC_OC |
    pub fn gate_drivers(&mut self, status_structures: &mut [u16; 10]) {
        todo!()
    }

    /// Sends the config over to the FPGA.
    /// 
    /// It is assumed that the fpga has already been initialized and the spi bus is cleared out
    /// 
    /// Return true  if the config send was successful
    pub fn send_config(&mut self) -> bool {
        todo!()
    }
}