#![no_std]

use core::fmt::Debug;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};

use fugit::{ExtU32};
use rtic_monotonics::systick::Systick;

// IMXRT_HAL Doesn't Implement Input from Embedded Hal
use imxrt_hal::gpio::Input;

mod init_command;
use init_command::FPGA_BYTES;

mod command;
use command::Command;

pub mod error;
use error::{FpgaError, convert_gpio_error, convert_spi_error};

// const FPGA_SPI_FREQUENCY_HZ: u32 = 100_000;
const MAX_DUTY_CYCLES: u16 = 511;

/// In FreeRTOS our vTaskDelay had a period of 1ms
const V_TASK_WAIT_PERIOD: u32 = 1;

// TODO: Check whether or not the spi rate actually needs to be tweaked

pub struct FPGA<GpioE, SpiE, INITB, DONE, CSN, PROGB, SPI>
    where GpioE: Debug,
          SpiE: Debug,
          CSN: OutputPin<Error = GpioE>,
          PROGB: OutputPin<Error = GpioE>,
          SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE> {
    csn: CSN,
    init_b: Input<INITB>,
    prog_b: PROGB,
    done: Input<DONE>,
    spi: SPI,
    is_init: bool,
}

impl<GpioE, SpiE, INITB, DONE, CSN, PROGB, SPI> FPGA<GpioE, SpiE, INITB, DONE, CSN, PROGB, SPI>
    where GpioE: Debug,
    SpiE: Debug,
    CSN: OutputPin<Error = GpioE>,
    PROGB: OutputPin<Error = GpioE>,
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE> {
    pub fn new(spi: SPI, mut csn: CSN, init_b: Input<INITB>, mut prog_b: PROGB, done: Input<DONE>) -> Result<Self, FpgaError<GpioE, SpiE>> {
        //prog_b has PullType None, PinMode OpenDrain, PinSpeed Low, and is not inverted
        // TODO: Check the prog_b has correct PinMode (i.e. Open Drain)

        // Set the correct SPI Frequency
        // spi.set_frequency(FPGA_SPI_FREQUENCY_HZ.Hz()).unwrap();

        prog_b.set_high().map_err(convert_gpio_error)?;

        csn.set_high().map_err(convert_gpio_error)?;

        Ok(Self {
            csn,
            init_b,
            prog_b,
            done,
            spi,
            is_init: false,
        })
    }

    /// Configure FPGA with the binary.  Must be called to initialize the fpga
    /// 
    /// Returns true on successful FPGA Initialization
    pub async fn configure(&mut self) -> Result<(), FpgaError<GpioE, SpiE>> {
        self.prog_b.set_high().map_err(convert_gpio_error)?;
        Systick::delay(V_TASK_WAIT_PERIOD.millis()).await;
        self.prog_b.set_low().map_err(convert_gpio_error)?;

        // Wait for the FPGA to tell us it's ready for the bitstream
        let mut fpga_init = false;
        for _ in 0..100 {
            Systick::delay(10 * V_TASK_WAIT_PERIOD.millis()).await;

            // We're ready to start configuration when init_b goes high
            if self.init_b.is_set() {
                fpga_init = true;
                break;
            }
        }

        if !fpga_init {
            return Err(FpgaError::InitTimeout);
        }

        self.send_config()?;

        let mut config_success = false;
        for _ in 0..1_000 {
            Systick::delay(100 * V_TASK_WAIT_PERIOD.millis()).await;
            if self.done.is_set() {
                config_success = true;
                break;
            }
        }

        if config_success {
            self.is_init = true;
            return Ok(());
        }

        return Err(FpgaError::UnableToSetConfiguration);
    }

    /// Return true if initialized and configured
    pub fn is_ready(&mut self) -> bool {
        self.is_init
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
    pub fn set_duty_get_enc(&mut self, duty_cycles: &[i16; 5], encoder_deltas: &mut [i16; 5]) -> Result<u8, FpgaError<GpioE, SpiE>> {
        // self.spi.set_frequency(400_000u32.Hz()).unwrap();

        let mut status = [0u8];

        for duty in duty_cycles {
            if duty.abs() > MAX_DUTY_CYCLES as i16 {
                return Err(FpgaError::InvalidDutyCycle);
            }
        }

        let duty_bytes = i16s_to_bytes(duty_cycles);
        let mut encoder_bytes = [0u8; 10];

        self.csn.set_low().map_err(convert_gpio_error)?;
        self.safe_write_spi(&[Command::ReadStatusWriteDuty as u8])?;
        self.safe_transfer_spi(&mut status)?;

        for (i, duty_byte) in duty_bytes.iter().enumerate() {
            let mut temp = [0u8];
            self.safe_write_spi(&[*duty_byte])?;
            self.safe_transfer_spi(&mut temp)?;
            encoder_bytes[i] = temp[0];
        }

        self.csn.set_high().map_err(convert_gpio_error)?;

        *encoder_deltas = bytes_to_i16s(&encoder_bytes);

        Ok(status[0])
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
    pub fn set_duty_cycles(&mut self, duty_cycles: &[i16; 5]) -> Result<u8, FpgaError<GpioE, SpiE>> {
        for duty in duty_cycles {
            if *duty as u16 > MAX_DUTY_CYCLES {
                return Err(FpgaError::InvalidDutyCycle);
            }
        }

        let mut status = [0u8];
        let duties = i16s_to_bytes(duty_cycles);

        self.csn.set_low().map_err(convert_gpio_error)?;
        self.safe_write_spi(&[Command::ReadStatusWriteDuty as u8])?;
        self.safe_transfer_spi(&mut status)?;
        self.safe_write_spi(&duties)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        Ok(status[0])
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
    pub fn read_duty_cycles(&mut self, duty_cycles: &mut [i16; 5]) -> Result<u8, FpgaError<GpioE, SpiE>> {
        let mut status = [0u8];
        let mut duty_cycle_bytes = [0u8; 10];

        self.csn.set_low().map_err(convert_gpio_error)?;
        self.safe_write_spi(&[Command::ReadDuty as u8])?;
        self.safe_transfer_spi(&mut status)?;
        self.safe_transfer_spi(&mut duty_cycle_bytes)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        *duty_cycles = bytes_to_i16s(&duty_cycle_bytes);

        Ok(status[0])
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
    pub fn read_encoders(&mut self, encoder_counts: &mut [i16; 5]) -> Result<u8, FpgaError<GpioE, SpiE>> {
        let mut status = [0u8];
        let mut encoder_count_bytes = [0u8; 10];

        self.csn.set_low().map_err(convert_gpio_error)?;
        self.safe_write_spi(&[Command::ReadEncoders as u8])?;
        self.safe_transfer_spi(&mut status)?;
        self.safe_transfer_spi(&mut encoder_count_bytes)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        *encoder_counts = bytes_to_i16s(&encoder_count_bytes);

        Ok(status[0])
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
    pub fn read_halls(&mut self, halls: &mut [u8; 5]) -> Result<u8, FpgaError<GpioE, SpiE>> {
        let mut status = [0u8];

        self.csn.set_low().map_err(convert_gpio_error)?;
        self.safe_write_spi(&[Command::ReadHalls as u8])?;
        self.safe_transfer_spi(&mut status)?;
        self.safe_transfer_spi(&mut halls[..])?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        Ok(status[0])
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
    pub fn set_motors_enabled(&mut self, on: bool) -> Result<(), FpgaError<GpioE, SpiE>> {
        let mut status = [0u8];

        self.csn.set_low().map_err(convert_gpio_error)?;
        self.safe_write_spi(&[Command::EnableDisableMotors as u8 | ((on as u8) << 7)])?;
        self.safe_transfer_spi(&mut status)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        Ok(())
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
    pub fn watchdog_reset(&mut self) -> Result<(), FpgaError<GpioE, SpiE>> {
        self.set_motors_enabled(false)?;
        self.set_motors_enabled(true)
    }

    /// Gets the git hash of the current fpga firmware
    /// 
    /// hash will be overwritten to store the hash into (21 characters)
    /// 
    /// Returns true if the current fpga firmware has been modified and not committed
    pub fn git_hash(&mut self, hash: &mut [u8; 21]) -> Result<(), FpgaError<GpioE, SpiE>> {
        // Store reversed git hash here
        let mut found_hash = [0u8; 21];

        self.csn.set_low().map_err(convert_gpio_error)?;
        self.safe_write_spi(&[Command::ReadHash1 as u8])?;
        self.safe_transfer_spi(&mut found_hash[0..10])?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        self.csn.set_low().map_err(convert_gpio_error)?;
        self.safe_write_spi(&[Command::ReadHash2 as u8])?;
        self.safe_transfer_spi(&mut found_hash[10..])?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        if (found_hash[found_hash.len() - 1] & 0x01) == 0 {
            return Err(FpgaError::InvalidGitHash);
        }

        *hash = found_hash;

        Ok(())
    }

    /// Get information from each of the DRV8303's
    /// 
    /// Status Structures is overwritten with 10 16bit status structures that structured as follows
    /// 
    ///          | nibble 3: | 0        | 0        | 0        | 0        |
    ///          | nibble 2: | GVDD_OV  | FAULT    | GVDD_UV  | PVDD_UV  |
    ///          | nibble 1: | OTSD     | OTW      | FETHA_OC | FETLA_OC |
    ///          | nibble 0: | FETHB_OC | FETLB_OC | FETHC_OC | FETLC_OC |
    pub fn gate_drivers(&mut self, status_structures: &mut [u16; 10]) -> Result<(), FpgaError<GpioE, SpiE>>{
        let mut gate_driver_bytes = [0u8; 20];

        self.csn.set_low().map_err(convert_gpio_error)?;
        self.safe_write_spi(&[Command::CheckDrive as u8])?;
        self.safe_transfer_spi(&mut gate_driver_bytes)?;

        for (i, status) in status_structures.iter_mut().enumerate() {
            *status = gate_driver_bytes[i*2] as u16 | gate_driver_bytes[i*2+1] as u16;
        }

        Ok(())
    }

    /// Sends the config over to the FPGA.
    /// 
    /// It is assumed that the fpga has already been initialized and the spi bus is cleared out
    /// 
    /// Return true  if the config send was successful
    pub fn send_config(&mut self) -> Result<(), FpgaError<GpioE, SpiE>> {
        self.csn.set_low().map_err(convert_gpio_error)?;
        // self.spi.set_frequency(16_000_000u32.Hz()).map_err(convert_spi_error).unwrap();
        self.safe_write_spi(&FPGA_BYTES)?;
        self.csn.set_high().map_err(convert_gpio_error)?;

        Ok(())
    }

    /// Checks that there was no error on spi transmission and set csn high on failure
    fn safe_write_spi(&mut self, data: &[u8]) -> Result<(), FpgaError<GpioE, SpiE>> {
        if let Err(err) = self.spi.write(data) {
            self.csn.set_high().unwrap();
            return Err(convert_spi_error(err));
        }

        Ok(())
    }

    fn safe_transfer_spi(&mut self, data: &mut [u8]) -> Result<(), FpgaError<GpioE, SpiE>> {
        if let Err(err) = self.spi.transfer(data) {
            self.csn.set_high().unwrap();
            return Err(convert_spi_error(err));
        }

        Ok(())
    }
}

// Convert the i16s to a bunch of bytes in LSB order
fn i16s_to_bytes(data: &[i16; 5]) -> [u8; 10] {
    let mut byte_slice = [0u8; 10];

    for (i, d) in data.iter().enumerate() {
        let unsigned_data = to_sign_magnitude::<9>(*d);
        byte_slice[2*i] = (unsigned_data & 0xff) as u8;
        byte_slice[2*i+1] = (unsigned_data >> 8) as u8;
    }

    return byte_slice;
}

fn bytes_to_i16s(data: &[u8; 10]) -> [i16; 5] {
    let mut output_slice = [0i16; 5];

    for i in 0..data.len() {
        let d = (data[i*2] as u16) << 8 | (data[i*2+1] as u16);
        output_slice[i] = from_sign_magnitude::<9>(d);
    }

    output_slice
}

fn to_sign_magnitude<const SIGN_INDEX: usize>(value: i16) -> u16 {
    if value > 0 {
        return (-value as u16) | 1 << SIGN_INDEX;
    } else {
        return value as u16;
    }
}

fn from_sign_magnitude<const SIGN_INDEX: usize>(mut value: u16) -> i16 {
    if value & (1 << SIGN_INDEX) > 0 {
        value ^= 1 << SIGN_INDEX;
        return (value as i16) * -1
    }

    return value.try_into().unwrap();
}