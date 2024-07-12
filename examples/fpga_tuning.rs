//!
//! This program tunes values sent to the fpga so that
//! telling the wheels of the fpga to run at x m/s 
//! actually makes the wheels run at x m/s.
//! 
//! All of the motor tuning factors are linear combinations.
//! For example, a possible tuning factor set would be:
//! actual_value = 0.15 * expected_value + 0.2 where
//!     actual_value is the value to be fed to the FPGA and
//!     expected_value is the speed we actually want to move at
//! 

#![no_std]
#![no_main]

use teensy4_panic as _;

use teensy4_bsp::{self as bsp, board, hal::timer::Blocking};
use imxrt_iomuxc::{configure, Config, OpenDrain};

use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use fpga_rs::{FPGA_SPI_FREQUENCY, FPGA_SPI_MODE, FPGA};

use main::{GPT_DIVIDER, GPT_CLOCK_SOURCE, GPT_FREQUENCY};

/// The number of iterations to allow for the wheel to stabilize before beginning
/// the tuning process (in multiples of 200us).
const STABLIZATION_ITERATIONS: i32 = 100;
/// The total number of motion control updates to stay at the specified velocity.
const ITERATIONS: i32 = 1_000;
/// The number of motors to tune
const MOTORS: usize = 4;
/// The number of trials (multiple of 0.25) to try per motor.
/// 
/// A TRIALS value of 8 would mean the motor would be tested at
/// 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, and 2.0 m/s
const TRIALS: usize = 8;

/// Calculate the least squares regression line given a motor error measurement.
/// 
/// Returns (slope, intercept)
fn least_squares(y: &[f32]) -> (f32, f32) {
    let mut x = [0.0; TRIALS];
    for trial in 0..TRIALS {
        x[trial] = 0.25 * (trial as f32) + 0.25;
    }

    let slope = 
        ((TRIALS as f32)
            * y.iter().zip(x.iter()).map(|v| v.0 * v.1).sum::<f32>()
            - x.iter().sum::<f32>() * y.iter().sum::<f32>()) /
        ((TRIALS as f32)
            * x.iter().map(|v| v * v).sum::<f32>()
            - x.iter().sum::<f32>() * x.iter().sum::<f32>());

    let intercept =
        (y.iter().sum::<f32>() - slope * x.iter().sum::<f32>()) /
            (TRIALS as f32);

    (slope, intercept)
}

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        mut pins,
        mut gpio2,
        mut gpio3,
        mut gpio4,
        usb,
        lpspi4,
        mut gpt1,
        mut gpt2,
        ..
    } = board::t41(board::instances());

    // Register usb for logging
    bsp::LoggingFrontend::default_log().register_usb(usb);

    // Initialize the fpga
    gpt1.disable();
    gpt1.set_divider(GPT_DIVIDER);
    gpt1.set_clock_source(GPT_CLOCK_SOURCE);
    let delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

    let mut spi = board::lpspi(
        lpspi4,
        board::LpspiPins {
            pcs0: pins.p10,
            sck: pins.p13,
            sdo: pins.p11,
            sdi: pins.p12,
        },
        FPGA_SPI_FREQUENCY,
    );
    spi.set_mode(FPGA_SPI_MODE);

    let cs = gpio2.output(pins.p9);
    let init_b = gpio4.input(pins.p29);
    let config = Config::zero().set_open_drain(OpenDrain::Enabled);
    configure(&mut pins.p28, config);
    let prog_b = gpio3.output(pins.p28);
    let done = gpio3.input(pins.p30);

    let mut fpga = match FPGA::new(spi, cs, init_b, prog_b, done, delay) {
        Ok(fpga) => fpga,
        Err(_) => panic!("Unable to initialize the FPGA"),
    };

    // Initialize a second delay
    gpt2.disable();
    gpt2.set_divider(GPT_DIVIDER);
    gpt2.set_clock_source(GPT_CLOCK_SOURCE);
    let mut delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

    // Tuning setup
    log::info!("Preparing for tuning...");
    delay.delay_ms(500u32);
    if fpga.configure().is_err() {
        panic!("Unable to configure the fpga");
    }

    if fpga.motors_en(true).is_err() {
        panic!("Unable to enable motors");
    }

    // Begin Tuning
    log::info!("Tuning Motor 1");

    let mut motor_errors = [[0.0; TRIALS]; MOTORS];
    for motor in 0..MOTORS {
        for trial in 0..TRIALS {
            log::info!("Tuning Motor {} at {} m/s", motor, 0.25 * (trial as f32) + 0.25);
            let target_velocity = match motor {
                0 => [0.25 * (trial as f32) + 0.25, 0.0, 0.0, 0.0],
                1 => [0.0, 0.25 * (trial as f32) + 0.25, 0.0, 0.0],
                2 => [0.0, 0.0, 0.25 * (trial as f32) + 0.25, 0.0],
                _ => [0.0, 0.0, 0.0, 0.25 * (trial as f32) + 0.25],
            };

            for i in 0..ITERATIONS {
                if let Ok(encoder_velocities) = fpga.set_velocities(target_velocity, 0.0) {
                    if i > STABLIZATION_ITERATIONS {
                        motor_errors[motor][trial] = 
                            (motor_errors[motor][trial] * (ITERATIONS - STABLIZATION_ITERATIONS) as f32 +
                                encoder_velocities[motor]) /
                            (ITERATIONS - STABLIZATION_ITERATIONS) as f32;
                    }
                }

                delay.delay_us(200u32);
            }

            log::info!(
                "Motor {} had an error of {} m/s at {} m/s",
                motor,
                motor_errors[motor][trial],
                0.25 * (trial as f32) + 0.25,
            );
        }
        delay.delay_ms(100u32);
    }

    // End Tuning

    // Calculate Line of Best Fit through motor error values
    let mut motor_equations = [(0.0, 0.0); MOTORS];
    for motor in 0..MOTORS {
        motor_equations[motor] = least_squares(&motor_errors[motor]);
    }

    loop {
        for motor in 0..MOTORS {
            log::info!("Motor {}: {} * x + {}", motor, motor_equations[motor].0, motor_equations[motor].1);
        }

        delay.delay_ms(1_000u32);
    }
}