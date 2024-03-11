//!
//! Use the AVR910 ISP Programmer to program the kicker board.
//! 
//! Note: This program is not an RTIC program to make compilation more
//! direct.
//! 

#![no_std]
#![no_main]

use teensy4_panic as _;

use embedded_hal::spi::MODE_0;
use embedded_hal::blocking::delay::DelayMs;

use teensy4_bsp as bsp;
use teensy4_bsp::board;
use teensy4_bsp::board::LPSPI_FREQUENCY;

use bsp::hal as hal;
use hal::gpt::ClockSource;
use hal::timer::Blocking;

use bsp::ral as ral;
use ral::lpspi::LPSPI3;

use kicker_programmer::{KickerProgrammer, KickerProgramError};

const GPT_FREQUENCY: u32 = 1_000;
const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

const START_TASK_DELAY_MS: u32 = 2_000;

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        usb,
        mut gpt1,
        mut gpt2,
        ..
    } = board::t41(board::instances());

    // usb logging setup
    bsp::LoggingFrontend::default_log().register_usb(usb);

    // gpt1 as blocking delay
    gpt1.disable();
    gpt1.set_divider(GPT_DIVIDER);
    gpt1.set_clock_source(GPT_CLOCK_SOURCE);
    let delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

    // gpt2 as blocking delay
    gpt2.disable();
    gpt2.set_divider(GPT_DIVIDER);
    gpt2.set_clock_source(GPT_CLOCK_SOURCE);
    let mut blocking_delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

    let spi_pins = hal::lpspi::Pins {
        pcs0: pins.p38,
        sck: pins.p27,
        sdo: pins.p26,
        sdi: pins.p39,
    };
    let spi = unsafe { LPSPI3::instance() };
    let mut spi = hal::lpspi::Lpspi::new(spi, spi_pins);
    spi.disabled(|spi| {
        spi.set_clock_hz(LPSPI_FREQUENCY, 100_000u32);
        spi.set_mode(MODE_0);
    });

    let kicker_csn = gpio2.output(pins.p34);
    let kicker_reset = gpio2.output(pins.p35);

    blocking_delay.delay_ms(START_TASK_DELAY_MS);

    let mut kicker_programmer = KickerProgrammer::new(kicker_csn, kicker_reset, spi, delay);

    match kicker_programmer.init() {
        Ok(_) => log::info!("Kicker Programmer Initialized"),
        Err(err) => match err {
            KickerProgramError::UnableToEnableProgramming => panic!("Unable to Enable Programming"),
            _ => panic!("Unknown Error Occurred"),
        }
    }

    match kicker_programmer.check_vendor_code() {
        Ok(_) => log::info!("Correct Vendor Code"),
        Err(err) => match err {
            KickerProgramError::InvalidVendorId { expected, found } => panic!("Invalid Vendor Id. Expected {}, Found {}", expected, found),
            _ => panic!("Unknown Error Occurred"),
        }
    }

    match kicker_programmer.check_part_family() {
        Ok(_) => log::info!("Correct Part Family"),
        Err(err) => match err {
            KickerProgramError::InvalidPartFamily { expected, found } => panic!("Invalid Part Family. Expected {}, Found {}", expected, found),
            _ => panic!("Unknown Error Occurred"),
        }
    }

    match kicker_programmer.check_part_number() {
        Ok(_) => log::info!("Correct Part Number"),
        Err(err) => match err {
            KickerProgramError::InvalidDeviceId { expected, found } => panic!("Invalid Device Id Found. Expected {} Found {}", expected, found),
            _ => panic!("Unknown Error Occurred"),
        }
    }

    let updated = match kicker_programmer.programs_different() {
        Ok(different) => {
            if different {
                match kicker_programmer.program() {
                    Ok(_) => true,
                    Err(err) => match err {
                        KickerProgramError::InvalidBinarySize { max, found } => panic!("Invalid Kicker Binary Size. Max {}, Found {}", max, found),
                        _ => panic!("Unknown Error Occurred Programming the Kicker"),
                    }
                }
            } else {
                log::info!("Firmware is up to date");
                false
            }
        },
        Err(_err) => panic!("Unable to Check Differences in Kicker Firmware"),
    };

    loop {
        if updated {
            log::info!("Kicker Firmware has been updated");
        } else {
            log::info!("Kicker Firmware is up to date");
        }

        blocking_delay.delay_ms(1_000u32);
    }
}