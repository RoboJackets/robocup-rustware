//!
//! Diagnose the robot and print the status of the various
//! devices connected to the Teensy
//! 

#![no_std]
#![no_main]

use teensy4_panic as _;

use core::mem::MaybeUninit;

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

const HEAP_SIZE: usize = 2048;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

use fpga_rs::{FPGA, FPGA_SPI_FREQUENCY, FPGA_SPI_MODE};
use icm42605_driver::IMU;
use rtic_nrf24l01::Radio;
use teensy4_bsp as bsp;

use imxrt_iomuxc::prelude::*;

use teensy4_bsp::hal as hal;
use hal::{
    lpspi::{Lpspi, Pins},
    timer::Blocking,
};

use teensy4_bsp::ral::lpspi::LPSPI3;

use teensy4_bsp::board::{self, PERCLK_FREQUENCY, LPSPI_FREQUENCY};

use embedded_hal::{
    spi::MODE_0,
    blocking::delay::{DelayMs, DelayUs},
};

use robojackets_robocup_rtp::{CONTROL_MESSAGE_SIZE, BASE_STATION_ADDRESS};

use main::{
    BASE_AMPLIFICATION_LEVEL,
    CHANNEL,
    RADIO_ADDRESS,
    GPT_FREQUENCY,
    GPT_CLOCK_SOURCE,
    GPT_DIVIDER,
};

#[bsp::rt::entry]
fn main() -> ! {
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

    let board::Resources {
        mut pins,
        mut gpio1,
        mut gpio2,
        mut gpio3,
        mut gpio4,
        usb,
        lpi2c1,
        lpspi4,
        mut gpt1,
        mut gpt2,
        pit: (pit, _, _, _),
        ..
    } = board::t41(board::instances());

    bsp::LoggingFrontend::default_log().register_usb(usb);

    // Configure Delays
    gpt1.disable();
    gpt1.set_divider(GPT_DIVIDER);
    gpt1.set_clock_source(GPT_CLOCK_SOURCE);
    let mut delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

    gpt2.disable();
    gpt2.set_divider(GPT_DIVIDER);
    gpt2.set_clock_source(GPT_CLOCK_SOURCE);
    let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

    let mut pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit);

    delay1.delay_ms(500u32);

    // Test the imu is connected correctly
    log::info!("Checking the IMU");

    let mut imu_success = true;
    let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
    let imu = IMU::new(i2c, &mut pit_delay);
    if let Ok(mut imu) = imu {
        log::info!("IMU initialized... Testing the IMU");
        if let Ok(gyro_z) = imu.gyro_z() {
            log::info!("Successfully read {} from the IMU Gyro Z", gyro_z);
        } else {
            imu_success = false;
            log::error!("Unable to read Gyro Z from IMU");
        }

        if let Ok(accel_x) = imu.accel_x() {
            log::info!("Successfully read {} from the IMU Acceleration X", accel_x);
        } else {
            imu_success = false;
            log::error!("Unable to read Acceleration X from the IMU");
        }

        if let Ok(accel_y) = imu.accel_y() {
            log::info!("Successfully read {} from the IMU Acceleration Y", accel_y);
        } else {
            imu_success = false;
            log::error!("Unable to read Acceleration Y from the IMU");
        }
    } else {
        imu_success = false;
        log::error!("IMU could not be initialized");
    }

    // Test the Radio is connected correctly
    log::info!("Checking the Radio");

    let shared_spi_pins = Pins {
        pcs0: pins.p38,
        sck: pins.p27,
        sdo: pins.p26,
        sdi: pins.p39,
    };
    let shared_spi_block = unsafe { LPSPI3::instance() };
    let mut shared_spi = Lpspi::new(shared_spi_block, shared_spi_pins);
    shared_spi.disabled(|spi| {
        spi.set_clock_hz(LPSPI_FREQUENCY, 5_000_000u32);
        spi.set_mode(MODE_0);
    });
    let radio_cs = gpio1.output(pins.p14);
    let radio_ce = gpio1.output(pins.p20);

    let mut radio_success = true;
    let mut radio = Radio::new(radio_ce, radio_cs);
    if radio.begin(&mut shared_spi, &mut delay2).is_err() {
        radio_success = false;
        log::error!("Radio could not be initialized");
    } else {
        log::info!("Radio was initialized succesfully");
        radio.set_pa_level(BASE_AMPLIFICATION_LEVEL, &mut shared_spi, &mut delay2);
        radio.set_channel(CHANNEL, &mut shared_spi, &mut delay2);
        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, &mut shared_spi, &mut delay2);
        radio.open_writing_pipe(BASE_STATION_ADDRESS, &mut shared_spi, &mut delay2);
        radio.open_reading_pipe(1, RADIO_ADDRESS, &mut shared_spi, &mut delay2);
        radio.start_listening(&mut shared_spi, &mut delay2);
        radio.stop_listening(&mut shared_spi, &mut delay2);
        log::info!("Radio was able to be successfully configured");
    }

    // Test the FPGA is connected correctly
    log::info!("Checking FPGA");

    let mut spi = board::lpspi(
        lpspi4,
        board::LpspiPins {
            pcs0: pins.p10,
            sck: pins.p13,
            sdo: pins.p11,
            sdi: pins.p12,
        },
        FPGA_SPI_FREQUENCY
    );
    spi.disabled(|spi| spi.set_mode(FPGA_SPI_MODE));

    let fpga_cs = gpio2.output(pins.p9);
    let init_b = gpio4.input(pins.p29);
    let config = Config::zero().set_open_drain(OpenDrain::Enabled);
    configure(&mut pins.p28, config);
    let prog_b = gpio3.output(pins.p28);
    let done = gpio3.input(pins.p30);

    let fpga_success = || {
        if let Ok(mut fpga) = FPGA::new(spi, fpga_cs, init_b, prog_b, done, delay1) {
            log::info!("FPGA was successfully initialzed");

            if fpga.configure().is_err() {
                log::error!("Unable to configure the FPGA");
                return false;
            } else {
                log::info!("FPGA was successfully configured");
            }

            if fpga.motors_en(true).is_err() {
                log::error!("Unable to enable motors");
                return false;
            } else {
                log::info!("Motors were enabled");
            }

            for _ in 0..200 {
                if fpga.set_velocities([1.0, 1.0, 1.0, 1.0], false).is_err() {
                    log::error!("Unable to move motors");
                    return false;
                }

                delay2.delay_us(200u32);
            }

            for _ in 0..800 {
                if let Ok(encoder_velocities) = fpga.set_velocities([1.0, 1.0, 1.0, 1.0], false) {
                    for (i, velocity) in encoder_velocities.iter().enumerate() {
                        if *velocity < 0.5 || *velocity > 1.5 {
                            log::error!("Encoder {} is not reading in range.  It reads {}", i, velocity);
                        }
                    }
                } else {
                    log::error!("Unable to move motors");
                    return false;
                }
            }

            return true;
        } else {
            log::error!("Unable to enable motors");
            return false;
        }
    };

    let fpga_success = fpga_success();

    loop {
        log::info!("IMU Initialization: {}", imu_success);
        log::info!("Radio Initialization: {}", radio_success);
        log::info!("FPGA Initialization: {}", fpga_success);

        delay2.delay_ms(1_000u32);
    }
}