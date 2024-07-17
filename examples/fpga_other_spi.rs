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

use embedded_hal::digital::v2::OutputPin;

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

    let mut led = gpio2.output(pins.p13);

    // Configure Delays
    gpt1.disable();
    gpt1.set_divider(GPT_DIVIDER);
    gpt1.set_clock_source(GPT_CLOCK_SOURCE);
    let mut delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

    led.set_high();
    delay1.delay_ms(100u32);

    gpt2.disable();
    gpt2.set_divider(GPT_DIVIDER);
    gpt2.set_clock_source(GPT_CLOCK_SOURCE);
    let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

    led.set_low();
    delay2.delay_ms(100u32);

    let mut pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit);

    let spi_pins = Pins {
        pcs0: pins.p38,
        sck: pins.p27,
        sdo: pins.p26,
        sdi: pins.p39,
    };
    let shared_spi_block = unsafe { LPSPI3::instance() };
    let mut fpga_spi = Lpspi::new(shared_spi_block, spi_pins);
    fpga_spi.disabled(|spi| {
        spi.set_clock_hz(LPSPI_FREQUENCY, 400_000);
        spi.set_mode(MODE_0);
    });

    let fpga_cs = gpio2.output(pins.p9);
    let init_b = gpio4.input(pins.p29);
    let config = Config::zero().set_open_drain(OpenDrain::Enabled);
    configure(&mut pins.p28, config);
    let prog_b = gpio3.output(pins.p28);
    let done = gpio3.input(pins.p30);

    let mut fpga = match FPGA::new(fpga_spi, fpga_cs, init_b, prog_b, done, delay2) {
        Ok(fpga) => fpga,
        Err(err) => panic!("{:?}", err),
    };

    led.set_high();

    match fpga.configure() {
        Ok(_) => log::info!("Fpga Configured"),
        Err(_) => panic!("Fpga was unable to be configured"),
    }
    
    led.set_low();

    match fpga.motors_en(true) {
        Ok(_) => log::info!("Motors enabled"),
        Err(_) => panic!("Unable to enable motors"),
    }

    led.set_high();

    loop {
        led.set_low();
        log::info!("Moving Forwards");
        for _ in 0..100 {
            fpga.set_velocities([1.0, 1.0, 1.0, 1.0], true);
            delay1.delay_us(100u32);
        }

        led.set_high();
        log::info!("Stopping");
        for _ in 0..100 {
            fpga.set_velocities([0.0, 0.0, 0.0, 0.0], false);
            delay1.delay_ms(100u32);
        }
    }
}