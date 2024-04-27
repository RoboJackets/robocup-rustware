//!
//! Collection Collects Data from Basic Movement by the Robot and stores the data
//! in flash memory that can be read via report.rs
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(bigint_helper_methods)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

pub fn from_sign_magnitude(value: u16) -> i16 {
    let mut value = value as i16;
    if (value & (1 << 9)) != 0 {
        value ^= 1 << 9;
        value *= -1;
    }
    value
}

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::convert::Infallible;
    use core::mem::MaybeUninit;

    use imxrt_iomuxc::prelude::*;

    use main::collect::{MotionControlHeader, MotionControlReading};

    use embedded_hal::spi::MODE_0;

    use nalgebra::Vector3;
    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::board::{Lpi2c1, PERCLK_FREQUENCY};

    use teensy4_pins::t41::*;

    use teensy4_bsp::hal as hal;
    use hal::lpspi::{LpspiError, Lpspi, Pins};
    use hal::gpio::Output;
    use hal::gpt::{ClockSource, Gpt1, Gpt2};
    use hal::timer::Blocking;
    use hal::pit::Chained01;
    
    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_monotonics::systick::*;

    use motion::MotionControl;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    use icm42605_driver::IMU;

    use w25q128::StorageModule;

    // Constants
    const GPT_FREQUENCY: u32 = 1_500;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 8192;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 200;

    // Type Definitions
    // FPGA Spi
    type FpgaSpi = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
    type Fpga = FPGA<FpgaSpi, Output<P9>, P29, Output<P28>, P30, Delay1, hal::lpspi::LpspiError, Infallible>;
    // Shared SPI
    type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
    // Delays
    type Delay1 = Blocking<Gpt1, GPT_FREQUENCY>;
    type Delay2 = Blocking<Gpt2, GPT_FREQUENCY>;

    #[local]
    struct Local {
        fpga: Fpga,
        motion_control: MotionControl,
        gpt: Gpt2,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        // Grab the board peripherals
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
            pit: (pit0, pit1, pit2, _pit3),
            ..
        } = board::t41(ctx.device);

        // Setup Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        let chained = Chained01::new(pit0, pit1);

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        gpt2.disable();
        gpt2.set_divider(board::PERCLK_FREQUENCY / 1_000_000);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        gpt2.enable();

        let mut fpga_spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            FPGA_SPI_FREQUENCY,
        );
        fpga_spi.disabled(|spi| {
            spi.set_mode(FPGA_SPI_MODE);
        });

        let cs = gpio2.output(pins.p9);
        let init_b = gpio4.input(pins.p29);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        let done = gpio3.input(pins.p30);

        let mut fpga = match FPGA::new(fpga_spi, cs, init_b, prog_b, done, delay1) {
            Ok(fpga) => fpga,
            Err(_) => panic!("Unable to initialize the FPGA"),
        };

        if fpga.configure().is_err() {
            panic!("Unable to Configure FPGA");
        }

        motion_control_update::spawn().ok();

        (
            Shared {

            },
            Local {
                fpga,
                motion_control: MotionControl::new(),
                gpt: gpt2,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        local = [fpga, motion_control, gpt, initialized: bool = false, last_gpt: u32 = 0],
        shared = [],
        priority=1,
    )]
    async fn motion_control_update(ctx: motion_control_update::Context) {
        if !*ctx.local.initialized {
            if ctx.local.fpga.reset_motors().is_err() {
                panic!("Motor Reset Failed");
            }
        }

        // if !*ctx.local.initialized {
        //     if ctx.local.fpga.configure().is_err() {
        //         panic!("Unable to configure fpga");
        //     }

        //     if ctx.local.fpga.motors_en(true).is_err() {
        //         panic!("Unable to enable motors");
        //     }

        //     *ctx.local.initialized = true;
        // }

        let body_velocity = Vector3::new(0.0, 0.5, 0.0);

        let wheel_velocities = ctx.local.motion_control.body_to_wheels(body_velocity);
    
        let encoder_values = match ctx.local.fpga.set_velocities([1.0, 1.0, 1.0, 1.0], 0.0) {
            Ok(encoder_values) => {
                log::info!("Encoder Values: {:?}", encoder_values);
                encoder_values
            },
            Err(_) => [0.0; 4],
        };
        // let encoder_values = match ctx.local.fpga.set_velocities(wheel_velocities.into(), 0.0) {
        //     Ok(encoder_values) => encoder_values,
        //     Err(_) => [0.0; 4],
        // };

        let now = ctx.local.gpt.count();
        // log::info!("Delta: {}", now - *ctx.local.last_gpt);
        *ctx.local.last_gpt = now;

        if let Ok(drv) = ctx.local.fpga.check_drv() {
            log::info!("DRV: [{:#010b}, {:#010b}, {:#010b}]", drv[0], drv[1], drv[2]);
        }

        log::info!("Fpga Status: {:#010b}", ctx.local.fpga.status);

        // log::info!("{:?}", encoder_values);

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_update::spawn().ok();
    }
}