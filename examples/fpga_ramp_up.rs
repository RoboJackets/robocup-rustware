//!
//! I have a theory the FPGA has issues moving the motors befause the target velocity is too far from
//! the current velocity.  If that is the case, this example should run close to perfectly.
//! 
//! This example ramps up the target speed sent to the FPGA reaching full speed over a set number of iterations
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

/// The number of iterations (~1ms) to reach the target speed
const MAX_SPEED_ITERATIONS: u32 = 100;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::convert::Infallible;
    use core::mem::MaybeUninit;

    use embedded_hal::blocking::delay::DelayMs;

    use fpga_rs::error::FpgaError;
    use imxrt_hal::lpspi::LpspiError;
    use imxrt_iomuxc::prelude::*;

    use nalgebra::{Vector3, Vector4};
    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::board::PERCLK_FREQUENCY;

    use teensy4_bsp::hal as hal;
    use hal::lpi2c::ControllerStatus;
    use hal::gpio::Trigger;
    use hal::timer::Blocking;
    use hal::pit::Chained01;

    use rtic_monotonics::systick::*;

    use robojackets_robocup_rtp::ControlMessage;

    use motion::MotionControl;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    use icm42605_driver::{IMU, ImuError};

    use main::{
        Fpga, Imu, PitDelay, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 200;

    #[local]
    struct Local {
        motion_controller: MotionControl,
        last_encoders: Vector4<f32>,
        chain_timer: Chained01,
    }

    #[shared]
    struct Shared {
        fpga: Fpga,
        imu: Imu,
        pit_delay: PitDelay,
        control_message: Option<ControlMessage>,
        counter: u32,

        // Errors
        imu_initialization_error: Option<ImuError<ControllerStatus>>,
        fpga_programming_error: Option<FpgaError<LpspiError, Infallible>>,
        fpga_initialization_error: Option<FpgaError<LpspiError, Infallible>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize the Heap
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
            pit: (pit0, pit1, pit2, _pit3),
            ..
        } = board::t41(ctx.device);

        // Setup USB Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        // Chained Pit<0> and Pit<1>
        let mut chained_timer = Chained01::new(pit0, pit1);
        chained_timer.enable();

        // Setup Rx Interrupt
        let rx_int = gpio1.input(pins.p15);
        gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));

        // Initialize Fpga SPI
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
        spi.disabled(|spi| spi.set_mode(FPGA_SPI_MODE));

        // Initialize IMU
        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let imu = IMU::new(i2c);

        // Initialize pins for the FPGA
        let cs = gpio2.output(pins.p9);
        let init_b = gpio4.input(pins.p29);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        let done = gpio3.input(pins.p30);

        // Initialize the FPGA
        let fpga = FPGA::new(spi, cs, init_b, prog_b, done, delay1).expect("Unable to modify peripheral pins for FPGA");

        rx_int.clear_triggered();

        initialize_imu::spawn().ok();

        (
            Shared {
                control_message: None,
                counter: 0,
                fpga,
                imu,
                pit_delay,

                imu_initialization_error: None,
                fpga_programming_error: None,
                fpga_initialization_error: None,
            },
            Local {
                motion_controller: MotionControl::new(),
                last_encoders: Vector4::zeros(),
                chain_timer: chained_timer,
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
        shared = [imu, pit_delay, imu_initialization_error],
        priority = 1
    )]
    async fn initialize_imu(ctx: initialize_imu::Context) {
        (ctx.shared.imu, ctx.shared.pit_delay, ctx.shared.imu_initialization_error).lock(|imu, pit_delay, imu_initialization_error| {
            if let Err(err) = imu.init(pit_delay) {
                *imu_initialization_error = Some(err);
            }
        });

        initialize_fpga::spawn().ok();
    }

    #[task(
        shared = [fpga, pit_delay, imu_initialization_error, fpga_programming_error, fpga_initialization_error],
        priority = 1,
    )]
    async fn initialize_fpga(ctx: initialize_fpga::Context) {
        let fully_initialized = (
            ctx.shared.fpga,
            ctx.shared.pit_delay,
            ctx.shared.imu_initialization_error,
            ctx.shared.fpga_programming_error,
            ctx.shared.fpga_initialization_error,
        ).lock(|fpga, pit_delay, imu_initialization_error, fpga_programming_error, fpga_initialization_error| {
            if let Err(err) = fpga.configure() {
                *fpga_programming_error = Some(err);
                return false;
            }
            pit_delay.delay_ms(10u8);
            if let Err(err) = fpga.motors_en(true) {
                *fpga_initialization_error = Some(err);
                return false;
            }

            if imu_initialization_error.is_some() {
                return false;
            }

            true
        });

        if fully_initialized {
            motion_control_loop::spawn().ok();
        } else {
            error_report::spawn().ok();
        }
    }

    #[task(
        shared = [control_message, counter, fpga, imu],
        local = [motion_controller, last_encoders, chain_timer, initialized: bool = false, iteration: u32 = 0, last_time: u64 = 0],
        priority = 1
    )]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            *ctx.local.initialized = true;
            *ctx.local.last_time = ctx.local.chain_timer.current_timer_value();
        }

        let body_velocities = if *ctx.local.iteration < MAX_SPEED_ITERATIONS {
            Vector3::new(0.0, (*ctx.local.iteration as f32) / (MAX_SPEED_ITERATIONS as f32), 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        };
        let wheel_velocities = ctx.local.motion_controller.body_to_wheels(body_velocities);

        #[cfg(feature = "debug")]
        log::info!("Moving at {:?}", wheel_velocities);

        let encoder_velocities = ctx.shared.fpga.lock(|fpga| {
            match fpga.set_velocities(wheel_velocities.into(), false) {
                Ok(encoder_velocities) => encoder_velocities,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Unable to Read Encoder Values");
                    [0.0; 4]
                }
            }
        });

        #[cfg(feature = "debug")]
        log::info!("Fpga Status: {:#010b}", ctx.local.fpga.status);

        *ctx.local.last_encoders = Vector4::new(
            encoder_velocities[0],
            encoder_velocities[1],
            encoder_velocities[2],
            encoder_velocities[3],
        );

        *ctx.local.iteration += 1;

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_loop::spawn().ok();
    }

    #[task(
        shared = [imu_initialization_error, fpga_programming_error, fpga_initialization_error],
        priority = 1
    )]
    async fn error_report(ctx: error_report::Context) {
        let (imu_initialization_error, fpga_programming_error, fpga_initialization_error) =
        (
            ctx.shared.imu_initialization_error,
            ctx.shared.fpga_programming_error,
            ctx.shared.fpga_initialization_error
        ).lock(|imu_init_error, fpga_prog_error, fpga_init_error| {
            (
                imu_init_error.take(),
                fpga_prog_error.take(),
                fpga_init_error.take(),
            )
        });

        for _ in 0..5 {
            log::error!("IMU-INIT: {:?}\nFPGA-PROG: {:?}\nFPGA-INIT: {:?}", imu_initialization_error, fpga_programming_error, fpga_initialization_error);
            Systick::delay(1_000u32.millis()).await;
        }

        panic!("IMU-INIT: {:?}\nFPGA-PROG: {:?}\nFPGA-INIT: {:?}", imu_initialization_error, fpga_programming_error, fpga_initialization_error);
    }
}