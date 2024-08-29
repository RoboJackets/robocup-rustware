//!
//! I have a theory that the FPGA has trouble switching directions dramatically.
//! 
//! Therefore, I think it might increase the performance of the FPGA by ramping up to
//! target velocities
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

/// Ramp up the current velocity over 100 iterations of the motion control loop.
const RAMP_UP_ITERATIONS: u32 = 100;

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
    use hal::gpio::Trigger;
    use hal::timer::Blocking;
    use hal::pit::Chained01;

    use rtic_monotonics::systick::*;

    use motion::MotionControl;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    use main::{
        Fpga, PitDelay, LedPin, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 200;

    #[local]
    struct Local {
        motion_controller: MotionControl,
        last_encoders: Vector4<f32>,
        chain_timer: Chained01,
        led_pin: LedPin,
    }

    #[shared]
    struct Shared {
        fpga: Fpga,
        pit_delay: PitDelay,
        counter: u32,

        // Errors
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

        // Initialize pins for the FPGA
        let cs = gpio2.output(pins.p9);
        let init_b = gpio4.input(pins.p29);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        let done = gpio3.input(pins.p30);

        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);

        // Initialize the FPGA
        let fpga = FPGA::new(spi, cs, init_b, prog_b, done, delay1).expect("Unable to modify peripheral pins for FPGA");

        rx_int.clear_triggered();

        let led_pin = gpio1.output(pins.p0);
        led_pin.clear();

        initialize_fpga::spawn().ok();

        (
            Shared {
                counter: 0,
                fpga,
                pit_delay,

                fpga_programming_error: None,
                fpga_initialization_error: None,
            },
            Local {
                led_pin,
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
        shared = [fpga, pit_delay, fpga_programming_error, fpga_initialization_error],
        priority = 1,
    )]
    async fn initialize_fpga(ctx: initialize_fpga::Context) {
        let fully_initialized = (
            ctx.shared.fpga,
            ctx.shared.pit_delay,
            ctx.shared.fpga_programming_error,
            ctx.shared.fpga_initialization_error,
        ).lock(|fpga, pit_delay, fpga_programming_error, fpga_initialization_error| {
            if let Err(err) = fpga.configure() {
                *fpga_programming_error = Some(err);
                return false;
            }
            pit_delay.delay_ms(10u8);
            if let Err(err) = fpga.motors_en(true) {
                *fpga_initialization_error = Some(err);
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
        shared = [counter, fpga],
        local = [motion_controller, last_encoders, chain_timer, led_pin, initialized: bool = false, iteration: u32 = 0],
        priority = 1
    )]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            *ctx.local.initialized = true;
            ctx.local.led_pin.set();
        }

        let body_velocities = if *ctx.local.iteration < RAMP_UP_ITERATIONS {
            Vector3::new(0.0, (*ctx.local.iteration as f32) / (RAMP_UP_ITERATIONS as f32), 0.0)
        } else {
            ctx.local.led_pin.clear();
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

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_loop::spawn().ok();
    }

    #[task(
        shared = [fpga_programming_error, fpga_initialization_error],
        priority = 1
    )]
    async fn error_report(ctx: error_report::Context) {
        let (fpga_programming_error, fpga_initialization_error) =
        (
            ctx.shared.fpga_programming_error,
            ctx.shared.fpga_initialization_error
        ).lock(|fpga_prog_error, fpga_init_error| {
            (
                fpga_prog_error.take(),
                fpga_init_error.take(),
            )
        });

        for _ in 0..5 {
            log::error!("FPGA-PROG: {:?}\nFPGA-INIT: {:?}", fpga_programming_error, fpga_initialization_error);
            Systick::delay(1_000u32.millis()).await;
        }

        panic!("FPGA-PROG: {:?}\nFPGA-INIT: {:?}", fpga_programming_error, fpga_initialization_error);
    }
}