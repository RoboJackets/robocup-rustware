//!
//! Controller uses a external controller (i.e. a breadboard with push buttons
//! to control the robot).  This can be used to tune the theoretical constants
//! for motion control of the robot.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::convert::Infallible;
    use core::mem::MaybeUninit;

    use imxrt_iomuxc::prelude::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use teensy4_pins::t41::*;

    use teensy4_bsp::hal as hal;
    use hal::gpio::Output;
    use hal::gpt::{ClockSource, Gpt1, Gpt2};
    use hal::timer::Blocking;
    use hal::lpspi::Lpspi;

    use rtic_monotonics::systick::*;

    use motion::MotionControl;

    use controller::Controller;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    // Constants
    const GPT_FREQUENCY: u32 = 1_000;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 200;

    // Type Definitions
    // FPGA Spi
    type SPI = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
    type Fpga = FPGA<SPI, Output<P9>, P29, Output<P28>, P30, Delay1, hal::lpspi::LpspiError, Infallible>;
    // Delays
    type Delay1 = Blocking<Gpt1, GPT_FREQUENCY>;
    type Control = Controller<P18, P19, P20, P21, P22, P23>;

    #[local]
    struct Local {
        motion_controller: MotionControl,
        fpga: Fpga,
        controller: Control,
        gpt2: Gpt2,
    }

    #[shared]
    struct Shared {

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
            mut gpt2,
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

        gpt2.disable();
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        gpt2.enable();

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
        spi.disabled(|spi| {
            spi.set_mode(FPGA_SPI_MODE);
        });

        // Initialize pins for the FPGA
        let cs = gpio2.output(pins.p9);
        let init_b = gpio4.input(pins.p29);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        let done = gpio3.input(pins.p30);

        // Initialize the FPGA
        let fpga = match FPGA::new(spi, cs, init_b, prog_b, done, delay1) {
            Ok(fpga) => fpga,
            Err(_) => panic!("Unable to initialize the FPGA"),
        };

        // TODO: Refactor into a driver
        let left = gpio1.input(pins.p18);
        let up = gpio1.input(pins.p19);
        let right = gpio1.input(pins.p20);
        let down = gpio1.input(pins.p21);
        let counterclockwise = gpio1.input(pins.p22);
        let clockwise = gpio1.input(pins.p23);

        let controller = Controller::new(left, up, right, down, counterclockwise, clockwise);

        motion_control_loop::spawn().ok();

        (
            Shared {

            },
            Local {
                motion_controller: MotionControl::new(),
                fpga,
                controller,
                gpt2,
            }
        )
    }

    // 1 count from the gpt2 ~1 microseconds
    #[task(local=[motion_controller, fpga, controller, gpt2], priority=1)]
    async fn motion_control_loop(ctx: motion_control_loop::Context) {
        if !ctx.local.fpga.is_initialized() {
            match ctx.local.fpga.configure() {
                Ok(_) => {
                    #[cfg(feature = "debug")]
                    log::info!("Fpga Configured");
                },
                Err(_) => panic!("Unable to Configure Fpga"),
            }

            match ctx.local.fpga.motors_en(true) {
                Ok(_status) => {
                    #[cfg(feature = "debug")]
                    log::info!("Motors Enabled: {:010b}", _status);
                },
                Err(_) => panic!("Unable to enable motors"),
            }
        }

        let movement = controller::UP;

        // let movement = ctx.local.controller.calculate_movement();

        let wheel_velocities = ctx.local.motion_controller.body_to_wheels(movement);

        let now = ctx.local.gpt2.count();
        let encoder_values = match ctx.local.fpga.set_duty_get_encoders(wheel_velocities.into(), 0.0) {
            Ok(encoder_values) => encoder_values,
            Err(_) => panic!("Unable to Communicate with the FPGA"),
        };

        ctx.local.motion_controller.add_encoders(encoder_values, now);

        if ctx.local.motion_controller.full() {
            let average_velocities = ctx.local.motion_controller.get_stats();
            log::info!("Average Velocities: {:?}", average_velocities);
        }

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_loop::spawn().ok();
    }
}