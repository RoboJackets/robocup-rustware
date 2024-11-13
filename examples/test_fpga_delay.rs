//!
//! Manual Control is an example that should be pretty close to the fully-working program.  However,
//! it is currently considered a work in progress so it is still considered a test.
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

/// Wheel Radius (m)
pub const WHEEL_RADIUS: f32 = 0.02786;
/// Duty Cycle to Velocity (m/s) (Tested.  See duty-to-wheel-data <https://docs.google.com/spreadsheets/d/1Y931pXyfOq7iaclSkPCwzVSvX_aOvcgqkb2LaQmUZGI/edit?usp=sharing>)
pub const DUTY_CYCLE_TO_VELOCITY: f32 = 7.37;
/// Velocity (m/s) to Duty Cycle
pub const VELOCITY_TO_DUTY_CYCLES: f32 = 1.0 / DUTY_CYCLE_TO_VELOCITY;
/// The maximum duty cycle
pub const MAX_DUTY_CYCLE: f32 = 511.0;


#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::convert::Infallible;
    use core::f32::MAX;
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
    const MOTION_CONTROL_TRANSITION_DELAY_US: u64 = 1_000_000;

    #[local]
    struct Local {
        gpt2: imxrt_hal::gpt::Gpt2,
        body_velocities: Vector3<f32>,
        next_body_velocities: Vector3<f32>,
        start_time: u32,
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
            mut gpt2,
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

        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        gpt2.enable();

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
                gpt2: gpt2,
                body_velocities: Vector3::new(0.0, 1.0, 0.0),
                next_body_velocities: Vector3::new(0.0, 0.0, 0.0),
                start_time: 0,
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
        local = [gpt2, body_velocities, next_body_velocities, start_time, motion_controller, last_encoders, chain_timer, initialized: bool = false, iteration: u32 = 0, last_time: u64 = 0, transitioning: bool = false, done: bool = false],
        priority = 1
    )]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            *ctx.local.initialized = true;
            *ctx.local.start_time = ctx.local.gpt2.count();
            log::info!("Testing and measuring FPGA Delay");
            *ctx.local.body_velocities = Vector3::new(0.0, 1.0, 0.0); // start at this velocity
            *ctx.local.transitioning = false;
        }
        //always wait 1 second between direction changes
        else if *ctx.local.transitioning {
            if (ctx.local.gpt2.count() - *ctx.local.start_time) > 1000000 { // FIGURE OUT HOW THE TIMER WORKS
                *ctx.local.body_velocities = *ctx.local.next_body_velocities;
                log::info!("LEAVING TRANSITIONING");
                *ctx.local.transitioning = false;
                *ctx.local.start_time = ctx.local.gpt2.count(); // reset start time to avoid overestimation
            }
        }
        
        
        //log::info!("running");

        let (gyro, accel_x, accel_y) = ctx.shared.imu.lock(|imu| {
            let gyro = match imu.gyro_z() {
                Ok(gyro) => gyro,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Unable to Read Gyro");
                    0.0
                }
            };
    
            let accel_x = match imu.accel_x() {
                Ok(accel_x) => accel_x,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Unable to Read Accel X");
                    0.0
                }
            };
    
            let accel_y = match imu.accel_y() {
                Ok(accel_y) => accel_y,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Unable to Read Accel Y");
                    0.0
                }
            };

            (gyro, accel_x, accel_y)
        });
        
        let now = ctx.local.chain_timer.current_timer_value();
        let delta = *ctx.local.last_time - now;
        *ctx.local.last_time = now;

        
        //let test_body_vel = Vector3::new(0.0, 1.0, 0.0);
        
        *ctx.local.last_encoders = Vector4::new(0.0,0.0,0.0,0.0);

        let wheel_velocities = ctx.local.motion_controller.control_update(
            Vector3::new(-accel_y, accel_x, gyro),
            *ctx.local.last_encoders, 
            *ctx.local.body_velocities,
            delta
        );
        



        //#[cfg(feature = "debug")]
        //log::info!("Moving at {:?}", wheel_velocities);

        
        // conduct SPI transfer to FPGA, feeding it duty cycles and receiving the current operating duty cycle
        let actual_phases = ctx.shared.fpga.lock(|fpga| {
            match fpga.set_velocities_rcv_duty(wheel_velocities.into(), false) {
                Ok(actual_phases) => actual_phases,
                Err(_err) => {
                    //#[cfg(feature = "debug")]
                    log::info!("Unable to Read Encoder Values");
                    [0.0; 4]
                }
            }
        });

        //log::info!("Phases at {:?}, should be {:?}", actual_phases, wheel_velocities * VELOCITY_TO_DUTY_CYCLES * MAX_DUTY_CYCLE);


        let delta_phase = Vector4::from(actual_phases) - wheel_velocities * VELOCITY_TO_DUTY_CYCLES * MAX_DUTY_CYCLE;

        if Vector4::abs(&delta_phase) < Vector4::new(2.5, 2.5, 2.5, 2.5) && !*ctx.local.done && !*ctx.local.transitioning {

            let seconds_between = ((ctx.local.gpt2.count() - *ctx.local.start_time) as f32) / 1_000_000.0;
            log::info!("time between sending duty cycle to reaching duty cycle is {} s", seconds_between);
            log::info!("Phases from FPGA: {:?}, phases expected: {:?}", actual_phases, wheel_velocities * VELOCITY_TO_DUTY_CYCLES * MAX_DUTY_CYCLE);

            // kind of a state machine set up here
            // add more steps in betweeen
            if *ctx.local.body_velocities == Vector3::new(0.0, 1.0, 0.0) {
                *ctx.local.next_body_velocities = Vector3::new(1.0, 0.0, 0.0);
                log::info!("Going from body velocity {:?} to {:?}", *ctx.local.body_velocities, *ctx.local.next_body_velocities);
            } 

            else if *ctx.local.body_velocities == Vector3::new(1.0, 0.0, 0.0) {
                *ctx.local.next_body_velocities = Vector3::new(0.0, 0.0, 0.0);
                log::info!("Going from body velocity {:?} to {:?}", *ctx.local.body_velocities, *ctx.local.next_body_velocities);
            }

            else if *ctx.local.body_velocities == Vector3::new(0.0, 0.0, 0.0) {
                *ctx.local.done = true;
                log::info!("Going from body velocity {:?} to {:?}", *ctx.local.body_velocities, *ctx.local.next_body_velocities);
            }

            // still print out all of the phases for the motors, to make sure that all match
            // only checking motor 0 for the time measurement, could implement checking all
            /*
            log::info!("Actual phase of motor 0 is {}", actual_phases[0]);
            log::info!("Expected phase of motor 0 is {}", wheel_velocities[0] * VELOCITY_TO_DUTY_CYCLES * MAX_DUTY_CYCLE);
            log::info!("Actual phase of motor 0 is {}", actual_phases[1]);
            log::info!("Expected phase of motor 0 is {}", wheel_velocities[1] * VELOCITY_TO_DUTY_CYCLES * MAX_DUTY_CYCLE);
            log::info!("Actual phase of motor 0 is {}", actual_phases[2]);
            log::info!("Expected phase of motor 0 is {}", wheel_velocities[2] * VELOCITY_TO_DUTY_CYCLES * MAX_DUTY_CYCLE);
            log::info!("Actual phase of motor 0 is {}", actual_phases[3]);
            log::info!("Expected phase of motor 0 is {}", wheel_velocities[3] * VELOCITY_TO_DUTY_CYCLES * MAX_DUTY_CYCLE);
            */

            *ctx.local.start_time = ctx.local.gpt2.count(); // reset the start time for the next stage, going to 0
            *ctx.local.transitioning = true;
        }

        #[cfg(feature = "debug")]
        log::info!("Fpga Status: {:#010b}", ctx.local.fpga.status);

        /* 
        *ctx.local.last_encoders = Vector4::new(
            encoder_velocities[0],
            encoder_velocities[1],
            encoder_velocities[2],
            encoder_velocities[3],
         );
        */
        
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

        for _ in 0..100 {
            log::error!("IMU-INIT: {:?}\nFPGA-PROG: {:?}\nFPGA-INIT: {:?}", imu_initialization_error, fpga_programming_error, fpga_initialization_error);
            Systick::delay(1_000u32.millis()).await;
        }

        panic!("IMU-INIT: {:?}\nFPGA-PROG: {:?}\nFPGA-INIT: {:?}", imu_initialization_error, fpga_programming_error, fpga_initialization_error);
    }
}