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

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use imxrt_iomuxc::prelude::*;

    use embedded_hal::spi::MODE_0;

    use nalgebra::{Vector3, Vector4};
    use teensy4_bsp as bsp;
    use bsp::board::{self, LPSPI_FREQUENCY};
    use bsp::board::PERCLK_FREQUENCY;

    use teensy4_bsp::hal as hal;
    use hal::lpspi::{Lpspi, Pins};
    use hal::gpio::Trigger;
    use hal::timer::Blocking;
    use hal::pit::Chained01;
    
    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::Radio;

    use rtic_monotonics::systick::*;

    use robojackets_robocup_rtp::{ControlMessage, CONTROL_MESSAGE_SIZE};
    use robojackets_robocup_rtp::BASE_STATION_ADDRESS;

    use motion::MotionControl;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    use icm42605_driver::IMU;

    use main::{
        Fpga, Imu, BASE_AMPLIFICATION_LEVEL, CHANNEL, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY, RADIO_ADDRESS
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 200;

    #[local]
    struct Local {
        motion_controller: MotionControl,
        fpga: Fpga,
        imu: Imu,
        last_encoders: Vector4<f32>,
        chain_timer: Chained01,
    }

    #[shared]
    struct Shared {
        control_message: Option<ControlMessage>,
        counter: u32,
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

        // Gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

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
        let mut pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let imu = match IMU::new(i2c, &mut pit_delay) {
            Ok(imu) => imu,
            Err(_err) => panic!("Unable to Initialize IMU"),
        };

        // Initialize Shared SPI
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

        // Init radio cs pin and ce pin
        let radio_cs = gpio1.output(pins.p14);
        let ce = gpio1.output(pins.p20);

        // Initialize radio
        let mut radio = Radio::new(ce, radio_cs);
        let radio_error = radio.begin(&mut shared_spi, &mut delay2);

        if !radio_error.is_err() {
            radio.set_pa_level(BASE_AMPLIFICATION_LEVEL, &mut shared_spi, &mut delay2);
            radio.set_channel(CHANNEL, &mut shared_spi, &mut delay2);
            radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, &mut shared_spi, &mut delay2);
            radio.open_writing_pipe(BASE_STATION_ADDRESS, &mut shared_spi, &mut delay2);
            radio.open_reading_pipe(1, RADIO_ADDRESS, &mut shared_spi, &mut delay2);
            radio.start_listening(&mut shared_spi, &mut delay2);
        } else {
            panic!("Unable to initialize radio");
        }

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

        rx_int.clear_triggered();

        motion_control_loop::spawn().ok();

        (
            Shared {
                control_message: None,
                counter: 0,
            },
            Local {
                motion_controller: MotionControl::new(),
                fpga,
                imu,
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
        shared = [control_message, counter],
        local = [fpga, motion_controller, imu, last_encoders, chain_timer, initialized: bool = false, iteration: u32 = 0, last_time: u64 = 0, cumulative_time: u64 = 0, body_velocity: Vector3<f32> = Vector3::new(0.0, 1.0, 0.0), correct_direction: bool = false],
        priority = 1
    )]
    async fn motion_control_loop(ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            if ctx.local.fpga.configure().is_err() {
                panic!("Unable to configure fpga");
            }

            Systick::delay(10u32.millis()).await;

            if ctx.local.fpga.motors_en(true).is_err() {
                panic!("Unable to enable motors");
            }

            *ctx.local.initialized = true;
            *ctx.local.last_time = ctx.local.chain_timer.current_timer_value();

            log::info!("Fpga Initialized");
        }

        let gyro = match ctx.local.imu.gyro_z() {
            Ok(gyro) => gyro,
            Err(_err) => {
                #[cfg(feature = "debug")]
                log::info!("Unable to Read Gyro");
                0.0
            }
        };

        let accel_x = match ctx.local.imu.accel_x() {
            Ok(accel_x) => accel_x,
            Err(_err) => {
                #[cfg(feature = "debug")]
                log::info!("Unable to Read Accel X");
                0.0
            }
        };

        let accel_y = match ctx.local.imu.accel_y() {
            Ok(accel_y) => accel_y,
            Err(_err) => {
                #[cfg(feature = "debug")]
                log::info!("Unable to Read Accel Y");
                0.0
            }
        };

        let now = ctx.local.chain_timer.current_timer_value();
        let delta = *ctx.local.last_time - now;
        *ctx.local.last_time = now;
        *ctx.local.cumulative_time += delta;

        // log::info!("Target Velocity: {:?}", *ctx.local.body_velocity);
        let wheel_velocities = ctx.local.motion_controller.control_update(
            Vector3::new(-accel_y, accel_x, gyro),
            *ctx.local.last_encoders,
            *ctx.local.body_velocity,
            delta,
        );

        #[cfg(feature = "debug")]
        log::info!("Moving at {:?}", wheel_velocities);

        if *ctx.local.iteration == 0 {
            log::info!("Starting FPGA");
        }

        let encoder_velocities = match ctx.local.fpga.set_velocities(wheel_velocities.into(), false) {
            Ok(encoder_velocities) => encoder_velocities,
            Err(_err) => {
                #[cfg(feature = "debug")]
                log::info!("Unable to Read Encoder Values");
                [0.0; 4]
            }
        };

        if ctx.local.body_velocity[0] == 0.0 {
            if !*ctx.local.correct_direction {
                if encoder_velocities[0] > 0.0 &&
                    encoder_velocities[1] > 0.0 &&
                    encoder_velocities[2] < 0.0 &&
                    encoder_velocities[3] < 0.0 {
                    log::info!("Moving Forward after {} us", ctx.local.cumulative_time);
                    *ctx.local.correct_direction = true;
                }
            }
        } else {
            if !*ctx.local.correct_direction {
                if encoder_velocities[0] < 0.0 &&
                    encoder_velocities[1] > 0.0 &&
                    encoder_velocities[2] > 0.0 &&
                    encoder_velocities[3] < 0.0 {
                    log::info!("Moving Left after {} us", ctx.local.cumulative_time);
                    *ctx.local.correct_direction = true;
                }
            }
        }

        #[cfg(feature = "debug")]
        log::info!("Fpga Status: {:#010b}", ctx.local.fpga.status);

        *ctx.local.last_encoders = Vector4::new(
            encoder_velocities[0],
            encoder_velocities[1],
            encoder_velocities[2],
            encoder_velocities[3],
        );

        *ctx.local.iteration += 1;
        if *ctx.local.correct_direction && *ctx.local.cumulative_time > 10_000 {
            if ctx.local.body_velocity[0] == 0.0 {
                log::info!("Moving Right");
                *ctx.local.body_velocity = Vector3::new(1.0, 0.0, 0.0);
            } else {
                log::info!("Moving Forward");
                *ctx.local.body_velocity = Vector3::new(0.0, 1.0, 0.0);
            }
            *ctx.local.iteration = 0;
            *ctx.local.correct_direction = false;
            *ctx.local.cumulative_time = 0;
        }

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_loop::spawn().ok();
    }
}