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

    use core::mem::MaybeUninit;

    use imxrt_iomuxc::prelude::*;

    use nalgebra::Vector3;
    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::board::PERCLK_FREQUENCY;

    use teensy4_bsp::hal as hal;
    use hal::timer::Blocking;
    use hal::pit::Chained01;

    use rtic_monotonics::systick::*;

    use motion::MotionControl;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    use nalgebra::*;

    use icm42605_driver::IMU;

    use main::{
        Fpga,
        Imu,
        GPT_FREQUENCY,
        GPT_CLOCK_SOURCE,
        GPT_DIVIDER,
    };

    const HEAP_SIZE: usize = 8192;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 1000;

    #[local]
    struct Local {
        fpga: Fpga,
        motion_control: MotionControl,
        last_encoders: Vector4<f32>,
        chain_timer: Chained01,
        imu: Imu,
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
            gpio1: _gpio1,
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

        // Setup Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        let mut chained = Chained01::new(pit0, pit1);
        chained.enable();

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
        let mut pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let imu = match IMU::new(i2c, &mut pit_delay) {
            Ok(imu) => imu,
            Err(_err) => panic!("Unable to Initialize IMU"),
        };

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

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
        fpga_spi.disabled(|spi| spi.set_mode(FPGA_SPI_MODE));

        let cs = gpio2.output(pins.p9);
        let init_b = gpio4.input(pins.p29);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        let done = gpio3.input(pins.p30);

        let fpga = match FPGA::new(fpga_spi, cs, init_b, prog_b, done, delay1) {
            Ok(fpga) => fpga,
            Err(_) => panic!("Unable to initialize the FPGA"),
        };

        motion_control_update::spawn().ok();

        (
            Shared {

            },
            Local {
                fpga,
                motion_control: MotionControl::new(),
                chain_timer: chained,
                last_encoders: Vector4::zeros(),
                imu,
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
        local = [fpga, motion_control, chain_timer, imu, last_encoders, initialized: bool = false, last_time: u64 = 0],
        shared = [],
        priority=1,
    )]
    async fn motion_control_update(ctx: motion_control_update::Context) {
        if !*ctx.local.initialized {
            if ctx.local.fpga.configure().is_err() {
                panic!("Unable to configure fpga");
            }

            if ctx.local.fpga.motors_en(true).is_err() {
                panic!("Unable to enable motors");
            }

            *ctx.local.initialized = true;
            *ctx.local.last_time = ctx.local.chain_timer.current_timer_value();
        }

        let body_velocities = Vector3::new(0.0, 1.0, 0.0);

        let gyro = match ctx.local.imu.gyro_z() {
            Ok(gyro) => gyro,
            Err(_err) => {
                log::info!("Unable to Read Gyro");
                0.0
            }
        };

        let accel_x = match ctx.local.imu.accel_x() {
            Ok(accel_x) => accel_x,
            Err(_err) => {
                log::info!("Unable to Read Accel X");
                0.0
            }
        };

        let accel_y = match ctx.local.imu.accel_y() {
            Ok(accel_y) => accel_y,
            Err(_err) => {
                log::info!("Unable to Read Accel Y");
                0.0
            }
        };

        let now = ctx.local.chain_timer.current_timer_value();
        let delta = *ctx.local.last_time - now;
        *ctx.local.last_time = now;

        let wheel_velocities = ctx.local.motion_control.control_update(
            Vector3::new(accel_x, accel_y, gyro),
            *ctx.local.last_encoders,
            body_velocities,
            delta,
        );
    
        let encoder_values = match ctx.local.fpga.set_velocities(wheel_velocities.into(), 0.0) {
            Ok(encoder_values) => {
                log::info!("Encoder Values: {:?}", encoder_values);
                encoder_values
            },
            Err(_) => [0.0; 4],
        };

        *ctx.local.last_encoders = Vector4::new(
            encoder_values[0],
            encoder_values[1],
            encoder_values[2],
            encoder_values[3],
        );

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_update::spawn().ok();
    }
}