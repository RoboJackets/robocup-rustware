//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use core::mem::MaybeUninit;

///
/// This is a demo example file that turns on and off the onboard led.
///
/// Please follow this example for future examples and sanity tests
///
use teensy4_panic as _;

use embedded_alloc::Heap;

const HEAP_SIZE: usize = 1024;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use super::*;

    use embedded_hal::blocking::delay::DelayMs;
    use icm42605_driver::{ImuError, IMU};

    use bsp::board;
    use teensy4_bsp::{self as bsp, board::PERCLK_FREQUENCY};

    use bsp::hal;
    use hal::lpi2c::ControllerStatus;
    use hal::timer::Blocking;

    use rtic_monotonics::{systick::*, Monotonic};

    use robojackets_robocup_control::{Display, Imu, PitDelay};

    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::Point,
        text::{Baseline, Text},
        Drawable,
    };

    use teensy4_pins::t41::*;

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        display: Display,
        imu: Imu,
        pit_delay: PitDelay,

        // Errors
        imu_init_error: Option<ImuError<ControllerStatus>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize the Heap
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins,
            usb,
            lpi2c1,
            pit: (_, _, pit2, _),
            mut gpio1,
            mut gpio2,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        //stop the motorboard from killing itself
        let v3_3_enable = gpio1.output(pins.p23);
        v3_3_enable.set();

        let v5_enable = gpio2.output(pins.p36);
        v5_enable.set();

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::MHz1);
        let i2c_bus: &'static _ = shared_bus::new_cortexm!(
            imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1> = i2c
        )
        .unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let imu = IMU::new(i2c_bus.acquire_i2c());

        let interface = I2CDisplayInterface::new(i2c_bus.acquire_i2c());
        let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        initialize_imu::spawn().ok();

        (
            Shared {
                display,
                imu,
                pit_delay: delay,
                imu_init_error: None,
            },
            Local {},
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [imu, pit_delay, imu_init_error,display],
        priority = 1
    )]
    async fn initialize_imu(mut ctx: initialize_imu::Context) {
        Systick::delay(4_000u32.millis()).await;

        let fully_initialized = (
            ctx.shared.imu,
            ctx.shared.pit_delay,
            ctx.shared.imu_init_error,
        )
            .lock(|imu, pit_delay, imu_init_error| {
                if let Err(err) = imu.init(pit_delay) {
                    *imu_init_error = Some(err);
                    return false;
                }
                true
            });

        if fully_initialized {
            log::info!("IMU Initialized");
        } else {
            log::error!("IMU Initialization Failed");
        }

        Systick::delay(1_000u32.millis()).await;

        (ctx.shared.display).lock(|display| {
            display.init().unwrap();
        });

        if fully_initialized {
            imu_test::spawn().ok();
        } else {
            error_report::spawn().ok();
        }
    }

    #[task(priority = 1)]
    async fn delay_tick(mut ctx: delay_tick::Context) {
        Systick::delay(1000u32.millis()).await;

        imu_test::spawn().ok();
    }

    #[task(
        shared = [imu, pit_delay,display],
        priority = 1,
    )]
    async fn imu_test(mut ctx: imu_test::Context) {
        let mut g_z = 0.0;
        let mut a_x = 0.0;
        let mut a_y = 0.0;

        (ctx.shared.imu).lock(|imu| {
            let gyro_z = match imu.gyro_z() {
                Ok(gyro_z) => gyro_z,
                Err(err) => {
                    log::info!("Unable to Read Gyro Z: {:?}", err);
                    0.0
                }
            };
            g_z = gyro_z;

            let accel_x = match imu.accel_x() {
                Ok(accel_x) => accel_x,
                Err(err) => {
                    log::info!("Unable to Read Accel X: {:?}", err);
                    0.0
                }
            };
            a_x = accel_x;

            let accel_y = match imu.accel_y() {
                Ok(accel_y) => accel_y,
                Err(err) => {
                    log::info!("Unable to Read Accel Y: {:?}", err);
                    0.0
                }
            };
            a_y = accel_y;
        });

        let mut display = (ctx.shared.display).lock(|display| {
            display.clear();

            let style = MonoTextStyleBuilder::new()
                .font(&FONT_6X10)
                .text_color(BinaryColor::On)
                .build();
            let text = alloc::fmt::format(format_args!(
                "Gyro Z: {:.2} \nAccel X: {:.2} \nAccel Y: {:.2}",
                g_z, a_x, a_y
            ));
            Text::with_baseline(text.as_str(), Point::zero(), style, Baseline::Top)
                .draw(display)
                .unwrap();
            display.flush().unwrap();
        });

        delay_tick::spawn().ok();
    }

    #[task(
        shared = [imu_init_error],
        priority = 1
    )]
    async fn error_report(mut ctx: error_report::Context) {
        let imu_initialization_error = ctx
            .shared
            .imu_init_error
            .lock(|imu_init_error| imu_init_error.take());

        for _ in 0..5 {
            log::error!("IMU-INIT: {:?}", imu_initialization_error);
            Systick::delay(1_000u32.millis()).await;
        }

        panic!("IMU-INIT: {:?}", imu_initialization_error);
    }
}
