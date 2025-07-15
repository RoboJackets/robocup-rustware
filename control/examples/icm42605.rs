//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

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

    use rtic_monotonics::systick::*;

    use robojackets_robocup_control::{
        Imu, Killn, MotorEn, PitDelay, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
    };

    use teensy4_pins::t41::{P18, P19};

    #[local]
    struct Local {
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {
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
            mut gpt2,
            mut gpio1,
            mut gpio2,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        // Gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        // End Initialize Timers //

        // Initialize Motor Board //

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();

        delay2.delay_ms(500u32);

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::MHz1);
        let i2c_bus: &'static _ = shared_bus::new_cortexm!(
            imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1> = i2c
        )
        .expect("Failed to initialize shared I2C bus LPI2C1");

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let imu = IMU::new(i2c_bus.acquire_i2c());

        initialize_imu::spawn().ok();

        (
            Shared {
                imu,
                pit_delay: delay,
                imu_init_error: None,
            },
            Local { poller },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [imu, pit_delay, imu_init_error],
        priority = 1
    )]
    async fn initialize_imu(ctx: initialize_imu::Context) {
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
            imu_test::spawn().ok();
        } else {
            error_report::spawn().ok();
        }
    }

    #[task(
        shared = [imu, pit_delay],
        priority = 1,
    )]
    async fn imu_test(mut ctx: imu_test::Context) {
        let (gyro_z, accel_x, accel_y) = ctx.shared.imu.lock(|imu| {
            (
                imu.gyro_z().unwrap_or_default(),
                imu.accel_x().unwrap_or_default(),
                imu.accel_y().unwrap_or_default(),
            )
        });

        log::info!("X: {}, Y: {}, Z: {}", accel_x, accel_y, gyro_z);

        short_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn short_delay(_ctx: short_delay::Context) {
        Systick::delay(100u32.millis()).await;
        imu_test::spawn().ok();
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

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
