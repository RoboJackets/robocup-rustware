//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

///
/// This is a demo example file that turns on and off the onboard led.
///
/// Please follow this example for future examples and sanity tests
///
use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use icm42605_driver::Icm42605;

    use bsp::board;
    use teensy4_bsp::{self as bsp, board::{Lpi2c1, PERCLK_FREQUENCY}};

    use bsp::hal;
    use hal::timer::Blocking;

    use rtic_monotonics::systick::*;

    type IMU = Icm42605<Lpi2c1>;

    #[local]
    struct Local {
        imu: IMU,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            lpi2c1,
            pit: (pit, _, _, _),
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let mut delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit);

        let imu = match Icm42605::new(i2c, &mut delay) {
            Ok(imu) => imu,
            Err(_err) => panic!("Unable to Initialize IMU"),
        };

        icm::spawn().ok();

        (
            Shared {},
            Local {
                imu,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [imu], priority = 1)]
    async fn icm(ctx: icm::Context) {
        Systick::delay(1_000u32.millis()).await;
        log::info!("Using IMU");

        loop {
            let gyro_z = match ctx.local.imu.gyro_z() {
                Ok(gyro_z) => gyro_z,
                Err(_err) => {
                    log::info!("Unable to Read Gyro Z");
                    0.0
                },
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

            log::info!("X: {}, Y: {}, Z: {}", accel_x, accel_y, gyro_z);

            Systick::delay(100u32.millis()).await;
        }
    }
}
