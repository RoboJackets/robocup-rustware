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
    use icm42605_driver::IMU;

    use bsp::board;
    use teensy4_bsp::{self as bsp, board::{Lpi2c1, PERCLK_FREQUENCY}};

    use bsp::hal;
    use hal::timer::Blocking;

    use rtic_monotonics::systick::*;

    type Imu = IMU<Lpi2c1>;

    #[local]
    struct Local {
        delay: Blocking<hal::pit::Pit<0>, PERCLK_FREQUENCY>,
        i2c: Option<Lpi2c1>,
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

        icm::spawn().ok();

        (
            Shared {},
            Local {
                delay,
                i2c: Some(i2c),
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [i2c, delay], priority = 1)]
    async fn icm(ctx: icm::Context) {
        Systick::delay(5_000u32.millis()).await;
        let i2c = ctx.local.i2c.take().unwrap();

        let mut imu = match IMU::new(i2c, ctx.local.delay) {
            Ok(imu) => imu,
            Err(_) => panic!("Unable to Initialize IMU"),
        };

        log::info!("Using IMU");

        loop {
            let gyro_z = match imu.gyro_z() {
                Ok(gyro_z) => gyro_z,
                Err(_err) => {
                    log::info!("Unable to Read Gyro Z");
                    0.0
                },
            };

            let accel_x = match imu.accel_x() {
                Ok(accel_x) => accel_x,
                Err(_err) => {
                    log::info!("Unable to Read Accel X");
                    0.0
                }
            };

            let accel_y = match imu.accel_y() {
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
