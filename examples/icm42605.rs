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
    use cortex_m::delay::Delay;
    use icm42605_driver::Icm42605;
    use teensy4_pins::common::*;

    use bsp::board;
    use teensy4_bsp::{self as bsp, board::Lpi2c1, hal::pit::Pit0};

    use bsp::hal;
    use hal::gpio::Output;

    use rtic_monotonics::systick::*;
    use you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::hal::timer::Blocking;

    type Led = Output<P13>;
    type Icm = Icm42605<Lpi2c1>;

    #[local]
    struct Local {
        led: Led,
        poller: bsp::logging::Poller,
        i2c: Option<Lpi2c1>,
        pit: Option<Pit0>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio2,
            usb,
            lpi2c1,
            pit: (pit, _, _, _),
            ..
        } = board::t41(ctx.device);

        let poller = bsp::logging::log::usbd(usb, bsp::logging::Interrupts::Enabled).unwrap();
        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let led = gpio2.output(pins.p13);

        icm::spawn().ok();

        (
            Shared {},
            Local {
                led,
                poller,
                i2c: Some(i2c),
                pit: Some(pit),
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = USB_OTG1, local = [poller])]
    fn poll_logger(cx: poll_logger::Context) {
        cx.local.poller.poll();
    }

    #[task(local = [i2c, pit, led], priority = 1)]
    async fn icm(ctx: icm::Context) {
        Systick::delay(7000u32.millis()).await;
        log::info!("creating icm");
        let mut icm = Icm42605::new(
            ctx.local.i2c.take().unwrap(),
            &mut Blocking::<_, { board::PERCLK_FREQUENCY }>::from_pit(
                ctx.local.pit.take().unwrap(),
            ),
        );

        if let Err(err) = icm {
            log::info!("{err:?}");
        }

        Systick::delay(1000u32.millis()).await;

        let mut icm = icm.unwrap();

        ctx.local.led.set();
        loop {
            let x = icm.accel_x();
            let y = icm.accel_y();
            let z = icm.gyro_z();

            log::info!("x: {x:?}, y: {y:?}, z: {z:?}");
            Systick::delay(100u32.millis()).await;
        }
    }
}
