//!
//! This test charges the kicker, printing the current voltage of the kicker while
//! powering up for 2 seconds.  Then kicks and stops charging the kicker.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

///
/// This is a demo example file that turns on and off the onboard led.
///
/// Please follow this example for future examples and sanity tests
///
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use bsp::board::{self, PERCLK_FREQUENCY};
    use imxrt_hal::timer::Blocking;
    use teensy4_bsp as bsp;

    use rtic_monotonics::systick::*;

    use kicker_controller::{KickTrigger, KickType, Kicker, KickerCommand};

    use main::{spi::FakeSpi, KickerCSn, KickerReset};

    #[local]
    struct Local {
        kicker_controller: Kicker<KickerCSn, KickerReset>,
        fake_spi: FakeSpi,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            mut gpio2,
            mut gpio4,
            pit: (_pit0, _pit1, _pit2, pit3),
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);
        let kicker = Kicker::new(
            gpio4.output(pins.p5),
            gpio2.output(pins.p6)
        );

        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit3);
        let fake_spi = FakeSpi::new(
            gpio4.output(pins.p2),
            gpio4.output(pins.p3),
            gpio4.input(pins.p4),
            pit_delay,
        );

        kicker_test::spawn().ok();

        (
            Shared {},
            Local {
                kicker_controller: kicker,
                fake_spi,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        local = [kicker_controller, fake_spi],
        priority = 1
    )]
    async fn kicker_test(ctx: kicker_test::Context) {
        Systick::delay(1_000u32.millis()).await;

        log::info!("Charging the Kicker");
        let kicker_command = KickerCommand {
            kick_type: KickType::Kick,
            kick_trigger: KickTrigger::Disabled,
            kick_strength: 20.0,
            charge_allowed: true,
        };
        for _ in 0..20 {
            let kicker_status = ctx
                .local
                .kicker_controller
                .service(kicker_command, ctx.local.fake_spi)
                .unwrap();
            log::info!("Kicker Status: {:?}", kicker_status);
            Systick::delay(100u32.millis()).await;
        }

        log::info!("KICKING!!!");
        let kicker_command = KickerCommand {
            kick_type: KickType::Kick,
            kick_trigger: KickTrigger::Immediate,
            kick_strength: 100.0,
            charge_allowed: true,
        };
        let kicker_status = ctx
            .local
            .kicker_controller
            .service(kicker_command, ctx.local.fake_spi)
            .unwrap();
        log::info!("Kicker Status: {:?}", kicker_status);

        Systick::delay(100u32.millis()).await;

        log::info!("Powering Down the Kicker");
        let kicker_command = KickerCommand {
            kick_type: KickType::Kick,
            kick_trigger: KickTrigger::Disabled,
            kick_strength: 0.0,
            charge_allowed: false,
        };

        for _ in 0..20 {
            let kicker_status = ctx.local.kicker_controller.service(kicker_command, ctx.local.fake_spi).unwrap();
            log::info!("Status: {:?}", kicker_status);
            Systick::delay(100u32.millis()).await;
        }
    }
}
