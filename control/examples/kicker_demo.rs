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
    use embedded_hal::blocking::delay::DelayMs;

    use rtic_monotonics::systick::*;

    use kicker_controller::{KickTrigger, KickType, Kicker, KickerCommand};

    use robojackets_robocup_control::{spi::FakeSpi, KickerCSn, KickerReset, MotorEn, Killn, GPT_DIVIDER, GPT_CLOCK_SOURCE, GPT_FREQUENCY};

    #[local]
    struct Local {
        kicker_controller: Kicker<KickerCSn, KickerReset>,
        fake_spi: FakeSpi,
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            mut gpt2,
            mut gpio1,
            mut gpio2,
            pit: (_pit0, _pit1, _pit2, pit3),
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();

        delay2.delay_ms(500u32);

        let kicker = Kicker::new(gpio1.output(pins.p38), gpio2.output(pins.p37));

        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit3);
        let fake_spi = FakeSpi::new(
            gpio1.output(pins.p27),
            gpio1.output(pins.p26),
            gpio1.input(pins.p39),
            pit_delay,
        );

        kicker_test::spawn().ok();

        (
            Shared {},
            Local {
                kicker_controller: kicker,
                fake_spi,
                poller,
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
            let kicker_status = ctx
                .local
                .kicker_controller
                .service(kicker_command, ctx.local.fake_spi)
                .unwrap();
            log::info!("Status: {:?}", kicker_status);
            Systick::delay(100u32.millis()).await;
        }
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
