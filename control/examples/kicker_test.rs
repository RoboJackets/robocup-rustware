//!
//! Test for the Robojackets Robocup Kicker Board
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_panic as _;

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::Empty();

use teensy4_bsp as bsp;

#[rtic::app(
    device=bsp,
    peripherals=true,
    dispatchers=[GPT2]
)]
mod app {
    use super::*;

    use main::spi::Spi3;

    use teensy4_bsp::board::{self, LPSPI_FREQUENCY};

    use kicker::{Kicker, KickType, KickTrigger, KickerError};


    #[local]
    struct Local {
        kicker: Kicker<Spi3, Output<P16>, LpspiError, Infallible>,
    }

    #[shared]
    struct Shared {
        spi: Spi3,
        control_message: ControlMessage,
        delay: Delay2,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb,
            mut gpt2,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let spi_pins = Pins {
            psc0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let spi_block = unsafe { LPSPI3::instance() };
        let spi = Spi3::new(Lpspi::new(spi_block, spi_pins), MODE_0, 100_000);

        let kicker_cs = gpio1.output(pins.p16);
        let mut kicker = Kicker::new(kicker_cs);

        kick_immediately::spawn().ok();
        kick_on_breakbeam::spawn().ok();
        chip_immediately::spawn().ok();
        chip_on_breakbeam::spawn().ok();

        (
            Shared {
                spi,
                control_message: ControLmessage,
            },
            Local {
                kicker,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(shared=[spi, control_message, delay2], local=[kicker], priority=1)]
    async fn service_kicker(ctx: service_kicker::Context) {
        ctx.local.kicker.set_charge_allowed(true);

        let result = (ctx.shared.spi, ctx.shared.control_message, ctx.shared.delay2).lock(|spi, control_message, delay| {
            ctx.local.kicker.set_from_control_message(control_message);
            ctx.local.kicker.service(spi, delay)
        });
        match result {
            Ok(_) => log::info!("Able to set the correct kicker command"),
            Err(_) => log::error!("Unable to set the current kicker command"),
        }

        // service_kicker_delay::spawn().ok();
    }

    #[task(shared=[control_message], priority=1)]
    async fn kick_immediately(ctx: kick_fpga_immediately::Context) {
        ctx.shared.control_message.lock(|control_message| {
            *control_message.kick_strength = 0x0F;
            *control_message.shoot_mode = false;
            *control_message.trigger_mode = 0b10;
        });
        service_kicker::spawn().ok();
    }

    #[task(shared=[control_message], priority=1)]
    async fn kick_on_breakbeam(ctx: kick_on_breakbeam::Context) {
        ctx.shared.control_message.lock(|control_message| {
            *control_message.kick_strength = 0x0F;
            *control_message.shoot_mode = false;
            *control_message.trigger_mode = 0b01;
        });
        service_kicker::spawn().ok();
    }

    #[task(shared=[control_message], priority=1)]
    async fn chip_immediately(ctx: chip_immediately::Context) {
        ctx.shared.control_message.lock(|control_message| {
            *control_message.kick_strength = 0x0F;
            *control_message.shoot_mode = true;
            *control_message.trigger_mode = 0b10;
        });
        service_kicker::spawn().ok();
    }

    #[task(shared=[control_message], priority=1)]
    async fn chip_on_breakbeam(ctx: chip_on_breakbeam::Context) {
        ctx.shared.control_message.lock(|control_message| {
            *control_message.kick_strength = 0x0F;
            *control_message.shoot_mode = true;
            *control_message.trigger_mode = 0b01;
        });
        service_kicker::spawn().ok();
    }

    #[task(priority=1)]
    async fn service_kicker_delay(ctx: service_kicker_delay::Context) {
        Systick::delay(20u32.millis()).await;

        service_kicker::spawn().ok();
    }
}