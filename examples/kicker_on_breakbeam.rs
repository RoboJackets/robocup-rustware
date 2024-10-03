//!
//! This program programs the kicker board with a program that automatically
//! kicks when the breakbeam is triggered.
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use bsp::board::{self, PERCLK_FREQUENCY};
    use teensy4_bsp as bsp;

    use hal::timer::Blocking;
    use teensy4_bsp::hal;

    use rtic_monotonics::systick::*;

    use kicker_programmer::KickerProgrammer;

    use main::{
        Delay2, KickerProg,
        GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY, spi::FakeSpi,
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {

    }

    #[shared]
    struct Shared {
        delay2: Delay2,
        fake_spi: FakeSpi,
        kicker_programmer: KickerProg,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins,
            mut gpio2,
            mut gpio4,
            usb,
            mut gpt2,
            pit: (_pit0, _pit1, _pit2, pit3),
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit3);
        let kicker_reset = gpio2.output(pins.p6);
        kicker_reset.set();
        let kicker_csn = gpio4.output(pins.p5);
        kicker_csn.set();
        let fake_spi = FakeSpi::new(
            gpio4.output(pins.p2),
            gpio4.output(pins.p3),
            gpio4.input(pins.p4),
            pit_delay,
        );

        let kicker_programmer = KickerProgrammer::new(kicker_csn, kicker_reset);

        program_kicker::spawn().ok();

        (
            Shared {
                fake_spi,
                delay2,
                kicker_programmer,
            },
            Local {

            }
        )
    }

    #[task(
        shared = [fake_spi, delay2, kicker_programmer],
        priority = 1
    )]
    async fn program_kicker(ctx: program_kicker::Context) {
        let result = (
            ctx.shared.fake_spi,
            ctx.shared.delay2,
            ctx.shared.kicker_programmer,
        ).lock(|spi, delay, kicker_programmer| {
            kicker_programmer.program_kick_on_breakbeam(spi, delay)
        });

        match result {
            Ok(_) => log::info!("Kicker Programming Successful!!!"),
            Err(err) => loop {
                log::error!("Kicker Programming Failed: {:?}", err);

                Systick::delay(1_000u32.millis()).await;
            }
        }
    }
}