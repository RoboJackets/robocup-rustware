//!
//! This program programs the kicker board with the most recently
//! built kicker binary.
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

    use robojackets_robocup_control::{
        spi::FakeSpi, Delay2, KickerProg, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        delay2: Delay2,
        fake_spi: FakeSpi,
        kicker_programmer: KickerProg,
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins,
            mut gpio1,
            mut gpio2,
            usb,
            mut gpt2,
            pit: (_pit0, _pit1, _pit2, pit3),
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit3);
        let kicker_reset = gpio2.output(pins.p37);
        kicker_reset.set();
        let kicker_csn = gpio1.output(pins.p38);
        kicker_csn.set();
        let fake_spi = FakeSpi::new(
            gpio1.output(pins.p27),
            gpio1.output(pins.p26),
            gpio1.input(pins.p39),
            pit_delay,
        );

        let kicker_programmer = KickerProgrammer::new(kicker_csn, kicker_reset);

        program_kicker::spawn().ok();

        (
            Shared {},
            Local {
                fake_spi,
                delay2,
                kicker_programmer,
                poller,
            },
        )
    }

    #[task(
        local = [fake_spi, delay2, kicker_programmer],
        priority = 1
    )]
    async fn program_kicker(ctx: program_kicker::Context) {
        let result = ctx
            .local
            .kicker_programmer
            .program_kicker(ctx.local.fake_spi, ctx.local.delay2);

        match result {
            Ok(_) => log::info!("Kicker Programming Successful!!!"),
            Err(err) => loop {
                log::error!("Kicker Programming Failed: {:?}", err);

                Systick::delay(1_000u32.millis()).await;
            },
        }
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
