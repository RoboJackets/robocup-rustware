//!
//! The Teensy SPI is too fast to interface with the
//! kicker so I'm going to try to write an LPSPI implementation
//! using gpio pins to interface with the kicker
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(
    device = teensy4_bsp,
    peripherals = true,
    dispatchers = [GPIO1_INT0]
)]
mod app {
    use core::mem::MaybeUninit;

    use super::*;

    use bsp::board::{self, PERCLK_FREQUENCY};
    use imxrt_iomuxc::{configure, Config, PullKeeper};
    use robojackets_robocup_control::spi::FakeSpi;
    use robojackets_robocup_control::KickerCSn;
    use teensy4_bsp as bsp;

    use bsp::hal::timer::Blocking;

    use rtic_monotonics::systick::*;

    use embedded_hal::blocking::spi::Transfer;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {
        fake_spi: FakeSpi,
        kicker_csn: KickerCSn,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            mut pins,
            mut gpio1,
            usb,
            pit: (_pit0, _pit1, _pit2, pit3),
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit3);

        let miso_config = Config::zero().set_pull_keeper(Some(PullKeeper::Pullup22k));
        configure(&mut pins.p4, miso_config);
        let fake_spi = FakeSpi::new(
            gpio1.output(pins.p27),
            gpio1.output(pins.p26),
            gpio1.input(pins.p39),
            pit_delay,
        );

        write_data_spi::spawn().ok();

        (
            Shared {
                fake_spi,
                kicker_csn: gpio1.output(pins.p38),
            },
            Local { poller },
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [fake_spi, kicker_csn],
        priority = 1,
    )]
    async fn write_data_spi(ctx: write_data_spi::Context) {
        Systick::delay(50u32.millis()).await;

        let result = (ctx.shared.fake_spi, ctx.shared.kicker_csn).lock(|spi, csn| {
            let mut buffer = [0x55, 0x55, 0x55, 0x55];
            csn.clear();
            spi.transfer(&mut buffer).unwrap();
            csn.set();
            buffer
        });

        log::info!(
            "Result: [{:#02x}, {:#02x}, {:#02x}, {:#02x}]",
            result[0],
            result[1],
            result[2],
            result[3]
        );

        delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn delay(_ctx: delay::Context) {
        Systick::delay(50u32.millis()).await;

        write_data_spi::spawn().ok();
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
