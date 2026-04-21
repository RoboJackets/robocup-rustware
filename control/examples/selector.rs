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
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use bsp::board;
    use teensy4_bsp as bsp;

    use imxrt_hal::gpio::Input;
    use teensy4_pins::t41::{P32, P33, P34, P35};

    use rtic_monotonics::systick::*;

    use robojackets_robocup_control::selector::{get_team_and_id, PIN_CONFIG};

    #[local]
    struct Local {
        poller: imxrt_log::Poller,
        hex0: Input<P35>,
        hex1: Input<P34>,
        hex2: Input<P33>,
        hex3: Input<P32>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            mut gpio4,
            mut pins,
            usb,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        imxrt_iomuxc::configure(&mut pins.p35, PIN_CONFIG);
        let hex0 = gpio2.input(pins.p35);
        imxrt_iomuxc::configure(&mut pins.p34, PIN_CONFIG);
        let hex1 = gpio2.input(pins.p34);
        imxrt_iomuxc::configure(&mut pins.p33, PIN_CONFIG);
        let hex2 = gpio4.input(pins.p33);
        imxrt_iomuxc::configure(&mut pins.p32, PIN_CONFIG);
        let hex3 = gpio2.input(pins.p32);

        read_selector::spawn().unwrap();

        (
            Shared {},
            Local {
                poller,
                hex0,
                hex1,
                hex2,
                hex3,
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
        priority = 1,
        local = [hex0, hex1, hex2, hex3]
    )]
    async fn read_selector(ctx: read_selector::Context) {
        loop {
            let (team, id) = get_team_and_id(
                &ctx.local.hex0,
                &ctx.local.hex1,
                &ctx.local.hex2,
                &ctx.local.hex3,
            );

            log::info!("Team: {:?}, ID: {}", team, id);

            Systick::delay(1000u32.millis()).await;
        }
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
