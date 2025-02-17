//!
//! Test for communicating with the new motor boards via UART
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use embedded_hal::blocking::serial::Write;
    use embedded_hal::serial::Read;
    use imxrt_hal::lpuart::{self, Direction};
    use teensy4_bsp::{self as bsp, board::Lpuart2};
    use bsp::board;

    use rtic_monotonics::systick::*;

    #[local]
    struct Local {

    }

    #[shared]
    struct Shared {
        uart: Lpuart2,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            lpuart2,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let mut uart = board::lpuart(
            lpuart2,
            pins.p14,
            pins.p15,
            115_200,
        );
        uart.disable(|uart| {
            uart.disable_fifo(Direction::Rx);
            uart.disable_fifo(Direction::Tx);
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            uart.set_parity(None);
        });

        send_data::spawn().ok();

        (
            Shared {
                uart,
            },
            Local {

            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [uart],
        priority = 1
    )]
    async fn send_data(mut ctx: send_data::Context) {
        ctx.shared.uart.lock(|uart| {
            uart.bwrite_all(&[0x00, 0x01, 0x02, 0x03, 0x04, 0x05]).unwrap();
        });
        log::info!("Finished Writing");
    }

    #[task(
        shared = [uart],
        priority = 1,
        binds = LPUART2
    )]
    fn receive_data(mut ctx: receive_data::Context) {
        ctx.shared.uart.lock(|uart| {
            let mut buffer = [0x00; 2];
            for i in 0..buffer.len() {
                buffer[i] = uart.read().unwrap();
            }
            log::info!("Received: {:?}", buffer);
        });
        send_data::spawn().unwrap();
    }
}