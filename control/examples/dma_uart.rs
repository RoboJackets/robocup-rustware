//!
//! Test for using dma with the uart peripherals on the teensy
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use core::{mem::MaybeUninit, num::NonZero};

    use cortex_m::prelude::_embedded_hal_blocking_serial_Write;
    use imxrt_hal::{dma::channel::{self, Channel}, lpuart::{self, Interrupts, Status}};
    use rtic_monotonics::systick::*;
    use teensy4_bsp::board::{self, Lpuart4, Lpuart6};

    use super::*;

    const HEAP_SIZE: usize = 4096;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
    }

    #[shared]
    struct Shared {
        uart: Lpuart6,
        receive_uart: Lpuart4,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        #[allow(static_mut_refs)]
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        let board::Resources {
            pins,
            lpuart6,
            lpuart4,
            usb,
            ..
        } = board::t41(ctx.device);

        teensy4_bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let mut uart = board::lpuart(
            lpuart6,
            pins.p1,
            pins.p0,
            115200
        );
        uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
            uart.set_interrupts(Interrupts::RECEIVE_FULL);
        });

        let mut receive_uart = board::lpuart(
            lpuart4,
            pins.p8,
            pins.p7,
            115200
        );
        receive_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            // uart.enable_fifo(lpuart::Watermark::rx(NonZero::new(15).unwrap()));
            uart.set_parity(None);
            uart.set_interrupts(Interrupts::RECEIVE_FULL);
        });

        send_uart::spawn().ok();

        (
            Shared {
                uart,
                receive_uart,
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
    async fn send_uart(mut ctx: send_uart::Context) {
        log::info!("Sending");
        let data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
        ctx.shared.uart.lock(|uart| {
            uart.bwrite_all(&data).unwrap();
        });

        send_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn send_delay(_ctx: send_delay::Context) {
        Systick::delay(200u32.millis()).await;
        send_uart::spawn().ok();
    }

    #[task(
        shared = [receive_uart],
        binds = LPUART4
    )]
    fn receive_uart(mut ctx: receive_uart::Context) {
        let mut buffer = [0u8; 8];

        ctx.shared.receive_uart.lock(|uart| {
            uart.clear_status(Status::W1C);
            let mut index = 0;
            loop {
                let data = uart.read_data();
                if data.flags().contains(lpuart::ReadFlags::RXEMPT) {
                    break;
                }

                buffer[index] = u8::from(data);
                index += 1;
                if index > 8 {
                    break;
                }
            }
        });

        log::info!("Received: {:?}", buffer);
    }
}
