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
    use core::num::NonZero;

    use cortex_m::prelude::_embedded_hal_blocking_serial_Write;
    use embedded_hal::serial::{Write, Read};
    use imxrt_hal::lpuart;
    use robojackets_robocup_control::MotorOneUart;
    use teensy4_bsp as bsp;
    use bsp::board;

    use rtic_monotonics::systick::*;

    #[local]
    struct Local {
        
    }

    #[shared]
    struct Shared {
        uart: MotorOneUart,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            mut gpio4,
            lpuart6,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let trigger = gpio4.output(pins.p2);
        trigger.set();

        let mut uart = board::lpuart(
            lpuart6,
            pins.p1,
            pins.p0,
            9600
        );
        uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.enable_fifo(lpuart::Watermark::rx(NonZero::new(2).unwrap()));
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            uart.set_parity(None);
        });
        uart.clear_status(lpuart::Status::all());

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
            uart.bwrite_all(&(2400i32.to_le_bytes())).unwrap();
        });
        log::info!("Finished Writing");
        wait_to_send_data::spawn().ok();
    }

    #[task(priority = 1)]
    async fn wait_to_send_data(_ctx: wait_to_send_data::Context) {
        Systick::delay(50u32.millis()).await;
        send_data::spawn().ok();
    }

    #[task(
        shared = [uart],
        binds = LPUART6,
        priority = 2,
    )]
    fn receive_data(mut ctx: receive_data::Context) {
        let ticks_per_second = ctx.shared.uart.lock(|uart| {
            uart.clear_status(lpuart::Status::W1C);

            let mut buffer = [0u8; 4];
            for i in 0..4 {
                let data = uart.read_data();
                buffer[i] = data.into();
            }
            i32::from_le_bytes(buffer)
        });

        log::info!("Ticks Per Second: {}", ticks_per_second);
    }
}