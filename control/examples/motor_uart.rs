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

    use embedded_hal::serial::{Write, Read};
    use imxrt_hal::lpuart;
    use robojackets_robocup_control::MotorOneUart;
    use teensy4_bsp::{self as bsp, board::UART_FREQUENCY};
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
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });

        start_delay::spawn().ok();

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

    #[task(priority = 1)]
    async fn start_delay(_ctx: start_delay::Context) {
        Systick::delay(2_000u32.millis()).await;
        transfer_data::spawn().ok();
    }

    #[task(
        shared = [uart],
        priority = 1,
    )]
    async fn transfer_data(mut ctx: transfer_data::Context) {
        let command = 24_000i32.to_le_bytes();
        ctx.shared.uart.lock(|uart| {
            for i in 0..4 {
                nb::block!(uart.write(command[i])).unwrap();
                nb::block!(uart.flush()).unwrap();
            }
        });

        wait_to_transfer::spawn().ok();
    }

    #[task(priority = 1)]
    async fn wait_to_transfer(ctx: wait_to_transfer::Context) {
        Systick::delay(100u32.millis()).await;
        transfer_data::spawn().ok();
    }

    // #[task(
    //     shared = [uart],
    //     priority = 1
    // )]
    // async fn send_data(mut ctx: send_data::Context) {
    //     ctx.shared.uart.lock(|uart| {
    //         uart.bwrite_all(&(2400i32.to_le_bytes())).unwrap();
    //     });
    //     log::info!("Finished Writing");
    //     wait_to_send_data::spawn().ok();
    // }

    // #[task(priority = 1)]
    // async fn wait_to_send_data(_ctx: wait_to_send_data::Context) {
    //     Systick::delay(50u32.millis()).await;
    //     send_data::spawn().ok();
    // }

    // #[task(
    //     shared = [uart],
    //     binds = LPUART6,
    //     priority = 2,
    // )]
    // fn receive_data(mut ctx: receive_data::Context) {
    //     let ticks_per_second = ctx.shared.uart.lock(|uart| {
    //         uart.clear_status(lpuart::Status::W1C);

    //         let mut buffer = [0u8; 4];
    //         for i in 0..4 {
    //             let data = uart.read_data();
    //             buffer[i] = data.into();
    //         }
    //         i32::from_le_bytes(buffer)
    //     });

    //     log::info!("Ticks Per Second: {}", ticks_per_second);
    // }
}