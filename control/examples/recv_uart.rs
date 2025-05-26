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
    use embedded_hal::blocking::serial::Write;

    use bsp::board;
    use embedded_hal::serial::Read;
    use robojackets_robocup_control::MotorOneUart;
    use teensy4_bsp as bsp;

    use teensy4_bsp::hal::lpuart;

    use rtic_monotonics::systick::*;

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        motor_one_uart: MotorOneUart,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins, usb, lpuart6, ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let mut motor_one_uart = board::lpuart(lpuart6, pins.p1, pins.p0, 9600);
        motor_one_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            uart.set_parity(None);
        });
        motor_one_uart.write_byte(0x12);
        // poll_uart::spawn().ok();

        (Shared { motor_one_uart }, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn blink_led(_ctx: blink_led::Context) {
        loop {
            log::info!("On");

            Systick::delay(1000u32.millis()).await;

            log::info!("Off");

            Systick::delay(1000u32.millis()).await;
        }
    }

    // #[task(
    //     shared = [motor_one_uart],
    //     local = [buffer: [u8; 100] = [0u8; 100]],
    //     priority = 1
    // )]
    // async fn poll_uart(mut ctx: poll_uart::Context) {
    //     ctx.shared.motor_one_uart.lock(|uart| {
    //         if uart.status().contains(lpuart::Status::RECEIVE_FULL) {
    //             let mut buffer = [0u8; 10];
    //             let mut success = true;
    //             for i in 0..10 {
    //                 match nb::block!(uart.read()) {
    //                     Ok(data) => buffer[i] = data,
    //                     Err(err) => {
    //                         log::error!("Error: {:?}", err);
    //                         success = false;
    //                         break;
    //                     }
    //                 }
    //             }

    //             uart.flush_fifo(lpuart::Direction::Rx);
    //             if success {
    //                 log::info!("Buffer: {:?}", buffer);
    //             }
    //         } else {
    //             log::info!("Not Data");
    //         }
    //         uart.clear_status(lpuart::Status::all());
    //     });

    //     wait_to_pull_uart::spawn().ok();
    // }

    // #[task(priority = 1)]
    // async fn wait_to_pull_uart(ctx: wait_to_pull_uart::Context) {
    //     Systick::delay(100u32.millis()).await;
    //     poll_uart::spawn().ok();
    // }

    #[task(
        binds = LPUART6,
        shared = [motor_one_uart],
    )]
    fn receive_data(mut ctx: receive_data::Context) {
        ctx.shared.motor_one_uart.lock(|uart| {
            let status = uart.status();
            uart.clear_status(lpuart::Status::W1C);

            if status.contains(lpuart::Status::RECEIVE_FULL) {
                loop {
                    let data = uart.read_data();
                    log::info!("Data: {}", data.raw());
                    if data.flags().contains(lpuart::ReadFlags::RXEMPT) {
                        break;
                    }
                    if uart.status().contains(lpuart::Status::TRANSMIT_EMPTY) {
                        uart.write_byte(data.into());
                    }
                }
            }
        });
    }

    // #[task(
    //     shared = [motor_one_uart],
    //     local = [buffer: [u8; 100] = [0u8; 100]],
    //     binds = LPUART6
    // )]
    // fn receive_data(mut ctx: receive_data::Context) {
    //     log::info!("HERE");
    //     ctx.shared.motor_one_uart.lock(|uart| {
    //         if uart.status().contains(lpuart::Status::RECEIVE_FULL) {
    //             let mut buffer = [0u8; 10];
    //             let mut success = true;
    //             for i in 0..10 {
    //                 match nb::block!(uart.read()) {
    //                     Ok(data) => buffer[i] = data,
    //                     Err(err) => {
    //                         log::error!("Error: {:?}", err);
    //                         success = false;
    //                         break;
    //                     }
    //                 }
    //             }

    //             uart.flush_fifo(lpuart::Direction::Rx);
    //             if success {
    //                 log::info!("Buffer: {:?}", buffer);
    //             }
    //         } else {
    //             log::info!("Not Data");
    //         }
    //         uart.clear_status(lpuart::Status::all());
    //     });
    // }
}
