//!
//! Test for communicating with the new motor boards via UART
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
extern crate alloc;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {

    use bsp::board;
    use embedded_hal::blocking::serial::Write;
    use embedded_hal::serial::Read;
    use imxrt_hal::lpuart::{self, Direction};
    use teensy4_bsp::{self as bsp, board::Lpuart2};

    use common::motor::{MotorCommand, MOTOR_COMMAND_SIZE, PID_RESPONSE_SIZE};
    use ncomm_utils::packing::Packable;
    use rtic_monotonics::systick::*;

    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        move_speed: u32,
    }

    #[shared]
    struct Shared {
        uart: Lpuart2,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins, usb, lpuart2, ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let mut uart = board::lpuart(lpuart2, pins.p14, pins.p15, 115_200);
        uart.disable(|uart| {
            uart.disable_fifo(Direction::Rx);
            uart.disable_fifo(Direction::Tx);
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            uart.set_parity(None);
        });

        delay_main::spawn().ok();

        (Shared { uart }, Local { move_speed: 0 })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [uart],
        local = [move_speed],
        priority = 1
    )]
    async fn send_data(mut ctx: send_data::Context) {
        ctx.shared.uart.lock(|uart| {
            let mut buf = [0x00; MOTOR_COMMAND_SIZE];
            let speed = *ctx.local.move_speed;
            let command = MotorCommand::Move {
                ticks_per_second: speed,
            };
            command.pack(&mut buf).unwrap();
            uart.bwrite_all(&buf).unwrap();
            log::info!("Sent speed command: {}", speed);
        });

        *ctx.local.move_speed += 10000;
        if *ctx.local.move_speed > 50000 {
            *ctx.local.move_speed = 0;
        }
        delay_main::spawn().ok();
    }

    #[task(priority = 1)]
    async fn delay_main(ctx: delay_main::Context) {
        Systick::delay(4000.millis()).await;

        send_data::spawn().ok();
    }
}
