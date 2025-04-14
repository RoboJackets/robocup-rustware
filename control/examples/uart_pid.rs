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

    use crate::common::motor::{MotorCommand, MOTOR_COMMAND_SIZE};
    use rtic_monotonics::systick::*;

    struct TestParams {
        kp: f32,
        ki: f32,
        kd: f32,
    }

    #[local]
    struct Local {
        current_params: TestParams,
    }

    #[shared]
    struct Shared {
        uart: Lpuart2,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

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

        send_data::spawn().ok();

        (
            Shared { uart },
            Local {
                current_params: TestParams {
                    kp: 1.0,
                    ki: 1.0,
                    kd: 1.0,
                },
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
        shared = [uart],
        priority = 1
    )]
    async fn send_data(mut ctx: send_data::Context) {
        ctx.shared.uart.lock(|uart| {
            let mut buf = [0x00; MOTOR_COMMAND_SIZE];
            let command = MotorCommand::SetPid {
                kp: ctx.local.current_params.kp,
                ki: ctx.local.current_params.ki,
                kd: ctx.local.current_params.kd,
            };
            command.pack(&mut buf).unwrap();
            uart.write(&buf).unwrap();
            log::info!(
                "Sent: kp: {}, ki: {}, kd: {}",
                ctx.local.current_params.kp,
                ctx.local.current_params.ki,
                ctx.local.current_params.kd
            );
        });
    }

    #[task(
        shared = [uart],
        priority = 1,
        binds = LPUART2
    )]
    fn receive_data(mut ctx: receive_data::Context) {
        let mut buffer = [0x00; PID_RESPONSE_SIZE];
        ctx.shared.uart.lock(|uart| {
            for i in 0..buffer.len() {
                buffer[i] = uart.read().unwrap();
            }
            log::info!("Received data!", buffer);
        });

        //start dumping it to the log
        #[task(priority = 1)]
        async fn delay_main(ctx: delay_main::Context) {
            Systick::delay(1000.millis()).await;

            send_data::spawn().ok();
        }
        for i in 0..1000 {
            let err = f32::from_le_bytes(buffer[i * 4..(i + 1) * 4].try_into().unwrap());
            log::info!("{}, {}", i, err);
        }
        log::info!("Done receiving data!");
    }
}
