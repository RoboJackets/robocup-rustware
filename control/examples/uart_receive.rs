//!
//! Test for receiving data via uart
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
    use super::*;
    use core::mem::MaybeUninit;

    use imxrt_hal::lpuart;
    use rtic_monotonics::systick::*;
    use teensy4_bsp::board;

    use robojackets_robocup_control::peripherals::*;

    const HEAP_SIZE: usize = 4096;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        motor_one_uart: MotorOneUart,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        #[allow(static_mut_refs)]
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins,
            lpuart4,
            lpuart6,
            lpuart8,
            usb,
            ..
        } = board::t41(ctx.device);

        teensy4_bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let mut motor_one_uart = board::lpuart(lpuart6, pins.p1, pins.p0, 9600);
        motor_one_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
        });
        motor_one_uart.clear_status(lpuart::Status::W1C);

        (Shared { motor_one_uart }, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [
            motor_one_uart,
        ],
        local = [
            buffer: [u8; 4] = [0u8; 4],
            idx: usize = 0,
        ],
        binds = LPUART6,
    )]
    fn motor_one_uart(mut ctx: motor_one_uart::Context) {
        ctx.shared.motor_one_uart.lock(|uart| {
            let data = uart.read_data();
            if data.flags().contains(lpuart::ReadFlags::IDLINE) {
                *ctx.local.idx = 0;
                log::info!("Received: {:#02x}", i32::from_le_bytes(*ctx.local.buffer));
            }
            log::info!("Flags: {:?}", data.flags());
            ctx.local.buffer[*ctx.local.idx] = data.into();
            uart.clear_status(lpuart::Status::W1C);
            *ctx.local.idx += 1;
        })
    }
}
