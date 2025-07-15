//!
//! Enable the motor board so we can do stuff with the kicker
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

    use imxrt_hal::{lpuart, timer::Blocking};
    use rtic_monotonics::systick::*;
    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };
    use teensy4_bsp::board;

    use robojackets_robocup_control::{
        motors::{motor_interrupt, send_command},
        peripherals::*,
        GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
    };

    use embedded_hal::blocking::delay::DelayMs;

    const HEAP_SIZE: usize = 4096;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        // The logging poller
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        #[allow(static_mut_refs)]
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins,
            mut gpio1,
            mut gpio2,
            usb,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();

        (Shared {}, Local { poller })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
