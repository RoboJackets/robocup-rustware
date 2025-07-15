//!
//! Testing program to test the frequency of the pit timer
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
    use bsp::board;
    use imxrt_hal::{gpio::Output, pit::Pit};
    use teensy4_bsp as bsp;
    use teensy4_pins::t41::P0;

    const PIT_VALUE: u32 = 100_000;

    #[local]
    struct Local {
        pit: Pit<0>,
        output: Output<P0>,
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            mut gpio1,
            pit: (mut pit0, _, _, _),
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let output = gpio1.output(pins.p0);

        pit0.clear_elapsed();
        pit0.set_load_timer_value(PIT_VALUE);
        pit0.set_interrupt_enable(true);
        pit0.enable();

        (
            Shared {},
            Local {
                pit: pit0,
                output,
                poller,
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
        local = [pit, output, last_state: bool = false],
        binds = PIT
    )]
    fn pit_interrupt(ctx: pit_interrupt::Context) {
        ctx.local.pit.clear_elapsed();
        ctx.local.pit.set_interrupt_enable(false);

        if *ctx.local.last_state {
            ctx.local.output.clear();
            *ctx.local.last_state = false;
        } else {
            ctx.local.output.set();
            *ctx.local.last_state = true;
        }

        ctx.local.pit.clear_elapsed();
        ctx.local.pit.set_load_timer_value(PIT_VALUE);
        ctx.local.pit.set_interrupt_enable(true);
        ctx.local.pit.enable();
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
