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

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1, GPT2])]
mod app {
    use super::*;
    use core::mem::MaybeUninit;

    use embedded_hal::digital::v2::InputPin;
    use imxrt_hal::gpio::Trigger;
    use imxrt_hal::gpio::{Input, Output};
    use rtic_monotonics::systick::*;
    use teensy4_bsp::board::{self, Led};
    use teensy4_pins::t41::*;

    use robojackets_robocup_control::peripherals::*;

    const HEAP_SIZE: usize = 4096;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        // The logging poller
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {
        led: Led,
        gpio1: Gpio1,
        gpio2: Gpio2,
        bank_a0: Output<P6>,
        bank_a1: Output<P7>,
        bank_b0: Output<P8>,
        bank_b1: Output<P9>,
        bank_c0: Output<P10>,
        bank_c1: Output<P11>,

        trigger_a: Input<P14>,
        trigger_b: Input<P15>,
        trigger_c: Input<P16>,
        bank_a_on: bool,
        bank_b_on: bool,
        bank_c_on: bool,
    }

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

        let trigger_a = gpio1.input(pins.p14);
        let bank_a0 = gpio2.output(pins.p6);
        let bank_a1 = gpio2.output(pins.p7);
        let bank_a_on = false;

        let trigger_b = gpio1.input(pins.p15);
        let bank_b0 = gpio2.output(pins.p8);
        let bank_b1 = gpio2.output(pins.p9);
        let bank_b_on = false;

        let trigger_c = gpio1.input(pins.p16);
        let bank_c0 = gpio2.output(pins.p10);
        let bank_c1 = gpio2.output(pins.p11);
        let bank_c_on = false;

        let led = gpio2.output(pins.p13);

        gpio1.set_interrupt(&trigger_a, Some(Trigger::RisingEdge));
        gpio1.set_interrupt(&trigger_b, Some(Trigger::RisingEdge));
        gpio1.set_interrupt(&trigger_c, Some(Trigger::RisingEdge));

        // run the first update to set outputs correctly
        update_outputs::spawn().unwrap();

        (
            Shared {
                led,
                gpio1,
                gpio2,
                bank_a0,
                bank_a1,
                bank_b0,
                bank_b1,
                bank_c0,
                bank_c1,
                trigger_a,
                trigger_b,
                trigger_c,
                bank_a_on,
                bank_b_on,
                bank_c_on,
            },
            Local { poller },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    ///This task kills the motor board when the power switch is pressed.
    #[task(binds = GPIO1_COMBINED_16_31, shared=[trigger_a,trigger_b,trigger_c,bank_a_on,bank_b_on,bank_c_on])]
    fn button_update_interrupt(ctx: button_update_interrupt::Context) {
        (
            ctx.shared.trigger_a,
            ctx.shared.trigger_b,
            ctx.shared.trigger_c,
            ctx.shared.bank_a_on,
            ctx.shared.bank_b_on,
            ctx.shared.bank_c_on,
        )
            .lock(
                |trigger_a, trigger_b, trigger_c, bank_a_on, bank_b_on, bank_c_on| {
                    trigger_a.clear_triggered();
                    trigger_b.clear_triggered();
                    trigger_c.clear_triggered();

                    if trigger_a.is_high().unwrap() {
                        *bank_a_on = !*bank_a_on;
                    }
                    if trigger_b.is_high().unwrap() {
                        *bank_b_on = !*bank_b_on;
                    }
                    if trigger_c.is_high().unwrap() {
                        *bank_c_on = !*bank_c_on;
                    }
                },
            );
        update_outputs::spawn().unwrap();
    }

    #[task(priority=1,shared = [bank_a0,bank_a1,bank_b0,bank_b1,bank_c0,bank_c1, bank_a_on,bank_b_on,bank_c_on])]
    async fn update_outputs(ctx: update_outputs::Context) {
        (
            ctx.shared.bank_a0,
            ctx.shared.bank_a1,
            ctx.shared.bank_b0,
            ctx.shared.bank_b1,
            ctx.shared.bank_c0,
            ctx.shared.bank_c1,
            ctx.shared.bank_a_on,
            ctx.shared.bank_b_on,
            ctx.shared.bank_c_on,
        )
            .lock(
                |bank_a0,
                 bank_a1,
                 bank_b0,
                 bank_b1,
                 bank_c0,
                 bank_c1,
                 bank_a_on,
                 bank_b_on,
                 bank_c_on| {
                    if *bank_a_on {
                        bank_a0.set();
                        bank_a1.clear();
                    } else {
                        bank_a0.clear();
                        bank_a1.set();
                    }

                    if *bank_b_on {
                        bank_b0.set();
                        bank_b1.clear();
                    } else {
                        bank_b0.clear();
                        bank_b1.set();
                    }

                    if *bank_c_on {
                        bank_c0.set();
                        bank_c1.clear();
                    } else {
                        bank_c0.clear();
                        bank_c1.set();
                    }
                },
            );
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
