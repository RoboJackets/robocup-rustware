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

    use imxrt_hal::gpio::Trigger;
    use rtic_monotonics::systick::*;
    use teensy4_bsp::board::{self, Led};

    use robojackets_robocup_control::peripherals::*;

    const HEAP_SIZE: usize = 4096;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        // The logging poller
        poller: imxrt_log::Poller,
        // True if suppling power to the motor board
        power_state: bool,
    }

    #[shared]
    struct Shared {
        kill_n: Killn,
        power_switch: PowerSwitch,
        led: Led,
        gpio1: Gpio1,
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

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();

        let power_switch = gpio1.input(pins.p40);

        //defult to suppling power on first boot
        let power_state = true;

        let led = gpio2.output(pins.p13);

        startup::spawn().ok();

        (
            Shared {
                kill_n,
                power_switch,
                led,
                gpio1,
            },
            Local {
                poller,
                power_state,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1,shared=[gpio1,power_switch])]
    async fn startup(ctx: startup::Context) {
        Systick::delay(2000.millis()).await;
        (ctx.shared.gpio1, ctx.shared.power_switch).lock(|gpio1, power_switch| {
            gpio1.set_interrupt(&power_switch, Some(Trigger::Low));
        });
    }

    ///This task kills the motor board when the power switch is pressed.
    #[task(binds = GPIO1_COMBINED_16_31, shared = [power_switch, kill_n,led],local=[power_state])]
    fn power_switch_interrupt(ctx: power_switch_interrupt::Context) {
        log::info!("Killing Motor Board!");
        (ctx.shared.kill_n, ctx.shared.led).lock(|kill_n, led| {
            kill_n.clear();
            led.set();
        });
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
