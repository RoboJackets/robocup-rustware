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

//board support package
use teensy4_bsp as bsp;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use bsp::board;
    use bsp::hal::adc::{Adc, AnalogInput};
    use teensy4_bsp as bsp;

    use teensy4_pins::t41::*; //adds the pins

    use rtic_monotonics::systick::*;

    #[local]
    struct Local {
        poller: imxrt_log::Poller,
        adc: Adc<1>,
        pot_a: AnalogInput<P14, 1>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        //updated this to also get Pins from the board
        let board::Resources {
            usb,
            pins,
            mut gpio1,
            adc1,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        // A0 pin representation
        let laser_pin = pins.p0;

        let mut laser_output = gpio1.output(laser_pin);
        laser_output.set();
        log::info!("Laser is ON (Pin A0)");

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        log_task::spawn().ok();
        // blink_led::spawn().ok();

        let pot_a = bsp::hal::adc::AnalogInput::new(pins.p14);

        log_task::spawn().ok();

        (
            Shared {},
            Local {
                poller,
                adc: adc1, // Pass the ADC controller
                pot_a,     // Pass the initialized pot pin
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    // The Working Task
    #[task(priority = 1, local = [adc,pot_a])]
    async fn log_task(ctx: log_task::Context) {
        let adc = ctx.local.adc;
        let pot_pin = ctx.local.pot_a;
        const THRESHOLD: u16 = 100;

        let value: u16 = adc.read_blocking(pot_pin);

        if value < THRESHOLD {
            // Not broken
            log::info!("Breakbeam is NOT broken. Value: {}", value);
        } else {
            // Broken
            log::info!("Breakbeam is broken. Value: {}", value);
        }

        break_task::spawn().ok();

        //Reads pot_a value
        // let value: u16 = adc.read_blocking(pot_pin);

        // log::info!("Potentiometer (14) Value: {}", value);

        // // This just keeps the log active so you can see "Laser is ON"
        // log::info!("Log task running...");
        // Systick::delay(5000u32.millis()).await;
    }

    // Break task
    #[task(priority = 1)]
    async fn break_task(_ctx: break_task::Context) {
        Systick::delay(1000u32.millis()).await;
        log_task::spawn().ok();
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
