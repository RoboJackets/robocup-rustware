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

    use teensy4_bsp::hal::gpio::Output;

    #[local]
    struct Local {
        poller: imxrt_log::Poller,
        adc: Adc<1>,
        pot_a: AnalogInput<P14, 1>,
        laser_output: Output<P0>,
    }

    #[shared]
    struct Shared {
        signal_strength: u16,
    }

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

        let laser_output = gpio1.output(laser_pin);

        // laser_output.set();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let pot_a = bsp::hal::adc::AnalogInput::new(pins.p14);

        pulse_and_read::spawn().ok();

        (
            Shared { signal_strength: 0 },
            Local {
                poller,
                adc: adc1, // Pass the ADC controller
                pot_a,     // Pass the initialized pot pin
                laser_output,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    // Pulsing/Reading Task
    #[task(priority = 1, shared = [signal_strength], local = [adc,pot_a,laser_output])]
    async fn pulse_and_read(ctx: pulse_and_read::Context) {
        let adc = ctx.local.adc;
        let pot_pin = ctx.local.pot_a;
        let mut laser_output = ctx.local.laser_output;

        laser_output.set();
        Systick::delay(1u32.millis()).await;
        let r_on: u16 = adc.read_blocking(pot_pin);

        laser_output.clear();
        Systick::delay(1u32.millis()).await;
        let r_off: u16 = adc.read_blocking(pot_pin);
        // log::info!("Potentiometer (14) Value: {}", value);

        { ctx.shared.signal_strength }.lock(|signal_strength| {
            *signal_strength = r_on - r_off;
        });

        determine_state::spawn().ok();

        // This just keeps the log active so you can see "Laser is ON"
        // log::info!("Log task running...");
        // Systick::delay(5000u32.millis()).await;
    }

    // Decision Logic
    #[task(priority = 1, shared = [signal_strength])]
    async fn determine_state(ctx: determine_state::Context) {
        { ctx.shared.signal_strength }.lock(|signal_strength| {
            if *signal_strength > 50 {
                log::info!("Beam is INTACT. Signal: {}", signal_strength);
            } else {
                log::info!("!!! BEAM BROKEN !!! Signal: {}", signal_strength);
            }
        });

        Systick::delay(50u32.millis()).await;
        pulse_and_read::spawn().ok();
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
