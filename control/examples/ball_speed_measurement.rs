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
use embedded_graphics::prelude::*;
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use super::*;
    use bsp::board;
    use graphics::error_screen::ErrorScreen;
    use embedded_hal::digital::v2::OutputPin;
    use teensy4_bsp::board::Lpi2c3;
    use teensy4_bsp::hal::adc::AnalogInput;
    use teensy4_bsp::hal::gpio::Output;
    use teensy4_bsp as bsp;
    use teensy4_pins::t41::*;

    use rtic_monotonics::systick::*;

    use core::mem::MaybeUninit;

    const HEAP_SIZE: usize = 1024 * 8;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const VCC: f32 = 3.3;
    const ADC_MAX: u16 = 1023; // max value for adc according to internet forums

    #[local]
    struct Local {
        adc: bsp::hal::adc::Adc<1>, // ADC1
        pin18: AnalogInput<P18, 1>,
        pin19: AnalogInput<P19, 1>,
        pin20: AnalogInput<P20, 1>,
        pin21: AnalogInput<P21, 1>,
        pin22: AnalogInput<P22, 1>,
        pin23: AnalogInput<P23, 1>,
        digital0: Output<P0>,
        poller: imxrt_log::Poller,
        error_screen: ErrorScreen<'static>
    }

    #[shared]
    struct Shared {
        voltages: [f32; 6],
        high_avg: f32,
        low_avg: f32,
        display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P16, P17>, 3>>,
            DisplaySize128x64,
            ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
        >,
    }
    fn read_all_volts(
        adc: &mut bsp::hal::adc::Adc<1>,
        pin18: &mut AnalogInput<P18, 1>,
        pin19: &mut AnalogInput<P19, 1>,
        pin20: &mut AnalogInput<P20, 1>,
        pin21: &mut AnalogInput<P21, 1>,
        pin22: &mut AnalogInput<P22, 1>,
        pin23: &mut AnalogInput<P23, 1>,
    ) -> [f32; 6] {
        let raw0 = adc.read_blocking(pin18);
        let raw1 = adc.read_blocking(pin19);
        let raw2 = adc.read_blocking(pin20);
        let raw3 = adc.read_blocking(pin21);
        let raw4 = adc.read_blocking(pin22);
        let raw5 = adc.read_blocking(pin23);

        [
            raw0 as f32 * VCC / ADC_MAX as f32,
            raw1 as f32 * VCC / ADC_MAX as f32,
            raw2 as f32 * VCC / ADC_MAX as f32,
            raw3 as f32 * VCC / ADC_MAX as f32,
            raw4 as f32 * VCC / ADC_MAX as f32,
            raw5 as f32 * VCC / ADC_MAX as f32,
        ]
    }

    fn avg_f32_array(arr: &[f32]) -> f32 {
        if arr.is_empty(){
            0.0
        } else {
            arr.iter().sum::<f32>() / arr.len() as f32
        }
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe { // as of now, the code does not run without this block. Figure out why!
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }
        let board::Resources {usb, lpi2c3, pins, mut gpio1, adc1, ..}= board::t41(ctx.device);
        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();


        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let pin18 = AnalogInput::<P18, 1>::new(pins.p18);
        let pin19 = AnalogInput::<P19, 1>::new(pins.p19);
        let pin20 = AnalogInput::<P20, 1>::new(pins.p20);
        let pin21 = AnalogInput::<P21, 1>::new(pins.p21);
        let pin22 = AnalogInput::<P22, 1>::new(pins.p22);
        let pin23 = AnalogInput::<P23, 1>::new(pins.p23);

        let digital0 = gpio1.output(pins.p0);

        let i2c: Lpi2c3 = board::lpi2c(lpi2c3, pins.p16, pins.p17, board::Lpi2cClockSpeed::MHz1);
        let interface = I2CDisplayInterface::new(i2c);
        let display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P16, P17>, 3>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        > = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        let error_screen = ErrorScreen::new(
            "Example Program",
            "Lorem ipsum dolor sit amet, consectetur adipiscing elit, 
            sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.",
        );

        calibration::spawn().ok();
        init_display::spawn().ok();
        // blink_led::spawn().ok();
        (Shared {voltages: [0.0; 6], high_avg: 0f32, low_avg: 0f32, display}, Local {
            adc: adc1,
            pin18,
            pin19,
            pin20,
            pin21,
            pin22,
            pin23,
            digital0,
            poller,
            error_screen
        })
    }

    #[task(priority=1, shared = [display], local = [error_screen])]
    async fn init_display(mut _cx: init_display::Context) {
    
        _cx.shared.display.lock(|display| {
            display.init().ok();
            display.clear();
            _cx.local.error_screen.draw(display).ok();
            display.flush().ok();
        });
    }

    #[task(priority = 1, local = [digital0, adc, pin18, pin19, pin20, pin21, pin22, pin23], shared = [voltages, high_avg, low_avg])]
    async fn calibration(mut cx: calibration::Context) {
        loop {
            cx.local.digital0.set_high().expect("TODO: panic message");
            Systick::delay(1000u32.millis()).await; // blocking delay
            let low_voltages = read_all_volts(
                cx.local.adc,
                cx.local.pin18,
                cx.local.pin19,
                cx.local.pin20,
                cx.local.pin21,
                cx.local.pin22,
                cx.local.pin23,
            );
            let low_avg_local = avg_f32_array(&low_voltages);
            cx.shared.high_avg.lock(|low_avg| {
                *low_avg = low_avg_local;
            });

            log::info!("Dark Voltages: {:?}", low_voltages);
            log::info!("Dark Avg: {}", low_avg_local);


            cx.local.digital0.set_low().expect("TODO: panic message");
             Systick::delay(1000u32.millis()).await; // blocking delay
            let high_voltages = read_all_volts(
                cx.local.adc,
                cx.local.pin18,
                cx.local.pin19,
                cx.local.pin20,
                cx.local.pin21,
                cx.local.pin22,
                cx.local.pin23,
            );
            let high_avg_local = avg_f32_array(&high_voltages);
            cx.shared.high_avg.lock(|high_avg| {
                *high_avg = high_avg_local;
            });

            log::info!("Bright Voltages: {:?}", high_voltages);
            log::info!("Bright Avg: {}", high_avg_local);
    }
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

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(priority = 2, binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
