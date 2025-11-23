#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
extern crate alloc;

use embedded_alloc::Heap;
use embedded_graphics::prelude::*;
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT1, GPT2])]
mod app {
    use alloc::format;
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
    use embedded_graphics::mono_font::ascii::FONT_6X10;
    use embedded_graphics::mono_font::MonoTextStyle;
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::text::Text;
    use rtic::Mutex;

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
    }

    #[shared]
    struct Shared {
        voltages: [f32; 6],
        dark_avg: f32,
        bright_avg: f32,
        display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P16, P17>, 3>>,
            DisplaySize128x64,
            ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
        >,
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
        unsafe { // needed, unfortunately
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
        let mut display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P16, P17>, 3>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        > = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().ok();

        read_voltages::spawn().ok();
        calibration::spawn().ok();

        (Shared {voltages: [0.0; 6], dark_avg: 0f32, bright_avg: 0f32, display}, Local {
            adc: adc1,
            pin18,
            pin19,
            pin20,
            pin21,
            pin22,
            pin23,
            digital0,
            poller,
        })
    }
    #[task(priority=1, shared = [voltages, dark_avg, bright_avg], local = [adc, pin18, pin19, pin20, pin21, pin22, pin23])]
    async fn read_voltages(mut cx: read_voltages::Context) {
        let v = [
            cx.local.adc.read_blocking(cx.local.pin18) as f32 * VCC / ADC_MAX as f32,
            cx.local.adc.read_blocking(cx.local.pin19) as f32 * VCC / ADC_MAX as f32,
            cx.local.adc.read_blocking(cx.local.pin20) as f32 * VCC / ADC_MAX as f32,
            cx.local.adc.read_blocking(cx.local.pin21) as f32 * VCC / ADC_MAX as f32,
            cx.local.adc.read_blocking(cx.local.pin22) as f32 * VCC / ADC_MAX as f32,
            cx.local.adc.read_blocking(cx.local.pin23) as f32 * VCC / ADC_MAX as f32
        ];
        cx.shared.voltages.lock(|voltages| *voltages = v);
        (cx.shared.dark_avg, cx.shared.bright_avg).lock(|dark, bright| {
            if (*dark != 0f32 && *bright != 0f32) { // calibration complete
                // TODO: check if any are low or high. write actual speed logic
            }
        })
    }


    #[task(priority = 2, local = [digital0], shared = [voltages, dark_avg, bright_avg, display])]
    async fn calibration(mut cx: calibration::Context) {
        cx.local.digital0.set_high().expect("TODO: panic message");
        Systick::delay(1000u32.millis()).await; // blocking delay
        // TODO: Verify that the prior task is actually running.

        let dark_voltages = (cx.shared.voltages).lock(|voltages| *voltages);
        let dark_avg_local: f32 = avg_f32_array(&dark_voltages);
        cx.shared.dark_avg.lock(|dark_avg| *dark_avg = dark_avg_local);


        cx.local.digital0.set_low().expect("TODO: panic message");
         Systick::delay(1000u32.millis()).await; // blocking delay
        // TODO: Verify that the prior task is actually running.

        let bright_voltages = (cx.shared.voltages).lock(|voltages| *voltages);
        let bright_avg_local = avg_f32_array(&bright_voltages);
        cx.shared.bright_avg.lock(|bright_avg| {
            *bright_avg = bright_avg_local;
        });

        log::info!("Dark Voltages: {:?}", dark_voltages);
        log::info!("Dark Avg: {}", dark_avg_local);
        log::info!("Bright Voltages: {:?}", bright_voltages);
        log::info!("Bright Avg: {}", bright_avg_local);
        cx.shared.display.lock(|display| {
            display.clear();
            Text::new(
                &format!(
                    "Dark Voltages: {:?}\nDark Avg: {}\nBright Voltages: {:?}\nBright Avg: {}\n",
                    dark_voltages, dark_avg_local, bright_voltages, bright_avg_local
                ),
                Point { x: 0, y: 32 },
                MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            ).draw(display).expect("TODO: panic message");
            display.flush().ok();
        });
    }


    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(priority = 2, binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
