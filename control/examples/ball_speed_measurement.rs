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
    use embedded_graphics::mono_font::ascii::*;
    use embedded_graphics::mono_font::MonoTextStyle;
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::text::Text;
    use rtic_monotonics::Monotonic; // for Systick::now()

    const HEAP_SIZE: usize = 1024 * 8;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const LASERS: usize = 4;
    const GAP_M: f32 = 5e-2f32;
    const MPH_PER_CM_PER_TENTH_MS: f32 = 223.69362920544;
    const METERS_PER_SECOND_PER_CM_PER_TENTH_MS: f32 = 100f32;
    const VCC: f32 = 3.3f32;
    const ADC_MAX: u16 = 1023u16; // max value for adc according to internet forums
    const CLOCK_RATE: u32 = 600_000_000;

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
        last_voltages: [f32; LASERS],
        timestamps: [u32; LASERS], // if not signed, subtraction underflow later on
    }

    #[shared]
    struct Shared {
        voltages: [f32; 4],
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

    fn regression_through_origin_slope(x_slice: &[f32], y_slice: &[f32]) -> f32 {
        // Zip the slices and calculate sums in a single pass
        let (sum_xy, sum_x2) = x_slice.iter().zip(y_slice)
            .fold((0.0, 0.0), |(s_xy, s_x2), (&x, &y)| {
                (s_xy + x * y, s_x2 + x * x)
            });

        if sum_x2 == 0.0 {
            0.0
        } else {
            sum_xy / sum_x2
        }
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        unsafe { // needed, unfortunately
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }
        let board::Resources {usb, lpi2c3, pins, mut gpio1, adc1, ..}= board::t41(ctx.device);
        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, CLOCK_RATE, systick_token);

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

        (Shared {voltages: [0f32; 4], dark_avg: 0f32, bright_avg: 0f32, display}, Local {
            adc: adc1,
            pin18,
            pin19,
            pin20,
            pin21,
            pin22,
            pin23,
            digital0,
            poller,
            last_voltages: [0f32; LASERS],
            timestamps: [0u32; LASERS]
        })
    }
    #[task(priority=1, shared = [voltages, dark_avg, bright_avg, display], local = [adc, pin18, pin19, pin20, pin21, pin22, pin23, last_voltages, timestamps])]
    async fn read_voltages(mut cx: read_voltages::Context) {
        loop {
            let v = [
                cx.local.adc.read_blocking(cx.local.pin18) as f32 * VCC / ADC_MAX as f32,
                cx.local.adc.read_blocking(cx.local.pin19) as f32 * VCC / ADC_MAX as f32,
                cx.local.adc.read_blocking(cx.local.pin20) as f32 * VCC / ADC_MAX as f32,
                cx.local.adc.read_blocking(cx.local.pin21) as f32 * VCC / ADC_MAX as f32,
                /*cx.local.adc.read_blocking(cx.local.pin22) as f32 * VCC / ADC_MAX as f32,
                cx.local.adc.read_blocking(cx.local.pin23) as f32 * VCC / ADC_MAX as f32*/
            ];
            cx.shared.voltages.lock(|voltages| {
                *cx.local.last_voltages = *voltages;
                *voltages = v
            });
            (cx.shared.dark_avg).lock(|dark| {
                if *dark != 0f32 {
                    (cx.shared.bright_avg).lock(|bright| {
                        if *bright != 0f32 { // calibration complete
                            'a: for i in 0..LASERS {
                                if ((cx.local.last_voltages[i] - *bright).abs() > (cx.local.last_voltages[i] - *dark).abs())
                                        && ((v[i] - *bright).abs() <= (v[i] - *dark).abs()) {

                                    cx.local.timestamps[i] = cortex_m::peripheral::DWT::cycle_count();
                                    log::info!("For {}, {}", i, cx.local.timestamps[i]);

                                    if i == LASERS - 1 { // last laser
                                        for i in 1..LASERS {
                                            cx.local.timestamps[i] = cx.local.timestamps[i].wrapping_sub(cx.local.timestamps[0]); // normalize
                                            let diff = cx.local.timestamps[i].wrapping_sub(cx.local.timestamps[i-1]);
                                            if diff <= 0 || diff > CLOCK_RATE / 2  { // error validation for longer than .5s between each. epsilon value included for nonnegative -1e-5f32
                                                break 'a;
                                            }
                                        } // if we exit loop cleanly, then we have a valid ball roll

                                        let mut timestamp_x_arr: [f32; LASERS - 1] = [0.0f32; LASERS - 1];
                                        let mut distance_y_arr: [f32; LASERS - 1] = [0.0f32; LASERS - 1];

                                        for i in 1..LASERS {
                                            timestamp_x_arr[i - 1] = cx.local.timestamps[i] as f32 / CLOCK_RATE as f32;
                                            distance_y_arr[i - 1] = i as f32 * GAP_M;
                                        }

                                        let speed = regression_through_origin_slope(&timestamp_x_arr, &distance_y_arr);

                                        log::info!("M/S: {}", speed);
                                        cx.shared.display.lock(|display| {
                                            display.clear();
                                            Text::new(
                                                &format!("Speed (m/s): {}", speed),
                                                Point { x: 0, y: 32 },
                                                MonoTextStyle::new(&FONT_8X13, BinaryColor::On),
                                            ).draw(display).expect("TODO: panic message");
                                            display.flush().ok();
                                        });
                                        
                                    }
                                }
                            }
                        }
                    });
                }            
            });
        }
    }


    #[task(priority = 2, local = [digital0], shared = [voltages, dark_avg, bright_avg, display])]
    async fn calibration(mut cx: calibration::Context) {
        cx.local.digital0.set_high().expect("TODO: panic message");
        Systick::delay(1000u32.millis()).await; // blocking delay
        // TODO: Verify that the prior task is actually running.

        let dark_voltages = (cx.shared.voltages).lock(|voltages| *voltages);
        let dark_avg_local: f32 = avg_f32_array(&dark_voltages);
        cx.shared.dark_avg.lock(|dark_avg| *dark_avg = dark_avg_local);
        log::info!("Dark Voltages: {:?}", dark_voltages);
        log::info!("Dark Avg: {}", dark_avg_local);

        cx.local.digital0.set_low().expect("TODO: panic message");
        Systick::delay(1000u32.millis()).await; // blocking delay
        // TODO: Verify that the prior task is actually running.

        let bright_voltages = (cx.shared.voltages).lock(|voltages| *voltages);
        let bright_avg_local = avg_f32_array(&bright_voltages);
        cx.shared.bright_avg.lock(|bright_avg| {
            *bright_avg = bright_avg_local;
        });
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
