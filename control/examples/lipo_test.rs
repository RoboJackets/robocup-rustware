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
use rtic_monotonics::systick::*;

use embedded_alloc::Heap;

use embedded_graphics::prelude::*;
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use super::*;

    use bsp::board;
    use cortex_m::delay;
    use imxrt_hal::adc::AnalogInput;
    use imxrt_hal::lpspi::Pins;
    use robojackets_robocup_control::peripherals::BatterySenseT;
    use robojackets_robocup_control::{
        Adc1, Gpio1, Gpio2, Killn, MotorEn, PowerSwitch, RadioSPI, MIN_BATTERY_VOLTAGE, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
    };
    use rtic_monotonics::systick::*;
    use teensy4_bsp as bsp;
    use teensy4_bsp::board::Lpi2c1;
    //display stuff
    use graphics::error_screen::ErrorScreen;
    use graphics::main_window::{self, MainWindow};
    use teensy4_bsp::hal::timer::Blocking;
    use embedded_hal::blocking::delay::DelayMs;
    use core::mem::MaybeUninit;

    use teensy4_pins::t41::{P14, P15, P16, P17, P18, P19};

    const HEAP_SIZE: usize = 1024 * 8;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        poller: imxrt_log::Poller,
        main_window: MainWindow<'static>,
        error_screen: ErrorScreen<'static>,
        latency_placeholder: u16,
    }

    #[shared]
    struct Shared {
        adc1: Adc1,
        batt_sense1: AnalogInput<P14, 1>,
        batt_sense2: AnalogInput<P15, 1>,
        batt_sense3: AnalogInput<P16, 1>,
        batt_sense4: AnalogInput<P17, 1>,
        display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
            DisplaySize128x64,
            ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
        >,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }
        let board::Resources {
            pins,
            usb,
            lpi2c1,
            mut adc1,
            mut gpt2,
            mut gpio1,
            mut gpio2,
            ..
        } = board::t41(ctx.device);
        let mut p41_input = bsp::hal::adc::AnalogInput::new(pins.p41);
        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        //let battery_sensor = BatterySenseT::new(adc1, p41_input);
        log::info!("log1");
        adc1.calibrate();
        let mut batt_sense1 = AnalogInput::new(pins.p14);
        let mut batt_sense2 = AnalogInput::new(pins.p15);
        let mut batt_sense3 = AnalogInput::new(pins.p16);
        let mut batt_sense4 = AnalogInput::new(pins.p17);
        adc1.read_blocking(&mut batt_sense1);
        adc1.read_blocking(&mut batt_sense2);
        adc1.read_blocking(&mut batt_sense3);
        adc1.read_blocking(&mut batt_sense4);

        adc1.read_blocking(&mut p41_input);
        let i2c: Lpi2c1 = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::MHz1);
        let interface = I2CDisplayInterface::new(i2c);
        let display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        > = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        let main_window = MainWindow::new(0, "Blue");
        let error_screen = ErrorScreen::new(
            "ERROR",
            "Attempting to restart...",
        );
        let latency_placeholder: u16 = 0;
        init_devices::spawn().ok();
        dummy::spawn().ok();
        (
            Shared {
                adc1,
                batt_sense1,
                batt_sense2,
                batt_sense3,
                batt_sense4,
                display,
            },
            Local { 
                main_window,
                error_screen,
                latency_placeholder,
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

    #[task(priority = 1)]
    async fn dummy(mut ctx: dummy::Context) {
        Systick::delay(10000000.micros()).await;
        measure_lipo::spawn().ok();
    }

    #[task(priority = 1, shared = [adc1, batt_sense1, batt_sense2, batt_sense3, batt_sense4, display], local =[latency_placeholder, main_window])]
    async fn measure_lipo(mut ctx: measure_lipo::Context) {
        //calculate raw analog value from teensy, then convert value into V
        (
            ctx.shared.adc1,
            ctx.shared.batt_sense1,
            ctx.shared.batt_sense2,
            ctx.shared.batt_sense3,
            ctx.shared.batt_sense4,
        )
            .lock(|adc, batt_sense1, batt_sense2, batt_sense3, batt_sense4| {
                let s1 = adc.read_blocking(batt_sense1);
                let s2 = adc.read_blocking(batt_sense2);
                let s3 =adc.read_blocking(batt_sense3);
                let s4 = adc.read_blocking(batt_sense4);

                let battery_voltage1 = (s1 as f32) * 3.3 / 1023.0;
                let battery_voltage2 = (s2 as f32) * 3.3 / 1023.0;
                let battery_voltage3 = (s3 as f32) * 3.3 / 1023.0;
                let battery_voltage4 = (s4 as f32) * 3.3 / 1023.0;

                let avg = (battery_voltage1 + battery_voltage2 + battery_voltage3 + battery_voltage4) / 4.0;
                ctx.local.main_window.battery_percent = ((avg / 12.6) * 100.0) as u32;
                //modify this to show if the kicker is charged/ball is sensed
                ctx.local.main_window.kicker_charged = false;
                ctx.local.main_window.ball_sense = false;
                *ctx.local.latency_placeholder += 1;

                ctx.local.main_window.team = "Georgia Tech";
                //implement OLED display here
                log::info!("Battery Voltage 1: {}", battery_voltage1);
                log::info!("Battery Voltage 2: {}", battery_voltage2);
                log::info!("Battery Voltage 3: {}", battery_voltage3);
                log::info!("Battery Voltage 4: {}", battery_voltage4);
                ctx.shared.display.lock(|display| {
                display.clear();
                ctx.local.main_window.draw(display).ok();
                display.flush().ok();
            });
            });
        // // Battery is under voltaged so we should die
        // if battery_voltage < MIN_BATTERY_VOLTAGE {awdawawd
        //     kill_self::spawn().ok();
        // }
        
        dummy::spawn().ok();
    }
    
    #[task(priority=1, shared = [display])]
    async fn init_devices(mut _cx: init_devices::Context) {
        _cx.shared.display.lock(|display| {
            display.init().ok();
        });
        dummy::spawn().ok();
    }


    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
