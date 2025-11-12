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

use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use bsp::board;
    use cortex_m::delay;
    use teensy4_bsp as bsp;
    use robojackets_robocup_control::{
        Adc1, Gpio1, Gpio2, Killn, MotorEn, PowerSwitch, RadioSPI, MIN_BATTERY_VOLTAGE,
    };
    use rtic_monotonics::systick::*;
    use imxrt_hal::adc::AnalogInput;
    use imxrt_hal::lpspi::Pins;
    use robojackets_robocup_control::peripherals::BatterySenseT;

    use teensy4_pins::t41::{P14, P15, P16, P17};

    #[local]
    struct Local {
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {
        adc1: Adc1,
        batt_sense1: AnalogInput<P14, 1>,batt_sense2: AnalogInput<P15, 1>,batt_sense3: AnalogInput<P16, 1>,batt_sense4: AnalogInput<P17, 1>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins, usb, mut adc1, ..
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
        dummy::spawn().ok();
        (Shared {adc1, batt_sense1, batt_sense2, batt_sense3, batt_sense4}, Local {poller})
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

    #[task(priority = 1, shared = [adc1, batt_sense1])]
    async fn measure_lipo(mut ctx: measure_lipo::Context) {
        //calculate raw analog value from teensy, then convert vlaue into V
        let battery_sense1 = (ctx.shared.adc1, ctx.shared.batt_sense1)
            .lock(|adc, batt_sense1| adc.read_blocking(batt_sense1));
        let battery_voltage = (battery_sense1 as f32) * 3.3 / 1023.0;

        //implement OLED display here
        log::info!("Battery Voltage: {}", battery_voltage);

        // // Battery is under voltaged so we should die
        // if battery_voltage < MIN_BATTERY_VOLTAGE {awdawawd
        //     kill_self::spawn().ok();
        // }
        dummy::spawn().ok();
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
