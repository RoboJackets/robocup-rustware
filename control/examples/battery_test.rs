//!
//! Test battery output from ADC and voltage/percent conversion
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use rtic_monotonics::systick::*;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    // this allows us to define our packages outside the app module
    // we're essetially "bringing them all in"
    use super::*;

    use robojackets_robocup_control::peripherals::BatterySenseT;

    #[local]
    struct Local {
        battery_sensor: BatterySenseT,
    }

    #[shared]
    struct Shared {
        battery_capacity: u16,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins, usb, adc1, ..
        } = board::t41(cx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 36_000_000, systick_token);

        // TODO change the pin input to be an analog input
        let p41_input = bsp::hal::adc::AnalogInput::new(pins.p41);
        let battery_sensor = BatterySenseT::new(adc1, p41_input);
        let current_capacity = 0;

        get_battery_capacity::spawn().ok();
        (
            Shared {
                battery_capacity: current_capacity,
            },
            Local { battery_sensor },
        )
    }

    // (optional) lowest priority tasks that runs only while no other task is running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // wfi: wait-for-interrupt
            cortex_m::asm::wfi();
        }
    }

    #[task(priority=1, shared = [battery_capacity], local = [battery_sensor])]
    async fn get_battery_capacity(mut cx: get_battery_capacity::Context) {
        let capacity = cx
            .local
            .battery_sensor
            .get_percent_capacity()
            .expect("Invalid response for get_percent_capacity");
        cx.shared.battery_capacity.lock(|battery_capacity| {
            *battery_capacity = capacity;
        });

        log::info!("Battreee capacity: {:?}", capacity);

        battery_dly::spawn().ok();
    }

    #[task(priority = 1)]
    async fn battery_dly(_cx: battery_dly::Context) {
        Systick::delay(25000000.micros()).await;
        get_battery_capacity::spawn().ok();
    }
}
