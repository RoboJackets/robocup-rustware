//!
//! Test battery output from ADC and voltage/percent conversion
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use imxrt_iomuxc::prelude::*;

use bsp::hal::timer::Blocking;

use rtic::app;
use rtic_monotonics::systick::*;

use rotary_switch_rs as rotary_switch;


use io_expander_rs as io_expander;




use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use core::convert::Infallible;

    // this allows us to define our packages outside the app module
    // we're essetially "bringing them all in"
    use super::*;

    // Pin with ADC for initial measurement

    // accounts for our syst_clock to be in 10 kHz (normal is 1 kHz)
    // this means that the granularity for the delay is 0.1 ms per tick
    // therefore we multiply our delay time by a factor of 10
    const SYST_MONO_FACTOR: u32 = 10;

    use main::{
        BatterySenseT
    };

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
            mut pins,
            mut usb,
            mut adc1,
            ..
        } = board::t41(cx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 36_000_000, systick_token);

        let mut battery_sensor = BatterySenseT::new(adc1, pins.p41);
        let mut current_capacity: u16 = 0;
        (
            Shared {
                battery_capacity: current_capacity,
            },
            Local {
                battery_sensor: battery_sensor,
            }
        )
    }

    // (optional) lowest priority tasks that runs only while no other task is running
    #[idle]
    fn idle(_: idle::Context) -> !{
        loop {
            // wfi: wait-for-interrupt
            cortex_m::asm::wfi();
        }
    }

    #[task(priority=1, shared = [battery_capacity], local = [battery_sensor])]
    async fn get_battery_capacity(cx: get_battery_capacity::Context) {
        let capacity: u16 = cx.local.battery_sensor.get_percent_capacity();
        cx.shared.battery_capacity.lock(| battery_capacity | {
            *battery_capacity = capacity;
        });

        Log::info!("Battreee capacity: {:?}", capacity);
    }


}