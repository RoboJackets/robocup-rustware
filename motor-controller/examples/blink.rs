//!
//! Basic Example that uses defmt logger to log on and off
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use rtic_monotonics::stm32::prelude::*;
    use motor_controller::TIM3_CLOCK_HZ;

    stm32_tim3_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {

    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(_ctx: init::Context) -> (Shared, Local) {
        Mono::start(TIM3_CLOCK_HZ);

        blink::spawn().ok();

        (
            Shared {

            },
            Local {

            }
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn blink(_ctx: blink::Context) {
        loop {
            defmt::info!("On!!!");
            Mono::delay(1_000.millis()).await;
            defmt::info!("Off!!!");
            Mono::delay(1_000.millis()).await;
        }
    }
}