//!
//! Motor driver firmware for the Robojackets robocup team
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use rtic_monotonics::stm32::prelude::*;

    use motor_controller::TIM2_CLOCK_HZ;

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {

    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(_ctx: init::Context) -> (Shared, Local) {
        Mono::start(TIM2_CLOCK_HZ);

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
    async fn print_debug(_ctx: print_debug::Context) {
        loop {
            defmt::info!("On!!!");
            Mono::delay(1_000.millis()).await;
            defmt::info!("Off!!!");
            Mono::delay(1_000.millis()).await;
        }
    }
}