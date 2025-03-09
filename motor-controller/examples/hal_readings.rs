//!
//! Example program to read the HAL-effect sensors
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use rtic_monotonics::stm32::prelude::*;
    use motor_controller::{HS1, HS2, HS3, TIM2_CLOCK_HZ};
    use stm32f0xx_hal::prelude::*;

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {

    }

    #[shared]
    struct Shared {
        hs1: HS1,
        hs2: HS2,
        hs3: HS3,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(TIM2_CLOCK_HZ);

        let mut rcc = ctx.device.RCC.configure().sysclk(48.mhz()).freeze(&mut ctx.device.FLASH);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);

        // Get the hall sensor peripherals
        let hs1 = cortex_m::interrupt::free(|cs| gpioa.pa0.into_pull_down_input(cs));
        let hs2 = cortex_m::interrupt::free(|cs| gpioa.pa1.into_pull_down_input(cs));
        let hs3 = cortex_m::interrupt::free(|cs| gpioa.pa2.into_pull_down_input(cs));

        print_hal_values::spawn().ok();

        (
            Shared {
                hs1,
                hs2,
                hs3,
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

    #[task(
        shared = [
            hs1,
            hs2,
            hs3,
        ],
        priority = 1
    )]
    async fn print_hal_values(ctx: print_hal_values::Context) {
        (
            ctx.shared.hs1,
            ctx.shared.hs2,
            ctx.shared.hs3
        ).lock(|hs1, hs2, hs3| {
            defmt::info!("HS1: {}, HS2: {}, HS3: {}", hs1.is_high(), hs2.is_high(), hs3.is_high());
        });

        wait_100ms::spawn().ok();
    }

    #[task(priority = 1)]
    async fn wait_100ms(_ctx: wait_100ms::Context) {
        Mono::delay(100.millis()).await;
        print_hal_values::spawn().ok();
    }
}