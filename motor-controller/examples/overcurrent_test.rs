//!
//! Test to ensure the overcurrent protection comparator is configured correctly
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use motor_controller::{OvercurrentComparator, OvercurrentThreshold, TIM2_CLOCK_HZ};
    use rtic_monotonics::stm32::prelude::*;
    use stm32f0xx_hal::{pac::EXTI, prelude::*};

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {
        overcurrent_comparator: OvercurrentComparator,
    }

    #[shared]
    struct Shared {
        exti: EXTI,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(TIM2_CLOCK_HZ);

        let mut rcc = ctx
            .device
            .RCC
            .configure()
            .sysclk(48.mhz())
            .freeze(&mut ctx.device.FLASH);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let gpiof = ctx.device.GPIOF.split(&mut rcc);

        let pa11 = cortex_m::interrupt::free(|cs| gpioa.pa11.into_push_pull_output(cs));
        let pb12 = cortex_m::interrupt::free(|cs| gpiob.pb12.into_pull_down_input(cs));
        let pf6 = cortex_m::interrupt::free(|cs| gpiof.pf6.into_push_pull_output(cs));
        let pf7 = cortex_m::interrupt::free(|cs| gpiof.pf7.into_push_pull_output(cs));

        let exti = ctx.device.EXTI;
        let syscfg = ctx.device.SYSCFG;

        let mut overcurrent_comparator = OvercurrentComparator::new(pa11, pf6, pf7, pb12);
        overcurrent_comparator.stop_gate_drivers(false);
        overcurrent_comparator.set_threshold(OvercurrentThreshold::T1);

        overcurrent_interrupt::spawn().ok();

        (
            Shared { exti },
            Local {
                overcurrent_comparator,
            },
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        local = [overcurrent_comparator],
        shared = [exti],
        priority = 1,
    )]
    async fn overcurrent_interrupt(ctx: overcurrent_interrupt::Context) {
        loop {
            defmt::info!(
                "Overcurrent Tripped: {}",
                ctx.local.overcurrent_comparator.is_tripped()
            );

            Mono::delay(100.millis()).await;
        }
    }
}
