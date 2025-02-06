//!
//! Example program to read the HAL-effect sensors and print out the current state
//! of the motor
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use rtic_monotonics::{stm32::prelude::*, stm32_tim2_monotonic};
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

        let hs1 = cortex_m::interrupt::free(|cs| gpioa.pa0.into_pull_down_input(cs));
        let hs2 = cortex_m::interrupt::free(|cs| gpioa.pa1.into_pull_down_input(cs));
        let hs3 = cortex_m::interrupt::free(|cs| gpioa.pa2.into_pull_down_input(cs));

        let exti = ctx.device.EXTI;
        let syscfg = ctx.device.SYSCFG;

        syscfg.exticr1.modify(|_, w| unsafe { w.exti1().bits(12) });
        exti.imr.modify(|_, w| w.mr12().set_bit());
        exti.rtsr.modify(|_, w| w.tr12().set_bit());

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
}