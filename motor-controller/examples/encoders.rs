//!
//! Basic Program to Read the Encoders and Periodically Calculate the Ticks/Second
//! the motors are moving
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
    use stm32f0xx_hal::{pac::TIM3, prelude::*, qei::Qei};

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {
        encoders: Qei<TIM3>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(TIM2_CLOCK_HZ);

        let mut rcc = ctx.device.RCC.configure().sysclk(48.mhz()).freeze(&mut ctx.device.FLASH);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);

        let pins = cortex_m::interrupt::free(move |cs| {
            (
                gpioa.pa6.into_alternate_af1(cs),
                gpioa.pa7.into_alternate_af1(cs),
            )
        });
        let encoders = Qei::tim3(ctx.device.TIM3, pins, &mut rcc);

        (
            Shared {

            },
            Local {
                encoders,
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
        local = [encoders],
        priority = 1
    )]
    async fn print_encoders(ctx: print_encoders::Context) {
        loop {
            defmt::info!("Count: {}, Direction: {}", ctx.local.encoders.count(), ctx.local.encoders.read_direction());

            Mono::delay(100.millis()).await;
        }
    }
}