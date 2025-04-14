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
    use motor_controller::TIM2_CLOCK_HZ;
    use stm32f0xx_hal::{gpio::{gpioa::PA8, gpiob::PB1, Output, PushPull}, prelude::*};

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {
        led: PB1<Output<PushPull>>,
        test: PA8<Output<PushPull>>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(TIM2_CLOCK_HZ);

        let mut rcc = ctx.device.RCC.configure().sysclk(48.mhz()).freeze(&mut ctx.device.FLASH);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);

        let led = cortex_m::interrupt::free(|cs| gpiob.pb1.into_push_pull_output(cs));
        let test = cortex_m::interrupt::free(|cs| gpioa.pa8.into_push_pull_output(cs));

        blink::spawn().ok();

        (
            Shared {

            },
            Local {
                led,
                test,
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
        local = [led, test],
        priority = 1
    )]
    async fn blink(ctx: blink::Context) {
        ctx.local.test.set_high().unwrap();
        loop {
            defmt::info!("On!!!");
            ctx.local.led.set_high().unwrap();
            Mono::delay(1_000.millis()).await;
            defmt::info!("Off!!!");
            ctx.local.led.set_low().unwrap();
            Mono::delay(1_000.millis()).await;
        }
    }
}