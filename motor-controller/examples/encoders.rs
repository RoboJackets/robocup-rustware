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
    use rtic_monotonics::{fugit::Instant, stm32::prelude::*};
    use motor_controller::TIM3_CLOCK_HZ;
    use stm32f0xx_hal::{prelude::*, timers::{Event, Timer}, gpio::{gpiob::{PB6, PB7}, Input, PullDown}, pac::{TIM2, TIM14}};

    stm32_tim3_monotonic!(Mono, 200_000);

    #[local]
    struct Local {
        e1: PB6<Input<PullDown>>,
        e2: PB7<Input<PullDown>>,
        last_e1: bool,
        last_e2: bool,
        tim2: Timer<TIM2>,
        last_time: Instant<u64, 1, 200_000>,
        tim14: Timer<TIM14>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(TIM3_CLOCK_HZ);

        let mut rcc = ctx.device.RCC.configure().sysclk(48.mhz()).freeze(&mut ctx.device.FLASH);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);

        let e1 = cortex_m::interrupt::free(|cs| gpiob.pb6.into_pull_down_input(cs));
        let last_e1 = e1.is_high().unwrap();
        let e2 = cortex_m::interrupt::free(|cs| gpiob.pb7.into_pull_down_input(cs));
        let last_e2 = e2.is_high().unwrap();

        let mut tim2 = Timer::tim2(ctx.device.TIM2, 1_000.hz(), &mut rcc);
        // tim2.listen(Event::TimeOut);

        let mut tim14 = Timer::tim14(ctx.device.TIM14, 1.hz(), &mut rcc);
        tim14.listen(Event::TimeOut);

        (
            Shared {

            },
            Local {
                e1,
                e2,
                last_e1,
                last_e2,
                tim2,
                last_time: Mono::now(),
                tim14,
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
        local = [e1, e2, last_e1, last_e2, tim2, last_time],
        binds = TIM2,
    )]
    /// Every ~430 us this is called by the tim2 interrupt
    fn encoder_update(ctx: encoder_update::Context) {
        // Calculate elapsed time
        let now = Mono::now();
        let elapsed_us = match now.ticks().checked_sub(ctx.local.last_time.ticks()) {
            Some(time) => time,
            None => u64::MAX - ctx.local.last_time.ticks() + now.ticks(),
        };
        *ctx.local.last_time = now;

        ctx.local.tim2.clear_irq();
    }

    #[task(
        local = [tim14],
        binds = TIM14,
    )]
    fn motor_speed_update(ctx: motor_speed_update::Context) {
        defmt::info!("Time: {}", Mono::now());

        ctx.local.tim14.clear_irq();
    }
}