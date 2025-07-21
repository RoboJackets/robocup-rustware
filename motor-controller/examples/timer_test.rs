//!
//! Test to determine the delay frequency of my timer interrupt
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use stm32f0xx_hal::{
        gpio::{Output, PushPull, gpiob::PB1},
        pac::TIM2,
        prelude::*,
        timers::{Event, Timer},
    };

    #[local]
    struct Local {
        // The onboard led for testing the interrupt frequency
        led: PB1<Output<PushPull>>,
        // Timer 3 (used for motor control interrupt)
        tim2: Timer<TIM2>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut rcc = ctx
            .device
            .RCC
            .configure()
            .sysclk(48.mhz())
            .freeze(&mut ctx.device.FLASH);

        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let led = cortex_m::interrupt::free(|cs| gpiob.pb1.into_push_pull_output(cs));

        let mut tim2 = Timer::tim2(ctx.device.TIM2, 2_000.hz(), &mut rcc);
        tim2.listen(Event::TimeOut);

        (Shared {}, Local { led, tim2 })
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        local = [led, tim2, last_led: bool = false],
        binds = TIM2
    )]
    fn motor_state_check(ctx: motor_state_check::Context) {
        if *ctx.local.last_led {
            ctx.local.led.set_low().unwrap();
            *ctx.local.last_led = false;
        } else {
            ctx.local.led.set_high().unwrap();
            *ctx.local.last_led = true;
        }
        ctx.local.tim2.clear_irq();
    }
}
