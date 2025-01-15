//!
//! Test for basic forward and backward motor movement
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
    use stm32f0xx_hal::{pac::TIM1, prelude::*, pwm::{PwmChannels, C1, C1N, C2, C2N, C3, C3N}};

    // The value given to eaech motor as the high value for duty cycles
    const MAX_DUTY: u16 = 125;
    // The amount of time between subsequent hall sensor checks for comutation
    const CHECK_DELAY_US: u64 = 5_000;

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {
        // Hall sensor 1
        hs1: HS1,
        // Hall sensor 2
        hs2: HS2,
        // Hall sensor 3
        hs3: HS3,

        // TIM1_CH1
        ch1: PwmChannels<TIM1, C1>,
        // TIM1_CH1N
        ch1n: PwmChannels<TIM1, C1N>,
        // TIM1_CH2
        ch2: PwmChannels<TIM1, C2>,
        // TIM1_CH2N
        ch2n: PwmChannels<TIM1, C2N>,
        // TIM1_CH3
        ch3: PwmChannels<TIM1, C3>,
        // TIM1_CH3N
        ch3n: PwmChannels<TIM1, C3N>,
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

        // Get the hall sensor peripherals
        let hs1 = cortex_m::interrupt::free(|cs| gpioa.pa0.into_pull_down_input(cs));
        let hs2 = cortex_m::interrupt::free(|cs| gpioa.pa1.into_pull_down_input(cs));
        let hs3 = cortex_m::interrupt::free(|cs| gpioa.pa2.into_pull_down_input(cs));

        let channels = cortex_m::interrupt::free(move |cs| {
            (
                gpioa.pa8.into_alternate_af2(cs),
                gpiob.pb13.into_alternate_af2(cs),
                gpioa.pa9.into_alternate_af2(cs),
                gpiob.pb14.into_alternate_af2(cs),
                gpioa.pa10.into_alternate_af2(cs),
                gpiob.pb15.into_alternate_af2(cs),
            )
        });

        let pwm = stm32f0xx_hal::pwm::tim1(
            ctx.device.TIM1,
            channels,
            &mut rcc,
            20u32.khz()
        );

        let (
            mut ch1,
            mut ch1n,
            mut ch2,
            mut ch2n,
            mut ch3,
            mut ch3n,
        ) = pwm;

        ch1.set_duty(0);
        ch1.enable();
        ch1n.set_duty(0);
        ch1n.enable();
        ch2.set_duty(0);
        ch2.enable();
        ch2n.set_duty(0);
        ch2n.enable();
        ch3.set_duty(0);
        ch3.enable();
        ch3n.set_duty(0);
        ch3.enable();

        (
            Shared {

            },
            Local {
                hs1,
                hs2,
                hs3,
                ch1,
                ch1n,
                ch2,
                ch2n,
                ch3,
                ch3n,
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
        local = [
            hs1,
            hs2,
            hs3,
            ch1,
            ch1n,
            ch2,
            ch2n,
            ch3,
            ch3n
        ],
        priority = 1
    )]
    /// Move the motor clockwise according to the following chart:
    /// 
    /// Step | Current | HS1 | HS2 | HS3 | Hall Status
    ///    1 | U -> V  |   1 |   0 |   1 |   5
    ///    2 | U -> W  |   1 |   0 |   0 |   4
    ///    3 | V -> W  |   1 |   1 |   0 |   6
    ///    4 | V -> U  |   0 |   1 |   0 |   2
    ///    5 | W -> U  |   0 |   1 |   1 |   3
    ///    6 | W -> V  |   0 |   0 |   1 |   1
    /// 
    /// Move the motor counter-clockwise according to the following chart:
    /// 
    /// Step | Current | HS1 | HS2 | HS3 | Hall Status
    ///    6 | W -> V  |   1 |   1 |   0 |   6
    ///    5 | W -> U  |   1 |   0 |   0 |   4
    ///    4 | V -> U  |   1 |   0 |   1 |   5
    ///    3 | V -> W  |   0 |   0 |   1 |   1
    ///    2 | U -> W  |   0 |   1 |   1 |   3
    ///    1 | U -> V  |   0 |   1 |   0 |   2
    async fn move_motors(ctx: move_motors::Context) {
        // Spin Clockwise for 5 seconds
        let start_time = Mono::now();
        while Mono::now() - start_time < fugit::Duration::<u64, 1, 1_000_000>::secs(5) {
            let (h1, h2, h3) = (ctx.local.hs1.is_high().unwrap(), ctx.local.hs2.is_high().unwrap(), ctx.local.hs3.is_high().unwrap());
            match (h1, h2, h3) {
                (true, false, true) => {
                    ctx.local.ch1.set_duty(0);
                    ctx.local.ch1n.set_duty(0);
                    ctx.local.ch2.set_duty(0);
                    ctx.local.ch2n.set_duty(MAX_DUTY);
                    ctx.local.ch3.set_duty(255);
                    ctx.local.ch3n.set_duty(0);
                },
                (true, false, false) => {
                    
                },
                (true, true, false) => todo!(),
                (false, true, false) => todo!(),
                (false, true, true) => todo!(),
                (false, false, true) => todo!(),
                _ => defmt::error!("Unknown Hal Sensor Configuration"),
            }

            Mono::delay(CHECK_DELAY_US.millis()).await;
        }

        // Spin CounterClockwise for 5 seconds
        let start_time = Mono::now();
        while Mono::now() - start_time < fugit::Duration::<u64, 1, 1_000_000>::secs(5) {
            let (h1, h2, h3) = (ctx.local.hs1.is_high().unwrap(), ctx.local.hs2.is_high().unwrap(), ctx.local.hs3.is_high().unwrap());
            match (h1, h2, h3) {
                (true, true, false) => todo!(),
                (true, false, false) => todo!(),
                (true, false, true) => todo!(),
                (false, false, true) => todo!(),
                (false, true, true) => todo!(),
                (false, true, false) => todo!(),
                _ => defmt::error!("Unknown Hal Sensor Configuration"),
            }

            Mono::delay(CHECK_DELAY_US.millis()).await;
        }
    }
}