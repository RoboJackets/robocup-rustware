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
    use motor_controller::TIM2_CLOCK_HZ;
    use rtic_monotonics::stm32::prelude::*;
    use stm32f0xx_hal::{
        pac::TIM3,
        prelude::*,
        qei::{Direction, Qei},
    };

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {
        encoders: Qei<TIM3>,
        // pa6: PA6<Input<Floating>>,
        // pa7: PA7<Input<Floating>>,
    }

    #[shared]
    struct Shared {}

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

        let pwm = stm32f0xx_hal::pwm::tim1(ctx.device.TIM1, channels, &mut rcc, 1u32.khz());

        let (mut ch1, mut ch1n, mut ch2, mut ch2n, mut ch3, mut ch3n) = pwm;

        ch1.set_duty(0);
        ch2.set_duty(0);
        ch3.set_duty(0);
        ch1.enable();
        ch1n.enable();
        ch2.enable();
        ch2n.enable();
        ch3.enable();
        ch3n.enable();

        let pins = cortex_m::interrupt::free(move |cs| {
            (
                gpioa.pa6.into_alternate_af1(cs),
                gpioa.pa7.into_alternate_af1(cs),
            )
        });
        let encoders = Qei::tim3(ctx.device.TIM3, pins, &mut rcc);

        print_encoders::spawn().ok();
        // print_pin_states::spawn().ok();

        (
            Shared {},
            Local {
                encoders, // pa6: pins.0,
                          // pa7: pins.1,
            },
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    // #[task(
    //     local = [pa6, pa7],
    //     priority = 1
    // )]
    // async fn print_pin_states(ctx: print_pin_states::Context) {
    //     loop {
    //         defmt::info!("PA6: {}, PA7: {}", ctx.local.pa6.is_high().unwrap(), ctx.local.pa7.is_high().unwrap());
    //     }
    // }

    #[task(
        local = [encoders],
        priority = 1
    )]
    async fn print_encoders(ctx: print_encoders::Context) {
        loop {
            defmt::info!(
                "Count: {}, Direction: {}",
                ctx.local.encoders.count(),
                match ctx.local.encoders.read_direction() {
                    Direction::Downcounting => "Down",
                    Direction::Upcounting => "Up",
                }
            );

            Mono::delay(100.millis()).await;
        }
    }
}
