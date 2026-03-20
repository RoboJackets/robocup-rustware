#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;

use motor_controller::OvercurrentComparator;
use stm32f0xx_hal::{
    delay::Delay,
    pac,
    prelude::*,
    pwm::{self, ComplementaryPwm},
};

#[entry]
fn main() -> ! {
    if let Some(mut dp) = pac::Peripherals::take() {
        let mut rcc = dp.RCC.configure().sysclk(48.mhz()).freeze(&mut dp.FLASH);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpiof = dp.GPIOF.split(&mut rcc);

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

        let pa11 = cortex_m::interrupt::free(|cs| gpioa.pa11.into_push_pull_output(cs));
        let pb12 = cortex_m::interrupt::free(|cs| gpiob.pb12.into_pull_down_input(cs));
        let pf6 = cortex_m::interrupt::free(|cs| gpiof.pf6.into_push_pull_output(cs));
        let pf7 = cortex_m::interrupt::free(|cs| gpiof.pf7.into_push_pull_output(cs));

        let mut overcurrent_comparator = OvercurrentComparator::new(pa11, pf6, pf7, pb12);
        overcurrent_comparator.stop_gate_drivers(false);
        overcurrent_comparator.set_threshold(motor_controller::OvercurrentThreshold::T1);
        overcurrent_comparator.set_interrupt(&dp.SYSCFG, &dp.EXTI);
        overcurrent_comparator.clear_interrupt(&dp.EXTI);

        let pwm = pwm::tim1(dp.TIM1, channels, &mut rcc, 1u32.khz());
        let (mut ch1, mut ch1n, mut ch2, mut ch2n, mut ch3, mut ch3n) = pwm;

        let max_duty = ch1.get_max_duty();
        defmt::info!("Max Duty: {}", max_duty);
        ch1.set_dead_time(pwm::DTInterval::DT_5);
        ch1.set_duty(max_duty / 2);
        ch1.enable();
        ch1n.enable();
        ch2.set_duty(max_duty / 4);
        ch2.enable();
        ch2n.enable();
        ch3.set_duty(max_duty / 8);
        ch3.enable();
        ch3n.enable();

        if let Some(cp) = cortex_m::Peripherals::take() {
            let mut delay = Delay::new(cp.SYST, &rcc);

            loop {
                defmt::info!(
                    "Duty Cycles: [{}, {}, {}]",
                    ch1.get_duty(),
                    ch2.get_duty(),
                    ch3.get_duty()
                );

                delay.delay_ms(100u32);
            }
        }

        loop {}
    }

    loop {
        cortex_m::asm::nop();
    }
}
