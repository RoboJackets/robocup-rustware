//!
//! Program to control the speed of the dribbler based on the 0-100%
//! of maximum speed (defined)
//! 

#![no_std]
#![no_main]

use panic_probe as _;

use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use motor_controller::{hall_to_phases, set_pwm_output, phase_number, OvercurrentComparator, HS1, HS2, HS3, Phase};
    use stm32f0xx_hal::{
        pac::{EXTI, TIM1, USART1, TIM2},
        prelude::*,
        pwm::{PwmChannels, C1, C1N, C2, C2N, C3, C3N},
        serial::{self, Serial},
        timers::{self, Timer},
        gpio::{gpioa::{PA14, PA15}, Alternate, AF1},
    };

    const PWM_DUTY_CYCLE: u16 = 200;

    #[local]
    struct Local {
        // Hall Sensor 1
        hs1: HS1,
        // Hall Sensor 2
        hs2: HS2,
        // Hall Sensor 3
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
        
        // Overcurrent Comparataor
        overcurrent_comparator: OvercurrentComparator,
        /// Timer 2 (used for motor control interrupt)
        tim2: Timer<TIM2>,
        
        // The serial peripheral
        // serial: Serial<USART1, PA14<Alternate<AF1>>, PA15<Alternate<AF1>>>,
    }

    #[shared]
    struct Shared {
        /// The pin interrupt register
        exti: EXTI,
        /// The number of rotations per millisecond
        rotations_per_ms: f32,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut rcc = ctx.device.RCC.configure().sysclk(48.mhz()).freeze(&mut ctx.device.FLASH);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let gpiof = ctx.device.GPIOF.split(&mut rcc);

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

        let hs1 = cortex_m::interrupt::free(|cs| gpioa.pa0.into_pull_down_input(cs));
        let hs2 = cortex_m::interrupt::free(|cs| gpioa.pa1.into_pull_down_input(cs));
        let hs3 = cortex_m::interrupt::free(|cs| gpioa.pa2.into_pull_down_input(cs));

        let pa11 = cortex_m::interrupt::free(|cs| gpioa.pa11.into_push_pull_output(cs));
        let pb12 = cortex_m::interrupt::free(|cs| gpiob.pb12.into_pull_down_input(cs));
        let pf6 = cortex_m::interrupt::free(|cs| gpiof.pf6.into_push_pull_output(cs));
        let pf7 = cortex_m::interrupt::free(|cs| gpiof.pf7.into_push_pull_output(cs));

        let exti = ctx.device.EXTI;
        let syscfg = ctx.device.SYSCFG;

        let mut overcurrent_comparator = OvercurrentComparator::new(pa11, pf6, pf7, pb12);
        overcurrent_comparator.stop_gate_drivers(true);
        overcurrent_comparator.set_threshold(motor_controller::OvercurrentThreshold::T1);
        overcurrent_comparator.set_interrupt(&syscfg, &exti);

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
        ch3n.enable();

        // let (tx, rx) = cortex_m::interrupt::free(move |cs| {
        //     (
        //         gpioa.pa14.into_alternate_af1(cs),
        //         gpioa.pa15.into_alternate_af1(cs)
        //     )
        // });

        // let mut serial = Serial::usart1(
        //     ctx.device.USART1,
        //     (tx, rx),
        //     115_200.bps(),
        //     &mut rcc
        // );
        // serial.listen(serial::Event::Rxne);

        let mut tim2 = Timer::tim2(ctx.device.TIM2, 1_000.hz(), &mut rcc);
        tim2.listen(timers::Event::TimeOut);

        (
            Shared {
                exti,
                rotations_per_ms: 0.0,
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
                overcurrent_comparator,
                tim2,
                // serial,
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
            ch3n,
            tim2,
            iteration: u32 = 0,
            last_phase: u8 = 0,
            rotations_idx: usize = 0,
            rotations_buffer: [u8; 10_000] = [0u8; 10_000],
        ],
        shared = [
            rotations_per_ms,
        ],
        binds = TIM2
    )]
    fn revs_per_second(ctx: revs_per_second::Context) {
        if *ctx.local.iteration == 0 {
            *ctx.local.last_phase = phase_number(
                ctx.local.hs1.is_high().unwrap(),
                ctx.local.hs2.is_high().unwrap(),
                ctx.local.hs3.is_high().unwrap(),
            );
            return;
        }

        // 100_000 iterations = ~1 second so 100 iterations = 1 millisecond
        if *ctx.local.iteration % 10_000 == 0 {
            let rotations_per_ms = ctx.local.rotations_buffer.iter().map(|v| *v as u64).sum::<u64>() as f32 / (6.0 * 100.0);
            defmt::info!("Speed (rotations/ms): {}", rotations_per_ms);
        }

        let hs1 = ctx.local.hs1.is_high().unwrap();
        let hs2 = ctx.local.hs2.is_high().unwrap();
        let hs3 = ctx.local.hs3.is_high().unwrap();

        // Update the phase rotations
        let phase_num = phase_number(hs1, hs2, hs3);
        let rotation = if phase_num >= *ctx.local.last_phase {
            phase_num - *ctx.local.last_phase
        } else {
            6 + phase_num - *ctx.local.last_phase
        };
        ctx.local.rotations_buffer[*ctx.local.rotations_idx] = rotation;
        *ctx.local.rotations_idx = (*ctx.local.rotations_idx + 1) % 10_000;

        let phases = hall_to_phases(hs1, hs2, hs3, true);

        set_pwm_output(
            ctx.local.ch1,
            ctx.local.ch1n,
            phases[0],
            PWM_DUTY_CYCLE,
        );

        set_pwm_output(
            ctx.local.ch2,
            ctx.local.ch2n,
            phases[1],
            PWM_DUTY_CYCLE,
        );

        set_pwm_output(
            ctx.local.ch3,
            ctx.local.ch3n,
            phases[2],
            PWM_DUTY_CYCLE,
        );

        *ctx.local.iteration += 1;
    }

    // #[task(
    //     local = [
    //         hs1,
    //         hs2,
    //         hs3,
    //         ch1,
    //         ch1n,
    //         ch2,
    //         ch2n,
    //         ch3,
    //         ch3n,
    //         tim2,
    //     ],
    //     binds = TIM2
    // )]
    // fn motor_state_check(ctx: motor_state_check::Context) {

    // }

    // #[task(
    //     local = [serial],
    //     binds = USART1,
    //     priority = 1
    // )]
    // fn serial_received(ctx: serial_received::Context) {

    // }

    #[task(
        local = [overcurrent_comparator],
        shared = [exti],
        binds = EXTI0_1
    )]
    fn overcurrent_interrupt(_ctx: overcurrent_interrupt::Context) {
        panic!("Overcurrent Detected");
    }
}