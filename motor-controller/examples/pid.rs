//!
//! PID Tuning for the motor board
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use motor_controller::{HS1, HS2, HS3, OvercurrentComparator, Phase, hall_to_phases, pid::Pid};
    use stm32f0xx_hal::{
        pac::{TIM1, TIM2, TIM3},
        prelude::*,
        pwm::{self, C1, C1N, C2, C2N, C3, C3N, ComplementaryPwm, PwmChannels},
        qei::{Direction, Qei},
        timers::{Event, Timer},
    };

    /// The maximum PWM that is sendable to the motors
    pub const MAXIMUM_OUTPUT: u16 = 100;

    /// The kp constant for the pid controller
    static mut KP: f32 = 1.0;
    /// The ki constant for the pid controller
    static mut KI: f32 = 0.0;
    /// the kd constant for the pid controller
    static mut KD: f32 = 0.0;

    #[local]
    struct Local {
        // The quadrature encoder interface
        encoders: Qei<TIM3>,

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

        // Overcurrent Comparator
        overcurrent_comparator: OvercurrentComparator,
        /// Timer 2 (used to schedule the motion control updates)
        tim2: Timer<TIM2>,
        /// The pid controller
        pid: Pid,
    }

    #[shared]
    struct Shared {
        /// The target number of ticks per second to move the motor at
        setpoint: i32,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut rcc = ctx
            .device
            .RCC
            .configure()
            .sysclk(48.mhz())
            .freeze(&mut ctx.device.FLASH);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let gpiof = ctx.device.GPIOF.split(&mut rcc);

        let pwm_channels = cortex_m::interrupt::free(move |cs| {
            (
                gpioa.pa8.into_alternate_af2(cs),
                gpiob.pb13.into_alternate_af2(cs),
                gpioa.pa9.into_alternate_af2(cs),
                gpiob.pb14.into_alternate_af2(cs),
                gpioa.pa10.into_alternate_af2(cs),
                gpiob.pb15.into_alternate_af2(cs),
            )
        });

        let pwm = pwm::tim1(ctx.device.TIM1, pwm_channels, &mut rcc, 30u32.khz());

        let (mut ch1, mut ch1n, mut ch2, mut ch2n, mut ch3, mut ch3n) = pwm;

        ch1.set_dead_time(pwm::DTInterval::DT_5);
        ch1.set_duty(0);
        ch2.set_duty(0);
        ch3.set_duty(0);
        ch1.disable();
        ch1n.disable();
        ch2.disable();
        ch2n.disable();
        ch3.disable();
        ch3n.disable();

        let qei_pins = cortex_m::interrupt::free(move |cs| {
            (
                gpioa.pa6.into_alternate_af1(cs),
                gpioa.pa7.into_alternate_af1(cs),
            )
        });
        let encoders = Qei::tim3(ctx.device.TIM3, qei_pins, &mut rcc);

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
        overcurrent_comparator.stop_gate_drivers(false);
        overcurrent_comparator.set_threshold(motor_controller::OvercurrentThreshold::T1);
        overcurrent_comparator.set_interrupt(&syscfg, &exti);
        overcurrent_comparator.clear_interrupt(&exti);

        let mut tim2 = Timer::tim2(ctx.device.TIM2, 2_000.hz(), &mut rcc);
        tim2.listen(Event::TimeOut);

        (
            Shared { setpoint: 2500 * 2 },
            Local {
                encoders,
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
                pid: Pid::new(MAXIMUM_OUTPUT, unsafe { KP }, unsafe { KI }, unsafe { KD }),
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
        local = [
            encoders,
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
            pid,
            last_encoders_value: u16 = 0,
            iteration: u32 = 0,
            errors: [f32; 500] = [0.0; 500],
        ],
        shared = [
            setpoint,
        ],
        binds = TIM2,
        priority = 1
    )]
    /// Update the speed of the motors.  The timer calls an interrupt every 1ms
    fn motion_control_update(mut ctx: motion_control_update::Context) {
        let encoder_count = ctx.local.encoders.count();
        let elapsed_encoders =
            if encoder_count < 1000 && *ctx.local.last_encoders_value > u16::MAX - 1000 {
                // Overflow
                (u16::MAX - *ctx.local.last_encoders_value + encoder_count) as i32
            } else if *ctx.local.last_encoders_value < 1000 && encoder_count > u16::MAX - 1000 {
                // Underflow
                -((*ctx.local.last_encoders_value + u16::MAX - encoder_count) as i32)
            } else {
                // Normal
                (encoder_count as i32) - (*ctx.local.last_encoders_value as i32)
            };
        let ticks_per_second = elapsed_encoders * 2_000;

        if *ctx.local.iteration % 100 == 0 {
            defmt::info!("Ticks Per Second: {}", ticks_per_second);
        }
        *ctx.local.last_encoders_value = encoder_count;

        let pwm = ctx.local.ch1.get_max_duty() / 5;
        let clockwise = true;

        // let setpoint = ctx.shared.setpoint.lock(|setpoint| *setpoint);

        // let (pwm, clockwise) = ctx.local.pid.update(
        //     setpoint,
        //     ticks_per_second
        // );
        // if *ctx.local.iteration % 2 == 0 {
        //     ctx.local.errors[*ctx.local.iteration as usize / 2] = ctx.local.pid.last_error();
        // }

        let phases = hall_to_phases(
            ctx.local.hs1.is_high().unwrap(),
            ctx.local.hs2.is_high().unwrap(),
            ctx.local.hs3.is_high().unwrap(),
            clockwise,
        );

        match phases[0] {
            Phase::Positive => {
                ctx.local.ch1.set_duty(pwm);
                ctx.local.ch1.enable();
                ctx.local.ch1n.enable();
            }
            Phase::Zero => {
                ctx.local.ch1.disable();
                ctx.local.ch1n.disable();
            }
            Phase::Negative => {
                ctx.local.ch1.set_duty(0);
                ctx.local.ch1.enable();
                ctx.local.ch1n.enable();
            }
        }

        match phases[1] {
            Phase::Positive => {
                ctx.local.ch2.set_duty(pwm);
                ctx.local.ch2.enable();
                ctx.local.ch2n.enable();
            }
            Phase::Zero => {
                ctx.local.ch2.disable();
                ctx.local.ch2n.disable();
            }
            Phase::Negative => {
                ctx.local.ch2.set_duty(0);
                ctx.local.ch2.enable();
                ctx.local.ch2n.enable();
            }
        }

        match phases[2] {
            Phase::Positive => {
                ctx.local.ch3.set_duty(pwm);
                ctx.local.ch3.enable();
                ctx.local.ch3n.enable();
            }
            Phase::Zero => {
                ctx.local.ch3.disable();
                ctx.local.ch3n.disable();
            }
            Phase::Negative => {
                ctx.local.ch3.set_duty(0);
                ctx.local.ch3.enable();
                ctx.local.ch3n.enable();
            }
        }

        *ctx.local.iteration += 1;
        ctx.local.tim2.clear_irq();
    }
}
