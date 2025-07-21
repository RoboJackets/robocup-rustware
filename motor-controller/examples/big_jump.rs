//!
//! Make a big jump in setpoint (0->X) so we can make sure the robot won't be blowing any
//! fuses
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use motion_control::Pid;
    use motor_controller::{HS1, HS2, HS3, OvercurrentComparator, Phase, hall_to_phases};
    use stm32f0xx_hal::{
        gpio::{Output, PushPull, gpiob::PB1},
        pac::{TIM1, TIM2, TIM3, TIM14},
        prelude::*,
        pwm::{self, C1, C1N, C2, C2N, C3, C3N, ComplementaryPwm, PwmChannels},
        qei::Qei,
        timers::{Event, Timer},
    };

    /// The maximum PWM that is sendable to the motors
    pub const MAXIMUM_OUTPUT: u16 = 100;
    /// The timeout (in milliseconds)
    pub const TIMEOUT_MS: u32 = 100;
    /// The frequency of the motion control loop
    pub const MOTION_CONTROL_FREQUENCY: u32 = 1000;

    /// The kp constant for the pid controller
    static mut KP: f32 = 2.5;
    /// The ki constant for the pid controller
    static mut KI: f32 = 0.2;
    /// the kd constant for the pid controller
    static mut KD: f32 = 0.5;

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
        // Timer 2 (used to schedule the motion control updates)
        tim2: Timer<TIM2>,
        // The pid controller
        pid: Pid,

        // The motor board led
        led: PB1<Output<PushPull>>,
        // The tim14 peripheral
        tim14: Timer<TIM14>,
    }

    #[shared]
    struct Shared {
        // The target number of ticks per second to move the motor at
        setpoint: i32,
        // The current velocity (in ticks per second) of the motor
        current_velocity: i32,
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

        let pwm = pwm::tim1(ctx.device.TIM1, pwm_channels, &mut rcc, 6u32.khz());

        let (mut ch1, mut ch1n, mut ch2, mut ch2n, mut ch3, mut ch3n) = pwm;

        ch1.set_dead_time(pwm::DTInterval::DT_7);
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

        let mut led = cortex_m::interrupt::free(|cs| gpiob.pb1.into_push_pull_output(cs));
        led.set_high().unwrap();

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
        overcurrent_comparator.set_threshold(motor_controller::OvercurrentThreshold::T2);
        overcurrent_comparator.set_interrupt(&syscfg, &exti);
        overcurrent_comparator.clear_interrupt(&exti);

        let mut tim2 = Timer::tim2(ctx.device.TIM2, MOTION_CONTROL_FREQUENCY.hz(), &mut rcc);
        let mut tim14 = Timer::tim14(ctx.device.TIM14, 1.hz(), &mut rcc);

        // Start Interrupts
        tim2.listen(Event::TimeOut);
        tim14.listen(Event::TimeOut);

        let max_duty = ch1.get_max_duty() / 4;
        let mut pid = Pid::new(
            max_duty,
            unsafe { KP },
            unsafe { KI },
            unsafe { KD },
            MOTION_CONTROL_FREQUENCY,
        );
        pid.set_i_limit(5_000.0);

        (
            Shared {
                setpoint: 0,
                current_velocity: 0,
            },
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
                led,
                pid,
                tim14,
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
            tim14,
            led,
            last_led_state: bool = true,
        ],
        shared = [
            setpoint,
        ],
        binds = TIM14
    )]
    fn setpoint_update(mut ctx: setpoint_update::Context) {
        if *ctx.local.last_led_state {
            ctx.local.led.set_low().unwrap();
            *ctx.local.last_led_state = false;
        } else {
            ctx.local.led.set_high().unwrap();
            *ctx.local.last_led_state = true;
        }

        ctx.shared.setpoint.lock(|setpoint| {
            match *setpoint {
                198400 => *setpoint = 0,
                0 => *setpoint = 198400,
                _ => {
                    *setpoint = 198400;
                }
            }
            defmt::info!("New Setpoint: {}", *setpoint);
        });
        ctx.local.tim14.clear_irq();
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
            overcurrent_comparator,
            last_setpoint: i32 = 0,
            iterations: u32 = 0,
        ],
        shared = [
            setpoint,
            current_velocity,
        ],
        binds = TIM2,
        priority = 1
    )]
    /// Update the speed of the motors.  The timer calls an interrupt every 1ms
    fn motion_control_update(mut ctx: motion_control_update::Context) {
        // Setpoint in ticks per second
        let setpoint = ctx.shared.setpoint.lock(|setpoint| *setpoint);
        if setpoint != *ctx.local.last_setpoint {
            defmt::info!(
                "Found New Setpoint: {}; Iterations: {}",
                setpoint,
                *ctx.local.iterations
            );
            *ctx.local.last_setpoint = setpoint;
            *ctx.local.iterations = 0;
        }

        let (pwm, clockwise) = ctx.local.pid.update(setpoint, ctx.local.encoders.count());

        ctx.shared
            .current_velocity
            .lock(|velocity| *velocity = ctx.local.pid.current_velocity);

        let phases = if ctx.local.overcurrent_comparator.is_tripped() {
            [Phase::Zero, Phase::Zero, Phase::Zero]
        } else {
            hall_to_phases(
                ctx.local.hs1.is_high().unwrap(),
                ctx.local.hs2.is_high().unwrap(),
                ctx.local.hs3.is_high().unwrap(),
                clockwise,
            )
        };

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

        *ctx.local.iterations += 1;
        ctx.local.tim2.clear_irq();
    }
}
