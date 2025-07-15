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
    use motion_control::Pid;
    use motor_controller::{
        HS1, HS2, HS3, MOTION_CONTROL_FREQUENCY, OvercurrentComparator, Phase, SerialInterface,
        hall_to_phases,
    };
    use stm32f0xx_hal::{
        gpio::{Output, PushPull, gpiob::PB1},
        pac::{TIM1, TIM2, TIM3},
        prelude::*,
        pwm::{self, C1, C1N, C2, C2N, C3, C3N, ComplementaryPwm, PwmChannels},
        qei::Qei,
        serial::{self, Serial},
        timers::{Event, Timer},
    };

    /// The maximum PWM that is sendable to the motors
    pub const MAXIMUM_OUTPUT: u16 = 100;
    /// The timeout (in milliseconds)
    pub const TIMEOUT_MS: u32 = 100;

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
        _overcurrent_comparator: OvercurrentComparator,
        // Timer 2 (used to schedule the motion control updates)
        tim2: Timer<TIM2>,

        // The motor board led
        led: PB1<Output<PushPull>>,

        // The usart serial interface to talk to the Teensy
        usart: SerialInterface,

        // THe pid motion controller
        pid: Pid,
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
        overcurrent_comparator.set_threshold(motor_controller::OvercurrentThreshold::T1);
        overcurrent_comparator.set_interrupt(&syscfg, &exti);
        overcurrent_comparator.clear_interrupt(&exti);

        let (tx, rx) = cortex_m::interrupt::free(|cs| {
            (
                gpioa.pa14.into_alternate_af1(cs),
                gpioa.pa15.into_alternate_af1(cs),
            )
        });
        let mut usart = Serial::usart1(ctx.device.USART1, (tx, rx), 115200.bps(), &mut rcc);

        let mut tim2 = Timer::tim2(ctx.device.TIM2, MOTION_CONTROL_FREQUENCY.hz(), &mut rcc);

        // Start Interrupts
        usart.listen(serial::Event::Rxne);
        tim2.listen(Event::TimeOut);

        let max_duty = ch1.get_max_duty();

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
                _overcurrent_comparator: overcurrent_comparator,
                tim2,
                led,
                usart,
                pid: Pid::new(max_duty, 3.0, 0.45, 0.25, MOTION_CONTROL_FREQUENCY),
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
            last_encoders_value: u16 = 0,
            iteration: u32 = 0,
            pid,
            led,
            last_led: bool = false,
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
        if *ctx.local.iteration % 2_000 == 0 {
            if *ctx.local.last_led {
                ctx.local.led.set_low().unwrap();
                *ctx.local.last_led = false;
            } else {
                ctx.local.led.set_high().unwrap();
                *ctx.local.last_led = true;
            }
        }
        // Setpoint in ticks per second
        let setpoint = ctx.shared.setpoint.lock(|setpoint| *setpoint);

        let (pwm, clockwise) = ctx.local.pid.update(setpoint, ctx.local.encoders.count());

        ctx.shared
            .current_velocity
            .lock(|velocity| *velocity = ctx.local.pid.current_velocity);

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

    #[task(
        local = [
            usart,
            buffer: [u8; 4] = [0u8; 4],
            idx: usize = 0,
            done: bool = false,
            sending: bool = false,
        ],
        shared = [
            setpoint,
            current_velocity,
        ],
        binds = USART1,
        priority = 1
    )]
    fn usart_interrupt(mut ctx: usart_interrupt::Context) {
        ctx.local.usart.unlisten(serial::Event::Rxne);
        match nb::block!(ctx.local.usart.read()) {
            Ok(data) => {
                if data == 0x11 {
                    *ctx.local.idx = 0;
                } else {
                    ctx.local.buffer[*ctx.local.idx] = data;
                    *ctx.local.idx += 1;
                }
            }
            Err(_err) => defmt::error!("Error Reading"),
        }
        if *ctx.local.idx == ctx.local.buffer.len() {
            let setpoint = i32::from_le_bytes(*ctx.local.buffer);
            ctx.shared.setpoint.lock(|s| *s = setpoint);

            ctx.shared
                .current_velocity
                .lock(|velocity| ctx.local.buffer.copy_from_slice(&velocity.to_le_bytes()));
            for idx in 0..4 {
                nb::block!(ctx.local.usart.write(ctx.local.buffer[idx])).unwrap();
                nb::block!(ctx.local.usart.flush()).unwrap();
            }
            *ctx.local.idx = 0;
        }
        ctx.local.usart.listen(serial::Event::Rxne);
    }
}
