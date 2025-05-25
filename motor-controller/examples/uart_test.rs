//!
//! Test program mimicking the main functionality of the robot when it is being commanded via uart
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;
use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use stm32f0xx_hal::{
        gpio::{gpiob::PB1, Output, PushPull}, pac::TIM2, prelude::*, serial::{self, Serial}, timers::{Event, Timer}
    };

    use motor_controller::{SerialInterface, MOTOR_COMMAND};

    #[local]
    struct Local {
        // The usart serial interface to talk to Teensy
        usart: SerialInterface,
        // The timer
        tim2: Timer<TIM2>,
        // The led
        led: PB1<Output<PushPull>>,
    }

    #[shared]
    struct Shared {
        // The setpoint for the motor (in ticks per second)
        setpoint: i32,
        // The current velocity (in ticks per second) of the motor
        current_velocity: i32,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut rcc = ctx.device.RCC.configure().sysclk(48.mhz()).freeze(&mut ctx.device.FLASH);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);

        let mut led = cortex_m::interrupt::free(|cs| gpiob.pb1.into_push_pull_output(cs));
        led.set_high().unwrap();

        let (tx, rx) = cortex_m::interrupt::free(|cs| (
            gpioa.pa14.into_alternate_af1(cs),
            gpioa.pa15.into_alternate_af1(cs),
        ));
        let mut usart = Serial::usart1(ctx.device.USART1, (tx ,rx), 9600.bps(), &mut rcc);
        usart.listen(serial::Event::Rxne);

        let mut tim2 = Timer::tim2(ctx.device.TIM2, 10.hz(), &mut rcc);
        tim2.listen(Event::TimeOut);

        (
            Shared {
                setpoint: 0,
                current_velocity: 0,
            },
            Local {
                led,
                usart,
                tim2,
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
            tim2,
            led,
            last_led: bool = true,
        ],
        shared = [
            setpoint,
            current_velocity,
        ],
        binds = TIM2,
    )]
    fn motion_update(ctx: motion_update::Context) {
        if *ctx.local.last_led {
            ctx.local.led.set_low().unwrap();
            *ctx.local.last_led = false;
        } else {
            ctx.local.led.set_high().unwrap();
            *ctx.local.last_led = true;
        }

        (
            ctx.shared.setpoint,
            ctx.shared.current_velocity
        ).lock(|setpoint, velocity| {
            *velocity = *setpoint;
        });
        ctx.local.tim2.clear_irq();
    }

    #[task(
        local = [
            usart,
            buffer: [u8; 5] = [0u8; 5],
            idx: usize = 0,
            done: bool = false,
            sending: bool = false,
        ],
        shared = [
            setpoint,
            current_velocity,
        ],
        binds = USART1,
    )]
    fn usart_interrupt(mut ctx: usart_interrupt::Context) {
        if *ctx.local.sending {
            ctx.local.usart.unlisten(serial::Event::Txe);
            if *ctx.local.idx == ctx.local.buffer.len() {
                *ctx.local.sending = false;
                ctx.local.usart.listen(serial::Event::Rxne);
            } else {
                nb::block!(ctx.local.usart.write(ctx.local.buffer[*ctx.local.idx])).unwrap();
                *ctx.local.idx += 1;
                ctx.local.usart.listen(serial::Event::Txe);
            }
        } else {
            ctx.local.usart.unlisten(serial::Event::Rxne);
            if let Ok(data) = nb::block!(ctx.local.usart.read()) {
                if data == MOTOR_COMMAND {
                    *ctx.local.idx = 0;
                    ctx.local.buffer[0] = data;
                    ctx.local.usart.listen(serial::Event::Rxne);
                } else if *ctx.local.idx == ctx.local.buffer.len() {
                    ctx.shared.setpoint.lock(|setpoint| {
                        *setpoint = i32::from_le_bytes(ctx.local.buffer[1..5].try_into().unwrap());
                    });

                    ctx.shared.current_velocity.lock(|velocity| {
                        ctx.local.buffer.copy_from_slice(&velocity.to_le_bytes());
                    });

                    *ctx.local.idx = 0;
                    *ctx.local.sending = true;
                    ctx.local.usart.listen(serial::Event::Txe);
                } else {
                    ctx.local.buffer[*ctx.local.idx] = data;
                    *ctx.local.idx += 1;
                    ctx.local.usart.listen(serial::Event::Rxne);
                }
            }
        }
    }
}