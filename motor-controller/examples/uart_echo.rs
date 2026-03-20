//!
//! Test program to echo back the uart data incoming to the motor controller
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use motor_controller::SerialInterface;
    use stm32f0xx_hal::{
        pac::TIM2,
        prelude::*,
        serial::{self, Serial},
        timers::{Event, Timer},
    };

    #[local]
    struct Local {
        usart: SerialInterface,
        tim2: Timer<TIM2>,
    }

    #[shared]
    struct Shared {
        setpoint: i32,
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

        let (tx, rx) = cortex_m::interrupt::free(|cs| {
            (
                gpioa.pa14.into_alternate_af1(cs),
                gpioa.pa15.into_alternate_af1(cs),
            )
        });
        let mut usart = Serial::usart1(ctx.device.USART1, (tx, rx), 9600.bps(), &mut rcc);
        usart.listen(serial::Event::Rxne);

        let mut tim2 = Timer::tim2(ctx.device.TIM2, 10.hz(), &mut rcc);
        tim2.listen(Event::TimeOut);

        (
            Shared {
                setpoint: 0,
                current_velocity: 0,
            },
            Local { usart, tim2 },
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [
            setpoint,
            current_velocity,
        ],
        local = [
            tim2,
        ],
        binds = TIM2
    )]
    fn motion_update(mut ctx: motion_update::Context) {
        let setpoint = ctx.shared.setpoint.lock(|setpoint| *setpoint);
        ctx.shared
            .current_velocity
            .lock(|velocity| *velocity = setpoint);

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
        binds = USART1
    )]
    fn uart_interrupt(mut ctx: uart_interrupt::Context) {
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
