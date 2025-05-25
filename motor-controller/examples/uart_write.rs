//!
//! Echo command coming over uart
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;
use defmt_rtt as _;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [TSC])]
mod app {
    use stm32f0xx_hal::{pac::TIM2, prelude::*, serial::{self, Serial}, timers::{Event, Timer}};
    use motor_controller::SerialInterface;

    #[local]
    struct Local {
        // The usart serial interface to talk to Teensy
        usart: SerialInterface,
        // the timer
        tim2: Timer<TIM2>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut rcc = ctx.device.RCC.configure().sysclk(48.mhz()).freeze(&mut ctx.device.FLASH);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);

        let (tx, rx) = cortex_m::interrupt::free(|cs| (
            gpioa.pa14.into_alternate_af1(cs),
            gpioa.pa15.into_alternate_af1(cs),
        ));
        let usart = Serial::usart1(ctx.device.USART1, (tx, rx), 9600.bps(), &mut rcc);

        let mut tim2 = Timer::tim2(ctx.device.TIM2, 10.hz(), &mut rcc);
        tim2.listen(Event::TimeOut);

        (
            Shared {

            },
            Local {
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
        local = [usart, tim2],
        binds = TIM2
    )]
    fn write_uart(ctx: write_uart::Context) {
        let command = [0x01, 0x02, 0x03, 0x04];
        for byte in command {
            nb::block!(ctx.local.usart.write(byte)).unwrap();
        }
        nb::block!(ctx.local.usart.flush()).unwrap();
        ctx.local.tim2.clear_irq();
    } 
}