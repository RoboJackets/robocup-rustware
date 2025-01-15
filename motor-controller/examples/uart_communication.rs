//!
//! Testing example detailing UART communication between the Teensy and the Motor Controller
//! Microcontroller
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
    use stm32f0xx_hal::{gpio::{gpioa::{PA2, PA3}, Alternate, AF1}, pac::USART1, prelude::*, serial::{Event, Serial}};

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {
        serial: Serial<USART1, PA2<Alternate<AF1>>, PA3<Alternate<AF1>>>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(TIM2_CLOCK_HZ);

        let mut rcc = ctx.device.RCC.configure().sysclk(48.mhz()).freeze(&mut ctx.device.FLASH);
        let gpioa = ctx.device.GPIOA.split(&mut rcc);

        let (tx, rx) = cortex_m::interrupt::free(move |cs| {
            (
                gpioa.pa2.into_alternate_af1(cs),
                gpioa.pa3.into_alternate_af1(cs),
            )
        });

        let mut serial = Serial::usart1(
            ctx.device.USART1,
            (tx, rx),
            115_200.bps(),
            &mut rcc
        );
        serial.listen(Event::Rxne);

        (
            Shared {

            },
            Local {
                serial,
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
        local = [serial],
        binds = USART1,
        priority = 1,
    )]
    fn serial_received(ctx: serial_received::Context) {
        defmt::info!("Reading Command");
        let mut buffer = [0u8; 2];
        for i in 0..buffer.len() {
            let data = ctx.local.serial.read();
            match data {
                Ok(data) => buffer[i] = data,
                Err(_err) => defmt::error!("Error Reading from Serial"),
            }
        }

        defmt::info!("Writing Response");
        let response = [0x00, 0x00];
        for data in response {
            ctx.local.serial.write(data).unwrap();
        }
    }
}