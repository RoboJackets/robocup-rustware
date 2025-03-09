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
    use stm32f0xx_hal::{gpio::{gpiob::{PB6, PB7}, Alternate, AF0}, pac::USART1, prelude::*, serial::{Event, Serial}};

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[local]
    struct Local {
        serial: Serial<USART1, PB6<Alternate<AF0>>, PB7<Alternate<AF0>>>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(TIM2_CLOCK_HZ);

        let mut rcc = ctx.device.RCC.configure().sysclk(48.mhz()).freeze(&mut ctx.device.FLASH);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);

        let (tx, rx) = cortex_m::interrupt::free(move |cs| {
            (
                gpiob.pb6.into_alternate_af0(cs),
                gpiob.pb7.into_alternate_af0(cs),
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
        let mut buffer = [0u8; 6];
        for i in 0..buffer.len() {
            let data = ctx.local.serial.read();
            match data {
                Ok(data) => buffer[i] = data,
                Err(_err) => defmt::error!("Error Reading from Serial"),
            }
        }
        defmt::info!("Read: {:?}", buffer);

        defmt::info!("Writing Response");
        let response = [0x44, 0x55];
        for data in response {
            ctx.local.serial.write(data).unwrap();
        }
    }
}