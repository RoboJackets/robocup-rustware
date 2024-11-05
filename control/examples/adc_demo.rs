//!
//! Demo program that reads a value from the Teensy's ADC on pin 1 every 1
//! second
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use rtic_monotonics::systick::*;
    use teensy4_pins::t41::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::adc::{Adc, AnalogInput};

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        adc: Adc<1>,
        a1: AnalogInput<P24, 1>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        let board::Resources {
            usb,
            pins,
            adc1,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let mut a1 = AnalogInput::new(pins.p24);

        read_adc::spawn().ok();

        (
            Shared {

            },
            Local {
                adc: adc1,
                a1,
            }
        )
    }

    #[task(local = [adc, a1], priority = 1)]
    async fn read_adc(ctx: read_adc::Context) {
        loop {
            let reading: u16 = ctx.local.adc.read_blocking(ctx.local.a1);

            log::info!("Read: {}", reading);

            Systick::delay(1_000u32.millis()).await;
        }
    }
}