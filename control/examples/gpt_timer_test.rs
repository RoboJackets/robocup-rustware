//!
//! Collection Collects Data from Basic Movement by the Robot and stores the data
//! in flash memory that can be read via report.rs
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(bigint_helper_methods)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use bsp::board;
    use imxrt_hal::gpt::Gpt;
    use teensy4_bsp as bsp;

    use hal::pit::Chained01;
    use hal::timer::Blocking;
    use teensy4_bsp::hal;

    use rtic_monotonics::systick::*;

    use robojackets_robocup_control::{Delay2, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY};

    const HEAP_SIZE: usize = 8192;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        delay: Delay2,
        pit: Chained01,
        gpt1: Gpt<1>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        // Grab the board peripherals
        let board::Resources {
            usb,
            mut gpt1,
            mut gpt2,
            pit: (pit0, pit1, _pit3, _pit4),
            ..
        } = board::t41(ctx.device);

        let mut chained = Chained01::new(pit0, pit1);
        chained.enable();

        // Setup Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        gpt1.enable();

        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        motion_control_update::spawn().ok();

        (
            Shared {},
            Local {
                pit: chained,
                delay: delay2,
                gpt1,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [delay, pit, gpt1], priority=1)]
    async fn motion_control_update(ctx: motion_control_update::Context) {
        Systick::delay(1000u32.millis()).await;

        let start_gpt = ctx.local.gpt1.count();
        let start_pit = ctx.local.pit.current_timer_value();
        ctx.local.delay.block_ms(1_000);
        let end_pit = ctx.local.pit.current_timer_value();
        let end_gpt = ctx.local.gpt1.count();

        log::info!("Elapsed PIT: {}", start_pit - end_pit);
        log::info!("Elapsed GPT: {}", end_gpt - start_gpt);
    }
}
