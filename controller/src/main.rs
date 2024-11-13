// Cargo.toml dependencies
/*
[dependencies]
teensy4-bsp = "0.4"
imxrt-hal = "0.5"
embedded-hal = "0.2.7"
rtic = { version = "2.0", features = ["thumbv7-backend"] }
rtic-monotonics = { version = "1.0", features = ["cortex-m-systick"] }
log = "0.4"
*/

#![no_std]
#![no_main]

use rtic::app;
use rtic_monotonics::systick::*;
use teensy4_panic as _;

#[app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use teensy4_bsp as bsp;
    use bsp::board;
    
    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        adc: bsp::hal::adc::Adc<1>,
        a1: bsp::hal::adc::AnalogInput<bsp::pins::t41::P16, 1>,
        a2: bsp::hal::adc::AnalogInput<bsp::pins::t41::P17,1>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Get board resources
        let board::Resources {
            pins,
            adc1,
            ..
        } = board::t41(ctx.device);

        // Configure systick for timing
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Configure ADC input on pin P0
        let a1 = bsp::hal::adc::AnalogInput::new(pins.p16);
        let a2 = bsp::hal::adc::AnalogInput::new(pins.p17);
        // Start the ADC reading task
        log::info!("logging");
        read_adc::spawn().ok();

        (
            Shared { },
            Local {
                adc: adc1,
                a1,
                a2,
            },
        )
    }

    #[task(local = [adc, a1, a2], priority = 1)]
    async fn read_adc(ctx: read_adc::Context) {
        loop {
            // Read ADC value using OneShot trait
            let readingX: u16 = ctx.local.adc.read_blocking(ctx.local.a1);

            let readingY: u16 = ctx.local.adc.read_blocking(ctx.local.a2);
            
            // Log the reading
            log::info!("ADC ReadingX: {}", readingX);

            log::info!("ADC ReadingY: {}", readingY);

            // Wait for 1 second before next reading
            Systick::delay(1_000u32.millis()).await;
        }
    }

    // Required idle task when using RTIC
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}