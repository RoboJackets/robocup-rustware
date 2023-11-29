#![no_std]
#![no_main]

use teensy4_panic as _;
use rbgLED::rbg_LED_Driver;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use teensy4_pins::common::*;

    use teensy4_bsp as bsp;
    use bsp::board as board;

    use bsp::hal as hal;
    use hal::gpio::Output;

    use rtic_monotonics::systick::*;

    //local because this is currently the only place where this LED is used
    #[local]
    struct Local {
        //Why are these the pins?
        led: rbg_LED_Driver<Output<P0>, Output<P8>, Output<P31>>
    }

    #[shared]
    struct Shared {
        //empty because LED currently isn't shared with anything
    }

    #[init]
    fn init (ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio2, //is this gpio correct and if it is/isn't how do I know?
            ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        let led = rbg_LED_Driver::new(
            gpio2.output(pins.p0),
            gpio2.output(pins.p8),
            gpio2.output(pins.p31),
        );

        blink_led::spawn().ok();

        (
            Shared {},
            Local {
                led,
            },
        )
    }

    //when idle wait for interrupt
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [ led ], priority = 1)]
    async fn blink_led(ctx: blink_led::Context) {
        ctx.local.led.red().unwrap();

        Systick::delay(1000u32.millis()).await;

        ctx.local.led.green().unwrap();

        Systick::delay(1000u32.millis()).await;

        ctx.local.led.blue().unwrap();

        Systick::delay(1000u32.millis()).await;

        ctx.local.led.yellow().unwrap();

        Systick::delay(1000u32.millis()).await;

        ctx.local.led.purple().unwrap();

        Systick::delay(1000u32.millis()).await;

        ctx.local.led.cyan().unwrap();

        Systick::delay(1000u32.millis()).await;
    }
}