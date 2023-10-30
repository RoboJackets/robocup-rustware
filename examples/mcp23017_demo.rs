//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

///
/// This is a demo example file that turns on and off the onboard led.
/// 
/// Please follow this example for future examples and sanity tests
/// 

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use teensy4_pins::common::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::gpio::Output;

    use rtic_monotonics::systick::*;

    use mcp23017_rs::{self, Mcp23017};

    #[local]
    struct Local {
        mcp23017: Mcp23017<hal::lpi2c::Lpi2c>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut lpi2c1,
            ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        let mcp23017 = Mcp23017::new(lpi2c1.instance(), 0x20).unwrap();  // address set with A0, A1, A2 pin connections, in this case all set to ground

        test::spawn().ok();

        (
            Shared {},
            Local {
                mcp23017,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [mcp23017], priority = 1)]
    async fn test(ctx: test::Context) {
        ctx.local.mcp23017.config(0b1010101010101010, 0b0000000101010101, 0b0001010101010001);
        ctx.local.mcp23017.write_mask(0b101010101010, 0b100000000000);  // sets all outputs high except the first

        Systick::delay(1_000u32.millis()).await;

        ctx.local.mcp23017.set_pin(mcp23017_rs::GPIOPin::A0);
        ctx.local.mcp23017.clear_pin(mcp23017_rs::GPIOPin::A2);
        ctx.local.mcp23017.toggle_pin(mcp23017_rs::GPIOPin::A4);

        // tests input of each combination of pullup and polarity of inputs
        let input1 = ctx.local.mcp23017.read_pin(mcp23017_rs::GPIOPin::A1).unwrap();
        let input2 = ctx.local.mcp23017.read_pin(mcp23017_rs::GPIOPin::A3).unwrap();
        let input3 = ctx.local.mcp23017.read_pin(mcp23017_rs::GPIOPin::A7).unwrap();
        let input4 = ctx.local.mcp23017.read_pin(mcp23017_rs::GPIOPin::B5).unwrap();

        // writes each input tested to an output
        ctx.local.mcp23017.write_pin(mcp23017_rs::GPIOPin::A6, input1).unwrap();
        ctx.local.mcp23017.write_pin(mcp23017_rs::GPIOPin::B0, input2).unwrap();
        ctx.local.mcp23017.write_pin(mcp23017_rs::GPIOPin::B2, input3).unwrap();
        ctx.local.mcp23017.write_pin(mcp23017_rs::GPIOPin::B4, input4).unwrap();

        Systick::delay(1_000u32.millis()).await;
    }
}