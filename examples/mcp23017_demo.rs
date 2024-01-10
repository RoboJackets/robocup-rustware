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

    use rtic_monotonics::systick::*;

    use mcp23017_rs::{self, Mcp23017};

    #[local]
    struct Local {
        mcp23017: Mcp23017<hal::lpi2c::Lpi2c<hal::lpi2c::Pins<P19, P18>, 1>>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            lpi2c1,
            ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        let mcp23017 = Mcp23017::new(bsp::board::lpi2c(lpi2c1,
            pins.p19,
            pins.p18,
            bsp::board::Lpi2cClockSpeed::KHz400,
        ), 0x21).unwrap();  // address set with A0, A1, A2 pin connections, in this case all set to ground except A0

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
        ctx.local.mcp23017.pin_mode(mcp23017_rs::GPIOPin::B0, mcp23017_rs::PinMode::Output).unwrap();

        ctx.local.mcp23017.toggle_pin(mcp23017_rs::GPIOPin::B0).unwrap();

        Systick::delay(1_000u32.millis()).await;


        /*
        ctx.local.mcp23017.config(0b0110101010101010, 0b0000001010101010, 0b000010101010100010).unwrap();
        ctx.local.mcp23017.pin_mode(mcp23017_rs::GPIOPin::A0, mcp23017_rs::PinMode::Input);
        ctx.local.mcp23017.pin_mode(mcp23017_rs::GPIOPin::A1, mcp23017_rs::PinMode::Output);
        ctx.local.mcp23017.write_mask(0b0101010101010101, 0b1000000000000000).unwrap();  // sets all outputs high except the first

        Systick::delay(1_000u32.millis()).await;

        ctx.local.mcp23017.set_pin(mcp23017_rs::GPIOPin::A1).unwrap();
        ctx.local.mcp23017.clear_pin(mcp23017_rs::GPIOPin::A3).unwrap();
        ctx.local.mcp23017.toggle_pin(mcp23017_rs::GPIOPin::A5).unwrap();

        // tests input of each combination of pullup and polarity of inputs
        let input1 = ctx.local.mcp23017.read_pin(mcp23017_rs::GPIOPin::A2).unwrap();  // pulldown noninverted
        let input2 = ctx.local.mcp23017.read_pin(mcp23017_rs::GPIOPin::A4).unwrap();  // pulldown inverted
        let input3 = ctx.local.mcp23017.read_pin(mcp23017_rs::GPIOPin::A6).unwrap();  // pullup inverted
        let input4 = ctx.local.mcp23017.read_pin(mcp23017_rs::GPIOPin::B6).unwrap();  // pullup noninverted

        // writes each input tested to an output
        ctx.local.mcp23017.write_pin(mcp23017_rs::GPIOPin::A7, input1).unwrap();
        ctx.local.mcp23017.write_pin(mcp23017_rs::GPIOPin::B1, input2).unwrap();
        ctx.local.mcp23017.write_pin(mcp23017_rs::GPIOPin::B3, input3).unwrap();
        ctx.local.mcp23017.write_pin(mcp23017_rs::GPIOPin::B5, input4).unwrap();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.mcp23017.toggle_pin(mcp23017_rs::GPIOPin::A5).unwrap();*/
    }
}