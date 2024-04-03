#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod led_control {
    use teensy4_bsp as bsp;
    use MCP23017::MCP23017Driver;

    #[local]
    struct Local {
        // instantiation is not correct here, but i'm not sure how to do it
        mcp23017_driver: MCP23017Driver,
    }

    // the shared group of resources for everything to use amongst all functions
    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {

        // Grab the board peripherals
        let board::Resources {
            pins,
            mut gpio2,
            usb,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // here i want to define the tangible pins that the MCP23017 will be connected to

        blink_rgb::spawn().ok();
        (
            Shared {

            },
            Local { 
                mcp23017_driver: mcp23017_driver,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [mcp23017_driver], priority = 1)]
    async fn do_something(_ctx: blink_rgb::Context) {
        let mcp23017_driver = _ctx.local.mcp23017_driver;

        // populate this method with what should be done with the MCP23017
    }
}