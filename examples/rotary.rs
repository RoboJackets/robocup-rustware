#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod rotary_driver {
    use rotary::RotaryPinDriver;
    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::hal::gpio; // gpio module
    use teensy4_pins::t41::*;

    #[local]
    struct Local {
        // lib specifies that the rotaryPinDriver should take in InputPins, not Input --> only pending question 
        // in the rgb driver, the lib.rs file specifies that the rgb driver should take in OutputPins, not Output
        rotary_driver: RotaryPinDriver<gpio::Input<P6>, gpio::Input<P7>, gpio::Input<P8>, gpio::Input<P9>>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources { 
            usb, 
            mut gpio2, 
            pins,
            .. 
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // create the rotary driver
        let input_one = gpio2.input(pins.p6);
        let input_two = gpio2.input(pins.p7);
        let input_three = gpio2.input(pins.p8);
        let input_four = gpio2.input(pins.p9);
        let rotary_driver = RotaryPinDriver::new(input_one, input_two, input_three, input_four);

        // spawn other tasks as needed
        read_pins::spawn().ok();
        (
            Shared {},
            Local { rotary_driver },
        )
    }

    #[task(local = [rotary_driver], priority = 1)]
    async fn read_pins(_ctx: read_pins::Context) {
        let rotary_driver = _ctx.local.rotary_driver;

        let result = rotary_driver.read();
        log::info!("Result: {}", result);
    }
}