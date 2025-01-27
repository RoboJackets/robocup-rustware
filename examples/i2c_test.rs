//!
//! Demos the Rotary Switch functionality
//! 
//! 


#![no_std]
#![no_main] // bsp-rt is used as the entry point of the program instead
#![feature(type_alias_impl_trait)] // this feature is needed for RTIC v2

//// BASIC BSP PACKAGES ///
use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _; // allows program to panic and print panic messages
//////////////////////////
/// Package used to configure pin configuration
use imxrt_iomuxc::prelude::*;

//// ASSOCIATED TPYES FOR INSTANCES ////
use bsp::hal::timer::Blocking;
////////////////////////////////////////

//// RTIC PKACAGES ///
use rtic::app;
use rtic_monotonics::systick::*;
////////////////////

// use rotary driver
use rotary_switch_rs as rotary_switch;


use io_expander_rs as io_expander;


use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

//// THE APP MODULE ////
//// device: board support package
//// perihperals: refers to a specific boards hardware e.g. gpio, spi, etc
//// dispatchers: interrupt handlers for SOFTWARE defined tasks
#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    
    // WHT DOES THIS DO
    //use main::{Fpga, GPT_FREQUENCY, GPT_CLOCK_SOURCE, GPT_DIVIDER};

    use core::convert::Infallible;

    // this allows us to define our packages outside the app module
    // we're essetially "bringing them all in"
    use super::*;

    use embedded_hal::blocking::i2c::{Write, Operation};
    use embedded_hal::blocking::{delay::DelayMs, i2c};
    use imxrt_hal::lpi2c::*;


    // accounts for our syst_clock to be in 10 kHz (normal is 1 kHz)
    // this means that the granularity for the delay is 0.1 ms per tick
    // therefore we multiply our delay time by a factor of 10
    const SYST_MONO_FACTOR: u32 = 10;

    use main::{
        Rotary, Expander, PitDelay, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY
    };

    // delay in miliseconds
    const TEN_MS_DELAY: u32 = SYST_MONO_FACTOR * 10;            // 10 ms
    const HUNDRED_MS_DELAY: u32 = SYST_MONO_FACTOR * 100;       // 100 ms ms
    const SECOND_DELAY: u32 = SYST_MONO_FACTOR * 1000;    // 1 s delay


    // struct that holds local resources which can be accessed via the context
    #[local]
    struct Local {
        i2c: board::Lpi2c1
    }

    // struct that holds shared resources which can be accessed via the context
    #[shared] 
    struct Shared {
    }

    // entry point of the "program"
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // allocate the resources needed
        let board::Resources {
            mut pins,
            mut gpio1,
            mut gpio2,
            mut gpio3,
            mut gpio4,
            usb,
            lpi2c1,
            mut gpt1,
            ..
        } = board::t40(cx.device);

        // usb logging setup
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // systick monotonic setup
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 36_000_000, systick_token);

        // gpt1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        // setup interrupt
        //let rx_int = gpio1.input(pins.p15);
        //gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);

        
        //i2c.set_controller_enable(true);
        

        //cortex_m::prelude::_embedded_hal_blocking_i2c_Write::write(&mut i2c, 8, &[8]);

        /*
        let rotary = match RotarySwitch::new(io_expander) {
            Ok(instance) => instance,
            Err(_) => panic!("Couldn't initialize instance of Rotary Switch")
        };
        */
        init_rotary::spawn().unwrap();

        (
            Shared {

            },
            Local {
                i2c
            }
        )
    }



    // (optional) lowest priority tasks that runs only while no other task is running
    #[idle]
    fn idle(_: idle::Context) -> !{
        loop {
            // wfi: wait-for-interrupt
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [i2c], priority = 1)]
    async fn init_rotary(cx: init_rotary::Context) {
        let i2c = cx.local.i2c;
        let arr = [0;1];
        let res = i2c.write(5, &[10]);
        
        let boolean = i2c.is_controller_enabled();
        log::info!("{boolean}");

        match res {
            Err(error) => {
                match error {
                    ControllerStatus::NACK_DETECTED => log::info!("NACK"),
                    _ => {log::info!("OTHER")}
                }},
            Ok(_) => log::info!("no problem")
        }
        rotary_dly::spawn().unwrap();
        
    }
    
    #[task(priority = 1)]
    async fn rotary_dly(_cx: rotary_dly::Context) {
        Systick::delay(10000000.micros()).await;
        init_rotary::spawn().ok();
    }
    

}
