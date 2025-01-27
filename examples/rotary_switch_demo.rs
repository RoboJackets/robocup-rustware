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

    use io_expander_rs::error::IOExpanderError;
    use rotary_switch::RotarySwitch;
    use rotary_switch::error::RotaryError;

    use io_expander::IoExpander;
    //use io_expander::error::IOExpanderError;

    use embedded_hal::blocking::i2c::*;
    use imxrt_hal::lpi2c::*;
    use imxrt_hal::gpio::Trigger;


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
        //rotary: Rotary,
        //io_expander: Expander
    }

    // struct that holds shared resources which can be accessed via the context
    #[shared] 
    struct Shared {
        rotary: Rotary,
        rotary_interrupt: bool
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
            lpi2c3,
            mut gpt2,
            ..
        } = board::t40(cx.device);

        // usb logging setup
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // systick monotonic setup
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 36_000_000, systick_token);

        // gpt1 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);
        
        // setup interrupt
        //let rx_int = gpio1.input(pins.p15);
        //gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));

        let mut i2c = board::lpi2c(lpi2c3, 
            pins.p16, // SCL, pin 25 on MBED
            pins.p17, // SDA, pin 26 on MBED
            board::Lpi2cClockSpeed::KHz100);
        

        let io_expander = match Expander::new(i2c) {
            Ok(instance) => instance,
            Err(_) => panic!("Couldn't initalize io expander instance")
        };
        
        let mut rotary = match RotarySwitch::new(io_expander) {
            Ok(instance) => instance,
            Err(_) => panic!("Couldn't initialize instance of Rotary Switch")
        };

        

        let value = rotary.init(&mut delay);
        match value {
            Ok(value) => log::info!("{value}"),
            Err(error) => {
                match error {
                    IOExpanderError::I2C(controller_status) => log::info!("{:?}", controller_status),
                    IOExpanderError::BadResponse(_, _) => log::info!("wtf")
                }
            }
        }


        let io_expander_int = gpio2.input(pins.p6);
        gpio2.set_interrupt(&io_expander_int, Some(Trigger::High));
        io_expander_int.clear_triggered();

        let rotary_interrupt = false;

        going::spawn().unwrap();

        (
            Shared {
                rotary,
                rotary_interrupt
            },
            Local {
                //rotary,
                //io_expander
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


    // HANLDES THE INTERRUPT. TO AVOID GETTING MULTIPLE INTERRUPTS, USE A DELAY THEN READ
    // NEED TO READ FROM THE INTERRUPT REGISTER TO CLEAR IT
    #[task(binds = GPIO2_COMBINED_0_15, shared = [rotary, rotary_interrupt], priority = 2)]
    fn rotary_interrupt(cx: rotary_interrupt::Context) {
        let rotary = cx.shared.rotary;
        let rotary_interrupt = cx.shared.rotary_interrupt;
        
        (rotary, rotary_interrupt).lock(| rotary, rotary_interrupt | {
            let interrupts = rotary.clear_interrupts();
            match interrupts {
                Ok(value) => {},//log::info!("ROTARY VALUE CHANGED, INTERRUPTS {value}"),
                Err(error) => log::info!("error")
            }

            if *rotary_interrupt == false {
                *rotary_interrupt = true;
                read_rotary_dly::spawn().ok();
            }
        });
    }

    //  Delay after receiving the interrupt
    // Avoids reading multiple times when it might be wrong
    #[task(priority = 1, shared = [rotary_interrupt])]
    async fn read_rotary_dly(cx: read_rotary_dly::Context) {
        Systick::delay(1000000.micros()).await;
        let mut rotary_interrupt = cx.shared.rotary_interrupt;
        rotary_interrupt.lock(|rotary_interrupt| {
            *rotary_interrupt = false;
        });
        read_rotary::spawn().ok();
    }

    // actually reads the rotary value
    #[task(priority = 1, shared = [rotary])]
    async fn read_rotary(cx: read_rotary::Context) {
        let mut rotary = cx.shared.rotary;
        rotary.lock(|rotary| {
            let value = rotary.read();
            match value {
                Ok(value) => log::info!("ROTARY CHANGED TO {value}"),
                Err(error) => {
                    match error {
                        IOExpanderError::I2C(controller_status) => log::info!("{:?}", controller_status),
                        IOExpanderError::BadResponse(_, _) => log::info!("wtf")
                    }
                }
            }
        });
        
    }

    #[task(priority = 1, shared = [rotary])]
    async fn going(cx: going::Context) {
        let mut rotary = cx.shared.rotary;

        rotary.lock(|rotary| {
            let value = rotary.read();
            match value {
                Ok(value) => log::info!("rotary is {value}"),
                Err(error) => {
                    match error {
                        IOExpanderError::I2C(controller_status) => log::info!("{:?}", controller_status),
                        IOExpanderError::BadResponse(_, _) => log::info!("wtf")
                    }
                }
            }
        });
        
        //log::info!("{value}");

        log::info!("running, nothing changed");


        rotary_dly::spawn().ok();
    }
    
    #[task(priority = 1)]
    async fn rotary_dly(_cx: rotary_dly::Context) {
        Systick::delay(25000000.micros()).await;
        //going::spawn().ok();
    }


    
    

}
