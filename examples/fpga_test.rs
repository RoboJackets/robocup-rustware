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

// use fpga driver
use fpga_rs as fpga;
use fpga::FPGA_SPI_FREQUENCY;
use fpga::FPGA_SPI_MODE;
use fpga::FPGA;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

//// THE APP MODULE ////
//// device: board support package
//// perihperals: refers to a specific boards hardware e.g. gpio, spi, etc
//// dispatchers: interrupt handlers for SOFTWARE defined tasks
#[app(
    device = bsp, 
    peripherals = true, 
    dispatchers = [GPT2]
)]
mod app {
    use fpga::error::FpgaError;

    use main::{Fpga, GPT_FREQUENCY, GPT_CLOCK_SOURCE, GPT_DIVIDER};

    // this allows us to define our packages outside the app module
    // we're essetially "bringing them all in"
    use super::*;

    // struct that holds local resources which can be accessed via the context
    #[local]
    struct Local {
        fpga: Fpga,
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
            // usedd to acces pin names
            mut pins,
            // used to control any pin from the gpio2 register
            // (e.g. pin13 for the on board LED)
            mut gpio2,
            mut gpio3,
            mut gpio4,
            // for usb logging :)
            usb,
            // resources to control spi
            lpspi4,
            // for blocking delays :)
            mut gpt1,
            ..
        } = board::t40(cx.device);

        // usb logging setup
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // systick monotonic setup
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 600_000_000, systick_token);

        // gpt1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        // initalize spi
        // use pin 9 as chop select manially :) maybe it'll fix the issue???
        let mut spi = board::lpspi(lpspi4, 
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            }, 
            FPGA_SPI_FREQUENCY);
        
        // custom CS pin to test. Maybe self asserted pcs = pin.10 is not working as intended
        let cs = gpio2.output(pins.p9);

        // configure SPI
        spi.disabled(|spi| spi.set_mode(FPGA_SPI_MODE));

        // initialize pins for FPGA
        let init_b = gpio4.input(pins.p29);

        // configure prog pin to open drain configuration
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        
        let done = gpio3.input(pins.p30);

        let fpga = match FPGA::new(spi, cs, init_b, prog_b, done, delay) {
            Ok(instance) => instance,
            Err(_) => panic!("Couldn't initialize instance of FPGA"),
        };

        // start the init fpga task
        init_fpga::spawn().unwrap();

        // return the local, and shared resources to be used from the context
        (
            Shared {

            },
            Local {
                fpga
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

    // init fpga task
    #[task(local = [fpga], priority = 1)]
    async fn init_fpga(cx: init_fpga::Context) {
        // acquire fpga instance from local resources
        let fpga = cx.local.fpga;
        
        // start the test :)
        log::info!("[INIT] FPGA ENCODER DEMO");
        Systick::delay(1_000u32.millis()).await;
        
        // attempt to configure the fpga :)
        match fpga.configure() {
            Ok(_) => log::info!("Configuration worked???"),
            Err(e) => match e {
                FpgaError::SPI(spi_e) => panic!("SPI error with info: {:?}", spi_e),
                FpgaError::CSPin(cs_e) => panic!("CS pin error: {:?}", cs_e), 
                FpgaError::InitPin(init_e) => panic!("Init pin error: {:?}", init_e),
                FpgaError::ProgPin(prog_e) => panic!("Prog pin error: {:?}", prog_e),
                FpgaError::DonePin(done_e) => panic!("Done pin error: {:?}", done_e),
                FpgaError::FPGATimeout(code) => panic!("Fpga timed out?? code: {:x}", code),
            },
        };
        Systick::delay(10u32.millis()).await;

        // enable motors
        match fpga.motors_en(true){
            Ok(status) => log::info!(" enabled motors fpga status: {:b}", status),
            Err(e) => panic!("error enabling motors... {:?}", e),
        };
        Systick::delay(10u32.millis()).await;

        
        // drive forward at different speeds -> should be measuring different encoder values
        loop {
            for motor in 0..5 {
                let (target_velocity, dribbler) = match motor {
                    0 => ([0.35, 0.0, 0.0, 0.0], false),
                    1 => ([0.0, 0.35, 0.0, 0.0], false),
                    2 => ([0.0, 0.0, 0.35, 0.0], false),
                    3 => ([0.0, 0.0, 0.0, 0.35], false),
                    _ => ([0.0, 0.0, 0.0, 0.0], true),
                };

                for i in 0..2_500 {
                    if i % 100 == 0 {
                        if let Ok(velocities) = fpga.set_velocities(target_velocity, dribbler) {
                            log::info!("Wheels are moving at {:?}", velocities);
                        } else {
                            log::error!("Unable to set FPGA wheel velocities");
                        }
                    } else {
                        if fpga.set_velocities(target_velocity, dribbler).is_err() {
                            log::error!("Unable to set FPGA wheel velocities");
                        }
                    }

                    Systick::delay(200u32.micros()).await;
                }

                for _ in 0..200 {
                    if fpga.set_velocities([0.0, 0.0, 0.0, 0.0], false).is_err() {
                        log::error!("Unable to Stop the FPGA wheels");
                    }

                    Systick::delay(200u32.micros()).await;
                }
            }


        }
    }

}

