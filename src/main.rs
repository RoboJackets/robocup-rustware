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
use teensy4_pins::common::*; // pad to pin definitions
use bsp::hal::gpio::{self, Trigger}; // gpio module
use bsp::hal::gpt::Gpt1; // tpye definition for GPT1
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


// spi traits to use the transfer and write transactions
use embedded_hal::blocking::spi::{Transfer, Write};
use bsp::hal::gpt::ClockSource;

//// THE APP MODULE ////
//// device: board support package
//// perihperals: ...?
//// dispatchers: interrupt handlers for software defined tasks
#[app(
    device = bsp, 
    peripherals = true, 
    dispatchers = [GPT2]
)]
mod app {

    use fpga::{error::FpgaError, instructions::DutyCycle};

    // this allows us to define our packages outside the app module
    // we're essetially "bringing them all in"
    use super::*;

    // accounts for our syst_clock to be in 10 kHz (normal is 1 kHz)
    // this means that the granularity for the delay is 0.1 ms per tick
    // therefore we multiply our delay time by a factor of 10
    const SYST_MONO_FACTOR: u32 = 10;

    // delay in miliseconds
    const DELAY_MS: u32 = SYST_MONO_FACTOR * 150;      // delay in ms
    const LONG_DELAY: u32 = SYST_MONO_FACTOR * 5000;  // 5 s delay

    // timer stuff
    const GPT1_FREQUENCY: u32 = 1_000;
    const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;

    // type definition for Led :)
    // this simplifies local and shared resource definitions
    type Led = gpio::Output<P6>;
    type Delay = Blocking<Gpt1, GPT1_FREQUENCY>;
    type Fpga = FPGA<board::Lpspi4, gpio::Output<P9>, P15, gpio::Output<P16>, P14>;

    // struct that holds local resources which can be accessed via the context
    #[local]
    struct Local {
        led: Led,
        fpga: Fpga,
        delay: Delay,
    }

    // struct that holds shared resources which can be accessed via the context
    #[shared]
    struct Shared {
        counter: u32,
    }

    // entry point of the "program"
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // allocate the resources needed
        let board::Resources {
            // usedd to acces pin names
            mut pins,
            // gpio1 for pin 1
            mut gpio1,
            // used to control any pin from the gpio2 register
            // (e.g. pin13 for the on board LED)
            mut gpio2,
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
        Systick::start(cx.core.SYST, 36_000_000, systick_token);

        // gpt1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT1_DIVIDER);
        gpt1.set_clock_source(GPT1_CLOCK_SOURCE);
        let mut delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

        // init led from gpio2 pin 7
        let led = gpio2.output(pins.p6);

        // init counter shared variable
        let counter = 0;

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
        spi.disabled(|spi| {
            spi.set_mode(FPGA_SPI_MODE);
        });

        // initialize pins for FPGA
        let init_b = gpio1.input(pins.p15);

        // configure prog pin to open drain configuration
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p16, config);
        let prog_b = gpio1.output(pins.p16);
        
        let done = gpio1.input(pins.p14);

        let fpga = match FPGA::new(spi, cs, init_b, prog_b, done) {
            Ok(instance) => instance,
            Err(_) => panic!("Couldn't initialize instance of FPGA"),
        };

        // start the init fpga task
        init_fpga::spawn().unwrap();

        // return the local, and shared resources to be used from the context
        (
            Shared {counter},
            Local {led, fpga, delay}
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

    //
    #[task(local = [led], shared = [counter], priority = 1)]
    async fn toggle(cx : toggle::Context) {
        // just renaming our shared variable into a local variable so it's easier to read
        let mut counter = cx.shared.counter;

        // infinite loop which is allowed as it contains a delay followed by a ".await"
        loop {
            // example locking the shared counter variable and updating it's value!
            counter.lock(|counter| {
                // increment the counter using an external function
                *counter += 1;
                
                // prints "blink!" to the usb serial port
                log::info!("blink # {}!", *counter);
            });

            // toggle the led
            cx.local.led.toggle();

            // generate a delay using the initialized systick monotonic
            // by calling the Systick::delay() function
            Systick::delay(DELAY_MS.millis()).await;
        }
    }

    // init fpga task
    #[task(local = [fpga, delay], priority = 1)]
    async fn init_fpga(cx: init_fpga::Context) {
        // acquire fpga instance from local resources
        let fpga = cx.local.fpga;
        let delay = cx.local.delay;
        Systick::delay(LONG_DELAY.millis()).await;

        // attempt to configure the fpga :)
        match fpga.configure(delay) {
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

        Systick::delay(DELAY_MS.millis()).await;

        let mut duty_cycles = [DutyCycle::from(0 as i16), 
                        DutyCycle::from(0 as i16), 
                        DutyCycle::from(0 as i16),
                        DutyCycle::from(0 as i16),
                        DutyCycle::from(0 as i16)];

        log::info!("delay has been set to: {} ms", DELAY_MS / SYST_MONO_FACTOR);
        
        // // enable motors
        // match fpga.motors_en(true){
        //     Ok(status) => log::info!(" enabled motors fpga status: {:b}", status),
        //     Err(e) => panic!("error enabling motors... {:?}", e),
        // };
        // Systick::delay(DELAY_MS.millis()).await;        

        // // let's try to get and print the git hash?
        // let mut gitHash: [u8;20] = [0;20];
        // match fpga.get_git_hash(&mut gitHash, delay) {
        //     Ok(dirty) => {
        //         log::info!("got git hash!");
        //         for i in 0..20 {
        //             log::info!("{}: {:X}", i, gitHash[i]);
        //         }
        //         log::info!("dirty is: {}", dirty);
        //     },
        //     Err(e) => panic!("eror reading git hash {:?}", e),
        // }

        loop {
            // enable motors
            match fpga.motors_en(true){
                Ok(status) => log::info!(" enabled motors fpga status: {:b}", status),
                Err(e) => panic!("error enabling motors... {:?}", e),
            };
            Systick::delay(DELAY_MS.millis()).await;

            // write duty cycle
            match fpga.set_duty_cycles(&mut duty_cycles) {
                Ok(status) => {
                    log::info!("wrote duty cycles fpga status: {:b}", status);
                    //let dt: f32 = (status[1] as f32) * 2.0 * 128.0 * (1.0 / 18432000.0);
                    //log::info!("delta value is: {}", dt);
                },
                Err(e) => panic!("error writing duty cycles... {:?}", e),
            };
            Systick::delay(DELAY_MS.millis()).await;

            // // disable motors
            // match fpga.motors_en(false){
            //     Ok(status) => log::info!("disabled motors fpga status: {:b}", status),
            //     Err(e) => panic!("error enabling motors... {:?}", e),
            // };
            // Systick::delay(DELAY_MS.millis()).await;
        }
    }
}

