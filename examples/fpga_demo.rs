#![no_std]
#![no_main] // bsp-rt is used as the entry point of the program instead
#![feature(type_alias_impl_trait)] // this feature is needed for RTIC v2

extern crate alloc;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

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
//// perihperals: refers to a specific boards hardware e.g. gpio, spi, etc
//// dispatchers: interrupt handlers for SOFTWARE defined tasks
#[app(
    device = bsp, 
    peripherals = true, 
    dispatchers = [GPT2]
)]
mod app {
    use fpga::{error::FpgaError, duty_cycle::DutyCycle};

    // this allows us to define our packages outside the app module
    // we're essetially "bringing them all in"
    use super::*;

    // accounts for our syst_clock to be in 10 kHz (normal is 1 kHz)
    // this means that the granularity for the delay is 0.1 ms per tick
    // therefore we multiply our delay time by a factor of 10
    const SYST_MONO_FACTOR: u32 = 10;

    // delay in miliseconds
    const TEN_MS_DELAY: u32 = SYST_MONO_FACTOR * 10;            // 10 ms
    const HUNDRED_MS_DELAY: u32 = SYST_MONO_FACTOR * 100;       // 100 ms ms
    const SECOND_DELAY: u32 = SYST_MONO_FACTOR * 1000;    // 1 s delay

    // MOTION SPEED in DUTY CYCLE
    const SPEED: i16 = 63;
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
        let mut spi = board::lpspi(lpspi4, 
            board::LpspiPins {
                pcs0: pins.p10, // NOT CURRENTLY USED
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

    // init fpga task
    #[task(local = [fpga, delay], priority = 1)]
    async fn init_fpga(cx: init_fpga::Context) {
        // acquire fpga instance from local resources
        let fpga = cx.local.fpga;
        let delay = cx.local.delay;
        Systick::delay(SECOND_DELAY.millis()).await;

        // attempt to configure the fpga :)
        match fpga.configure().await {
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
        Systick::delay(TEN_MS_DELAY.millis()).await;

        // enable motors
        match fpga.enable_motors(true) {
            Ok(status) => log::info!(" enabled motors fpga status: {:b}", status),
            Err(e) => panic!("error enabling motors... {:?}", e),
        };
        Systick::delay(TEN_MS_DELAY.millis()).await;

        // TODO: Verify that each duty cycle corresponds to each respective motor
        loop {
            // move forward for 2 seconds
            log::info!("moving forward!");
            Systick::delay(TEN_MS_DELAY.millis()).await;
            for _ in 0..100 {
                // forward direction duty cycles
                let mut duty_cycles = [DutyCycle::from(SPEED as i16),     // motor 1
                                DutyCycle::from(SPEED as i16),                            // motor 2
                                DutyCycle::from(-SPEED as i16),                             // motor 3
                                DutyCycle::from(-SPEED as i16),                             // motor 4
                                DutyCycle::from(256 as i16)];                            // dribbler
                // write duty cycle
                match fpga.set_duty_cycles(&mut duty_cycles) {
                    Ok(status) => log::info!("wrote duty cycles fpga status: {:b}", status),
                    Err(e) => panic!("error writing duty cycles... {:?}", e),
                };
                Systick::delay(HUNDRED_MS_DELAY.millis()).await;
            }

            // move backwards for 2 seconds
            log::info!("moving backwards!");
            Systick::delay(TEN_MS_DELAY.millis()).await;
            for _ in 0..100 {
                // forward direction duty cycles
                let mut duty_cycles = [DutyCycle::from(-SPEED as i16),     // motor 1
                                DutyCycle::from(-SPEED as i16),                            // motor 2
                                DutyCycle::from(SPEED as i16),                           // motor 3
                                DutyCycle::from(SPEED as i16),                           // motor 4
                                DutyCycle::from(256 as i16)];                           // dribbler
                // write duty cycle
                match fpga.set_duty_cycles(&mut duty_cycles) {
                    Ok(status) => log::info!("wrote duty cycles fpga status: {:b}", status),
                    Err(e) => panic!("error writing duty cycles... {:?}", e),
                };
                Systick::delay(HUNDRED_MS_DELAY.millis()).await;
            }

            // spin for 5 seconds
            log::info!("spinning!");
            Systick::delay(TEN_MS_DELAY.millis()).await;
            for _ in 0..100 {
                // forward direction duty cycles
                let mut duty_cycles = [DutyCycle::from(SPEED as i16), 
                                DutyCycle::from(SPEED as i16), 
                                DutyCycle::from(SPEED as i16),
                                DutyCycle::from(SPEED as i16),
                                DutyCycle::from(256 as i16)];
                // write duty cycle
                match fpga.set_duty_cycles(&mut duty_cycles) {
                    Ok(status) => log::info!("wrote duty cycles fpga status: {:b}", status),
                    Err(e) => panic!("error writing duty cycles... {:?}", e),
                };
                Systick::delay(HUNDRED_MS_DELAY.millis()).await;
            }

            // still for 3 seconds
            log::info!("standby...");
            Systick::delay(TEN_MS_DELAY.millis()).await;
            for _ in 0..100 {
                // forward direction duty cycles
                let mut duty_cycles = [DutyCycle::from(0 as i16), 
                                DutyCycle::from(0 as i16), 
                                DutyCycle::from(0 as i16),
                                DutyCycle::from(0 as i16),
                                DutyCycle::from(256 as i16)];
                // write duty cycle
                match fpga.set_duty_cycles(&mut duty_cycles) {
                    Ok(status) => log::info!("wrote duty cycles fpga status: {:b}", status),
                    Err(e) => panic!("error writing duty cycles... {:?}", e),
                };
                Systick::delay(HUNDRED_MS_DELAY.millis()).await;
            }
        }
    }

}

