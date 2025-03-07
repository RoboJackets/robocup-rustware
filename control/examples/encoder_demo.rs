#![no_std]
#![no_main] // bsp-rt is used as the entry point of the program instead
#![feature(type_alias_impl_trait)] // this feature is needed for RTIC v2

/// BASIC BSP PACKAGES ///
use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _; // allows program to panic and print panic messages
                        //////////////////////////
/// Package used to configure pin configuration
use imxrt_iomuxc::prelude::*;

/// ASSOCIATED TPYES FOR INSTANCES ////
use bsp::hal::timer::Blocking;
////////////////////////////////////////

/// RTIC PKACAGES ///
use rtic::app;
use rtic_monotonics::systick::*;
////////////////////

// use fpga driver
use fpga::FPGA;
use fpga::FPGA_SPI_FREQUENCY;
use fpga::FPGA_SPI_MODE;
use fpga_rs as fpga;

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

    use robojackets_robocup_control::{Delay1, Fpga, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY};

    // this allows us to define our packages outside the app module
    // we're essetially "bringing them all in"
    use super::*;

    // accounts for our syst_clock to be in 10 kHz (normal is 1 kHz)
    // this means that the granularity for the delay is 0.1 ms per tick
    // therefore we multiply our delay time by a factor of 10
    const SYST_MONO_FACTOR: u32 = 10;

    // delay in miliseconds
    const TEN_MS_DELAY: u32 = SYST_MONO_FACTOR * 10; // 10 ms
    const HUNDRED_MS_DELAY: u32 = SYST_MONO_FACTOR * 100; // 100 ms ms
    const SECOND_DELAY: u32 = SYST_MONO_FACTOR * 1000; // 1 s delay

    // MOTION SPEED in DUTY CYCLE
    const SPEED: f32 = 1.0;
    const SLOW_SPEED: f32 = 0.32;
    const FAST_SPEED: f32 = 0.128;

    // struct that holds local resources which can be accessed via the context
    #[local]
    struct Local {
        fpga: Fpga,
        delay: Delay1,
    }

    // struct that holds shared resources which can be accessed via the context
    #[shared]
    struct Shared {}

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
        Systick::start(cx.core.SYST, 36_000_000, systick_token);

        // gpt1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        // initalize spi
        // use pin 9 as chop select manially :) maybe it'll fix the issue???
        let mut spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            FPGA_SPI_FREQUENCY,
        );

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

        let fpga = match FPGA::new(spi, cs, init_b, prog_b, done) {
            Ok(instance) => instance,
            Err(_) => panic!("Couldn't initialize instance of FPGA"),
        };

        // start the init fpga task
        init_fpga::spawn().unwrap();

        // return the local, and shared resources to be used from the context
        (Shared {}, Local { fpga, delay })
    }

    // (optional) lowest priority tasks that runs only while no other task is running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // wfi: wait-for-interrupt
            cortex_m::asm::wfi();
        }
    }

    // init fpga task
    #[task(local = [fpga, delay], priority = 1)]
    async fn init_fpga(ctx: init_fpga::Context) {
        // acquire fpga instance from local resources
        let fpga = ctx.local.fpga;

        // start the test :)
        log::info!("[INIT] FPGA ENCODER DEMO");
        Systick::delay(3 * SECOND_DELAY.millis()).await;

        // attempt to configure the fpga :)
        match fpga.configure(ctx.local.delay) {
            Ok(_) => log::info!("Configuration worked???"),
            Err(e) => match e {
                FpgaError::SPI(spi_e) => panic!("SPI error with info: {:?}", spi_e),
                FpgaError::FPGATimeout(code) => panic!("Fpga timed out?? code: {:x}", code),
            },
        };
        Systick::delay(TEN_MS_DELAY.millis()).await;

        // enable motors
        match fpga.motors_en(true, ctx.local.delay) {
            Ok(status) => log::info!(" enabled motors fpga status: {:b}", status),
            Err(e) => panic!("error enabling motors... {:?}", e),
        };
        Systick::delay(TEN_MS_DELAY.millis()).await;

        // drive forward at different speeds -> should be measuring different encoder values
        loop {
            // move forward at SPEED
            log::info!("moving forward!");
            Systick::delay(TEN_MS_DELAY.millis()).await;
            for _ in 0..500 {
                // forward direction duty cycles
                let duty_cycles = [SPEED, SPEED, SPEED, SPEED]; // dribbler
                                                                // write duty cycle
                match fpga.set_duty_get_encoders(duty_cycles, 0.0, ctx.local.delay) {
                    Ok(encoders) => {
                        log::info!("Encoder Values: {:?}", encoders);
                        log::info!("Status: {:#010b}", fpga.status());
                    }
                    Err(e) => panic!("error writing duty cycles... {:?}", e),
                };
                Systick::delay(HUNDRED_MS_DELAY.millis()).await;
            }

            // move forward at SLOW_SPEED
            log::info!("moving backwards!");
            Systick::delay(TEN_MS_DELAY.millis()).await;
            for _ in 0..500 {
                let duty_cycles = [SLOW_SPEED, SLOW_SPEED, SLOW_SPEED, SLOW_SPEED]; // dribbler
                                                                                    // write duty cycle
                match fpga.set_duty_get_encoders(duty_cycles, 0.0, ctx.local.delay) {
                    Ok(encoders) => {
                        log::info!("Encoder Values: {:?}", encoders);
                        log::info!("Status: {:#010b}", fpga.status());
                    }
                    Err(e) => panic!("error writing duty cycles... {:?}", e),
                };
                Systick::delay(HUNDRED_MS_DELAY.millis()).await;
            }

            // move forward at FAST_SPEED
            log::info!("spinning!");
            Systick::delay(TEN_MS_DELAY.millis()).await;
            for _ in 0..100 {
                let duty_cycles = [FAST_SPEED, FAST_SPEED, FAST_SPEED, FAST_SPEED];
                // write duty cycle
                match fpga.set_duty_get_encoders(duty_cycles, 0.0, ctx.local.delay) {
                    Ok(encoders) => {
                        log::info!("Encoder Values: {:?}", encoders);
                        log::info!("Status: {:#010b}", fpga.status());
                    }
                    Err(e) => panic!("error writing duty cycles... {:?}", e),
                };
                Systick::delay(HUNDRED_MS_DELAY.millis()).await;
            }

            // stand still
            log::info!("standby...");
            Systick::delay(TEN_MS_DELAY.millis()).await;
            for _ in 0..100 {
                let duty_cycles = [0.0, 0.0, 0.0, 0.0];
                // write duty cycle
                match fpga.set_duty_get_encoders(duty_cycles, 0.0, ctx.local.delay) {
                    Ok(encoders) => {
                        log::info!("Encoder Values: {:?}", encoders);
                        log::info!("Status: {:#010b}", fpga.status());
                    }
                    Err(e) => panic!("error writing duty cycles... {:?}", e),
                };
                Systick::delay(HUNDRED_MS_DELAY.millis()).await;
            }
        }
    }
}
