#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_bsp as bsp;

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(
    device = bsp,
    peripherals = true,
    dispatchers = [GPT2]
)]
mod app {
    use super::*;

    use core::convert::Infallible;
    use bsp::board;
    use main::{SharedSPI, Delay1, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY};
    use rtic_monotonics::systick::*;
    use bsp::hal::timer::Blocking;
    use bsp::hal::lpspi::{Lpspi, Pins, LpspiError};
    use teensy4_bsp::ral::lpspi::LPSPI3;
    use bsp::board::LPSPI_FREQUENCY;
    use embedded_hal::spi::MODE_0;
    use imxrt_iomuxc::prelude::*;
    use fpga_rs::{FPGA, FPGA_SPI_FREQUENCY, FPGA_SPI_MODE};
    use teensy4_pins::t41::*;
    use bsp::hal::gpio::Output;

    #[local]
    struct Local {
        fpga: FPGA<SharedSPI, Output<P9>, P29, Output<P28>, P30, Delay1, LpspiError, Infallible>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut pins,
            mut gpio1,
            mut gpio2,
            mut gpio3,
            mut gpio4,
            usb,
            mut gpt1,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        let spi_pins = Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut fpga_spi = Lpspi::new(shared_spi_block, spi_pins);
        fpga_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY / 8, 400_000);
            spi.set_mode(MODE_0);
        });

        let fpga_cs = gpio2.output(pins.p9);
        let init_b = gpio4.input(pins.p29);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        let done = gpio3.input(pins.p30);

        let fpga = match FPGA::new(fpga_spi, fpga_cs, init_b, prog_b, done, delay) {
            Ok(fpga) => fpga,
            Err(err) => panic!("{:?}", err),
        };

        init_fpga::spawn().ok();

        (
            Shared {

            },
            Local {
                fpga,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

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
            Err(_) => panic!("Unable to configure fpga"),
        }
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
                    0 => ([1.0, 0.0, 0.0, 0.0], false),
                    1 => ([0.0, 1.0, 0.0, 0.0], false),
                    2 => ([0.0, 0.0, 1.0, 0.0], false),
                    3 => ([0.0, 0.0, 0.0, 1.0], false),
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