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
    use core::convert::Infallible;

    use teensy4_pins::common::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::gpio::Output;
    use hal::iomuxc::{self, Config};
    use hal::lpspi::{Lpspi, LpspiError};

    use rtic_monotonics::systick::*;

    use fpga::FPGA;
    use teensy4_pins::imxrt_iomuxc::OpenDrain;

    const SYSCLK: u32 = 600_000_000;

    const OPEN_DRAIN_CONFIG: Config = Config::zero().set_open_drain(OpenDrain::Enabled);

    type Led = Output<P7>;
    type Fpga = FPGA<Infallible, LpspiError, P32, P8, Output<P9>, Output<P6>, Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>>;

    #[local]
    struct Local {
        led: Led,
    }

    #[shared]
    struct Shared {
        fpga: Fpga,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio2,
            lpspi4,
            ..
        } = board::t41(ctx.device);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, SYSCLK, systick_token);

        let led = gpio2.output(pins.p7);

        let spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            1_000_000
        );

        let mut prog_b = pins.p6;
        iomuxc::configure(&mut prog_b, OPEN_DRAIN_CONFIG);
        let prog_b = gpio2.output(prog_b);

        let done = pins.p8;
        let done = gpio2.input(done);

        let csn = gpio2.output(pins.p9);
        let init_b = gpio2.input(pins.p32);

        let fpga =  match FPGA::new(spi, csn, init_b, prog_b, done) {
            Ok(fpga) => fpga,
            Err(_) => panic!("Error Initializing FPGA"),
        };

        blink_led::spawn().ok();

        (
            Shared {
                fpga,
            },
            Local {
                led,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [fpga],
        priority = 1
    )]
    async fn init_fpga(mut ctx: init_fpga::Context) {
        Systick::delay(3_000u32.millis()).await;
        ctx.shared.fpga.lock(|fpga| {
            fpga.configure().await;
        });
    }

    #[task(local = [led], priority = 1)]
    async fn blink_led(ctx: blink_led::Context) {
        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.toggle();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.toggle();
    }
}