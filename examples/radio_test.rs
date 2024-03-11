#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use embedded_hal::spi::MODE_0;
    use rtic_nrf24l01::{Radio, config::*};

    use teensy4_bsp::hal::gpio::Output;
    use teensy4_bsp::hal::lpspi::Lpspi;
    use teensy4_pins::t41::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::gpt::{Gpt1, ClockSource};
    use hal::timer::Blocking;

    use rtic_monotonics::systick::*;

    const GPT1_FREQUENCY: u32 = 1_000;
    const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;

    type Delay = Blocking<Gpt1, GPT1_FREQUENCY>;
    type SPI = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
    type CE = Output<P0>;
    type CSN = Output<P9>;

    #[local]
    struct Local {
        spi: SPI,
        delay: Delay,
        ce: Option<CE>,
        csn: Option<CSN>
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio1,
            mut gpio2,
            usb,
            lpspi4,
            mut gpt1,
            ..
        } = board::t41(ctx.device);

        // usb logging setup
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // systic setup
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // gpt 1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT1_DIVIDER);
        gpt1.set_clock_source(GPT1_CLOCK_SOURCE);
        let delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

        // initialize spi
        let mut spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            10_000_000
        );

        spi.disabled(|spi| {
            spi.set_clock_hz(board::LPSPI_FREQUENCY, 10_000_000u32);
            spi.set_mode(MODE_0);
        });

        let ce = gpio1.output(pins.p0);
        let csn = gpio2.output(pins.p9);

        // init fake CS pin (TEMPORARY) and required reset pin

        init_radio::spawn().unwrap();

        (
            Shared {

            },
            Local {
                delay,
                spi,
                ce: Some(ce),
                csn: Some(csn),
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [delay, spi, ce, csn], priority = 1)]
    async fn init_radio(ctx: init_radio::Context) {
        Systick::delay(2_000u32.millis()).await;

        let csn = ctx.local.csn.take().unwrap();
        let ce = ctx.local.ce.take().unwrap();

        let mut radio = Radio::new(ce, csn);

        if radio.begin(ctx.local.spi, ctx.local.delay).is_err() {
            panic!("Unable to Initialize the radio");
        }

        radio.set_pa_level(power_amplifier::PowerAmplifier::PALow, ctx.local.spi, ctx.local.delay);

        radio.set_payload_size(4, ctx.local.spi, ctx.local.delay);

        radio.open_writing_pipe([0xE7, 0xE7, 0xE7, 0xE7, 0xE7], ctx.local.spi, ctx.local.delay);

        radio.open_reading_pipe(1, [0xC3, 0xC3, 0xC3, 0xC3, 0xC3], ctx.local.spi, ctx.local.delay);

        radio.start_listening(ctx.local.spi, ctx.local.delay);

        Systick::delay(1_000u32.millis()).await;

        radio.stop_listening(ctx.local.spi, ctx.local.delay);

        log::info!("Configuration: {:?}", radio.get_registers(ctx.local.spi, ctx.local.delay));

        let mut payload = 0u32;
        loop {
            let p = crate::to_bytes(&payload);
            log::info!("Sending P: {:?}", p);
            let report = radio.write(&p, ctx.local.spi, ctx.local.delay);

            if report {
                log::info!("Sent Successfully");
                log::info!("Sent {:?}", payload);
                payload += 1;
            } else {
                log::info!("Transmission Failed");
            }

            Systick::delay(1_000u32.millis()).await;

            log::info!("Configuration: {:?}", radio.get_registers(ctx.local.spi, ctx.local.delay));
        }
    }
}

fn to_bytes(value: &u32) -> [u8; 4] {
    [
        (value & 0xFF) as u8,
        ((value & (0xFF << 8)) >> 8) as u8,
        ((value & (0xFF << 16)) >> 16) as u8,
        ((value & (0xFF << 24)) >> 24) as u8,
    ]
}