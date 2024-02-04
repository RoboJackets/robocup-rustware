#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use core::convert::Infallible;

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

    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_monotonics::systick::*;
    use you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::board::LPSPI_FREQUENCY;

    const GPT1_FREQUENCY: u32 = 1_000;
    const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;

    type Delay = Blocking<Gpt1, GPT1_FREQUENCY>;
    type CE = Output<P0>;
    type CSN = Output<P9>;
    type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;

    #[local]
    struct Local {
        radio: Radio<CE, CSN, SharedSPI, Delay, Infallible, hal::lpspi::LpspiError>,
    }

    #[shared]
    struct Shared {
        shared_spi: SharedSPI,
        delay: Delay,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio1,
            mut gpio2,
            usb,
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
        let mut delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

        let spi_pins = hal::lpspi::Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = hal::lpspi::Lpspi::new(shared_spi_block, spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 10_000_000u32);
            spi.set_mode(MODE_0);
        });

        let ce = gpio1.output(pins.p0);
        let csn = gpio2.output(pins.p9);

        // Initialize the Radio
        let mut radio = Radio::new(ce, csn);
        if radio.begin(&mut shared_spi, &mut delay).is_err() {
            panic!("Unable to Initialize the Radio");
        }
        radio.set_pa_level(power_amplifier::PowerAmplifier::PALow, &mut shared_spi, &mut delay);
        radio.set_payload_size(4, &mut shared_spi, &mut delay);
        radio.open_writing_pipe([0xE7, 0xE7, 0xE7, 0xE7, 0xE7], &mut shared_spi, &mut delay);
        radio.open_reading_pipe(1, [0xC3, 0xC3, 0xC3 ,0xC3, 0xC3], &mut shared_spi, &mut delay);
        radio.start_listening(&mut shared_spi, &mut delay);

        ping_pong::spawn().unwrap();

        (
            Shared {
                shared_spi,
                delay,
            },
            Local {
                radio,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn wait_one_second(_ctx: wait_one_second::Context) {
        Systick::delay(1_000u32.millis()).await;

        ping_pong::spawn().ok();
    }

    #[task(local=[radio, listening: bool =true, payload: u32 =0], shared=[delay, shared_spi], priority = 1)]
    async fn ping_pong(ctx: ping_pong::Context) {
        if *ctx.local.listening {
            (ctx.shared.delay, ctx.shared.shared_spi).lock(|delay, spi| {
                if ctx.local.radio.available(spi, delay) {
                    let mut read_buffer = [0u8; 4];
                    ctx.local.radio.read(&mut read_buffer, spi, delay);
                    *ctx.local.payload = crate::from_bytes(&read_buffer);
                    log::info!("Received: {}", *ctx.local.payload);

                    *ctx.local.listening = false;
                    ctx.local.radio.stop_listening(spi, delay);
                    *ctx.local.payload += 1;
                }
            });
        } else {
            let p = crate::to_bytes(ctx.local.payload);
            (ctx.shared.delay, ctx.shared.shared_spi).lock(|delay, spi| {
                let report = ctx.local.radio.write(&p, spi, delay);

                if report {
                    log::info!("Sent: {}", *ctx.local.payload);
                    *ctx.local.listening = true;
                    ctx.local.radio.start_listening(spi, delay);
                } else {
                    log::info!("Transmission Timed Out");
                }
            });
        }

        wait_one_second::spawn().ok();
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

fn from_bytes(value: &[u8]) -> u32 {
    (value[0] as u32) | ((value[1] as u32) << 8) |
        ((value[2] as u32) << 16) | ((value[3] as u32) << 24)
}