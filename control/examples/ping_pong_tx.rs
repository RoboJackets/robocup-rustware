#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_panic as _;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use embedded_hal::spi::MODE_0;
    use rtic_nrf24l01::Radio;

    use bsp::board::{self, LPSPI_FREQUENCY};
    use teensy4_bsp as bsp;

    use bsp::hal;
    use hal::timer::Blocking;

    use bsp::ral;
    use ral::lpspi::LPSPI3;

    use rtic_monotonics::systick::*;

    use robojackets_robocup_control::robot::TEAM_NUM;
    use robojackets_robocup_rtp::BASE_STATION_ADDRESSES;

    use robojackets_robocup_control::{
        Delay2, RFRadio, SharedSPI, BASE_AMPLIFICATION_LEVEL, CHANNEL, GPT_CLOCK_SOURCE,
        GPT_DIVIDER, GPT_FREQUENCY, RADIO_ADDRESS,
    };

    #[local]
    struct Local {
        radio: RFRadio,
    }

    #[shared]
    struct Shared {
        shared_spi: SharedSPI,
        delay: Delay2,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio1,
            usb,
            mut gpt2,
            ..
        } = board::t41(ctx.device);

        // usb logging setup
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // systic setup
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // gpt 1 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let spi_pins = hal::lpspi::Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = hal::lpspi::Lpspi::new(shared_spi_block, spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 5_000_000);
            spi.set_mode(MODE_0);
        });

        let ce = gpio1.output(pins.p20);
        let csn = gpio1.output(pins.p14);

        // Initialize the Radio
        let mut radio = Radio::new(ce, csn);
        if radio.begin(&mut shared_spi, &mut delay).is_err() {
            panic!("Unable to Initialize the Radio");
        }
        radio.set_pa_level(BASE_AMPLIFICATION_LEVEL, &mut shared_spi, &mut delay);
        radio.set_channel(CHANNEL, &mut shared_spi, &mut delay);
        radio.set_payload_size(4, &mut shared_spi, &mut delay);
        radio.open_writing_pipe(RADIO_ADDRESS, &mut shared_spi, &mut delay);
        radio.open_reading_pipe(
            1,
            BASE_STATION_ADDRESSES[TEAM_NUM],
            &mut shared_spi,
            &mut delay,
        );
        radio.stop_listening(&mut shared_spi, &mut delay);

        ping_pong::spawn().unwrap();

        (Shared { shared_spi, delay }, Local { radio })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn wait_one_second(_ctx: wait_one_second::Context) {
        log::info!("Waiting");

        Systick::delay(1_000u32.millis()).await;

        ping_pong::spawn().ok();
    }

    #[task(local=[radio, listening: bool = false, payload: u32 =0], shared=[delay, shared_spi], priority = 1)]
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
    (value[0] as u32)
        | ((value[1] as u32) << 8)
        | ((value[2] as u32) << 16)
        | ((value[3] as u32) << 24)
}
