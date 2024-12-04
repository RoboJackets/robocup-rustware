//!
//! Benchmark the Radio by continually receiving packets.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;
use rtic_nrf24l01::config::power_amplifier::PowerAmplifier;

#[global_allocator]
static HEAP: Heap = Heap::empty();

// PA Level
const PA_LEVEL: PowerAmplifier = PowerAmplifier::PALow;

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use embedded_hal::spi::MODE_0;

    use bsp::board::{self, LPSPI_FREQUENCY};
    use teensy4_bsp as bsp;

    use hal::gpio::Trigger;
    use hal::lpspi::{Lpspi, Pins};
    use hal::timer::Blocking;
    use teensy4_bsp::hal;

    use bsp::ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::Radio;

    use rtic_monotonics::systick::*;

    use ncomm_utils::packing::Packable;

    use robojackets_robocup_rtp::BASE_STATION_ADDRESSES;
    use robojackets_robocup_rtp::{ControlMessage, CONTROL_MESSAGE_SIZE};
    use robojackets_robocup_rtp::{RobotStatusMessage, RobotStatusMessageBuilder};

    use robojackets_robocup_control::{
        Delay2, RFRadio, SharedSPI, CHANNEL, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
        RADIO_ADDRESS,
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        total_packets: usize,
    }

    #[shared]
    struct Shared {
        radio: RFRadio,
        shared_spi: SharedSPI,
        delay2: Delay2,
        robot_status: RobotStatusMessage,
        control_message: Option<ControlMessage>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins,
            mut gpio1,
            usb,
            mut gpt2,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let shared_spi_pins = Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = Lpspi::new(shared_spi_block, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 1_000_000u32);
            spi.set_mode(MODE_0);
        });

        let radio_cs = gpio1.output(pins.p14);
        let ce = gpio1.output(pins.p20);

        let mut radio = Radio::new(ce, radio_cs);
        if radio.begin(&mut shared_spi, &mut delay2).is_err() {
            panic!("Unable to Initialize the Radio");
        }

        radio.set_pa_level(PA_LEVEL, &mut shared_spi, &mut delay2);
        radio.set_channel(CHANNEL, &mut shared_spi, &mut delay2);
        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, &mut shared_spi, &mut delay2);
        radio.open_writing_pipe(BASE_STATION_ADDRESSES[0], &mut shared_spi, &mut delay2);
        radio.open_reading_pipe(1, RADIO_ADDRESS, &mut shared_spi, &mut delay2);
        radio.start_listening(&mut shared_spi, &mut delay2);

        let initial_radio_status = RobotStatusMessageBuilder::new().build();

        let rx_int = gpio1.input(pins.p15);
        gpio1.set_interrupt(&rx_int, Some(Trigger::Low));
        rx_int.clear_triggered();

        poll_receive::spawn().ok();
        blink::spawn().ok();

        (
            Shared {
                shared_spi,
                delay2,
                robot_status: initial_radio_status,
                control_message: None,
                radio,
            },
            Local { total_packets: 0 },
        )
    }

    #[task]
    async fn blink(_ctx: blink::Context) {
        loop {
            log::info!("On");

            Systick::delay(1000u32.millis()).await;

            log::info!("Off");

            Systick::delay(1000u32.millis()).await;
        }
    }

    #[task(
        local = [total_packets],
        shared = [shared_spi, delay2, control_message, robot_status, radio],
    )]
    async fn poll_receive(ctx: poll_receive::Context) {
        (
            ctx.shared.shared_spi,
            ctx.shared.delay2,
            ctx.shared.control_message,
            ctx.shared.robot_status,
            ctx.shared.radio,
        )
            .lock(|spi, delay, _control_message, _robot_status, radio| {
                if radio.packet_ready(spi, delay) {
                    let mut buffer = [0u8; CONTROL_MESSAGE_SIZE];
                    radio.read(&mut buffer, spi, delay);

                    match ControlMessage::unpack(&buffer[..]) {
                        Ok(data) => log::info!("Received: {:?}", data),
                        Err(err) => log::info!("Unable to Unpack Data: {:?}", err),
                    }

                    radio.flush_rx(spi, delay);
                }
            });

        wait::spawn().ok();
    }

    #[task]
    async fn wait(_ctx: wait::Context) {
        Systick::delay(50u32.millis()).await;

        poll_receive::spawn().ok();
    }
}
