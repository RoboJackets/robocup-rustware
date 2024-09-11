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
const PA_LEVEL: PowerAmplifier = PowerAmplifier::PAMin;

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use embedded_hal::spi::MODE_0;

    use teensy4_bsp as bsp;
    use bsp::board::{self, LPSPI_FREQUENCY};

    use teensy4_bsp::hal as hal;
    use hal::lpspi::{Lpspi, Pins};
    use hal::gpio::Trigger;
    use hal::timer::Blocking;

    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::Radio;

    use ncomm_utils::packing::Packable;

    use rtic_monotonics::systick::*;

    use robojackets_robocup_rtp::{ControlMessage, CONTROL_MESSAGE_SIZE};
    use robojackets_robocup_rtp::{RobotStatusMessage, RobotStatusMessageBuilder};
    use robojackets_robocup_rtp::BASE_STATION_ADDRESS;

    use main::{
        SharedSPI,
        RFRadio,
        RadioInterrupt,
        Delay2,
        Gpio1,
        CHANNEL,
        RADIO_ADDRESS,
        GPT_FREQUENCY,
        GPT_CLOCK_SOURCE,
        GPT_DIVIDER,
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
        rx_int: RadioInterrupt,
        robot_status: RobotStatusMessage,
        control_message: Option<ControlMessage>,
        gpio1: Gpio1,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

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
            spi.set_clock_hz(LPSPI_FREQUENCY, 5_000_000u32);
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
        radio.open_writing_pipe(BASE_STATION_ADDRESS, &mut shared_spi, &mut delay2);
        radio.open_reading_pipe(1, RADIO_ADDRESS, &mut shared_spi, &mut delay2);
        radio.start_listening(&mut shared_spi, &mut delay2);

        let initial_radio_status = RobotStatusMessageBuilder::new().build();

        let rx_int = gpio1.input(pins.p15);
        gpio1.set_interrupt(&rx_int, Some(Trigger::Low));

        blink::spawn().ok();

        (
            Shared {
                shared_spi,
                delay2,
                rx_int,
                robot_status: initial_radio_status,
                control_message: None,
                gpio1,
                radio,
            },
            Local {
                total_packets: 0,
            }
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
        binds = GPIO1_COMBINED_16_31,
        shared = [rx_int, gpio1],
        priority = 1
    )]
    fn radio_interrupt(ctx: radio_interrupt::Context) {
        log::info!("Command Received");

        if (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            if rx_int.is_triggered() {
                rx_int.clear_triggered();
                gpio1.set_interrupt(&rx_int, None);
                return true;
            }
            false
        }) {
            receive_command::spawn().ok();
        }
    }

    #[task(
        local = [total_packets],
        shared = [rx_int, gpio1, shared_spi, delay2, control_message, robot_status, radio],
        priority = 1,
    )]
    async fn receive_command(ctx: receive_command::Context) {
        (
            ctx.shared.shared_spi,
            ctx.shared.delay2,
            ctx.shared.control_message,
            ctx.shared.robot_status,
            ctx.shared.radio,
        ).lock(|spi, delay, _control_message, _robot_status, radio| {
            let mut read_buffer = [0u8; CONTROL_MESSAGE_SIZE];
            radio.read(&mut read_buffer, spi, delay);

            let control_message = ControlMessage::unpack(&read_buffer).unwrap();

            *ctx.local.total_packets += 1;

            log::info!("Control Command Received: {:?}", control_message);

            radio.flush_rx(spi, delay);
        });

        (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            rx_int.clear_triggered();
            gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));
        });

        log::info!("Received {} Total Packets", *ctx.local.total_packets);
    }
}