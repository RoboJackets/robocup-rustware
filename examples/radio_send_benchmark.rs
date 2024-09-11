//!
//! Benchmark the Radio by Sending X Packets and seeing how many are
//! acknowledged by the receiver.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;
use rtic_nrf24l01::config::power_amplifier::PowerAmplifier;

#[global_allocator]
static HEAP: Heap = Heap::empty();

// Number of Packets to Send
const TOTAL_SEND_PACKETS: usize = 100;
// PA Level
const PA_LEVEL: PowerAmplifier = PowerAmplifier::PALow;
// Delay between Packet Sends
const SEND_DELAY_MS: u32 = 50;

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use main::ROBOT_ID;

    use embedded_hal::spi::MODE_0;

    use rtic_nrf24l01::error::RadioError;

    use bsp::board::{self, LPSPI_FREQUENCY};
    use teensy4_bsp as bsp;

    use hal::lpspi::{Lpspi, Pins};
    use hal::timer::Blocking;
    use teensy4_bsp::hal;

    use bsp::ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::Radio;

    use rtic_monotonics::systick::*;

    use ncomm_utils::packing::Packable;

    use robojackets_robocup_rtp::Team;
    use robojackets_robocup_rtp::BASE_STATION_ADDRESS;
    use robojackets_robocup_rtp::{
        RobotStatusMessage, RobotStatusMessageBuilder, ROBOT_STATUS_SIZE,
    };

    use main::{
        Delay2, RFRadio, SharedSPI, CHANNEL, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
        RADIO_ADDRESS,
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        error: Result<(), RadioError>,
    }

    #[shared]
    struct Shared {
        radio: RFRadio,
        shared_spi: SharedSPI,
        delay2: Delay2,
        robot_status: RobotStatusMessage,
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

        let success = radio.begin(&mut shared_spi, &mut delay2);

        let initial_robot_status = RobotStatusMessageBuilder::new().build();

        if !success.is_err() {
            radio.set_pa_level(PA_LEVEL, &mut shared_spi, &mut delay2);
            radio.set_channel(CHANNEL, &mut shared_spi, &mut delay2);
            radio.set_payload_size(ROBOT_STATUS_SIZE as u8, &mut shared_spi, &mut delay2);
            radio.open_writing_pipe(BASE_STATION_ADDRESS, &mut shared_spi, &mut delay2);
            radio.open_reading_pipe(1, RADIO_ADDRESS, &mut shared_spi, &mut delay2);
            radio.stop_listening(&mut shared_spi, &mut delay2);

            send_status::spawn().ok();
        } else {
            print_error::spawn().ok();
        }

        (
            Shared {
                shared_spi,
                delay2,
                robot_status: initial_robot_status,
                radio,
            },
            Local { error: success },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [robot_status, radio, shared_spi, delay2,],
        local = [
            total_sends: usize = 0,
            successful_sends: usize = 0,
            last_ball_sense: bool = false,
            last_kick_status: bool = true,
        ],
        priority = 2,
    )]
    async fn send_status(ctx: send_status::Context) {
        Systick::delay(1_000u32.millis()).await;

        log::info!("Sending Statuses");
        (
            ctx.shared.robot_status,
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.delay2,
        )
            .lock(|robot_status, radio, spi, delay| {
                let new_robot_status = RobotStatusMessageBuilder::new()
                    .robot_id(ROBOT_ID)
                    .team(Team::Yellow)
                    .ball_sense_status(!*ctx.local.last_ball_sense)
                    .kick_status(!*ctx.local.last_kick_status)
                    .build();

                *ctx.local.last_ball_sense = !*ctx.local.last_ball_sense;
                *ctx.local.last_kick_status = !*ctx.local.last_kick_status;

                *robot_status = new_robot_status;

                log::info!("Sending {:?}", new_robot_status);

                let mut packed_data = [0u8; ROBOT_STATUS_SIZE];
                robot_status.pack(&mut packed_data).unwrap();

                let report = radio.write(&packed_data, spi, delay);
                radio.flush_tx(spi, delay);

                if report {
                    log::info!("Received Acknowledgement From Transmission");
                    *ctx.local.successful_sends += 1;
                } else {
                    log::info!("No Ack Received");
                }

                *ctx.local.total_sends += 1;

                if *ctx.local.total_sends >= TOTAL_SEND_PACKETS {
                    log::info!(
                        "{} / {} Packets Successfully Acknowledged",
                        ctx.local.successful_sends,
                        ctx.local.total_sends,
                    )
                } else {
                    wait_for_next_send::spawn().ok();
                }
            });
    }

    #[task(priority = 1)]
    async fn wait_for_next_send(_ctx: wait_for_next_send::Context) {
        Systick::delay(SEND_DELAY_MS.millis()).await;

        send_status::spawn().ok();
    }

    #[task(
        shared = [shared_spi, radio, delay2],
        local = [error],
        priority = 1
    )]
    async fn print_error(ctx: print_error::Context) {
        Systick::delay(1_000u32.millis()).await;

        log::info!("ERROR");

        (ctx.shared.shared_spi, ctx.shared.radio, ctx.shared.delay2).lock(|spi, radio, delay| {
            log::info!("Configuration: {:?}", radio.get_registers(spi, delay));
        });

        // panic!("Error Occurred: {:?}", error);
    }
}
