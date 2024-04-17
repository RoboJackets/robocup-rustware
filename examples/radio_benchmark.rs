//!
//! Benchmark the radio by receiving packets
//! 
//! Note: The Receiver simply receives packets, while the Sender prints
//! out the results after sending 100 packets at a given channel
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Sender,
    Receiver,
}

#[cfg(feature = "sender")]
pub const ROLE: Role = Role::Sender;
#[cfg(any(feature = "receiver", not(feature = "sender")))]
pub const ROLE: Role = Role::Sender;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::convert::Infallible;
    use core::mem::MaybeUninit;

    use main::ROBOT_ID;

    use embedded_hal::spi::MODE_0;

    use teensy4_pins::t41::*;

    use teensy4_bsp as bsp;
    use bsp::board::{self, LPSPI_FREQUENCY};

    use teensy4_bsp::hal as hal;
    use hal::lpspi::{LpspiError, Lpspi, Pins};
    use hal::gpio::{Output, Input, Trigger, Port};
    use hal::gpt::{ClockSource, Gpt2};
    use hal::timer::Blocking;

    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::{Radio, config::*};

    use rtic_monotonics::systick::*;

    use packed_struct::prelude::*;

    use robojackets_robocup_rtp::{ControlMessage, Team, CONTROL_MESSAGE_SIZE};
    use robojackets_robocup_rtp::{RobotStatusMessage, RobotStatusMessageBuilder, ROBOT_STATUS_SIZE};
    use robojackets_robocup_rtp::{BASE_STATION_ADDRESS, ROBOT_RADIO_ADDRESSES};

    // Constants
    const GPT_FREQUENCY: u32 = 1_000;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const SEND_DELAY_MS: u32 = 100;

    // Radio Channel
    const RADIO_CHANNEL: u8 = 0;
    // Total Packets to Send
    const TOTAL_SEND_PACKETS: usize = 100;

    // Type Definitions
    type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
    type RadioCE = Output<P0>;
    type RadioCSN = Output<P6>;
    type RadioInterrupt = Input<P1>;
    type Delay2 = Blocking<Gpt2, GPT_FREQUENCY>;
    type Gpio1 = Port<1>;

    #[local]
    struct Local {
    }

    #[shared]
    struct Shared {
        radio: Radio<RadioCE, RadioCSN, SharedSPI, Delay2, Infallible, LpspiError>,
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
            mut pins,
            mut gpio1,
            mut gpio2,
            mut gpio3,
            mut gpio4,
            usb,
            lpi2c1,
            lpspi4,
            mut gpt1,
            mut gpt2,
            pit: (pit0, pit1, pit2, pit3),
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
            sck: pins.p17,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = Lpspi::new(shared_spi_block, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 5_000_000u32);
            spi.set_mode(MODE_0);
        });

        let radio_cs = gpio2.output(pins.p6);
        let ce = gpio1.output(pins.p0);

        let mut radio = Radio::new(ce, radio_cs);
        if radio.begin(&mut shared_spi, &mut delay2).is_err() {
            panic!("Unable to Initialize the Radio");
        }

        radio.set_pa_level(power_amplifier::PowerAmplifier::PALow, &mut shared_spi, &mut delay2);
        radio.open_writing_pipe(BASE_STATION_ADDRESS, &mut shared_spi, &mut delay2);
        radio.open_reading_pipe(1, ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize], &mut shared_spi, &mut delay2);
        radio.set_channel(RADIO_CHANNEL, &mut shared_spi, &mut delay2);

        let initial_radio_status = RobotStatusMessageBuilder::new().build();

        let rx_int = gpio1.input(pins.p1);

        match ROLE {
            Role::Receiver => {
                gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));
                rx_int.clear_triggered();

                radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, &mut shared_spi, &mut delay2);
                radio.start_listening(&mut shared_spi, &mut delay2);
                
                // Task is interrupt based
            },
            Role::Sender => {
                radio.set_payload_size(ROBOT_STATUS_SIZE as u8, &mut shared_spi, &mut delay2);
                radio.stop_listening(&mut shared_spi, &mut delay2);
                
                // Spawn Sender Role Task
                send_status::spawn().ok();
            }
        }

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
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        binds = GPIO1_COMBINED_0_15,
        shared = [rx_int, gpio1],
        priority = 2
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
        shared = [rx_int, gpio1, shared_spi, delay2, control_message, robot_status, radio],
        priority = 2,
    )]
    async fn receive_command(ctx: receive_command::Context) {
        (
            ctx.shared.shared_spi,
            ctx.shared.delay2,
            ctx.shared.control_message,
            ctx.shared.robot_status,
            ctx.shared.radio,
        ).lock(|spi, delay, control_message, robot_status, radio| {
            let mut read_buffer = [0u8; CONTROL_MESSAGE_SIZE];
            radio.read(&mut read_buffer, spi, delay);

            let control_message = match ControlMessage::unpack_from_slice(&read_buffer[..]) {
                Ok(control_message) => control_message,
                Err(_err) => {
                    log::info!("Error Unpacking Control Command: {:?}", _err);
                    return;
                }
            };

            log::info!("Control Command Received: {:?}", control_message);

            radio.flush_rx(spi, delay);
        });

        (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            rx_int.clear_triggered();
            gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));
        });
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
        (
            ctx.shared.robot_status,
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.delay2,
        ).lock(|robot_status, radio, spi, delay| {
            let robot_status = RobotStatusMessageBuilder::new()
                .robot_id(ROBOT_ID)
                .team(Team::Blue)
                .ball_sense_status(!*ctx.local.last_ball_sense)
                .kick_status(!*ctx.local.last_kick_status)
                .build();

            *ctx.local.last_ball_sense = !*ctx.local.last_ball_sense;
            *ctx.local.last_kick_status != *ctx.local.last_kick_status;

            *ctx.shared.robot_status = robot_status;

            let packed_data = match robot_status.pack() {
                Ok(bytes) => bytes,
                Err(_err) => {
                    panic!("Error Packing Robot Status: {:?}", _err);
                }
            };

            let report = radio.write(&packed_data, spi, delay);

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
    async fn wait_for_next_send(ctx: wait_for_next_send::Context) {
        Systick::delay(SEND_DELAY_MS.millis()).await;

        send_status::spawn().ok();
    }
}