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

// PA Level
const PA_LEVEL: PowerAmplifier = PowerAmplifier::PALow;
// Delay between Packet Sends
const SEND_DELAY_MS: u32 = 50;

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use robojackets_robocup_control::ROBOT_ID;

    use embedded_hal::spi::MODE_0;

    use rtic_nrf24l01::error::RadioError;

    use bsp::board::{self, LPSPI_FREQUENCY};
    use teensy4_bsp as bsp;

    use hal::lpspi::Pins;
    use hal::timer::Blocking;
    use teensy4_bsp::hal;

    use rtic_nrf24l01::Radio;

    use rtic_monotonics::systick::*;

    use ncomm_utils::packing::Packable;

    use robojackets_robocup_rtp::Team;
    use robojackets_robocup_rtp::BASE_STATION_ADDRESSES;
    use robojackets_robocup_rtp::{
        RobotStatusMessage, RobotStatusMessageBuilder, ROBOT_STATUS_SIZE,
    };

    use robojackets_robocup_control::{
        Delay2, RFRadio, RadioSPI, CHANNEL, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
        RADIO_ADDRESS,
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        error: Result<(), RadioError>,
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {
        radio: RFRadio,
        shared_spi: RadioSPI,
        delay2: Delay2,
        robot_status: RobotStatusMessage,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins,
            mut gpio1,
            usb,
            mut gpt2,
            lpspi4,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let shared_spi_pins = Pins {
            pcs0: pins.p10,
            sck: pins.p13,
            sdo: pins.p11,
            sdi: pins.p12,
        };
        let mut shared_spi = hal::lpspi::Lpspi::new(lpspi4, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 1_000_000u32);
            spi.set_mode(MODE_0);
        });

        let radio_cs = gpio1.output(pins.p14);
        let ce = gpio1.output(pins.p41);

        let mut radio = Radio::new(ce, radio_cs);

        let success = radio.begin(&mut shared_spi, &mut delay2);

        let initial_robot_status = RobotStatusMessageBuilder::new().build();

        if success.is_ok() {
            radio.set_pa_level(PA_LEVEL, &mut shared_spi, &mut delay2);
            radio.set_channel(CHANNEL, &mut shared_spi, &mut delay2);
            radio.set_payload_size(ROBOT_STATUS_SIZE as u8, &mut shared_spi, &mut delay2);
            radio.open_writing_pipe(BASE_STATION_ADDRESSES[0], &mut shared_spi, &mut delay2);
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
            Local {
                error: success,
                poller,
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
        shared = [robot_status, radio, shared_spi, delay2,],
        local = [
            total_sends: usize = 0,
            successful_sends: usize = 0,
            last_ball_sense: bool = false,
            last_kick_status: bool = true,
            success_buffer: [bool; 20] = [false; 20],
            success_idx: usize = 0,
        ],
        priority = 2,
    )]
    async fn send_status(ctx: send_status::Context) {
        (
            ctx.shared.robot_status,
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.delay2,
        )
            .lock(|robot_status, radio, spi, delay| {
                let new_robot_status = RobotStatusMessageBuilder::new()
                    .robot_id(ROBOT_ID)
                    .team(Team::Blue)
                    .ball_sense_status(!*ctx.local.last_ball_sense)
                    .kick_status(!*ctx.local.last_kick_status)
                    .build();

                *ctx.local.last_ball_sense = !*ctx.local.last_ball_sense;
                *ctx.local.last_kick_status = !*ctx.local.last_kick_status;

                *robot_status = new_robot_status;

                let mut packed_data = [0u8; ROBOT_STATUS_SIZE];
                robot_status.pack(&mut packed_data).unwrap();

                let report = radio.write(&packed_data, spi, delay);
                radio.flush_tx(spi, delay);

                ctx.local.success_buffer[*ctx.local.success_idx] = report;
                *ctx.local.success_idx = (*ctx.local.success_idx + 1) % ctx.local.success_buffer.len();
            });

        if *ctx.local.success_idx == 0 {
            log::info!("Transmit Success Percent: {}%", ctx.local.success_buffer.iter().filter(|v| **v).count() * 5);
        }

        wait_for_next_send::spawn().ok();
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

        let config = (ctx.shared.shared_spi, ctx.shared.radio, ctx.shared.delay2).lock(|spi, radio, delay| {
            radio.get_registers(spi, delay)
        });

        loop {
            log::error!("Unexpected Configuration");
            log::error!("Found: {:?}", config);
        }
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
