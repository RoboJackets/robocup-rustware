//!
//! Benchmark the Radio by continually receiving packets.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use alloc::format;

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

    use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X10};
    use embedded_graphics::mono_font::MonoTextStyle;
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::text::Text;
    use embedded_hal::spi::MODE_0;

    use bsp::board::{self, LPSPI_FREQUENCY};
    use graphics::startup_screen::StartScreen;
    use rtic_monotonics::systick::fugit::{Duration, Instant};
    use teensy4_pins::common::{P19, P18};
    use teensy4_bsp as bsp;

    use hal::gpio::Trigger;
    use hal::lpspi::Pins;
    use hal::timer::Blocking;
    use teensy4_bsp::hal;

    use rtic_nrf24l01::Radio;

    use ncomm_utils::packing::Packable;

    use rtic_monotonics::{systick::*, Monotonic};

    use robojackets_robocup_rtp::{BASE_STATION_ADDRESSES, ROBOT_RADIO_ADDRESSES};
    use robojackets_robocup_rtp::{ControlMessage, CONTROL_MESSAGE_SIZE};
    use robojackets_robocup_rtp::{RobotStatusMessage, RobotStatusMessageBuilder};

    use embedded_graphics::prelude::*;
    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

    use robojackets_robocup_control::{
        Delay2, Display, Gpio2, RFRadio, RadioInterrupt, RadioSPI, CHANNEL, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY, RADIO_ADDRESS,
        MotorEn, Killn
    };

    use embedded_hal::blocking::delay::DelayMs;

    const HEAP_SIZE: usize = 8192;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        total_packets: usize,
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {
        display: Display,
        radio: RFRadio,
        shared_spi: RadioSPI,
        delay2: Delay2,
        rx_int: RadioInterrupt,
        robot_status: RobotStatusMessage,
        control_message: Option<ControlMessage>,
        gpio2: Gpio2,
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
            mut gpio2,
            usb,
            mut gpt2,
            lpspi4,
            lpi2c1,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();

        delay2.delay_ms(1_500u32);

        let shared_spi_pins = Pins {
            pcs0: pins.p10,
            sck: pins.p13,
            sdo: pins.p11,
            sdi: pins.p12,
        };
        let mut shared_spi = hal::lpspi::Lpspi::new(lpspi4, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 5_000_000u32);
            spi.set_mode(MODE_0);
        });

        let radio_cs = gpio1.output(pins.p14);
        let ce = gpio1.output(pins.p41);

        let mut radio = Radio::new(ce, radio_cs);
        if radio.begin(&mut shared_spi, &mut delay2).is_err() {
            panic!("Unable to Initialize the Radio");
        }

        radio.set_pa_level(PA_LEVEL, &mut shared_spi, &mut delay2);
        radio.set_channel(CHANNEL, &mut shared_spi, &mut delay2);
        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, &mut shared_spi, &mut delay2);
        radio.open_writing_pipe(BASE_STATION_ADDRESSES[0], &mut shared_spi, &mut delay2);
        radio.open_reading_pipe(1, ROBOT_RADIO_ADDRESSES[0][0], &mut shared_spi, &mut delay2);

        let initial_radio_status = RobotStatusMessageBuilder::new().build();

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
        let i2c_bus: &'static _ = shared_bus::new_cortexm!(
            imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1> = i2c
        )
        .expect("Failed to initialize shared I2C bus LPI2C1");

        let display_interface = I2CDisplayInterface::new(i2c_bus.acquire_i2c());
        let display: Display = Ssd1306::new(
            display_interface,
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();

        let rx_int = gpio2.input(pins.p9);

        blink::spawn().ok();
        initialize_display::spawn().ok();

        (
            Shared {
                shared_spi,
                delay2,
                rx_int,
                robot_status: initial_radio_status,
                control_message: None,
                gpio2,
                radio,
                display,
            },
            Local {
                total_packets: 0,
                poller,
            },
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

    /// Initialize the display
    #[task(shared=[display, radio, rx_int, shared_spi, delay2, gpio2])]
    async fn initialize_display(mut ctx: initialize_display::Context) {
        ctx.shared.display.lock(|display| {
            display.init().ok();
            display.clear();
            let start_scrn = StartScreen::new(Point::new(0, 0), Point::new(24, 8));
            let _ = start_scrn.draw(display);
            let _ = display.flush();
        });

        (
            ctx.shared.rx_int,
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.delay2,
            ctx.shared.gpio2
        ).lock(|rx_int, radio, spi, delay, gpio2| {
            radio.start_listening(spi, delay);
            gpio2.set_interrupt(&rx_int, Some(Trigger::FallingEdge));
        });
    }

    #[task(
        binds = GPIO2_COMBINED_0_15,
        shared = [rx_int, gpio2],
        priority = 1
    )]
    fn radio_interrupt(ctx: radio_interrupt::Context) {
        if (ctx.shared.rx_int, ctx.shared.gpio2).lock(|rx_int, gpio2| {
            if rx_int.is_triggered() {
                rx_int.clear_triggered();
                gpio2.set_interrupt(rx_int, None);
                return true;
            }
            false
        }) {
            receive_command::spawn().ok();
        }
    }

    #[task(
        local = [
            total_packets,
            rx_timestamps: [Instant<u32, 1, 1_000>; 20] = [Instant::<u32, 1, 1_000>::from_ticks(0u32); 20],
            rx_idx: usize = 0,
        ],
        shared = [rx_int, gpio2, shared_spi, delay2, control_message, robot_status, radio, display],
        priority = 1,
    )]
    async fn receive_command(mut ctx: receive_command::Context) {
        log::info!("Command Received");

        ctx.local.rx_timestamps[*ctx.local.rx_idx] = Systick::now();
        *ctx.local.rx_idx = (*ctx.local.rx_idx + 1) % ctx.local.rx_timestamps.len();

        (
            ctx.shared.shared_spi,
            ctx.shared.delay2,
            ctx.shared.control_message,
            ctx.shared.robot_status,
            ctx.shared.radio,
        )
            .lock(|spi, delay, _control_message, _robot_status, radio| {
                let mut read_buffer = [0u8; CONTROL_MESSAGE_SIZE];
                radio.read(&mut read_buffer, spi, delay);

                let _ = ControlMessage::unpack(&read_buffer).unwrap();

                *ctx.local.total_packets += 1;

                radio.flush_rx(spi, delay);
            });

        (ctx.shared.rx_int, ctx.shared.gpio2).lock(|rx_int, gpio2| {
            rx_int.clear_triggered();
            gpio2.set_interrupt(rx_int, Some(Trigger::FallingEdge));
        });

        if *ctx.local.rx_idx == 0 {
            let char_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
            ctx.shared.display.lock(|display| {
                display.clear();
                let _ = Text::new(
                    &format!("RX Delay: {}ms", calculate_rx_delay(ctx.local.rx_timestamps).to_millis()),
                    Point { x: 0, y: 32 },
                    char_style
                )
                .draw(display);
                display.flush().ok();
            });
        }
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }

    fn calculate_rx_delay(instants: &[Instant<u32, 1, 1_000>]) -> Duration<u32, 1, 1_000> {
        let mut total_duration = Duration::<u32, 1, 1_000>::millis(0);
        for i in 0..(instants.len() - 1) {
            total_duration += instants[i+1] - instants[i];
        }
        total_duration / ((instants.len() - 1) as u32)
    }
}
