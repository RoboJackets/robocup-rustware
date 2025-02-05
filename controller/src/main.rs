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
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use embedded_hal::spi::MODE_0;
    use rtic_nrf24l01::Radio;

    use bsp::board::{self, LPSPI_FREQUENCY};
    use teensy4_bsp as bsp;

    use teensy4_bsp::hal::gpio::{Input, Output};
    use teensy4_pins::t41::*;

    use rtic_monotonics::systick::*;

    use ncomm_utils::packing::Packable;

    use bsp::hal;
    use bsp::hal::adc::{Adc, AnalogInput};
    use hal::timer::Blocking;

    use bsp::ral;
    use ral::lpspi::LPSPI3;

    use robojackets_robocup_rtp::BASE_STATION_ADDRESSES;
    use robojackets_robocup_rtp::{ControlMessageBuilder, CONTROL_MESSAGE_SIZE};

    use robojackets_robocup_control::{
        Delay2, RFRadio, SharedSPI, BASE_AMPLIFICATION_LEVEL, CHANNEL, GPT_CLOCK_SOURCE,
        GPT_DIVIDER, GPT_FREQUENCY, RADIO_ADDRESS,
    };

    use core::fmt::Write;
    use embedded_graphics::draw_target::DrawTargetExt;
    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    #[local]
    struct Local {
        radio: RFRadio,
    }

    #[shared]
    struct Shared {
        shared_spi: SharedSPI,
        delay: Delay2,
        adc: Adc<1>,
        joyLX: AnalogInput<P16, 1>,
        joyLY: AnalogInput<P17, 1>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb,
            pins,
            mut gpio1,
            mut gpt2,
            adc1,
            lpi2c1,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

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
        radio.set_pa_level(PowerAmplifier::PALow, &mut shared_spi, &mut delay);
        radio.set_channel(CHANNEL, &mut shared_spi, &mut delay);
        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, &mut shared_spi, &mut delay);
        radio.open_writing_pipe(RADIO_ADDRESS, &mut shared_spi, &mut delay);
        radio.open_reading_pipe(1, BASE_STATION_ADDRESSES[0], &mut shared_spi, &mut delay);
        radio.stop_listening(&mut shared_spi, &mut delay);

        //set screen
        //set i2c
        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();
        Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();

        //start ADC
        let joyLX = bsp::hal::adc::AnalogInput::new(pins.p16);
        let joyLY = bsp::hal::adc::AnalogInput::new(pins.p17);

        log::info!("logging");
        send_tests::spawn().ok();

        (
            Shared {
                shared_spi,
                delay,
                joyLX,
                joyLY,
                adc: adc1,
            },
            Local { radio },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn delay(ctx: delay::Context) {
        Systick::delay(500u32.millis()).await;

        send_tests::spawn().ok();
    }

    #[task(priority = 1,local=[radio],shared=[adc,joyLX,joyLY,delay,shared_spi])]
    async fn send_tests(ctx: send_tests::Context) {
        log::info!("Tick!");
        let mut readingX: u16 = 0;
        let mut readingY: u16 = 0;
        (ctx.shared.adc, ctx.shared.joyLX, ctx.shared.joyLY).lock(|adc, joyLX, joyLY| {
            readingX = adc.read_blocking(joyLX);
            readingY = adc.read_blocking(joyLY);
        });

        log::info!("ADC X: {}", readingX);
        log::info!("ADC Y: {}", readingY);

        (ctx.shared.shared_spi, ctx.shared.delay).lock(|spi, delay| {
            let control_message = ControlMessageBuilder::new()
                .body_x(readingX as f32 / 1024.0)
                .body_y(readingY as f32 / 1024.0)
                .robot_id(1)
                .build();

            log::info!("Sending {:?}", control_message);
            let mut packed_data = [0u8; CONTROL_MESSAGE_SIZE];
            control_message.pack(&mut packed_data).unwrap();

            let report = ctx.local.radio.write(&packed_data, spi, delay);
            ctx.local.radio.flush_tx(spi, delay);

            if report {
                log::info!("Received Acknowledgement From Transmission");
            } else {
                log::info!("No Ack Received");
            }
        });

        delay::spawn().ok();
    }
}
