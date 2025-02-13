//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use embedded_alloc::Heap;

mod drive_mod;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use crate::drive_mod::InputState;

    use super::*;

    use drive_mod::DriveMod;
    use imxrt_hal::gpio::Input;

    use core::mem::MaybeUninit;

    use embedded_hal::spi::MODE_0;
    use rtic_nrf24l01::config::power_amplifier::PowerAmplifier;
    use rtic_nrf24l01::Radio;

    use bsp::board::{self, LPSPI_FREQUENCY};
    use teensy4_bsp as bsp;

    use teensy4_pins::t41::*;

    use rtic_monotonics::systick::*;

    use ncomm_utils::packing::Packable;

    use bsp::hal;
    use bsp::hal::adc::{Adc, AnalogInput};
    use hal::gpio::Trigger;
    use hal::timer::Blocking;

    use bsp::ral;
    use ral::lpspi::LPSPI3;

    use robojackets_robocup_rtp::BASE_STATION_ADDRESSES;
    use robojackets_robocup_rtp::{ControlMessageBuilder, CONTROL_MESSAGE_SIZE};

    use robojackets_robocup_control::{
        Delay2, RFRadio, SharedSPI, CHANNEL, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
        RADIO_ADDRESS,
    };

    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {}

    pub struct IOInputs {
        adc: Adc<1>,
        joy_lx: AnalogInput<P24, 1>,
        joy_ly: AnalogInput<P25, 1>,
        joy_rx: AnalogInput<P16, 1>,
        joy_ry: AnalogInput<P17, 1>,

        btn_up: Input<P2>,
        btn_down: Input<P5>,
        btn_left: Input<P3>,
        btn_right: Input<P4>,
    }

    #[shared]
    struct Shared {
        shared_spi: SharedSPI,
        delay: Delay2,
        inputs: IOInputs,
        radio: RFRadio,
        display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
            DisplaySize128x64,
            ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
        >,
        drive_mod: DriveMod,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            usb,
            pins,
            mut gpio1,
            mut gpio4,
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
        let delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

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
        let radio = Radio::new(ce, csn);

        //set screen
        //set i2c
        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::MHz1);
        let interface = I2CDisplayInterface::new(i2c);
        let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        //start ADC
        let joy_lx = bsp::hal::adc::AnalogInput::new(pins.p24);
        let joy_ly = bsp::hal::adc::AnalogInput::new(pins.p25);
        let joy_rx = bsp::hal::adc::AnalogInput::new(pins.p16);
        let joy_ry = bsp::hal::adc::AnalogInput::new(pins.p17);

        //buttons
        let btn_up = gpio4.input(pins.p2);
        let btn_down = gpio4.input(pins.p5);
        let btn_left = gpio4.input(pins.p3);
        let btn_right = gpio4.input(pins.p4);

        //set interrupt for buttons
        gpio4.set_interrupt(&btn_up, Some(Trigger::EitherEdge));
        gpio4.set_interrupt(&btn_down, Some(Trigger::EitherEdge));
        gpio4.set_interrupt(&btn_left, Some(Trigger::EitherEdge));
        gpio4.set_interrupt(&btn_right, Some(Trigger::EitherEdge));

        let inputs = IOInputs {
            adc: adc1,
            joy_lx,
            joy_ly,
            joy_rx,
            joy_ry,
            btn_up,
            btn_down,
            btn_left,
            btn_right,
        };

        let drive_mod = DriveMod::new();

        //init devices
        init_devices::spawn().ok();

        (
            Shared {
                shared_spi,
                delay,
                inputs,
                radio,
                display,
                drive_mod,
            },
            Local {},
        )
    }

    #[task(priority = 1, shared=[display,radio, shared_spi,delay])]
    async fn init_devices(mut ctx: init_devices::Context) {
        //wait 1s to allow debugging to capture output
        Systick::delay(1000u32.millis()).await;

        //init the display
        ctx.shared.display.lock(|display| {
            display.init().unwrap();
        });

        //init the radio
        (ctx.shared.radio, ctx.shared.shared_spi, ctx.shared.delay).lock(
            |radio, shared_spi, delay| {
                if radio.begin(shared_spi, delay).is_err() {
                    panic!("Unable to Initialize the Radio");
                }
                radio.set_pa_level(PowerAmplifier::PALow, shared_spi, delay);
                radio.set_channel(CHANNEL, shared_spi, delay);
                radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, shared_spi, delay);
                radio.open_writing_pipe(RADIO_ADDRESS, shared_spi, delay);
                radio.open_reading_pipe(1, BASE_STATION_ADDRESSES[0], shared_spi, delay);
                radio.stop_listening(shared_spi, delay);
            },
        );

        //spawn the actual task
        send_tests::spawn().ok();
    }

    #[task(
        binds = GPIO4_COMBINED_0_15,
        priority = 1, shared=[inputs,drive_mod])]
    fn btn_changed(ctx: btn_changed::Context) {
        let mut btns = 0u8;
        (ctx.shared.inputs, ctx.shared.drive_mod).lock(|inputs, drive_mod| {
            //clear what was pressed
            inputs.btn_left.clear_triggered();
            inputs.btn_right.clear_triggered();
            inputs.btn_up.clear_triggered();
            inputs.btn_down.clear_triggered();

            //debounce

            //Teensy 4 runs at 600MHz, so this is ~10ms
            cortex_m::asm::delay(6_000_000);

            let btn_left = inputs.btn_left.is_set();
            let btn_right = inputs.btn_right.is_set();
            let btn_up = inputs.btn_up.is_set();
            let btn_down = inputs.btn_down.is_set();

            //encode this into the u8 that drive_mod expects
            let mut btn = 0u8;
            if btn_left {
                btn |= 1 << 0;
            }
            if btn_right {
                btn |= 1 << 1;
            }
            if btn_up {
                btn |= 1 << 2;
            }
            if btn_down {
                btn |= 1 << 3;
            }

            let input_state = InputState { btn };
            btns = input_state.btn;

            drive_mod.update_inputs(input_state);
        });

        log::info!("Button Changed");
        log::info!("Buttons: {}", btns);
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn delay(_ctx: delay::Context) {
        Systick::delay(100u32.millis()).await;

        send_tests::spawn().ok();
    }

    #[task(priority = 1,shared=[inputs,delay,shared_spi,radio,display, drive_mod])]
    async fn send_tests(mut ctx: send_tests::Context) {
        log::info!("Tick!");
        let mut readingX: u16 = 0;
        let mut readingY: u16 = 0;
        (ctx.shared.inputs).lock(|inputs| {
            readingX = inputs.adc.read_blocking(&mut inputs.joy_lx);
            readingY = inputs.adc.read_blocking(&mut inputs.joy_ly);
        });

        let mut got_ack = false;

        (ctx.shared.shared_spi, ctx.shared.delay, ctx.shared.radio).lock(|spi, delay, radio| {
            let control_message = ControlMessageBuilder::new()
                .body_x(readingX as f32 / 1024.0)
                .body_y(readingY as f32 / 1024.0)
                .robot_id(1)
                .build();

            let mut packed_data = [0u8; CONTROL_MESSAGE_SIZE];
            control_message.pack(&mut packed_data).unwrap();

            let report = radio.write(&packed_data, spi, delay);
            radio.flush_tx(spi, delay);

            if report {
                log::info!("Received Acknowledgement From Transmission");
                got_ack = true;
            } else {
                log::info!("No Ack Received");
            }
        });

        (ctx.shared.display, ctx.shared.drive_mod).lock(|display, drive_mod| {
            display.clear();
            drive_mod.render(display);
            display.flush().unwrap();
        });

        delay::spawn().ok();
    }
}
