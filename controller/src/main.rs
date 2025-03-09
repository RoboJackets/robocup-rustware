//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use embedded_alloc::Heap;

mod module_drive;
mod module_types;
mod module_util;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2,GPT1])]
mod app {

    use crate::module_types::{InputStateUpdate, ModuleArr};

    use super::*;

    use alloc::boxed::Box;
    use imxrt_hal::gpio::Input;
    use module_drive::DriveMod;
    use module_types::{ControllerModule, RadioState};

    use core::mem::MaybeUninit;

    use embedded_hal::spi::MODE_0;
    use rtic_nrf24l01::config::power_amplifier::PowerAmplifier;
    use rtic_nrf24l01::Radio;

    use bsp::board::{self, LPSPI_FREQUENCY};
    use teensy4_bsp as bsp;

    use teensy4_pins::t41::*;

    use rtic_monotonics::{systick::*, Monotonic};

    use bsp::hal;
    use bsp::hal::adc::{Adc, AnalogInput};
    use hal::gpio::Trigger;
    use hal::timer::Blocking;

    use bsp::ral;
    use ral::lpspi::LPSPI3;

    use robojackets_robocup_rtp::BASE_STATION_ADDRESSES;
    use robojackets_robocup_rtp::{CONTROL_MESSAGE_SIZE, ROBOT_RADIO_ADDRESSES};

    use robojackets_robocup_control::{
        Delay2, RFRadio, SharedSPI, CHANNEL, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
    };

    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        dispatcher_tick: u32,
        last_tick_ms: u32,
    }

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
        radio_settings: RadioState,
        modules: ModuleArr,
        active_module: usize,
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

        let modules: [Box<dyn ControllerModule>; 1] = [Box::new(DriveMod::new())];

        let active_module = 0;

        //init devices
        init_devices::spawn().ok();

        let dispatcher_tick = 0u32;
        let last_tick_ms = Systick::now().ticks();

        //set radio settings
        let radio_settings = RadioState {
            team: 0,
            robot_id: 0,
            conn_acks_results: [false; 100],
            conn_acks_attempts: 0,
        };

        (
            Shared {
                shared_spi,
                delay,
                inputs,
                radio,
                display,
                modules,
                active_module: active_module,
                radio_settings,
            },
            Local {
                dispatcher_tick,
                last_tick_ms,
            },
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
                radio.open_writing_pipe(ROBOT_RADIO_ADDRESSES[0][0], shared_spi, delay);
                radio.open_reading_pipe(1, BASE_STATION_ADDRESSES[0], shared_spi, delay);
                radio.start_listening(shared_spi, delay);
            },
        );

        //start the main dispatcher
        dispatcher::spawn().ok();
    }

    #[task(
        binds = GPIO4_COMBINED_0_15,
        priority = 1, shared=[inputs,modules,active_module])]
    fn btn_changed_int(ctx: btn_changed_int::Context) {
        log::info!("Button Interrupt");
        (
            ctx.shared.inputs,
            ctx.shared.modules,
            ctx.shared.active_module,
        )
            .lock(|inputs, module, active_module| {
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

                let state_update = InputStateUpdate {
                    btn_left: Some(btn_left),
                    btn_right: Some(btn_right),
                    btn_up: Some(btn_up),
                    btn_down: Some(btn_down),
                    joy_lx: None,
                    joy_ly: None,
                    joy_rx: None,
                    joy_ry: None,
                };

                module[*active_module].update_inputs(state_update);
            });
    }

    #[task(priority = 1, shared=[inputs,modules,active_module])]
    async fn poll_input_status(ctx: poll_input_status::Context) {
        (
            ctx.shared.inputs,
            ctx.shared.modules,
            ctx.shared.active_module,
        )
            .lock(|inputs, modules, active_module| {
                //debounce

                //Teensy 4 runs at 600MHz, so this is ~10ms
                cortex_m::asm::delay(6_000_000);

                let btn_left = inputs.btn_left.is_set();
                let btn_right = inputs.btn_right.is_set();
                let btn_up = inputs.btn_up.is_set();
                let btn_down = inputs.btn_down.is_set();

                //update joysticks
                let joy_lx = inputs.adc.read_blocking(&mut inputs.joy_lx);
                let joy_ly = inputs.adc.read_blocking(&mut inputs.joy_ly);
                let joy_rx = inputs.adc.read_blocking(&mut inputs.joy_rx);
                let joy_ry = inputs.adc.read_blocking(&mut inputs.joy_ry);

                let state_update = InputStateUpdate {
                    btn_left: Some(btn_left),
                    btn_right: Some(btn_right),
                    btn_up: Some(btn_up),
                    btn_down: Some(btn_down),
                    joy_lx: Some(joy_lx),
                    joy_ly: Some(joy_ly),
                    joy_rx: Some(joy_rx),
                    joy_ry: Some(joy_ry),
                };

                modules[*active_module].update_inputs(state_update);
            });
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1,local=[last_tick_ms])]
    async fn delay_main(ctx: delay_main::Context) {
        let delay_ms: u32 = 50;

        let now = Systick::now().ticks();
        let diff: i32 = *ctx.local.last_tick_ms as i32 + delay_ms as i32 - now as i32;
        *ctx.local.last_tick_ms = now;

        let await_time: u32 = if diff > 0 { diff as u32 } else { 0 };

        if await_time > 0 {
            Systick::delay(await_time.millis()).await;
        }
        dispatcher::spawn().ok();
    }

    #[task(priority = 1,local=[dispatcher_tick],shared=[modules,active_module])]
    async fn dispatcher(ctx: dispatcher::Context) {
        *ctx.local.dispatcher_tick += 1;

        (ctx.shared.modules, ctx.shared.active_module).lock(|modules, active_module| {
            let next_module = modules[*active_module].next_module();
            if next_module != module_types::NextModule::None {
                *active_module = next_module as usize;
            }
        });

        //200 ms update
        if *ctx.local.dispatcher_tick % 4 == 0 {
            update_display::spawn().ok();
            poll_input_status::spawn().ok();
        }

        //100 ms update
        if *ctx.local.dispatcher_tick % 2 == 0 {
            update_radio::spawn().ok();
        }

        delay_main::spawn().ok();
    }

    #[task(priority = 1,shared=[display, modules,active_module])]
    async fn update_display(ctx: update_display::Context) {
        (
            ctx.shared.modules,
            ctx.shared.display,
            ctx.shared.active_module,
        )
            .lock(|modules, display, active_module| {
                display.clear();
                modules[*active_module].update_display(display);
                display.flush().unwrap();
            });
    }

    #[task(priority = 1,shared=[modules, radio, shared_spi,delay,radio_settings,active_module])]
    async fn update_radio(ctx: update_radio::Context) {
        log::info!("Time (ms): {}", Systick::now().ticks());

        (
            ctx.shared.shared_spi,
            ctx.shared.delay,
            ctx.shared.radio,
            ctx.shared.modules,
            ctx.shared.radio_settings,
            ctx.shared.active_module,
        )
            .lock(
                |spi, delay, radio, modules, radio_settings, active_module| {
                    modules[*active_module].update_settings(radio_settings);
                    modules[*active_module].radio_update(radio, spi, delay);
                },
            );
    }
}
