//!
//! Mock Manual is an example that has everything except for the FPGA moving.
//! 
//! Any motion controls will be logged to the console
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::convert::Infallible;
    use core::mem::MaybeUninit;

    use main::ROBOT_ID;

    use embedded_hal::spi::MODE_0;

    use nalgebra::Vector3;
    use teensy4_bsp as bsp;
    use bsp::board::{self, LPSPI_FREQUENCY};

    use teensy4_pins::t41::*;

    use teensy4_bsp::hal as hal;
    use hal::lpspi::{LpspiError, Lpspi, Pins};
    use hal::gpio::{Output, Input, Trigger, Port};
    use hal::gpt::{ClockSource, Gpt1, Gpt2};
    use hal::timer::Blocking;
    
    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::{Radio, config::*};

    use rtic_monotonics::systick::*;

    use packed_struct::prelude::*;

    use robojackets_robocup_rtp::{ControlMessage, CONTROL_MESSAGE_SIZE};
    use robojackets_robocup_rtp::{RobotStatusMessage, RobotStatusMessageBuilder, ROBOT_STATUS_SIZE};
    use robojackets_robocup_rtp::{BASE_STATION_ADDRESS, ROBOT_RADIO_ADDRESSES};

    use main::motion_control::MotionControl;

    // Constants
    const GPT_FREQUENCY: u32 = 1_000;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_MS: u32 = 1000;

    // Type Definitions
    // FPGA Spi
    type _SPI = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
    // Shared Spi
    type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
    type RadioCE = Output<P0>;
    type RadioCSN = Output<P2>;
    type RadioInterrupt = Input<P1>;
    // Delays
    type Delay1 = Blocking<Gpt1, GPT_FREQUENCY>;
    type Delay2 = Blocking<Gpt2, GPT_FREQUENCY>;
    // GPIO Ports
    type Gpio1 = Port<1>;

    #[local]
    struct Local {
        radio: Radio<RadioCE, RadioCSN, SharedSPI, Delay2, Infallible, LpspiError>,
        motion_controller: MotionControl,
    }

    #[shared]
    struct Shared {
        shared_spi: SharedSPI,
        delay1: Delay1,
        delay2: Delay2,
        rx_int: RadioInterrupt,
        gpio1: Gpio1,
        robot_status: RobotStatusMessage,
        control_message: Option<ControlMessage>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize the Heap
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        // Grab the board peripherals
        let board::Resources {
            pins,
            mut gpio1,
            mut gpio4,
            usb,
            mut gpt1,
            mut gpt2,
            ..
        } = board::t41(ctx.device);

        // Setup USB Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        // Gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        // Setup Rx Interrupt
        let rx_int = gpio1.input(pins.p1);
        gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));

        // Initialize Shared SPI
        let shared_spi_pins = Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = Lpspi::new(shared_spi_block, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 10_000_000u32);
            spi.set_mode(MODE_0);
        });

        // Init radio cs pin and ce pin
        let radio_cs = gpio4.output(pins.p2);
        let ce = gpio1.output(pins.p0);

        // Initialize radio
        let mut radio = Radio::new(ce, radio_cs);
        if radio.begin(&mut shared_spi, &mut delay2).is_err() {
            panic!("Unable to Initialize the Radio");
        }

        radio.set_pa_level(power_amplifier::PowerAmplifier::PALow, &mut shared_spi, &mut delay2);
        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, &mut shared_spi, &mut delay2);
        radio.open_writing_pipe(BASE_STATION_ADDRESS, &mut shared_spi, &mut delay2);
        radio.open_reading_pipe(1, ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize], &mut shared_spi, &mut delay2);
        radio.start_listening(&mut shared_spi, &mut delay2);

        // Set an initial robot status
        let initial_robot_status = RobotStatusMessageBuilder::new().build();

        rx_int.clear_triggered();

        motion_control_loop::spawn().ok();

        (
            Shared {
                shared_spi,
                delay1,
                delay2,
                rx_int,
                gpio1,
                robot_status: initial_robot_status,
                control_message: None,
            },
            Local {
                radio,
                motion_controller: MotionControl::new(),
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = GPIO1_COMBINED_0_15, shared = [rx_int, gpio1], priority = 2)]
    fn radio_interrupt(ctx: radio_interrupt::Context) {
        log::info!("Received Interrupt");
        if (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            if rx_int.is_triggered() {
                log::info!("Interrupt Triggered");
                rx_int.clear_triggered();
                gpio1.set_interrupt(&rx_int, None);
                return true;
            }
            false
        }) {
            receive_command::spawn().ok();
        }
    }

    #[task(shared = [rx_int, gpio1, shared_spi, delay2, control_message, robot_status], local = [radio], priority = 2)]
    async fn receive_command(ctx: receive_command::Context) {
        (
            ctx.shared.shared_spi,
            ctx.shared.delay2,
            ctx.shared.control_message,
            ctx.shared.robot_status,
        ).lock(|spi, delay, command, robot_status| {
            let mut read_buffer = [0u8; CONTROL_MESSAGE_SIZE];
            ctx.local.radio.read(&mut read_buffer, spi, delay);

            log::info!("Buffer: {:?}", read_buffer);

            let control_message = match ControlMessage::unpack_from_slice(&read_buffer[..]) {
                Ok(control_message) => control_message,
                Err(err) => {
                    log::info!("Error Unpacking Control Command: {:?}", err);
                    return;
                },
            };

            log::info!("Control Command Received: {:?}", control_message);

            *command = Some(control_message);

            ctx.local.radio.set_payload_size(ROBOT_STATUS_SIZE as u8, spi, delay);
            ctx.local.radio.stop_listening(spi, delay);

            let packed_data = match robot_status.pack() {
                Ok(bytes) => bytes,
                Err(err) => {
                    log::info!("Error Packing Robot Status: {:?}", err);
                    return;
                }
            };

            for _ in 0..5 {
                let report = ctx.local.radio.write(&packed_data, spi, delay);
                if report {
                    break;
                }
            }

            ctx.local.radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, spi, delay);
            ctx.local.radio.start_listening(spi, delay);
        });

        (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            rx_int.clear_triggered();
            gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));
        });
    }

    #[task(shared = [delay1, control_message], local = [motion_controller], priority = 1)]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        let body_velocities = ctx.shared.control_message.lock(|control_message| {
            match control_message {
                Some(control_message) => control_message.get_velocity(),
                None => Vector3::new(0.0, 0.0, 0.0),
            }
        });

        let wheel_velocities = ctx.local.motion_controller.body_to_wheels(body_velocities);

        log::info!("Wheel Velocities: {:?}", wheel_velocities);

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_MS.millis()).await;

        motion_control_loop::spawn().ok();
    }
}