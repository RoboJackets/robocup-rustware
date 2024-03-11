<<<<<<< HEAD
//! The manual control example can be run to manually control the robots movements
//! via software's make run-manual.
=======
//!
//! Manual Control is an example that should be pretty close to the fully-working program.  However,
//! it is currently considered a work in progress so it is still considered a test.
//! 
>>>>>>> main

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

<<<<<<< HEAD
extern crate alloc;

=======
>>>>>>> main
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

<<<<<<< HEAD
#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2, GPIO1_COMBINED_16_31])]
mod app {
    use super::*;

    use main::{BASE_STATION_ADDRESS, ROBOT_RADIO_ADDRESSES, ROBOT_ID};

    use embedded_hal::spi::MODE_0;
    use embedded_hal::digital::v2::InputPin;
    use teensy4_bsp::hal::lpspi::LpspiError;
    use teensy4_pins::t41::*;

    use core::convert::Infallible;
    use core::mem::MaybeUninit;

    use imxrt_iomuxc::prelude::*;

    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::board::LPSPI_FREQUENCY;

    use bsp::hal as hal;
    use hal::gpio::{Output, Input, Trigger, Port};
    use hal::gpt::{ClockSource, Gpt1, Gpt2};
    use hal::timer::Blocking;
    use hal::lpspi::Lpspi;

=======
#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::convert::Infallible;
    use core::mem::MaybeUninit;

    use fpga::DutyCycle;
    use imxrt_iomuxc::prelude::*;

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
    
>>>>>>> main
    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::{Radio, config::*};

    use rtic_monotonics::systick::*;

    use packed_struct::prelude::*;

<<<<<<< HEAD
    use fpga_rs::{error::FpgaError, duty_cycle::DutyCycle, FPGA};
    use fpga_rs::{FPGA_SPI_MODE, FPGA_SPI_FREQUENCY};

    // Communication
    use robojackets_robocup_rtp::Team;
    use robojackets_robocup_rtp::control_message::ControlMessage;
    use robojackets_robocup_rtp::robot_status_message::RobotStatusMessage;

    use main::motion_control::MotionControl;

    /// Constants
=======
    use robojackets_robocup_rtp::{ControlMessage, CONTROL_MESSAGE_SIZE};
    use robojackets_robocup_rtp::{RobotStatusMessage, RobotStatusMessageBuilder, ROBOT_STATUS_SIZE};
    use robojackets_robocup_rtp::{BASE_STATION_ADDRESS, ROBOT_RADIO_ADDRESSES};

    use main::motion_control::MotionControl;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    // Constants
>>>>>>> main
    const GPT_FREQUENCY: u32 = 1_000;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

<<<<<<< HEAD
    /// Type Definitions
    // SPI 1
    type SPI = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
    type Fpga = FPGA<SPI, Output<P9>, P15, Output<P16>, P14>;
    // SPI 2
=======
    const MOTION_CONTROL_DELAY_US: u32 = 200;

    // Type Definitions
    // FPGA Spi
    type SPI = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
    type Fpga = FPGA<SPI, Output<P9>, P29, Output<P28>, P30, Delay1>;
    // Shared Spi
>>>>>>> main
    type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
    type RadioCE = Output<P0>;
    type RadioCSN = Output<P2>;
    type RadioInterrupt = Input<P1>;
<<<<<<< HEAD
    // DELAYS
=======
    // Delays
>>>>>>> main
    type Delay1 = Blocking<Gpt1, GPT_FREQUENCY>;
    type Delay2 = Blocking<Gpt2, GPT_FREQUENCY>;
    // GPIO Ports
    type Gpio1 = Port<1>;

    #[local]
    struct Local {
<<<<<<< HEAD
        fpga: Fpga,
        radio: Radio<RadioCE, RadioCSN, SharedSPI, Delay2, Infallible, LpspiError>,
        motion_controller: MotionControl,
=======
        radio: Radio<RadioCE, RadioCSN, SharedSPI, Delay2, Infallible, LpspiError>,
        motion_controller: MotionControl,
        fpga: Fpga,
>>>>>>> main
    }

    #[shared]
    struct Shared {
        shared_spi: SharedSPI,
<<<<<<< HEAD
        delay1: Delay1,
=======
>>>>>>> main
        delay2: Delay2,
        rx_int: RadioInterrupt,
        gpio1: Gpio1,
        robot_status: RobotStatusMessage,
<<<<<<< HEAD
        control_command: Option<ControlMessage>,
=======
        control_message: Option<ControlMessage>,
>>>>>>> main
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
<<<<<<< HEAD
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

=======
        // Initialize the Heap
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        // Grab the board peripherals
>>>>>>> main
        let board::Resources {
            mut pins,
            mut gpio1,
            mut gpio2,
<<<<<<< HEAD
=======
            mut gpio3,
>>>>>>> main
            mut gpio4,
            usb,
            lpspi4,
            mut gpt1,
            mut gpt2,
            ..
        } = board::t41(ctx.device);

<<<<<<< HEAD
        // usb logging setup
        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // gpt 1 as blocking delay
=======
        // Setup USB Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 1 as blocking delay
>>>>>>> main
        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

<<<<<<< HEAD
        // gpt 2 as blocking delay
=======
        // Gpt 2 as blocking delay
>>>>>>> main
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

<<<<<<< HEAD
        // RX DONE Interrupt setup
        let rx_int = gpio1.input(pins.p1);
        gpio4.set_interrupt(&rx_int, Some(Trigger::RisingEdge));

        // initialize spi 1
        let mut kicker_spi = board::lpspi(
=======
        // Setup Rx Interrupt
        let rx_int = gpio1.input(pins.p1);
        gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));

        // Initialize Fpga SPI
        let mut spi = board::lpspi(
>>>>>>> main
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            FPGA_SPI_FREQUENCY,
        );
<<<<<<< HEAD

        // Initialize Shared SPI
        let shared_spi_pins = hal::lpspi::Pins {
=======
        spi.disabled(|spi| {
            spi.set_mode(FPGA_SPI_MODE);
        });

        // Initialize pins for the FPGA
        let cs = gpio2.output(pins.p9);
        let init_b = gpio4.input(pins.p29);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        let done = gpio3.input(pins.p30);

        // Initialize the FPGA
        let fpga = match FPGA::new(spi, cs, init_b, prog_b, done, delay1) {
            Ok(fpga) => fpga,
            Err(_) => panic!("Unable to initialize the FPGA"),
        };

        // Initialize Shared SPI
        let shared_spi_pins = Pins {
>>>>>>> main
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
<<<<<<< HEAD
        let mut shared_spi = hal::lpspi::Lpspi::new(shared_spi_block, shared_spi_pins);
=======
        let mut shared_spi = Lpspi::new(shared_spi_block, shared_spi_pins);
>>>>>>> main

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 10_000_000u32);
            spi.set_mode(MODE_0);
        });

<<<<<<< HEAD
        // init fake CS pin and required reset pin
        let fake_cs = gpio4.output(pins.p2);
        let ce = gpio1.output(pins.p0);

        // initialize radio
        let mut radio = Radio::new(ce, fake_cs);
        if radio.begin(&mut shared_spi, &mut delay2).is_err() {
            panic!("Unable to Initialize the Radio");
        }
        radio.set_pa_level(power_amplifier::PowerAmplifier::PALow, &mut shared_spi, &mut delay2);
        radio.set_payload_size(4, &mut shared_spi, &mut delay2);
=======
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
>>>>>>> main
        radio.open_writing_pipe(BASE_STATION_ADDRESS, &mut shared_spi, &mut delay2);
        radio.open_reading_pipe(1, ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize], &mut shared_spi, &mut delay2);
        radio.start_listening(&mut shared_spi, &mut delay2);

<<<<<<< HEAD
        // Configure SPI
        kicker_spi.disabled(|kicker_spi| {
            kicker_spi.set_mode(FPGA_SPI_MODE);
        });

        // Initialize FPGA
        let cs = gpio2.output(pins.p9);
        let init_b = gpio1.input(pins.p15);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p16, config);
        let prog_b = gpio1.output(pins.p16);
        let done = gpio1.input(pins.p14);

        let fpga = match FPGA::new(kicker_spi, cs, init_b, prog_b, done) {
            Ok(instance) => instance,
            Err(_) => panic!("Couldn't initialize instance of FPGA"),
        };

        let initial_robot_status = RobotStatusMessage::new(Team::Blue, 0u8, false, false, true, 0u8, 0u8, false, [0u16; 18]);
=======
        // Set an initial robot status
        let initial_robot_status = RobotStatusMessageBuilder::new().build();

        rx_int.clear_triggered();

        motion_control_loop::spawn().ok();
>>>>>>> main

        (
            Shared {
                shared_spi,
<<<<<<< HEAD
                delay1,
=======
>>>>>>> main
                delay2,
                rx_int,
                gpio1,
                robot_status: initial_robot_status,
<<<<<<< HEAD
                control_command: None,
            },
            Local {
                fpga,
                radio,
                motion_controller: MotionControl::new(),
            },
=======
                control_message: None,
            },
            Local {
                radio,
                motion_controller: MotionControl::new(),
                fpga,
            }
>>>>>>> main
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

<<<<<<< HEAD
    #[task(binds = GPIO1_COMBINED_0_15, shared = [rx_int, gpio1], priority = 1)]
    fn gpio_interrupt(mut ctx: gpio_interrupt::Context) {
=======
    #[task(binds = GPIO1_COMBINED_0_15, shared = [rx_int, gpio1], priority = 2)]
    fn radio_interrupt(ctx: radio_interrupt::Context) {
>>>>>>> main
        if (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            if rx_int.is_triggered() {
                rx_int.clear_triggered();
                gpio1.set_interrupt(&rx_int, None);
                return true;
            }
            false
        }) {
<<<<<<< HEAD
            radio_handler::spawn().ok();
        }
    }

    #[task(shared = [rx_int, gpio1, shared_spi, delay2, control_command, robot_status], local = [radio], priority = 2)]
    async fn radio_handler(ctx: radio_handler::Context) {
        (
            ctx.shared.shared_spi,
            ctx.shared.delay2,
            ctx.shared.control_command,
            ctx.shared.robot_status,
        ).lock(|spi, delay, command, robot_status| {
            let mut read_buffer = [0u8; 10];
            ctx.local.radio.read(&mut read_buffer, spi, delay);

            // Convert the slice into a ControlMessage
            // TODO: Can Panic so we should check this
            let control_command = ControlMessage::unpack_from_slice(&read_buffer[..]).unwrap();
            *command = Some(control_command);

            ctx.local.radio.stop_listening(spi, delay);

            // TODO: Fix encoder deltas to not be 18 u16s long
            let packed_data = match robot_status.pack() {
                Ok(bytes) => bytes,
                Err(err) => panic!("Could not pack robot status message: {:?}", err),
            };

            // Retry Transmission 5 Times
=======
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

            let control_message = match ControlMessage::unpack_from_slice(&read_buffer[..]) {
                Ok(control_message) => control_message,
                Err(err) => {
                    log::info!("Error Unpacking Control Command: {:?}", err);
                    return;
                },
            };

            // log::info!("Control Command Received: {:?}", control_message);

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

>>>>>>> main
            for _ in 0..5 {
                let report = ctx.local.radio.write(&packed_data, spi, delay);
                if report {
                    break;
                }
            }

<<<<<<< HEAD
=======
            ctx.local.radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, spi, delay);
>>>>>>> main
            ctx.local.radio.start_listening(spi, delay);
        });

        (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            rx_int.clear_triggered();
<<<<<<< HEAD
            gpio1.set_interrupt(&rx_int, Some(Trigger::RisingEdge));
        });
    }

    #[task(shared = [delay1], local=[fpga, motion_controller, initialized: bool = false], priority = 1)]
    async fn fpga(mut ctx: fpga::Context) {
        if !*ctx.local.initialized {
            ctx.shared.delay1.lock(|delay| {
                match ctx.local.fpga.configure(delay) {
                    Ok(_) => log::info!("Fpga Configured"),
                    Err(_) => panic!("Unable to Configure the FPGA"),
                }
            });
    
            if ctx.local.fpga.motors_en(true).is_err() {
                panic!("Unable to Start the Fpga");
            }

            *ctx.local.initialized = true;
        }

        // TODO: Motion Controlled Move
=======
            gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));
        });
    }

    #[task(shared = [control_message], local = [fpga, motion_controller, initialized: bool = false], priority = 1)]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            match ctx.local.fpga.configure() {
                Ok(_) => log::info!("Fpga Configured"),
                Err(_) => panic!("Unable to Configure Fpga"),
            }

            match ctx.local.fpga.motors_en(true) {
                Ok(status) => log::info!("Enabled motors fpga: {:010b}", status),
                Err(_) => panic!("Unable to Enable Motors"),
            }
            *ctx.local.initialized = true;
        }

        let body_velocities = ctx.shared.control_message.lock(|control_message| {
            match control_message {
                Some(control_message) => control_message.get_velocity(),
                None => Vector3::new(0.0, 0.0, 0.0),
            }
        });

        let wheel_velocities = ctx.local.motion_controller.body_to_wheels(body_velocities);

        let mut duty_cycles = [
            DutyCycle::from(wheel_velocities[0] as i16),
            DutyCycle::from(wheel_velocities[1] as i16),
            DutyCycle::from(wheel_velocities[2] as i16),
            DutyCycle::from(wheel_velocities[3] as i16),
            DutyCycle::from(256 as i16),
        ];

        log::info!("Duty Cycles: {:?}", duty_cycles);

        let status = match ctx.local.fpga.set_duty_cycles(&mut duty_cycles) {
            Ok(status) => status,
            Err(_) => {
                log::info!("Unable to set duty cycles");
                return;
            }
        };

        log::info!("Wheel Velocities: {:?} --- status: {:08b}", wheel_velocities, status);

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_loop::spawn().ok();
>>>>>>> main
    }
}