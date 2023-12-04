//! The manual control example can be run to manually control the robots movements
//! via software's make run-manual.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use super::*;

    use teensy4_pins::t41::*;

    use core::mem::MaybeUninit;

    use imxrt_iomuxc::prelude::*;

    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::board::LPSPI_FREQUENCY;

    use bsp::hal as hal;
    use hal::gpio::{self, Output, Trigger, Port};
    use hal::gpt::{ClockSource, Gpt1, Gpt2};
    use hal::timer::Blocking;
    use hal::lpspi::Lpspi;

    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_monotonics::systick::*;

    use sx127::RadioMode;

    use packed_struct::prelude::*;
    use packed_struct::types::bits::ByteArray;

    use fpga_rs::{error::FpgaError, duty_cycle::DutyCycle, FPGA};
    use fpga_rs::{FPGA_SPI_MODE, FPGA_SPI_FREQUENCY};

    // Communication
    use robojackets_robocup_rtp::{Team, MessageType, RTPHeader};
    use robojackets_robocup_rtp::control_command::{ControlCommand, CommandTypes};
    use robojackets_robocup_rtp::control_message::ControlMessage;
    use robojackets_robocup_rtp::robot_status_message::RobotStatusMessage;

    use main::motion_control::MotionControl;

    /// Constants
    const DELAY_MS: u32 = 10_000;
    const FREQUENCY: i64 = 915;
    const GPT1_FREQUENCY: u32 = 1_000;
    const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[cfg(feature = "robot-0")]
    const ROBOT_ID: u8 = 0;
    #[cfg(feature = "robot-1")]
    const ROBOT_ID: u8 = 1;
    #[cfg(feature = "robot-2")]
    const ROBOT_ID: u8 = 2;
    #[cfg(feature = "robot-3")]
    const ROBOT_ID: u8 = 3;
    #[cfg(feature = "robot-4")]
    const ROBOT_ID: u8 = 4;
    #[cfg(feature = "robot-5")]
    const ROBOT_ID: u8 = 5;


    /// Type Definitions
    type Led = Output<P7>;
    type Delay = Blocking<Gpt1, GPT1_FREQUENCY>;
    type BlockingDelay = Blocking<Gpt2, GPT1_FREQUENCY>;
    type RadioInterrupt = gpio::Input<P29>;
    type Radio = sx127::LoRa<Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>, gpio::Output<P8>, gpio::Output<P28>, Delay>;
    type Gpio1 = Port<1>;
    type Fpga = FPGA<board::Lpspi4, gpio::Output<P9>, P15, gpio::Output<P16>, P14>;

    #[local]
    struct Local {
        led: Led,
        fpga: Fpga,
        motion_controller: MotionControl,
    }

    #[shared]
    struct Shared {
        radio: Radio,
        blocking_delay: Option<BlockingDelay>,
        rx_int: RadioInterrupt,
        gpio1: Gpio1,
        awake: bool,
        current_control_message: Option<ControlMessage>,
        status: RobotStatusMessage,
        last_received_count: u32,
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
            lpspi4,
            mut gpt1,
            mut gpt2,
            ..
        } = board::t41(ctx.device);

        // usb logging setup
        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // gpt 1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT1_DIVIDER);
        gpt1.set_clock_source(GPT1_CLOCK_SOURCE);
        let delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

        gpt2.disable();
        gpt2.set_divider(GPT1_DIVIDER);
        gpt2.set_clock_source(GPT1_CLOCK_SOURCE);
        let blocking_delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt2);

        // RX DONE Interrupt setup
        let rx_int = gpio4.input(pins.p29);
        gpio4.set_interrupt(&rx_int, Some(Trigger::RisingEdge));

        // initialize spi
        let shared_spi_pins = hal::lpspi::Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = hal::lpspi::Lpspi::new(shared_spi_block, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 1_000_000);
        });

        // init fake CS pin (TEMPORARY) and required reset pin
        let fake_cs = gpio2.output(pins.p8);
        let reset = gpio3.output(pins.p28);

        // initialize radio
        let radio = match sx127::LoRa::new(shared_spi, fake_cs, reset, FREQUENCY, delay) {
            Ok(radio) => radio,
            Err(err) => match err {
                sx127::Error::VersionMismatch(version) => panic!("Version Mismatch Error {:?}", version),
                sx127::Error::CS(_) => panic!("Chip select issue"),
                sx127::Error::Reset(_) => panic!("Rest Issue"),
                sx127::Error::SPI(_) => panic!("SPI problem"),
                sx127::Error::Transmitting => panic!("Error during spi transmission"),
                sx127::Error::Uninformative => panic!("Uninformative error RIP"),
            }
        };

        let mut kicker_spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            FPGA_SPI_FREQUENCY,
        );

        // Configure SPI
        kicker_spi.disabled(|kicker_spi| {
            kicker_spi.set_mode(FPGA_SPI_MODE);
        });

        // Initialize Pins
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

        let led = gpio2.output(pins.p7);

        blink_led::spawn().ok();

        let initial_robot_status = RobotStatusMessage::new(Team::Blue, 0u8, false, false, true, 0u8, 0u8, false, [0u16; 18]);

        init_radio::spawn().ok();
        motor_control::spawn().ok();

        (
            Shared {
                radio,
                blocking_delay: Some(blocking_delay),
                rx_int,
                gpio1,
                awake: false,
                current_control_message: None,
                status: initial_robot_status,
                last_received_count: 0u32,
            },
            Local {
                led,
                fpga,
                motion_controller: MotionControl::new(),
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [led], priority = 1)]
    async fn blink_led(ctx: blink_led::Context) {
        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.toggle();

        Systick::delay(1_000u32.millis()).await;

        ctx.local.led.toggle();
    }

    #[task(shared = [radio], priority = 1)]
    async fn init_radio(mut ctx: init_radio::Context) {
        Systick::delay(DELAY_MS.millis()).await;

        ctx.shared.radio.lock(|radio| {
            log::info!("Lora Version: {:#04x}", radio.get_radio_version().unwrap());

            match radio.set_tx_power(17, 1) {
                Ok(_) => log::info!("Successfully set TX power"),
                Err(_) => panic!("Error setting tx power"),
            };

            log::info!("Successfully Initialized Radio");

            match radio.set_mode(RadioMode::RxContinuous) {
                Ok(_) => log::info!("Radio Listening..."),
                Err(_) => panic!("Couldn't set radio to listening"),
            }
        });
    }

    #[task(
        binds = GPIO4_COMBINED_16_31,
        shared = [radio, blocking_delay, rx_int, gpio1, awake, current_control_message, last_received_count]
    )]
    fn receive_data(mut ctx: receive_data::Context) {
        log::info!("Interrupt");
        (ctx.shared.rx_int, ctx.shared.radio, ctx.shared.blocking_delay, ctx.shared.gpio1).lock(|rx_int, radio, delay, gpio1| {
            if rx_int.is_triggered() {
                // Disable Radio Interrupt
                gpio1.set_interrupt(rx_int, None);
                rx_int.clear_triggered();

                match radio.read_packet() {
                    Ok(buffer) => {
                        match MessageType::from(buffer[0]) {
                            MessageType::ControlCommand => {
                                let control_command_length = <ControlCommand as PackedStruct>::ByteArray::len();
                                let control_command = match ControlCommand::unpack_from_slice(&buffer[1..(1+control_command_length)]) {
                                    Ok(control_command) => control_command,
                                    Err(err) => panic!("Error Occurred Extracting Control Command: {:?}", err),
                                };

                                log::info!("Received {:?}", control_command);

                                if control_command.robot_id == ROBOT_ID.into() {
                                    match control_command.command_type() {
                                        CommandTypes::WakeUp => ctx.shared.awake.lock(|awake| { *awake = true; }),
                                        CommandTypes::PowerDown => ctx.shared.awake.lock(|awake| { *awake = false; }),
                                        CommandTypes::Unknown => panic!("Unknown Control Command Received"),
                                    }

                                    // Spawn software task to send robot status and reenable interrupts
                                    send_robot_status::spawn().ok();
                                }
                            },
                            MessageType::ControlMessage => {
                                let control_message_length = <ControlMessage as PackedStruct>::ByteArray::len();
                                let control_message = match ControlMessage::unpack_from_slice(&buffer[1..(1+control_message_length)]) {
                                    Ok(control_message) => control_message,
                                    Err(err) => panic!("Error Occurred Extracting Control Message: {:?}", err),
                                };

                                if control_message.robot_id == ROBOT_ID.into() {
                                    ctx.shared.current_control_message.lock(|current_control_message| { *current_control_message = Some(control_message); });

                                    // Keep Track of Last Received Communication Timestamp
                                    ctx.shared.last_received_count.lock(|last_count| { 
                                        if let Some(taken_delay) = delay.take() {
                                            let gpt = taken_delay.release();
                                            *last_count = gpt.count();
                                            *delay = Some(Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt));
                                        }
                                    });

                                    // Spawn software task to send robot status and reenable interrupts
                                    send_robot_status::spawn().ok();
                                }
                            },
                            // Status Messages will be picked up by robots so this is to allow them to be dropped
                            _ => {
                                rx_int.clear_triggered();
                                gpio1.set_interrupt(rx_int, Some(Trigger::RisingEdge));
                            },
                        }
                    },
                    Err(_) => panic!("Error while reading data"),
                }
            }
        });
    }

    #[task(
        shared = [radio, blocking_delay, rx_int, gpio1, status],
        priority = 1,
    )]
    async fn send_robot_status(ctx: send_robot_status::Context) {
        (ctx.shared.status, ctx.shared.radio).lock(|status, radio| {
            let packed_data = match status.pack() {
                Ok(bytes) => bytes,
                Err(err) => panic!("Unable to Pack Status: {:?}", err),
            };

            let packed_data = packed_data.as_bytes_slice();

            let mut packet = [0u8; 255];
            packet[0] = RobotStatusMessage::get_header() as u8;
            packet[1..1+packed_data.len()].copy_from_slice(packed_data);

            match radio.transmit_payload_busy(packet, packed_data.len()) {
                Ok(_) => log::info!("Successfully Sent Status"),
                Err(_) => panic!("Unable to Send Response"),
            }

            if radio.set_mode(RadioMode::RxContinuous).is_err() {
                panic!("Unable to Reset Radio to Read");
            }
        });

        // Re-enable interrupts
        (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            rx_int.clear_triggered();
            gpio1.set_interrupt(rx_int, Some(Trigger::RisingEdge));
        });
    }

    #[task(
        shared = [status, current_control_message, blocking_delay],
        local = [fpga, motion_controller],
        priority = 1,
    )]
    async fn motor_control(mut ctx: motor_control::Context) {
        ctx.shared.blocking_delay.lock(|delay| {
            if let Some(mut taken_delay) = delay.take() {
                match ctx.local.fpga.configure(&mut taken_delay) {
                    Ok(_) => log::info!("Configured FPGA"),
                    Err(e) => match e {
                        FpgaError::SPI(spi_e) => panic!("SPI error with info: {:?}", spi_e),
                        FpgaError::CSPin(cs_e) => panic!("CS pin error: {:?}", cs_e), 
                        FpgaError::InitPin(init_e) => panic!("Init pin error: {:?}", init_e),
                        FpgaError::ProgPin(prog_e) => panic!("Prog pin error: {:?}", prog_e),
                        FpgaError::DonePin(done_e) => panic!("Done pin error: {:?}", done_e),
                        FpgaError::FPGATimeout(code) => panic!("Fpga timed out?? code: {:x}", code),
                    }
                }
                *delay = Some(taken_delay);
            } else {
                panic!("Unable to Initialize FPGA");
            }
        });

        Systick::delay(10u32.millis()).await;

        // enable motors
        match ctx.local.fpga.motors_en(true) {
            Ok(status) => log::info!("Enable Motors FPGA Status: {:b}", status),
            Err(e) => panic!("Error Enabling Motor... {:?}", e),
        }

        Systick::delay(10u32.millis()).await;

        ctx.shared.status.lock(|status| { status.fpga_status = true; });

        loop {
            // Get the Current Control Message
            let control_message = ctx.shared.current_control_message.lock(|message| { *message });
            if control_message.is_none() {
                log::info!("No Control Message");
                ctx.shared.blocking_delay.lock(|delay| {
                    if let Some(mut taken_delay) = delay.take() {
                        if ctx.local.fpga.watchdog_reset(&mut taken_delay).is_err() {
                            log::info!("Unable to Reset Watchdog");
                        }
                        *delay = Some(taken_delay);
                    }
                });
                Systick::delay(10u32.millis()).await;
                continue;
            }
            let control_message = control_message.unwrap();
            let target_global_velocity: [f32; 3] = [control_message.body_x.to_be() as f32, control_message.body_y.to_be() as f32, control_message.body_w.to_be() as f32];

            let target_velocity = ctx.local.motion_controller.body_to_wheel(&target_global_velocity);

            log::info!("Target Velocity: {:?}", target_velocity);
            log::info!("Control Message: {:?}", control_message);
            log::info!("Velocity: [{}, {}, {}, {}]", target_velocity[0] as i16, target_velocity[1] as i16, target_velocity[2] as i16, target_velocity[3] as i16);

            // Convert control_message x, y, and w to motion commands
            let mut duty_cycles = [
                DutyCycle::from(target_velocity[0] as i16),
                DutyCycle::from(target_velocity[1] as i16),
                DutyCycle::from(target_velocity[2] as i16),
                DutyCycle::from(target_velocity[3] as i16),
                DutyCycle::from(0i16),
            ];

            log::info!("Velocities: {:?}", duty_cycles);
            // Write Duty Cycles
            match ctx.local.fpga.set_duty_cycles(&mut duty_cycles) {
                Ok(status) => log::info!("Wrote Duty Cycles to FPGA. Status: {:b}", status),
                Err(e) => panic!("error writing duty cycles... {:?}", e),
            }

            Systick::delay(10u32.millis()).await;
        }
    }
}