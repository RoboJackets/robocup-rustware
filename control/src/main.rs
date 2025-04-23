//!
//! Manual Control is an example that should be pretty close to the fully-working program.  However,
//! it is currently considered a work in progress so it is still considered a test.
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

    use core::mem::MaybeUninit;

    use imxrt_hal::gpt::Gpt;
    use imxrt_hal::pit::Pit;
    use imxrt_iomuxc::prelude::*;

    use embedded_hal::{blocking::delay::DelayMs, spi::MODE_0};

    use bsp::board::PERCLK_FREQUENCY;
    use bsp::board::{self, LPSPI_FREQUENCY};
    use nalgebra::{Vector3, Vector4};
    use teensy4_bsp as bsp;

    use hal::gpio::Trigger;
    use hal::lpspi::{Lpspi, Pins};
    use hal::timer::Blocking;
    use teensy4_bsp::hal;

    use bsp::ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::Radio;

    use rtic_monotonics::systick::*;

    use ncomm_utils::packing::Packable;

    use shared_bus;

    // Includes for display module
    use embedded_graphics::prelude::*;
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
    use teensy4_bsp::board::Lpi2c1;
    use teensy4_pins::t41::{P18, P19};

    use robojackets_robocup_control::robot::{TEAM, TEAM_NUM};
    use robojackets_robocup_rtp::BASE_STATION_ADDRESSES;
    use robojackets_robocup_rtp::{
        control_message::Mode,
        control_test_message::{ControlTestMessage, CONTROL_TEST_MESSAGE_SIZE},
        imu_test_message::{ImuTestMessage, IMU_MESSAGE_SIZE},
        kicker_program_message::{KickerProgramMessage, KICKER_PROGRAM_MESSAGE},
        kicker_testing::{KickerTestingMessage, KICKER_TESTING_SIZE},
        radio_benchmarks::{
            RadioReceiveBenchmarkMessage, RadioSendBenchmarkMessage, RADIO_RECEIVE_BENCHMARK_SIZE,
            RADIO_SEND_BENCHMARK_SIZE,
        },
        ControlMessage, RobotStatusMessage, RobotStatusMessageBuilder, CONTROL_MESSAGE_SIZE,
        ROBOT_STATUS_SIZE,
    };

    use motion::MotionControl;

    use fpga::FPGA;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga_rs as fpga;

    use icm42605_driver::IMU;

    use robojackets_robocup_control::{
        spi::FakeSpi, Delay2, DisplayT, FPGAInitError, FPGAProgError, Fpga, Gpio1, Imu,
        ImuInitError, KickerCSn, KickerProg, KickerProgramError, KickerReset, KickerServicingError,
        PitDelay, RFRadio, RadioInitError, RadioInterrupt, SharedSPI, State,
        BASE_AMPLIFICATION_LEVEL, CHANNEL, GPT_1_DIVIDER, GPT_CLOCK_SOURCE, GPT_DIVIDER,
        GPT_FREQUENCY, RADIO_ADDRESS, ROBOT_ID,
    };

    use kicker_controller::{KickTrigger, KickType, Kicker, KickerCommand};
    use kicker_programmer::KickerProgrammer;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    /// The amount of time (in ms) between each motion control delay
    const MOTION_CONTROL_DELAY_MS: u32 = 1;
    /// The amount of time (in ms) between servicing the kicker
    const KICKER_SERVICE_DELAY_MS: u32 = 50;
    /// The number of motion control updates between servicing the kicker
    const KICKER_SERVICE_DELAY_TICKS: u32 = KICKER_SERVICE_DELAY_MS / MOTION_CONTROL_DELAY_MS;
    /// The amount of time the robot should continue moving without receiving a
    /// new message from the base station before it stops moving
    const DIE_TIME_US: u32 = 500_000;

    /// Helper method to disable radio interrupts and prepare to
    /// send messages of `message_size` over the radio
    pub fn disable_radio_interrupts(
        message_size: usize,
        rx_int: &mut RadioInterrupt,
        gpio1: &mut Gpio1,
        radio: &mut RFRadio,
        spi: &mut SharedSPI,
        radio_delay: &mut Delay2,
    ) {
        rx_int.clear_triggered();
        gpio1.set_interrupt(rx_int, None);
        radio.clear_interrupts(spi, radio_delay);
        radio.flush_rx(spi, radio_delay);
        radio.flush_tx(spi, radio_delay);
        radio.set_payload_size(message_size as u8, spi, radio_delay);
        radio.stop_listening(spi, radio_delay);
    }

    /// Helper method to re-enable radio interrupts and begin
    /// receiving incoming control messages
    pub fn enable_radio_interrupts(
        rx_int: &mut RadioInterrupt,
        gpio1: &mut Gpio1,
        radio: &mut RFRadio,
        spi: &mut SharedSPI,
        radio_delay: &mut Delay2,
    ) {
        rx_int.clear_triggered();
        gpio1.set_interrupt(rx_int, Some(Trigger::FallingEdge));
        radio.clear_interrupts(spi, radio_delay);
        radio.flush_rx(spi, radio_delay);
        radio.flush_tx(spi, radio_delay);
        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, spi, radio_delay);
        radio.start_listening(spi, radio_delay);
    }

    #[local]
    struct Local {
        motion_controller: MotionControl,
        last_encoders: Vector4<f32>,
    }

    #[shared]
    struct Shared {
        // Peripherals
        pit0: Pit<0>,
        gpt: Gpt<1>,
        shared_spi: SharedSPI,
        blocking_delay: Delay2,
        rx_int: RadioInterrupt,
        gpio1: Gpio1,
        pit_delay: PitDelay,
        fake_spi: FakeSpi,
        kicker_programmer: Option<KickerProg>,
        kicker_controller: Option<Kicker<KickerCSn, KickerReset>>,

        // Drivers
        fpga: Fpga,
        imu: Imu,
        radio: RFRadio,

        // Motion Control
        robot_status: RobotStatusMessage,
        control_message: Option<ControlMessage>,
        counter: u32,
        elapsed_time: u32,

        //Display
        display: DisplayT,

        // State
        state: State,

        // Errors
        imu_init_error: Option<ImuInitError>,
        fpga_prog_error: Option<FPGAProgError>,
        fpga_init_error: Option<FPGAInitError>,
        radio_init_error: Option<RadioInitError>,
        kicker_program_error: Option<KickerProgramError>,
        kicker_service_error: Option<KickerServicingError>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize the Heap
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        // Grab the board peripherals
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
            pit: (pit0, _pit1, pit2, pit3),
            ..
        } = board::t41(ctx.device);

        // Setup USB Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT_1_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        gpt1.enable();

        // Gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        // Setup Rx Interrupt
        let rx_int = gpio1.input(pins.p15);
        gpio1.set_interrupt(&rx_int, None);

        // Initialize Fpga SPI
        let mut spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            FPGA_SPI_FREQUENCY,
        );
        spi.disabled(|spi| spi.set_mode(FPGA_SPI_MODE));

        // Initialize IMU

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
        let i2c_bus: &'static _ = shared_bus::new_cortexm!(
            imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1> = i2c
        )
        .expect("Failed to initialize shared I2C bus LPI2C1");
        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let imu = IMU::new(i2c_bus.acquire_i2c());

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
            spi.set_clock_hz(LPSPI_FREQUENCY, 5_000_000u32);
            spi.set_mode(MODE_0);
        });

        // Init radio cs pin and ce pin
        let radio_cs = gpio1.output(pins.p14);
        let ce = gpio1.output(pins.p20);

        // Initialize radio
        let radio = Radio::new(ce, radio_cs);

        // Initialize pins for the FPGA
        let cs = gpio2.output(pins.p9);
        let init_b = gpio4.input(pins.p29);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        let done = gpio3.input(pins.p30);

        // Initialize the FPGA
        let fpga = match FPGA::new(spi, cs, init_b, prog_b, done) {
            Ok(fpga) => fpga,
            Err(_) => panic!("Unable to initialize the FPGA"),
        };

        // Set an initial robot status
        let initial_robot_status = RobotStatusMessageBuilder::new().robot_id(ROBOT_ID).build();

        let fake_spi_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit3);
        let fake_spi = FakeSpi::new(
            gpio4.output(pins.p2),
            gpio4.output(pins.p3),
            gpio4.input(pins.p4),
            fake_spi_delay,
        );

        let kicker_controller = Kicker::new(gpio4.output(pins.p5), gpio2.output(pins.p6));

        rx_int.clear_triggered();

        let display_interface = I2CDisplayInterface::new(i2c_bus.acquire_i2c());
        let display: DisplayT = Ssd1306::new(
            display_interface,
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();

        initialize_imu::spawn().ok();

        (
            Shared {
                pit0,
                gpt: gpt1,
                shared_spi,
                blocking_delay: delay2,
                rx_int,
                fpga,
                gpio1,
                robot_status: initial_robot_status,
                control_message: None,
                counter: 0,
                elapsed_time: 0,
                radio,
                imu,
                pit_delay,
                kicker_programmer: None,
                kicker_controller: Some(kicker_controller),
                fake_spi,
                display,
                state: State::default(),

                // Errors
                imu_init_error: None,
                radio_init_error: None,
                fpga_prog_error: None,
                fpga_init_error: None,
                kicker_program_error: None,
                kicker_service_error: None,
            },
            Local {
                motion_controller: MotionControl::new(),
                last_encoders: Vector4::zeros(),
            },
        )
    }

    /// Initialize the IMU
    #[task(
        shared = [imu, pit_delay, imu_init_error],
        priority = 1
    )]
    async fn initialize_imu(ctx: initialize_imu::Context) {
        (
            ctx.shared.imu,
            ctx.shared.pit_delay,
            ctx.shared.imu_init_error,
        )
            .lock(|imu, pit_delay, imu_init_error| {
                if let Err(err) = imu.init(pit_delay) {
                    *imu_init_error = Some(err);
                }
            });

        initialize_radio::spawn().ok();
    }

    /// Initialize the nRF24l01 Radio
    #[task(
        shared = [radio, shared_spi, blocking_delay, radio_init_error],
        priority = 1
    )]
    async fn initialize_radio(ctx: initialize_radio::Context) {
        (
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
            ctx.shared.radio_init_error,
        )
            .lock(
                |radio, spi, delay, radio_init_error| match radio.begin(spi, delay) {
                    Ok(_) => {
                        radio.set_pa_level(BASE_AMPLIFICATION_LEVEL, spi, delay);
                        radio.set_channel(CHANNEL, spi, delay);
                        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, spi, delay);
                        radio.open_writing_pipe(BASE_STATION_ADDRESSES[TEAM_NUM], spi, delay);
                        radio.open_reading_pipe(1, RADIO_ADDRESS, spi, delay);
                        radio.stop_listening(spi, delay);
                    }
                    Err(err) => *radio_init_error = Some(err),
                },
            );

        initialize_fpga::spawn().ok();
    }

    /// Initialize the FPGA Motor Driver
    #[task(
        shared = [fpga, blocking_delay, imu_init_error, fpga_init_error, fpga_prog_error, radio_init_error],
        priority = 1
    )]
    async fn initialize_fpga(ctx: initialize_fpga::Context) {
        (
            ctx.shared.fpga,
            ctx.shared.blocking_delay,
            ctx.shared.fpga_init_error,
            ctx.shared.fpga_prog_error,
        )
            .lock(|fpga, delay, fpga_init_error, fpga_prog_error| {
                if let Err(err) = fpga.configure(delay) {
                    *fpga_prog_error = Some(err);
                    return;
                }

                delay.delay_ms(10u8);

                if let Err(err) = fpga.motors_en(true, delay) {
                    *fpga_init_error = Some(err);
                }
            });

        initialize_kicker::spawn().ok();
    }

    /// Initialize the kicker and kicker controller
    #[task(
        shared = [
            kicker_programmer,
            kicker_controller,
            pit_delay,
            fake_spi,
            kicker_program_error,
            kicker_service_error,
            robot_status,
        ],
        priority = 1
    )]
    async fn initialize_kicker(ctx: initialize_kicker::Context) {
        (
            ctx.shared.kicker_programmer,
            ctx.shared.kicker_controller,
            ctx.shared.pit_delay,
            ctx.shared.robot_status,
            ctx.shared.fake_spi,
            ctx.shared.kicker_program_error,
            ctx.shared.kicker_service_error,
        )
            .lock(
                |programmer,
                 controller,
                 delay,
                 robot_status,
                 spi,
                 kicker_program_error,
                 kicker_service_error| {
                    let mut kicker_programmer = match programmer.take() {
                        Some(kicker_programmer) => kicker_programmer,
                        None => {
                            let (cs, reset) = controller.take().unwrap().destroy();
                            KickerProgrammer::new(cs, reset)
                        }
                    };

                    match kicker_programmer.program_kicker(spi, delay) {
                        Ok(_) => {
                            log::info!("Kicker Programmed");
                            let (cs, reset) = kicker_programmer.destroy();
                            let mut kicker_controller = Kicker::new(cs, reset);

                            match kicker_controller.service(
                                KickerCommand {
                                    kick_type: KickType::Kick,
                                    kick_trigger: KickTrigger::Disabled,
                                    kick_strength: 0.0,
                                    charge_allowed: false,
                                },
                                spi,
                            ) {
                                Ok(status) => {
                                    log::info!("Kicker Serviced");
                                    robot_status.kick_status = false;
                                    robot_status.ball_sense_status = status.ball_sensed;
                                    robot_status.kick_healthy = status.healthy;
                                }
                                Err(err) => {
                                    log::error!("Unable to Service Kicker: {:?}", err);
                                    *kicker_service_error = Some(err);
                                }
                            }

                            *controller = Some(kicker_controller);
                        }
                        Err(err) => {
                            log::error!("Unable to program kicker: {:?}", err);
                            *kicker_program_error = Some(err);
                        }
                    }
                },
            );

        check_for_errors::spawn().ok();
    }

    /// Check for any errors with the peripheral drivers
    #[task(
        shared = [
            imu_init_error,
            fpga_init_error,
            fpga_prog_error,
            radio_init_error,
            kicker_program_error,
            kicker_service_error,
        ],
        priority = 1
    )]
    async fn check_for_errors(ctx: check_for_errors::Context) {
        if (
            ctx.shared.imu_init_error,
            ctx.shared.fpga_init_error,
            ctx.shared.fpga_prog_error,
            ctx.shared.radio_init_error,
            ctx.shared.kicker_program_error,
            ctx.shared.kicker_service_error,
        )
            .lock(
                |imu_init_error,
                 fpga_init_error,
                 fpga_prog_error,
                 radio_init_error,
                 kicker_program_error,
                 kicker_service_error| {
                    imu_init_error.is_some()
                        || fpga_init_error.is_some()
                        || fpga_prog_error.is_some()
                        || radio_init_error.is_some()
                        || kicker_program_error.is_some()
                        || kicker_service_error.is_some()
                },
            )
        {
            error_report::spawn().ok();
        } else {
            start_listening::spawn().ok();
        }
    }

    /// Report (eventually via the OLED Display) any errors that occurred during
    /// initialization.
    #[task(
        shared = [
            imu_init_error,
            fpga_prog_error,
            fpga_init_error,
            radio_init_error,
            kicker_program_error,
            kicker_service_error,
        ],
        priority = 1
    )]
    async fn error_report(ctx: error_report::Context) {
        let (
            imu_init_error,
            fpga_prog_error,
            fpga_init_error,
            radio_init_error,
            kicker_program_error,
            kicker_service_error,
        ) = (
            ctx.shared.imu_init_error,
            ctx.shared.fpga_prog_error,
            ctx.shared.fpga_init_error,
            ctx.shared.radio_init_error,
            ctx.shared.kicker_program_error,
            ctx.shared.kicker_service_error,
        )
            .lock(
                |imu_init_error,
                 fpga_prog_error,
                 fpga_init_error,
                 radio_init_error,
                 kicker_program_error,
                 kicker_service_error| {
                    (
                        imu_init_error.take(),
                        fpga_prog_error.take(),
                        fpga_init_error.take(),
                        radio_init_error.take(),
                        kicker_program_error.take(),
                        kicker_service_error.take(),
                    )
                },
            );

        for _ in 0..5 {
            log::error!("IMU-INIT: {:?}", imu_init_error);
            log::error!("FPGA-PROG: {:?}", fpga_prog_error);
            log::error!("FPGA-INIT: {:?}", fpga_init_error);
            log::error!("RADIO-INIT: {:?}", radio_init_error);
            log::error!("KICKER-PROG: {:?}", kicker_program_error);
            log::error!("KICKER-SERVICE: {:?}", kicker_service_error);
            Systick::delay(1_000u32.millis()).await;
        }

        log::error!("IMU-INIT: {:?}", imu_init_error);
        log::error!("FPGA-PROG: {:?}", fpga_prog_error);
        log::error!("FPGA-INIT: {:?}", fpga_init_error);
        log::error!("RADIO-INIT: {:?}", radio_init_error);
        log::error!("KICKER-PROG: {:?}", kicker_program_error);
        panic!("KICKER-SERVICE: {:?}", kicker_service_error);
    }

    /// Have the radio start listening for incoming commands
    #[task(
        shared = [radio, shared_spi, blocking_delay, gpio1, rx_int, pit0],
        priority = 1
    )]
    async fn start_listening(mut ctx: start_listening::Context) {
        (
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
            ctx.shared.gpio1,
            ctx.shared.rx_int,
        )
            .lock(|radio, spi, delay, gpio1, rx_int| {
                rx_int.clear_triggered();
                gpio1.set_interrupt(rx_int, Some(Trigger::FallingEdge));
                radio.start_listening(spi, delay);
            });

        ctx.shared.pit0.lock(|pit| {
            pit.clear_elapsed();
            pit.set_load_timer_value(MOTION_CONTROL_DELAY_MS);
            pit.set_interrupt_enable(true);
            pit.enable();
        });
    }

    /// Interrupt called when the radio receives new data
    #[task(binds = GPIO1_COMBINED_16_31, shared = [rx_int, gpio1, elapsed_time], priority = 2)]
    fn radio_interrupt(ctx: radio_interrupt::Context) {
        if (ctx.shared.rx_int, ctx.shared.gpio1, ctx.shared.elapsed_time).lock(
            |rx_int, gpio1, elapsed_time| {
                if rx_int.is_triggered() {
                    *elapsed_time = 0;
                    rx_int.clear_triggered();
                    gpio1.set_interrupt(rx_int, None);
                    return true;
                }
                false
            },
        ) {
            receive_command::spawn().ok();
        }
    }

    /// Task that decodes the command received via the radio.  This task is directly
    /// spawned by the radio_interrupt hardware task.
    #[task(
        shared = [radio, rx_int, gpio1, shared_spi, blocking_delay, control_message, robot_status, counter, state],
        priority = 2
    )]
    async fn receive_command(mut ctx: receive_command::Context) {
        (
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
            ctx.shared.control_message,
            ctx.shared.robot_status,
            ctx.shared.counter,
            ctx.shared.radio,
        )
            .lock(|spi, delay, command, robot_status, counter, radio| {
                let mut read_buffer = [0u8; CONTROL_MESSAGE_SIZE];
                radio.read(&mut read_buffer, spi, delay);

                let control_message = ControlMessage::unpack(&read_buffer).unwrap();

                *counter = 0;

                #[cfg(feature = "debug")]
                log::info!("Control Command Received: {:?}", control_message);

                ctx.shared.state.lock(|state| match control_message.mode {
                    Mode::Default => *state = State::Default,
                    Mode::ImuTest => *state = State::IMUTesting,
                    Mode::ReceiveBenchmark => *state = State::ReceiveBenchmark,
                    Mode::SendBenchmark => *state = State::SendBenchmark,
                    Mode::ProgramKickOnBreakbeam => *state = State::ProgramKickOnBreakbeam,
                    Mode::ProgramKicker => *state = State::ProgramKicker,
                    Mode::KickerTest => *state = State::KickerTesting,
                    Mode::FpgaTest => *state = State::FpgaTesting,
                });
                *command = Some(control_message);

                let mut packed_data = [0u8; ROBOT_STATUS_SIZE];
                robot_status.pack(&mut packed_data).unwrap();

                radio.set_payload_size(ROBOT_STATUS_SIZE as u8, spi, delay);
                radio.stop_listening(spi, delay);

                for _ in 0..5 {
                    let report = radio.write(&packed_data, spi, delay);
                    if report {
                        break;
                    }
                }

                radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, spi, delay);
                radio.start_listening(spi, delay);
            });

        (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            rx_int.clear_triggered();
            gpio1.set_interrupt(rx_int, Some(Trigger::Low));
        });
    }

    /// State machine task that handles running the currently selected task using a
    /// pit countdown timer
    #[task(
        shared = [state, pit0],
        priority = 1,
        binds = PIT
    )]
    fn state_machine(mut ctx: state_machine::Context) {
        ctx.shared.pit0.lock(|pit| {
            pit.clear_elapsed();
            pit.set_load_timer_value(MOTION_CONTROL_DELAY_MS);
            pit.set_interrupt_enable(true);
            pit.enable();
        });

        // Most of the testing programs ceil the priority and should not
        // be interruptable but the only way for spawning them to error is if
        // they are already running so we can safely ignore their failure.
        match ctx.shared.state.lock(|state| *state) {
            State::Default => {
                // Default operation is running the motion control loop
                if motion_control_loop::spawn().is_err() {
                    log::error!("Motion Control Loop Already Running");
                }
            }
            State::Idle => (),
            State::IMUTesting => {
                let _ = imu_test::spawn();
            }
            State::ReceiveBenchmark => {
                let _ = benchmark_radio_receive::spawn();
            }
            State::SendBenchmark => {
                let _ = benchmark_radio_send::spawn();
            }
            State::ProgramKickOnBreakbeam => {
                let _ = program_kick_on_breakbeam::spawn();
            }
            State::ProgramKicker => {
                let _ = program_kicker_normal::spawn();
            }
            State::KickerTesting => {
                let _ = test_kicker::spawn();
            }
            State::FpgaTesting => {
                let _ = test_fpga_movement::spawn();
            }
        };
    }

    /// The motion control loop.
    ///
    /// This is the normal `main` operation of our robot.  This calculates a target velocity and
    /// moves the robot at what is believed to be the target velocity.
    ///
    /// The motion control loop is triggered by the PIT to have maximum reliability
    #[task(
        shared = [imu, control_message, counter, elapsed_time, fpga, rx_int, gpio1, blocking_delay, gpt, kicker_controller, kicker_programmer, robot_status, fake_spi],
        local = [motion_controller, last_encoders, initialized: bool = false, iteration: u32 = 0, last_time: u32 = 0],
        priority = 1,
    )]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            *ctx.local.initialized = true;
            *ctx.local.last_time = ctx.shared.gpt.lock(|gpt| gpt.count());
        }

        let (mut body_velocities, dribbler_enabled) =
            ctx.shared
                .control_message
                .lock(|control_message| match control_message {
                    Some(control_message) => (
                        control_message.get_velocity(),
                        control_message.dribbler_speed != 0,
                    ),
                    None => (Vector3::new(0.0, 0.0, 0.0), false),
                });

        let (gyro, accel_x, accel_y) = ctx.shared.imu.lock(|imu| {
            let gyro = imu.gyro_z().unwrap_or(0.0);
            let accel_x = imu.accel_x().unwrap_or(0.0);
            let accel_y = imu.accel_y().unwrap_or(0.0);

            (gyro, accel_x, accel_y)
        });

        let now = ctx.shared.gpt.lock(|gpt| gpt.count());
        let delta = now - *ctx.local.last_time;
        *ctx.local.last_time = now;
        let elapsed_time = ctx.shared.elapsed_time.lock(|elapsed_time| {
            *elapsed_time += delta;
            *elapsed_time
        });

        if elapsed_time > DIE_TIME_US {
            body_velocities = Vector3::zeros();
            #[cfg(feature = "debug")]
            log::info!("DEAD: {}", elapsed_time);
        }

        let wheel_velocities = ctx.local.motion_controller.control_update(
            Vector3::new(-accel_y, accel_x, gyro),
            *ctx.local.last_encoders,
            body_velocities,
            delta,
        );

        #[cfg(feature = "debug")]
        log::info!("Moving at {:?}", wheel_velocities);

        // TODO: Eventually it may be useful to update the fpga_status every tick, however I feel this might be unnecessary
        // so I'm currently only updating the fpga status field on the robot status whenever the kicker is serviced to make
        // rust borrowing easier.
        let (encoder_velocities, fpga_status) =
            (ctx.shared.fpga, ctx.shared.blocking_delay).lock(|fpga, delay| {
                (
                    fpga.set_velocities(wheel_velocities.into(), dribbler_enabled, delay)
                        .unwrap_or([0.0; 4]),
                    fpga.status,
                )
            });

        *ctx.local.last_encoders = Vector4::new(
            encoder_velocities[0],
            encoder_velocities[1],
            encoder_velocities[2],
            encoder_velocities[3],
        );

        // Service the kicker
        if *ctx.local.iteration % KICKER_SERVICE_DELAY_TICKS == 0 {
            let kicker_command = ctx.shared.control_message.lock(|control_message| {
                if let Some(control_message) = control_message {
                    (*control_message).into()
                } else {
                    KickerCommand::default()
                }
            });
            (
                ctx.shared.kicker_controller,
                ctx.shared.kicker_programmer,
                ctx.shared.robot_status,
                ctx.shared.fake_spi,
            )
                .lock(
                    |controller, programmer, robot_status, fake_spi| match controller.take() {
                        Some(mut kicker_controller) => {
                            let state =
                                kicker_controller.service(kicker_command, fake_spi).unwrap();
                            robot_status.kick_status =
                                kicker_command.kick_trigger != KickTrigger::Disabled;
                            robot_status.ball_sense_status = state.ball_sensed;
                            robot_status.kick_healthy = state.healthy;
                            robot_status.motor_errors = fpga_status & 0x1F;
                            robot_status.fpga_status = fpga_status & 0x80 != 0;
                            *controller = Some(kicker_controller);
                        }
                        None => {
                            let (cs, reset) = programmer.take().unwrap().destroy();
                            let mut kicker_controller = Kicker::new(cs, reset);
                            let state =
                                kicker_controller.service(kicker_command, fake_spi).unwrap();
                            robot_status.kick_status =
                                kicker_command.kick_trigger != KickTrigger::Disabled;
                            robot_status.ball_sense_status = state.ball_sensed;
                            robot_status.kick_healthy = state.healthy;
                            robot_status.motor_errors = fpga_status & 0x1F;
                            robot_status.fpga_status = fpga_status & 0x80 != 0;
                            *controller = Some(kicker_controller);
                        }
                    },
                );
        }

        *ctx.local.iteration = ctx.local.iteration.wrapping_add(1);
    }

    /// Stop the motors from moving and discharge the kicker.
    ///
    /// Basically, put the robot into `idle`.
    ///
    /// Note: Idle also disables the main control interrupts so testing
    /// can be conducted without being interrupted.
    #[task(
        shared = [
            pit0, state
        ],
        priority = 1
    )]
    async fn to_idle(ctx: to_idle::Context) {
        (ctx.shared.pit0, ctx.shared.state).lock(|pit, state| {
            // Disable the motion control loop
            pit.clear_elapsed();
            pit.disable();
            *state = State::Idle;
        });
    }

    /// Re-enable the main control interrupt so normal operation can be
    /// entered again
    #[task(
        shared = [
            pit0, state
        ],
        priority = 1
    )]
    async fn from_idle(ctx: from_idle::Context) {
        (ctx.shared.pit0, ctx.shared.state).lock(|pit, state| {
            pit.set_load_timer_value(MOTION_CONTROL_DELAY_MS);
            pit.set_interrupt_enable(true);
            pit.enable();
            *state = State::Default
        });
    }

    /// Test that the IMU on the Teensy is reading reasonable
    /// values
    #[task(
        shared = [imu, pit_delay, state, radio, rx_int, gpio1, shared_spi, blocking_delay],
        priority = 1
    )]
    async fn imu_test(ctx: imu_test::Context) {
        (
            ctx.shared.imu,
            ctx.shared.pit_delay,
            ctx.shared.state,
            ctx.shared.radio,
            ctx.shared.rx_int,
            ctx.shared.gpio1,
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
        )
            .lock(
                |imu, delay, state, radio, rx_int, gpio1, spi, radio_delay| {
                    // Disable Radio Interrupts for Testing
                    disable_radio_interrupts(
                        IMU_MESSAGE_SIZE,
                        rx_int,
                        gpio1,
                        radio,
                        spi,
                        radio_delay,
                    );

                    let mut buffer = [0u8; IMU_MESSAGE_SIZE];
                    for i in 0..100 {
                        let gyro_z = imu.gyro_z().unwrap_or(0.0);
                        let accel_x = imu.accel_x().unwrap_or(0.0);
                        let accel_y = imu.accel_y().unwrap_or(0.0);

                        // Write the update message
                        let imu_message: ImuTestMessage = ImuTestMessage {
                            first_message: i == 0,
                            last_message: i == 99,
                            gyro_z,
                            accel_x,
                            accel_y,
                        };
                        imu_message.pack(&mut buffer).unwrap();
                        radio.write(&buffer, spi, radio_delay);
                        log::info!("X: {}, Y: {}, Z: {}", accel_x, accel_y, gyro_z);

                        delay.delay_ms(10u8);
                    }

                    *state = State::Idle;

                    // Re-enable the radio interrupts
                    enable_radio_interrupts(rx_int, gpio1, radio, spi, radio_delay);
                },
            );
    }

    /// Benchmark the number of radio packets received
    /// over the next 5 seconds
    #[task(
        shared = [
            radio, shared_spi, blocking_delay, gpt, state, rx_int, gpio1,
        ],
        priority = 1
    )]
    async fn benchmark_radio_receive(ctx: benchmark_radio_receive::Context) {
        log::info!("Listening for packets");
        let received_packets = (
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
            ctx.shared.gpt,
            ctx.shared.state,
            ctx.shared.rx_int,
            ctx.shared.gpio1,
        )
            .lock(|radio, spi, delay, gpt, state, rx_int, gpio1| {
                disable_radio_interrupts(CONTROL_MESSAGE_SIZE, rx_int, gpio1, radio, spi, delay);
                let mut received_packets = 0;

                radio.start_listening(spi, delay);

                let start_time = gpt.count();

                while start_time - gpt.count() < 5_000_000 {
                    if radio.packet_ready(spi, delay) {
                        let mut buffer = [0u8; CONTROL_MESSAGE_SIZE];
                        radio.read(&mut buffer, spi, delay);
                        received_packets += 1;
                        radio.flush_rx(spi, delay);
                    }

                    delay.delay_ms(10u32);
                }

                *state = State::Idle;

                let message = RadioReceiveBenchmarkMessage {
                    receive_time_ms: 5_000,
                    received_packets,
                };
                let mut buffer = [0u8; RADIO_RECEIVE_BENCHMARK_SIZE];
                message.pack(&mut buffer).unwrap();

                for _ in 0..3 {
                    if radio.write(&buffer, spi, delay) {
                        break;
                    }

                    delay.delay_ms(50u32);
                }

                enable_radio_interrupts(rx_int, gpio1, radio, spi, delay);

                received_packets
            });

        // TODO: It might be useful to send this data over the radio
        log::info!("Received {} Packets over 5 Seconds", received_packets);
    }

    /// Send 100 packets with 50ms of delay between each packets and
    /// report the total number of packets acknowledged by the other radio.
    #[task(
        shared = [
            radio, shared_spi, blocking_delay, state, rx_int, gpio1
        ],
        priority = 1
    )]
    async fn benchmark_radio_send(ctx: benchmark_radio_send::Context) {
        log::info!("Sending 100 Packets");
        let successful_sends = (
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
            ctx.shared.state,
            ctx.shared.rx_int,
            ctx.shared.gpio1,
        )
            .lock(|radio, spi, delay, state, rx_int, gpio1| {
                disable_radio_interrupts(
                    RADIO_SEND_BENCHMARK_SIZE,
                    rx_int,
                    gpio1,
                    radio,
                    spi,
                    delay,
                );
                radio.stop_listening(spi, delay);

                let mut successful_sends = 0;

                let mut last_ball_sense: bool = false;
                let mut last_kick_status: bool = true;

                for _ in 0..100 {
                    let robot_status = RobotStatusMessageBuilder::new()
                        .robot_id(ROBOT_ID)
                        .team(TEAM)
                        .ball_sense_status(!last_ball_sense)
                        .kick_status(!last_kick_status)
                        .build();

                    last_ball_sense = !last_ball_sense;
                    last_kick_status = !last_kick_status;

                    let mut packed_data = [0u8; ROBOT_STATUS_SIZE];
                    robot_status.pack(&mut packed_data).unwrap();

                    let report = radio.write(&packed_data, spi, delay);
                    radio.flush_tx(spi, delay);

                    if report {
                        successful_sends += 1;
                    }

                    delay.delay_ms(50u32);
                }

                *state = State::Idle;

                let message = RadioSendBenchmarkMessage {
                    acknowledged_packets: successful_sends,
                    sent_packets: 100,
                };
                let mut buffer = [0u8; RADIO_SEND_BENCHMARK_SIZE];
                message.pack(&mut buffer).unwrap();
                for _ in 0..3 {
                    if radio.write(&buffer, spi, delay) {
                        break;
                    }

                    delay.delay_ms(50u32);
                }

                enable_radio_interrupts(rx_int, gpio1, radio, spi, delay);

                successful_sends
            });

        log::info!("{} / 100 Packets Acknowledged", successful_sends);
    }

    /// Program the kicker with kick-on-breakbeam
    #[task(
        shared = [
            fake_spi,
            kicker_programmer,
            kicker_controller,
            pit_delay,
            state,
            rx_int,
            gpio1,
            radio,
            shared_spi,
            blocking_delay,
        ],
        priority = 1
    )]
    async fn program_kick_on_breakbeam(ctx: program_kick_on_breakbeam::Context) {
        (
            ctx.shared.fake_spi,
            ctx.shared.kicker_programmer,
            ctx.shared.kicker_controller,
            ctx.shared.pit_delay,
            ctx.shared.state,
            ctx.shared.rx_int,
            ctx.shared.gpio1,
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
        )
            .lock(
                |spi,
                 kicker_programmer,
                 kicker_controller,
                 delay,
                 state,
                 rx_int,
                 gpio1,
                 radio,
                 radio_spi,
                 radio_delay| {
                    disable_radio_interrupts(
                        KICKER_PROGRAM_MESSAGE,
                        rx_int,
                        gpio1,
                        radio,
                        radio_spi,
                        radio_delay,
                    );

                    let mut programmer = match kicker_programmer.take() {
                        Some(kicker_programmer) => kicker_programmer,
                        None => {
                            let (cs, reset) = kicker_controller.take().unwrap().destroy();
                            KickerProgrammer::new(cs, reset)
                        }
                    };

                    let mut buffer = [0u8; KICKER_PROGRAM_MESSAGE];
                    let message = KickerProgramMessage {
                        kick_on_breakbeam: true,
                        finished: false,
                        page: 0,
                    };
                    message.pack(&mut buffer).unwrap();
                    radio.write(&buffer, radio_spi, radio_delay);

                    // TODO: Send updates while programming the kicker
                    match programmer.program_kick_on_breakbeam(spi, delay) {
                        Ok(_) => log::info!("Kicker Programmed with Kick on Breakbeam"),
                        Err(err) => log::error!("Unable to Program Kicker: {:?}", err),
                    }

                    let message = KickerProgramMessage {
                        kick_on_breakbeam: true,
                        finished: true,
                        page: 0,
                    };
                    message.pack(&mut buffer).unwrap();
                    radio.write(&buffer, radio_spi, radio_delay);

                    *kicker_programmer = Some(programmer);

                    *state = State::Idle;

                    enable_radio_interrupts(rx_int, gpio1, radio, radio_spi, radio_delay);
                },
            );
    }

    /// Program the kicker with normal operations
    #[task(
        shared = [
            fake_spi,
            kicker_programmer,
            kicker_controller,
            pit_delay,
            state,
            rx_int,
            gpio1,
            radio,
            shared_spi,
            blocking_delay,
        ],
        priority = 1
    )]
    async fn program_kicker_normal(ctx: program_kicker_normal::Context) {
        (
            ctx.shared.fake_spi,
            ctx.shared.kicker_programmer,
            ctx.shared.kicker_controller,
            ctx.shared.pit_delay,
            ctx.shared.state,
            ctx.shared.rx_int,
            ctx.shared.gpio1,
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
        )
            .lock(
                |spi,
                 kicker_programmer,
                 kicker_controller,
                 delay,
                 state,
                 rx_int,
                 gpio1,
                 radio,
                 radio_spi,
                 radio_delay| {
                    disable_radio_interrupts(
                        KICKER_PROGRAM_MESSAGE,
                        rx_int,
                        gpio1,
                        radio,
                        radio_spi,
                        radio_delay,
                    );

                    let mut programmer = match kicker_programmer.take() {
                        Some(kicker_programmer) => kicker_programmer,
                        None => {
                            let (cs, reset) = kicker_controller.take().unwrap().destroy();
                            KickerProgrammer::new(cs, reset)
                        }
                    };

                    let mut buffer = [0u8; KICKER_PROGRAM_MESSAGE];
                    let message = KickerProgramMessage {
                        kick_on_breakbeam: false,
                        finished: false,
                        page: 0,
                    };
                    message.pack(&mut buffer).unwrap();
                    radio.write(&buffer, radio_spi, radio_delay);

                    // TODO: Send updates while programming the kicker
                    match programmer.program_kicker(spi, delay) {
                        Ok(_) => {
                            log::info!("Kicker Programmed");
                            let (cs, reset) = programmer.destroy();
                            *kicker_controller = Some(Kicker::new(cs, reset));
                        }
                        Err(err) => {
                            // TODO: Spawn a task to print error
                            log::error!("Unable to Program Kicker: {:?}", err);
                            *kicker_programmer = Some(programmer);
                        }
                    }

                    let message = KickerProgramMessage {
                        kick_on_breakbeam: false,
                        finished: true,
                        page: 0,
                    };
                    message.pack(&mut buffer).unwrap();
                    radio.write(&buffer, radio_spi, radio_delay);

                    *state = State::Idle;

                    enable_radio_interrupts(rx_int, gpio1, radio, radio_spi, radio_delay);
                },
            );
    }

    /// Test the kicker is working properly
    #[task(
        shared = [
            fake_spi,
            kicker_programmer,
            kicker_controller,
            pit_delay,
            state,
            rx_int,
            gpio1,
            radio,
            shared_spi,
            blocking_delay,
        ],
        priority = 1
    )]
    async fn test_kicker(ctx: test_kicker::Context) {
        program_kicker_normal::spawn().ok();

        (
            ctx.shared.fake_spi,
            ctx.shared.kicker_programmer,
            ctx.shared.kicker_controller,
            ctx.shared.pit_delay,
            ctx.shared.state,
            ctx.shared.rx_int,
            ctx.shared.gpio1,
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
        )
            .lock(
                |spi,
                 kicker_programmer,
                 kicker_controller,
                 delay,
                 state,
                 rx_int,
                 gpio1,
                 radio,
                 radio_spi,
                 radio_delay| {
                    disable_radio_interrupts(
                        KICKER_TESTING_SIZE,
                        rx_int,
                        gpio1,
                        radio,
                        radio_spi,
                        radio_delay,
                    );

                    let mut kicker = match kicker_controller.take() {
                        Some(kicker_controller) => kicker_controller,
                        None => {
                            let (cs, reset) = kicker_programmer.take().unwrap().destroy();
                            Kicker::new(cs, reset)
                        }
                    };

                    log::info!("Charging the Kicker");
                    let kicker_command = KickerCommand {
                        kick_type: KickType::Kick,
                        kick_trigger: KickTrigger::Disabled,
                        kick_strength: 20.0,
                        charge_allowed: true,
                    };
                    let mut buffer = [0u8; KICKER_TESTING_SIZE];
                    for i in 0..20 {
                        let kicker_state = match kicker.service(kicker_command, spi) {
                            Ok(status) => {
                                log::info!("Kicker Status: {:?}", status);
                                Some(status)
                            }
                            Err(err) => {
                                log::error!("Error Servicing Kicker: {:?}", err);
                                None
                            }
                        };
                        if let Some(state) = kicker_state {
                            let message = KickerTestingMessage {
                                healthy: state.healthy,
                                ball_sense: state.ball_sensed,
                                kicking: i == 19,
                                kick_on_ball_sense: false,
                                kick_immediately: false,
                                voltage: state.current_voltage,
                            };
                            message.pack(&mut buffer).unwrap();
                            radio.write(&buffer, radio_spi, radio_delay);
                        }
                        delay.delay_ms(100u32);
                    }

                    log::info!("KICKING!!!");
                    let kicker_command = KickerCommand {
                        kick_type: KickType::Kick,
                        kick_trigger: KickTrigger::Immediate,
                        kick_strength: 100.0,
                        charge_allowed: true,
                    };
                    let kicker_state = match kicker.service(kicker_command, spi) {
                        Ok(status) => {
                            log::info!("Kicker Status: {:?}", status);
                            Some(status)
                        }
                        Err(err) => {
                            log::error!("Error Kicking: {:?}", err);
                            None
                        }
                    };
                    if let Some(state) = kicker_state {
                        let message = KickerTestingMessage {
                            healthy: state.healthy,
                            ball_sense: state.ball_sensed,
                            kicking: true,
                            kick_immediately: true,
                            kick_on_ball_sense: false,
                            voltage: state.current_voltage,
                        };
                        message.pack(&mut buffer).unwrap();
                        radio.write(&buffer, radio_spi, radio_delay);
                    }
                    delay.delay_ms(100u32);

                    log::info!("Powering Down the Kicker");
                    let kicker_command = KickerCommand {
                        kick_type: KickType::Kick,
                        kick_trigger: KickTrigger::Disabled,
                        kick_strength: 0.0,
                        charge_allowed: false,
                    };
                    for _ in 0..20 {
                        let kicker_state = match kicker.service(kicker_command, spi) {
                            Ok(status) => {
                                log::info!("Kicker Status: {:?}", status);
                                Some(status)
                            }
                            Err(err) => {
                                log::error!("Error Powering Down Kicker: {:?}", err);
                                None
                            }
                        };
                        if let Some(state) = kicker_state {
                            let message = KickerTestingMessage {
                                healthy: state.healthy,
                                ball_sense: state.ball_sensed,
                                kicking: true,
                                kick_immediately: false,
                                kick_on_ball_sense: false,
                                voltage: state.current_voltage,
                            };
                            message.pack(&mut buffer).unwrap();
                            radio.write(&buffer, radio_spi, radio_delay);
                        }
                        delay.delay_ms(100u32);
                    }

                    *kicker_controller = Some(kicker);

                    *state = State::Idle;

                    enable_radio_interrupts(rx_int, gpio1, radio, radio_spi, radio_delay);
                },
            );
    }

    /// Test the fpga moving at a given velocity for 1 second.
    #[task(
        shared = [
            fpga,
            imu,
            blocking_delay,
            control_message,
            gpt,
            state,
            radio,
            rx_int,
            gpio1,
            shared_spi
        ],
        priority = 1
    )]
    async fn test_fpga_movement(ctx: test_fpga_movement::Context) {
        (
            ctx.shared.fpga,
            ctx.shared.imu,
            ctx.shared.blocking_delay,
            ctx.shared.control_message,
            ctx.shared.gpt,
            ctx.shared.state,
            ctx.shared.radio,
            ctx.shared.rx_int,
            ctx.shared.gpio1,
            ctx.shared.shared_spi,
        )
            .lock(
                |fpga,
                 imu,
                 delay,
                 control_message,
                 gpt,
                 state,
                 radio,
                 rx_int,
                 gpio1,
                 shared_spi| {
                    // Disable Radio Interrupts
                    disable_radio_interrupts(
                        CONTROL_TEST_MESSAGE_SIZE,
                        rx_int,
                        gpio1,
                        radio,
                        shared_spi,
                        delay,
                    );

                    let mut motion_controller = MotionControl::new();
                    let mut last_encoder_values = Vector4::zeros();

                    let setpoint = match control_message {
                        Some(control_message) => control_message.get_velocity(),
                        None => Vector3::new(1.0, 0.0, 0.0),
                    };

                    let mut buffer = [0u8; CONTROL_TEST_MESSAGE_SIZE];

                    let mut last_time = gpt.count();
                    for _ in 0..1_000 {
                        let gyro = imu.gyro_z().unwrap_or(0.0);
                        let accel_x = imu.accel_x().unwrap_or(0.0);
                        let accel_y = imu.accel_y().unwrap_or(0.0);

                        let now = gpt.count();
                        let delta = last_time - now;
                        last_time = now;

                        let wheel_velocities = motion_controller.control_update(
                            Vector3::new(-accel_y, accel_x, gyro),
                            last_encoder_values,
                            setpoint,
                            delta,
                        );
                        let encoder_velocities = fpga
                            .set_velocities(wheel_velocities.into(), false, delay)
                            .unwrap_or([0.0; 4]);

                        last_encoder_values = Vector4::new(
                            encoder_velocities[0],
                            encoder_velocities[1],
                            encoder_velocities[2],
                            encoder_velocities[3],
                        );

                        // TODO: Include delta in ControlTestMessage
                        let message = ControlTestMessage {
                            gyro_z: gyro,
                            accel_x,
                            accel_y,
                            motor_encoders: encoder_velocities,
                            delta,
                        };
                        message.pack(&mut buffer).unwrap();
                        radio.write(&buffer, shared_spi, delay);

                        delay.delay_ms(1u32);
                    }

                    let _ = fpga.set_velocities([0.0, 0.0, 0.0, 0.0], false, delay);

                    delay.delay_ms(500u32);

                    *state = State::Idle;

                    enable_radio_interrupts(rx_int, gpio1, radio, shared_spi, delay);
                },
            );
    }
}
