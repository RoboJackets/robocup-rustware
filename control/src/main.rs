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
    use main::spi::FakeSpi;
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

    use robojackets_robocup_rtp::{BASE_STATION_ADDRESS, TEAM};
    use robojackets_robocup_rtp::{ControlMessage, CONTROL_MESSAGE_SIZE};
    use robojackets_robocup_rtp::{
        RobotStatusMessage, RobotStatusMessageBuilder, ROBOT_STATUS_SIZE,
    };

    use motion::MotionControl;

    use fpga::FPGA;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga_rs as fpga;

    use icm42605_driver::IMU;

    use main::{
        Delay2, FPGAInitError, FPGAProgError, Fpga, Gpio1, Imu, ImuInitError, KickerCSn, KickerProg, KickerReset, PitDelay, RFRadio, RadioInitError, RadioInterrupt, SharedSPI, BASE_AMPLIFICATION_LEVEL, CHANNEL, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY, RADIO_ADDRESS, ROBOT_ID, State
    };

    use kicker_controller::{KickTrigger, KickType, Kicker, KickerCommand};
    use kicker_programmer::KickerProgrammer;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_MS: u32 = 1;
    const DIE_TIME_US: u32 = 250_000;

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

        // State
        state: State,

        // Errors
        imu_init_error: Option<ImuInitError>,
        fpga_prog_error: Option<FPGAProgError>,
        fpga_init_error: Option<FPGAInitError>,
        radio_init_error: Option<RadioInitError>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize the Heap
        unsafe {
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
        gpt1.set_divider(GPT_DIVIDER);
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
        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let imu = IMU::new(i2c);

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
            fake_spi_delay
        );

        let kicker_controller = Kicker::new(gpio4.output(pins.p5), gpio2.output(pins.p6));

        rx_int.clear_triggered();

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
                state: State::default(),

                // Errors
                imu_init_error: None,
                radio_init_error: None,
                fpga_prog_error: None,
                fpga_init_error: None,
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
                        radio.open_writing_pipe(BASE_STATION_ADDRESS, spi, delay);
                        radio.open_reading_pipe(1, RADIO_ADDRESS, spi, delay);
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

        check_for_errors::spawn().ok();
    }

    /// Check for any errors with the peripheral drivers
    #[task(
        shared = [imu_init_error, fpga_init_error, fpga_prog_error, radio_init_error],
        priority = 1
    )]
    async fn check_for_errors(ctx: check_for_errors::Context) {
        if (
            ctx.shared.imu_init_error,
            ctx.shared.fpga_init_error,
            ctx.shared.fpga_prog_error,
            ctx.shared.radio_init_error,
        )
            .lock(
                |imu_init_error, fpga_init_error, fpga_prog_error, radio_init_error| {
                    imu_init_error.is_some()
                        || fpga_init_error.is_some()
                        || fpga_prog_error.is_some()
                        || radio_init_error.is_some()
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
        shared = [imu_init_error, fpga_prog_error, fpga_init_error, radio_init_error],
        priority = 1
    )]
    async fn error_report(ctx: error_report::Context) {
        let (imu_init_error, fpga_prog_error, fpga_init_error, radio_init_error) = (
            ctx.shared.imu_init_error,
            ctx.shared.fpga_prog_error,
            ctx.shared.fpga_init_error,
            ctx.shared.radio_init_error,
        )
            .lock(
                |imu_init_error, fpga_prog_error, fpga_init_error, radio_init_error| {
                    (
                        imu_init_error.take(),
                        fpga_prog_error.take(),
                        fpga_init_error.take(),
                        radio_init_error.take(),
                    )
                },
            );

        for _ in 0..5 {
            log::error!("IMU-INIT: {:?}", imu_init_error);
            log::error!("FPGA-PROG: {:?}", fpga_prog_error);
            log::error!("FPGA-INIT: {:?}", fpga_init_error);
            log::error!("RADIO-INIT: {:?}", radio_init_error);
            Systick::delay(1_000u32.millis()).await;
        }

        log::error!("IMU-INIT: {:?}", imu_init_error);
        log::error!("FPGA-PROG: {:?}", fpga_prog_error);
        log::error!("FPGA-INIT: {:?}", fpga_init_error);
        panic!("RADIO-INIT: {:?}", radio_init_error);
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
        shared = [radio, rx_int, gpio1, shared_spi, blocking_delay, control_message, robot_status, counter],
        priority = 2
    )]
    async fn receive_command(ctx: receive_command::Context) {
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
            gpio1.set_interrupt(rx_int, Some(Trigger::FallingEdge));
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
            },
            State::Idle => (),
            State::IMUTesting => {
                let _ = imu_test::spawn();
            },
            State::ReceiveBenchmark => {
                let _ = benchmark_radio_receive::spawn();
            },
            State::SendBenchmark => {
                let _ = benchmark_radio_send::spawn();
            },
            State::ProgramKickOnBreakbeam => {
                let _ = program_kick_on_breakbeam::spawn();
            },
            State::ProgramKicker => {
                let _ = program_kicker_normal::spawn();
            }
            State::KickerTesting => {
                let _ = test_kicker::spawn();
            },
            State::FpgaTesting => {
                let _ = test_fpga_movement::spawn();
            },
        };
    }

    /// The motion control loop.
    /// 
    /// This is the normal `main` operation of our robot.  This calculates a target velocity and
    /// moves the robot at what is believed to be the target velocity.
    /// 
    /// The motion control loop is triggered by the PIT to have maximum reliability
    #[task(
        shared = [imu, control_message, counter, elapsed_time, fpga, rx_int, gpio1, blocking_delay, gpt],
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
            let gyro = match imu.gyro_z() {
                Ok(gyro) => gyro,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Unable to Read Gyro");
                    0.0
                }
            };

            let accel_x = match imu.accel_x() {
                Ok(accel_x) => accel_x,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Unable to Read Accel X");
                    0.0
                }
            };

            let accel_y = match imu.accel_y() {
                Ok(accel_y) => accel_y,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Unable to Read Accel Y");
                    0.0
                }
            };

            (gyro, accel_x, accel_y)
        });

        let now = ctx.shared.gpt.lock(|gpt| gpt.count());
        let delta = *ctx.local.last_time - now;
        *ctx.local.last_time = now;
        let elapsed_time = ctx.shared.elapsed_time.lock(|elapsed_time| {
            *elapsed_time += delta;
            *elapsed_time
        });

        if elapsed_time > DIE_TIME_US {
            body_velocities = Vector3::zeros();
        }

        let wheel_velocities = ctx.local.motion_controller.control_update(
            Vector3::new(-accel_y, accel_x, gyro),
            *ctx.local.last_encoders,
            body_velocities,
            delta,
        );

        #[cfg(feature = "debug")]
        log::info!("Moving at {:?}", wheel_velocities);

        let encoder_velocities = (ctx.shared.fpga, ctx.shared.blocking_delay).lock(|fpga, delay| {
            match fpga.set_velocities(wheel_velocities.into(), dribbler_enabled, delay) {
                Ok(encoder_velocities) => encoder_velocities,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Unable to Read Encoder Values");
                    [0.0; 4]
                }
            }
        });

        #[cfg(feature = "debug")]
        log::info!("Fpga Status: {:#010b}", ctx.local.fpga.status);

        *ctx.local.last_encoders = Vector4::new(
            encoder_velocities[0],
            encoder_velocities[1],
            encoder_velocities[2],
            encoder_velocities[3],
        );
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
        shared = [imu, pit_delay, state],
        priority = 1
    )]
    async fn imu_test(ctx: imu_test::Context) {
        (ctx.shared.imu, ctx.shared.pit_delay, ctx.shared.state).lock(|imu, delay, state| {
            for _ in 0..100 {
                let gyro_z = match imu.gyro_z() {
                    Ok(gyro_z) => gyro_z,
                    Err(err) => {
                        log::info!("Unable to Read Gyro Z: {:?}", err);
                        0.0
                    }
                };
    
                let accel_x = match imu.accel_x() {
                    Ok(accel_x) => accel_x,
                    Err(err) => {
                        log::info!("Unable to Read Accel X: {:?}", err);
                        0.0
                    }
                };
    
                let accel_y = match imu.accel_y() {
                    Ok(accel_y) => accel_y,
                    Err(err) => {
                        log::info!("Unable to Read Accel Y: {:?}", err);
                        0.0
                    }
                };

                log::info!("X: {}, Y: {}, Z: {}", accel_x, accel_y, gyro_z);

                delay.delay_ms(10u8);
            }

            *state = State::Idle;
        });
    }

    /// Benchmark the number of radio packets received
    /// over the next 5 seconds
    #[task(
        shared = [
            radio, shared_spi, blocking_delay, gpt, state
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
            ctx.shared.state
        ).lock(|radio, spi, delay, gpt, state| {
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

            received_packets
        });

        log::info!("Received {} Packets over 5 Seconds", received_packets);
    }

    /// Send 100 packets with 50ms of delay between each packets and
    /// report the total number of packets acknowledged by the other radio.
    #[task(
        shared = [
            radio, shared_spi, blocking_delay, state
        ],
        priority = 1
    )]
    async fn benchmark_radio_send(ctx: benchmark_radio_send::Context) {
        log::info!("Sending 100 Packets");
        let successful_sends = (
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.blocking_delay,
            ctx.shared.state
        ).lock(|radio, spi, delay, state| {
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
            }

            *state = State::Idle;

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
        ).lock(|spi, kicker_programmer, kicker_controller, delay, state| {
            let mut programmer = match kicker_programmer.take() {
                Some(kicker_programmer) => kicker_programmer,
                None => {
                    let (cs, reset) = kicker_controller.take().unwrap().destroy();
                    KickerProgrammer::new(cs, reset)
                }
            };

            match programmer.program_kick_on_breakbeam(spi, delay) {
                Ok(_) => log::info!("Kicker Programmed with Kick on Breakbeam"),
                Err(err) => log::error!("Unable to Program Kicker: {:?}", err),
            }

            *kicker_programmer = Some(programmer);

            *state = State::Idle;
        });
    }

    /// Program the kicker with normal operations
    #[task(
        shared = [
            fake_spi,
            kicker_programmer,
            kicker_controller,
            pit_delay,
            state,
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
        ).lock(|spi, kicker_programmer, kicker_controller, delay, state| {
            let mut programmer = match kicker_programmer.take() {
                Some(kicker_programmer) => kicker_programmer,
                None => {
                    let (cs, reset) = kicker_controller.take().unwrap().destroy();
                    KickerProgrammer::new(cs, reset)
                }
            };

            match programmer.program_kicker(spi, delay) {
                Ok(_) => {
                    log::info!("Kicker Programmed");
                    let (cs, reset) = programmer.destroy();
                    *kicker_controller = Some(Kicker::new(cs, reset));
                },
                Err(err) => {
                    // TODO: Spawn a task to print error
                    log::error!("Unable to Program Kicker: {:?}", err);
                    *kicker_programmer = Some(programmer);
                }
            }

            *state = State::Idle;
        });
    }

    /// Test the kicker is working properly
    #[task(
        shared = [
            fake_spi,
            kicker_programmer,
            kicker_controller,
            pit_delay,
            state,
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
        ).lock(|spi, kicker_programmer, kicker_controller, delay, state| {
            let mut kicker = match kicker_controller.take() {
                Some(kicker_controller) => kicker_controller,
                None => {
                    let (cs, reset) = kicker_programmer.take().unwrap().destroy();
                    Kicker::new(cs, reset)
                },
            };

            log::info!("Charging the Kicker");
            let kicker_command = KickerCommand {
                kick_type: KickType::Kick,
                kick_trigger: KickTrigger::Disabled,
                kick_strength: 20.0,
                charge_allowed: true,
            };
            for _ in 0..20 {
                match kicker.service(kicker_command, spi) {
                    Ok(status) => log::info!("Kicker Status: {:?}", status),
                    Err(err) => log::error!("Error Servicing Kicker: {:?}", err),
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
            match kicker.service(kicker_command, spi) {
                Ok(status) => log::info!("Kicker Status: {:?}", status),
                Err(err) => log::error!("Error Kicking: {:?}", err),
            };

            delay.delay_ms(100u32);

            log::info!("Powering Down the Kicker");
            let kicker_command = KickerCommand {
                kick_type: KickType::Kick,
                kick_trigger: KickTrigger::Disabled,
                kick_strength: 0.0,
                charge_allowed: false,
            };
            for _ in 0..20 {
                match kicker.service(kicker_command, spi) {
                    Ok(status) => log::info!("Kicker Status: {:?}", status),
                    Err(err) => log::error!("Error Powering Down Kicker: {:?}", err),
                }
                delay.delay_ms(100u32);
            }

            *kicker_controller = Some(kicker);

            *state = State::Idle;
        });
    }

    /// Test the fpga moving at a given velocity for 1 second.
    #[task(
        shared = [
            fpga, imu, blocking_delay, control_message, gpt, state
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
        ).lock(|fpga, imu, delay, control_message, gpt, state| {
            let mut motion_controller = MotionControl::new();
            let mut last_encoder_values = Vector4::zeros();

            let setpoint = match control_message {
                Some(control_message) => control_message.get_velocity(),
                None => Vector3::new(1.0, 0.0, 0.0),
            };

            let mut last_time = gpt.count();
            for _ in 0..1_000 {
                let gyro = match imu.gyro_z() {
                    Ok(gyro) => gyro,
                    Err(_) => 0.0
                };

                let accel_x = match imu.accel_x() {
                    Ok(accel_x) => accel_x,
                    Err(_) => 0.0
                };

                let accel_y = match imu.accel_y() {
                    Ok(accel_y) => accel_y,
                    Err(_) => 0.0,
                };

                let now = gpt.count();
                let delta = last_time - now;
                last_time = now;

                let wheel_velocities = motion_controller.control_update(
                    Vector3::new(-accel_y, accel_x, gyro),
                    last_encoder_values,
                    setpoint,
                    delta,
                );

                let encoder_velocities = match fpga.set_velocities(wheel_velocities.into(), false, delay) {
                    Ok(encoder_velocities) => encoder_velocities,
                    Err(_) => [0.0; 4],
                };

                last_encoder_values = Vector4::new(
                    encoder_velocities[0],
                    encoder_velocities[1],
                    encoder_velocities[2],
                    encoder_velocities[3],
                );

                delay.delay_ms(1u32);
            }

            *state = State::Idle;
        });
    }
}
