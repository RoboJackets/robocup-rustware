//!
//! Manual Control is an example that should be pretty close to the fully-working program.  However,
//! it is currently considered a work in progress so it is still considered a test.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use alloc::format;
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

    use embedded_hal::{blocking::delay::DelayMs, spi::MODE_0};

    use bsp::board::PERCLK_FREQUENCY;
    use bsp::board::{self, LPSPI_FREQUENCY};
    use nalgebra::{Vector3, Vector4};
    use robojackets_robocup_control::{Killn, MotorEn, RadioSPI};
    use teensy4_bsp as bsp;

    use hal::gpio::Trigger;
    use hal::lpspi::Pins;
    use hal::timer::Blocking;
    use hal::lpuart;
    use teensy4_bsp::hal;

    use rtic_nrf24l01::Radio;

    use rtic_monotonics::systick::*;

    use ncomm_utils::packing::Packable;

    use shared_bus;

    // Includes for display module
    use embedded_graphics::prelude::*;
    use graphics::{error_screen::ErrorScreen, startup_screen::StartScreen};
    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
    use teensy4_pins::t41::{P18, P19};

    use robojackets_robocup_control::robot::{TEAM, TEAM_NUM};
    use robojackets_robocup_rtp::{ControlMessageBuilder, BASE_STATION_ADDRESSES};
    use robojackets_robocup_rtp::{
        control_message::Mode,
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

    use icm42605_driver::IMU;

    use robojackets_robocup_control::{
        spi::FakeSpi, Delay2, Display, Gpio1, Imu, ImuInitError, KickerCSn, KickerProg,
        KickerProgramError, KickerReset, KickerServicingError, PitDelay, RFRadio, RadioInitError,
        RadioInterrupt, State, BASE_AMPLIFICATION_LEVEL, CHANNEL, GPT_1_DIVIDER, GPT_CLOCK_SOURCE,
        GPT_DIVIDER, GPT_FREQUENCY, RADIO_ADDRESS, ROBOT_ID, MotorOneUart, MotorTwoUart,
        MotorThreeUart, MotorFourUart, DribblerUart, motors::{motor_interrupt, send_command}
    };

    use kicker_controller::{KickTrigger, KickType, Kicker, KickerCommand};
    use kicker_programmer::KickerProgrammer;

    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    /// The amount of time (in ms) between each motion control delay
    const MOTION_CONTROL_DELAY_MS: u32 = 200_000;
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
        spi: &mut RadioSPI,
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
        spi: &mut RadioSPI,
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
        poller: imxrt_log::Poller,

        // Motor 1
        motor_one_tx: Sender<'static, [u8; 4], 3>,
        motor_one_rx: Receiver<'static, [u8; 4], 3>,

        // Motor 2
        motor_two_tx: Sender<'static, [u8; 4], 3>,
        motor_two_rx: Receiver<'static, [u8; 4], 3>,

        // Motor 3
        motor_three_tx: Sender<'static, [u8; 4], 3>,
        motor_three_rx: Receiver<'static, [u8; 4], 3>,

        // Motor 4
        motor_four_tx: Sender<'static, [u8; 4], 3>,
        motor_four_rx: Receiver<'static, [u8; 4], 3>,

        // Dribbler
        dribbler_tx: Sender<'static, [u8; 4], 3>,
        dribbler_rx: Receiver<'static, [u8; 4], 3>,
    }

    #[shared]
    struct Shared {
        // Peripherals
        pit0: Pit<0>,
        gpt: Gpt<1>,
        radio_spi: RadioSPI,
        blocking_delay: Delay2,
        rx_int: RadioInterrupt,
        gpio1: Gpio1,
        pit_delay: PitDelay,
        fake_spi: FakeSpi,
        kicker_programmer: Option<KickerProg>,
        kicker_controller: Option<Kicker<KickerCSn, KickerReset>>,

        // Drivers
        imu: Imu,
        radio: RFRadio,

        // Motion Control
        robot_status: RobotStatusMessage,
        control_message: Option<ControlMessage>,
        counter: u32,
        elapsed_time: u32,

        //Display
        display: Display,

        // State
        state: State,

        // Errors
        imu_init_error: Option<ImuInitError>,
        radio_init_error: Option<RadioInitError>,
        kicker_program_error: Option<KickerProgramError>,
        kicker_service_error: Option<KickerServicingError>,

        // Motor 1
        motor_one_uart: MotorOneUart,
        motor_one_velocity: i32,

        // Motor 2
        motor_two_uart: MotorTwoUart,
        motor_two_velocity: i32,

        // Motor 3
        motor_three_uart: MotorThreeUart,
        motor_three_velocity: i32,

        // Motor 4
        motor_four_uart: MotorFourUart,
        motor_four_velocity: i32,

        // Dribbler
        dribbler_uart: DribblerUart,
        dribbler_velocity: i32,
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
            pins,
            mut gpio1,
            mut gpio2,
            usb,
            lpi2c1,
            lpspi4,
            mut gpt1,
            mut gpt2,
            pit: (pit0, _pit1, pit2, pit3),
            lpuart1,
            lpuart4,
            lpuart6,
            lpuart7,
            lpuart8,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        // Initialize Timers //

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
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let fake_spi_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit3);

        // End Initialize Timers //

        // Initialize Motor Board //

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();

        let mut motor_one_uart = board::lpuart(lpuart6, pins.p1, pins.p0, 9600);
        motor_one_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_one_uart.clear_status(lpuart::Status::W1C);
        let (motor_one_tx, motor_one_rx) = make_channel!([u8; 4], 3);

        let mut motor_two_uart = board::lpuart(lpuart4, pins.p8, pins.p7, 9600);
        motor_two_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_two_uart.clear_status(lpuart::Status::W1C);
        let (motor_two_tx, motor_two_rx) = make_channel!([u8; 4], 3);

        let mut motor_three_uart = board::lpuart(
            lpuart1,
            pins.p24,
            pins.p25,
            9600,
        );
        motor_three_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_three_uart.clear_status(lpuart::Status::W1C);
        let (motor_three_tx, motor_three_rx) = make_channel!([u8; 4], 3);

        let mut motor_four_uart = board::lpuart(
            lpuart7,
            pins.p29,
            pins.p28,
            9600,
        );
        motor_four_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_four_uart.clear_status(lpuart::Status::W1C);
        let (motor_four_tx, motor_four_rx) = make_channel!([u8; 4], 3);

        let mut dribbler_uart = board::lpuart(lpuart8, pins.p20, pins.p21, 9600);
        dribbler_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        dribbler_uart.clear_status(lpuart::Status::W1C);
        let (dribbler_tx, dribbler_rx) = make_channel!([u8; 4], 3);

        // End Initialize Motor Board //

        delay2.delay_ms(500u32);

        // Initialize I2C Devices //

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
        let i2c_bus: &'static _ = shared_bus::new_cortexm!(
            imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1> = i2c
        )
        .expect("Failed to initialize shared I2C bus LPI2C1");
        let imu = IMU::new(i2c_bus.acquire_i2c());

        let display_interface = I2CDisplayInterface::new(i2c_bus.acquire_i2c());
        let display: Display = Ssd1306::new(
            display_interface,
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();

        // End Initialize I2C Devices //

        // Initialize Radio //

        let radio_spi_pins = Pins {
            pcs0: pins.p10,
            sck: pins.p13,
            sdo: pins.p11,
            sdi: pins.p12,
        };
        let mut radio_spi = hal::lpspi::Lpspi::new(lpspi4, radio_spi_pins);
        radio_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 5_000_000u32);
            spi.set_mode(MODE_0);
        });
        let radio_cs = gpio1.output(pins.p14);
        let ce = gpio1.output(pins.p41);
        let radio = Radio::new(ce, radio_cs);
        let rx_int = gpio2.input(pins.p9);
        gpio2.set_interrupt(&rx_int, None);

        // End Initialize Radio //

        // Initialize Kicker //

        let fake_spi = FakeSpi::new(
            gpio1.output(pins.p27),
            gpio1.output(pins.p26),
            gpio1.input(pins.p39),
            fake_spi_delay,
        );

        let kicker_controller = Kicker::new(gpio1.output(pins.p38), gpio2.output(pins.p37));

        // End Initialize Kicker //

        rx_int.clear_triggered();

        initialize_imu::spawn().ok();

        (
            Shared {
                pit0,
                gpt: gpt1,
                radio_spi,
                blocking_delay: delay2,
                rx_int,
                gpio1,
                robot_status: RobotStatusMessageBuilder::new().robot_id(ROBOT_ID).build(),
                control_message: Some(ControlMessageBuilder::new().body_y(1.0).build()),
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

                // Motor Board
                motor_one_uart,
                motor_one_velocity: 0i32,
                motor_two_uart,
                motor_two_velocity: 0i32,
                motor_three_uart,
                motor_three_velocity: 0i32,
                motor_four_uart,
                motor_four_velocity: 0i32,
                dribbler_uart,
                dribbler_velocity: 0i32,

                // Errors
                imu_init_error: None,
                radio_init_error: None,
                kicker_program_error: None,
                kicker_service_error: None,
            },
            Local {
                motion_controller: MotionControl::new(),
                poller,
                motor_one_tx,
                motor_one_rx,
                motor_two_tx,
                motor_two_rx,
                motor_three_rx,
                motor_three_tx,
                motor_four_rx,
                motor_four_tx,
                dribbler_tx,
                dribbler_rx,
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
        Systick::delay(1000.millis()).await;
        initialize_display::spawn().ok();
    }

    /// Initialize the display
    #[task(shared=[display])]
    async fn initialize_display(mut ctx: initialize_display::Context) {
        ctx.shared.display.lock(|display| {
            display.init().ok();
            display.clear();
            let start_scrn = StartScreen::new(Point::new(0, 0), Point::new(24, 8));
            let _ = start_scrn.draw(display);
            let _ = display.flush();
        });
        check_for_errors::spawn().ok();
    }

    /// Check for any errors with the peripheral drivers
    #[task(
        shared = [
            imu_init_error,
            radio_init_error,
            kicker_program_error,
            kicker_service_error,
            pit0,
        ],
        priority = 1
    )]
    async fn check_for_errors(mut ctx: check_for_errors::Context) {
        if (
            ctx.shared.imu_init_error,
            ctx.shared.radio_init_error,
            ctx.shared.kicker_program_error,
            ctx.shared.kicker_service_error,
        )
            .lock(
                |imu_init_error, radio_init_error, kicker_program_error, kicker_service_error| {
                    imu_init_error.is_some()
                        || radio_init_error.is_some()
                        || kicker_program_error.is_some()
                        || kicker_service_error.is_some()
                },
            )
        {
            error_report::spawn().ok();
        } else {
            ctx.shared.pit0.lock(|pit| {
                pit.clear_elapsed();
                pit.set_load_timer_value(MOTION_CONTROL_DELAY_MS);
                pit.set_interrupt_enable(true);
                pit.enable();
            });
        }
    }

    /// Report (eventually via the OLED Display) any errors that occurred during
    /// initialization.
    #[task(
        shared = [
            imu_init_error,
            radio_init_error,
            kicker_program_error,
            kicker_service_error,
            display,
        ],
        priority = 1
    )]
    async fn error_report(mut ctx: error_report::Context) {
        let (imu_init_error, radio_init_error, kicker_program_error, kicker_service_error) = (
            ctx.shared.imu_init_error,
            ctx.shared.radio_init_error,
            ctx.shared.kicker_program_error,
            ctx.shared.kicker_service_error,
        )
            .lock(
                |imu_init_error, radio_init_error, kicker_program_error, kicker_service_error| {
                    (
                        imu_init_error.take(),
                        radio_init_error.take(),
                        kicker_program_error.take(),
                        kicker_service_error.take(),
                    )
                },
            );

        log::error!("IMU-INIT: {:?}", imu_init_error);
        log::error!("RADIO-INIT: {:?}", radio_init_error);
        log::error!("KICKER-PROG: {:?}", kicker_program_error);
        log::error!("KICKER-SERVICE: {:?}", kicker_service_error);

        loop {
            ctx.shared.display.lock(|display| {
                let err_txt = &format!("{:?}", imu_init_error);
                let err_scrn = ErrorScreen::new("IMU Init Error", err_txt);
                display.clear();
                let _ = err_scrn.draw(display);
                display.flush().ok();
            });
            Systick::delay(3000u32.millis()).await;

            Systick::delay(3000u32.millis()).await;
            ctx.shared.display.lock(|display| {
                let err_txt = &format!("{:?}", radio_init_error);
                let err_scrn = ErrorScreen::new("Radio Init Error", err_txt);
                display.clear();
                let _ = err_scrn.draw(display);
                display.flush().ok();
            });
            Systick::delay(3000u32.millis()).await;
            ctx.shared.display.lock(|display| {
                let err_txt = &format!("{:?}", kicker_program_error);
                let err_scrn = ErrorScreen::new("Kicker Prog Error", err_txt);
                display.clear();
                let _ = err_scrn.draw(display);
                display.flush().ok();
            });
            Systick::delay(3000u32.millis()).await;
            ctx.shared.display.lock(|display| {
                let err_txt = &format!("{:?}", kicker_service_error);
                let err_scrn = ErrorScreen::new("Kicker Serv Error", err_txt);
                display.clear();
                let _ = err_scrn.draw(display);
                display.flush().ok();
            });
            Systick::delay(3000u32.millis()).await;
        }
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

        motion_control_loop::spawn().ok();
    }

    /// The motion control loop.
    ///
    /// This is the normal `main` operation of our robot.  This calculates a target velocity and
    /// moves the robot at what is believed to be the target velocity.
    ///
    /// The motion control loop is triggered by the PIT to have maximum reliability
    #[task(
        shared = [
            imu,
            control_message,
            counter,
            elapsed_time,
            rx_int,
            gpio1,
            blocking_delay,
            gpt,
            kicker_controller,
            kicker_programmer,
            robot_status,
            fake_spi,
            motor_one_uart,
            motor_one_velocity,
            motor_two_uart,
            motor_two_velocity,
            motor_three_uart,
            motor_three_velocity,
            motor_four_uart,
            motor_four_velocity,
            dribbler_uart,
        ],
        local = [
            motor_one_tx,
            motor_two_tx,
            motor_three_tx,
            motor_four_tx,
            dribbler_tx,
            motion_controller,
            initialized: bool = false,
            iteration: u32 = 0,
            last_time: u32 = 0
        ],
        priority = 1,
    )]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            *ctx.local.initialized = true;
            *ctx.local.last_time = ctx.shared.gpt.lock(|gpt| gpt.count());
        }

        log::info!("Iteration: {}", *ctx.local.iteration);

        let (body_velocities, dribbler_enabled) =
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

        let last_encoders = (
            ctx.shared.motor_one_velocity,
            ctx.shared.motor_two_velocity,
            ctx.shared.motor_three_velocity,
            ctx.shared.motor_four_velocity,
        ).lock(|one, two, three, four| Vector4::new(*one, *two, *three, *four));

        let wheel_velocities = ctx.local.motion_controller.control_update(
            Vector3::new(-accel_y, accel_x, gyro),
            last_encoders,
            body_velocities,
            delta,
        );

        ctx.shared.dribbler_uart
            .lock(|uart| send_command(if dribbler_enabled { 6200 } else { 0 }, ctx.local.dribbler_tx, uart, 0));
        ctx.shared.motor_one_uart
            .lock(|uart| send_command(wheel_velocities[0], ctx.local.motor_one_tx, uart, 0));
        ctx.shared.motor_two_uart
            .lock(|uart| send_command(wheel_velocities[1], ctx.local.motor_two_tx, uart, 0));
        ctx.shared.motor_three_uart
            .lock(|uart| send_command(wheel_velocities[2], ctx.local.motor_three_tx, uart, 0));
        ctx.shared.motor_four_uart
            .lock(|uart| send_command(wheel_velocities[3], ctx.local.motor_four_tx, uart, 0));

        // #[cfg(feature = "debug")]
        //log::info!("Moving at {:?}", wheel_velocities);

        *ctx.local.iteration = ctx.local.iteration.wrapping_add(1);
    }

    #[task(
        shared = [
            motor_one_uart,
            motor_one_velocity,
        ],
        local = [
            motor_one_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART6,
    )]
    // Interrupt driven task to actually send and receive data via the uart
    fn motor_one_uart(ctx: motor_one_uart::Context) {
        motor_interrupt(
            ctx.shared.motor_one_uart,
            ctx.shared.motor_one_velocity,
            ctx.local.motor_one_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    #[task(
        shared = [
            motor_two_uart,
            motor_two_velocity,
        ],
        local = [
            motor_two_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART4
    )]
    fn motor_two_uart(ctx: motor_two_uart::Context) {
        motor_interrupt(
            ctx.shared.motor_two_uart,
            ctx.shared.motor_two_velocity,
            ctx.local.motor_two_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    #[task(
        shared = [
            motor_three_uart,
            motor_three_velocity,
        ],
        local = [
            motor_three_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART1
    )]
    fn motor_three_uart(ctx: motor_three_uart::Context) {
        motor_interrupt(
            ctx.shared.motor_three_uart,
            ctx.shared.motor_three_velocity,
            ctx.local.motor_three_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    #[task(
        shared = [
            motor_four_uart,
            motor_four_velocity,
        ],
        local = [
            motor_four_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART7
    )]
    fn motor_four_uart(ctx: motor_four_uart::Context) {
        motor_interrupt(
            ctx.shared.motor_four_uart,
            ctx.shared.motor_four_velocity,
            ctx.local.motor_four_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    #[task(
        shared = [
            dribbler_uart,
            dribbler_velocity,
        ],
        local = [
            dribbler_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART8
    )]
    fn dribbler_uart(ctx: dribbler_uart::Context) {
        motor_interrupt(
            ctx.shared.dribbler_uart,
            ctx.shared.dribbler_velocity,
            ctx.local.dribbler_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
