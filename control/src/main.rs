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

    use teensy4_pins::t41::{P32, P33, P34, P35};

    use core::mem::MaybeUninit;
    use once_cell::sync::OnceCell;

    use imxrt_hal::gpt::Gpt;
    use imxrt_hal::pit::Pit;

    use embedded_hal::{blocking::delay::DelayMs, spi::MODE_0};

    use bsp::board::PERCLK_FREQUENCY;
    use bsp::board::{self, LPSPI_FREQUENCY};
    use nalgebra::Vector3;
    use robojackets_robocup_control::{
        Adc1, Gpio1, Gpio2, Killn, MotorEn, PowerSwitch, RadioSPI, MIN_BATTERY_VOLTAGE,
    };
    use teensy4_bsp as bsp;

    use hal::adc::AnalogInput;
    use hal::gpio::{Input, Trigger};
    use hal::lpspi::{Lpspi, Pins};
    use hal::lpuart;
    use hal::timer::Blocking;
    use teensy4_bsp::hal;

    use bsp::ral::lpspi::LPSPI3;
    use embedded_hal::spi::MODE_3;
    use hal::iomuxc;

    use rtic_nrf24l01::Radio;

    use rtic_monotonics::systick::*;

    use shared_bus;

    // Includes for display module
    use graphics::screen::Screen;
    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
    use teensy4_pins::t41::{P18, P19};

    use robojackets_robocup_rtp::{BASE_STATION_ADDRESSES, ROBOT_RADIO_ADDRESSES};

    use motion::MotionControl;

    use icm42605_driver::IMU;

    use robojackets_robocup_control::{
        motors::{motor_interrupt, send_command},
        selector::{get_team_and_id, PIN_CONFIG},
        radio::{Team, robot_status::{RobotStatusMessage, RobotStatusMessageBuilder}, control_message::ControlMessage},
        Delay2, Display, DribblerUart, Imu, ImuInitError, KickerCSn,
        KickerReset, KickerServicingError, KickerSpi, MotorFourUart,
        MotorOneUart, MotorThreeUart, MotorTwoUart, PitDelay, RFRadio, RadioInitError,
        RadioInterrupt, BASE_AMPLIFICATION_LEVEL, CHANNEL, GPT_1_DIVIDER, GPT_CLOCK_SOURCE,
        GPT_DIVIDER, GPT_FREQUENCY,
    };

    use kicker_controller::{KickTrigger, KickType, Kicker, KickerCommand};

    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };
    use teensy4_pins::tmm::P15;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    static TEAM: OnceCell<Team> = OnceCell::new();
    static ROBOT_ID: OnceCell<u8> = OnceCell::new();

    /// The amount of time (in us) between each motion control delay
    const MOTION_CONTROL_DELAY_US: u32 = 1_000_000 / 60;
    /// The amount of time (in us) between each not-time-critical interrupt
    const NON_CRITICAL_DELAY_US: u32 = 1_000_000 / 2;
    /// The amount of time (in ms) between servicing the kicker
    const KICKER_SERVICE_DELAY_US: u32 = 50_000;
    /// The number of motion control updates between servicing the kicker
    const KICKER_SERVICE_DELAY_TICKS: u32 = KICKER_SERVICE_DELAY_US / MOTION_CONTROL_DELAY_US;
    /// The amount of time the robot should continue moving without receiving a
    /// new message from the base station before it stops moving
    const DIE_TIME_US: u32 = 1_000_000;
    // Number of consecutive readings under voltage threshold before shutting down
    const BATT_UVLO_THRESHOLD: u32 = 10;

    /// Helper method to disable radio interrupts and prepare to
    /// send messages of `message_size` over the radio
    pub fn disable_radio_interrupts(
        message_size: usize,
        rx_int: &mut RadioInterrupt,
        gpio2: &mut Gpio2,
        radio: &mut RFRadio,
        spi: &mut RadioSPI,
        radio_delay: &mut Delay2,
    ) {
        rx_int.clear_triggered();
        gpio2.set_interrupt(rx_int, None);
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
        gpio2: &mut Gpio2,
        radio: &mut RFRadio,
        spi: &mut RadioSPI,
        radio_delay: &mut Delay2,
    ) {
        rx_int.clear_triggered();
        gpio2.set_interrupt(rx_int, Some(Trigger::FallingEdge));
        radio.clear_interrupts(spi, radio_delay);
        radio.flush_rx(spi, radio_delay);
        radio.flush_tx(spi, radio_delay);
        radio.set_payload_size(ControlMessage::size() as u8, spi, radio_delay);
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

        // Battery low voltage checks
        batt_uvlo_counter: u32,

        hex0: Input<P35>,
        hex1: Input<P34>,
        hex2: Input<P33>,
        hex3: Input<P32>,
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
        gpio2: Gpio2,
        pit_delay: PitDelay,
        kicker_spi: KickerSpi,
        kicker_controller: Kicker<KickerCSn, KickerReset>,

        kill_n: Killn,
        power_switch: PowerSwitch,

        // Drivers
        imu: Imu,
        radio: RFRadio,

        // Motion Control
        robot_status: RobotStatusMessage,
        control_message: Option<ControlMessage>,
        counter: u32,
        elapsed_time: u32,

        // Periodic Interupt for non-time critical stuff
        pit1: Pit<1>,
        adc1: Adc1,
        batt_sense: AnalogInput<P15, 1>,
        //Display
        screen: Screen<'static, Display>,

        // Errors
        imu_init_error: Option<ImuInitError>,
        radio_init_error: Option<RadioInitError>,
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
            mut pins,
            mut gpio1,
            mut gpio2,
            mut gpio4,
            usb,
            lpi2c1,
            lpspi4,
            mut gpt1,
            mut gpt2,
            pit: (pit0, pit1, pit2, _pit3),
            lpuart1,
            lpuart4,
            lpuart6,
            lpuart7,
            lpuart8,
            mut adc1,
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

        // End Initialize Timers //

        // Initialize Motor Board //

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();
        let power_switch: PowerSwitch = gpio1.input(pins.p40);

        delay2.delay_ms(1_500u32);

        let mut motor_one_uart = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);
        motor_one_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_one_uart.clear_status(lpuart::Status::W1C);
        let (motor_one_tx, motor_one_rx) = make_channel!([u8; 4], 3);

        let mut motor_two_uart = board::lpuart(lpuart4, pins.p8, pins.p7, 115200);
        motor_two_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_two_uart.clear_status(lpuart::Status::W1C);
        let (motor_two_tx, motor_two_rx) = make_channel!([u8; 4], 3);

        let mut motor_three_uart = board::lpuart(lpuart1, pins.p24, pins.p25, 115200);
        motor_three_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_three_uart.clear_status(lpuart::Status::W1C);
        let (motor_three_tx, motor_three_rx) = make_channel!([u8; 4], 3);

        let mut motor_four_uart = board::lpuart(lpuart7, pins.p29, pins.p28, 115200);
        motor_four_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_four_uart.clear_status(lpuart::Status::W1C);
        let (motor_four_tx, motor_four_rx) = make_channel!([u8; 4], 3);

        let mut dribbler_uart = board::lpuart(lpuart8, pins.p20, pins.p21, 115200);
        dribbler_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        dribbler_uart.clear_status(lpuart::Status::W1C);
        let (dribbler_tx, dribbler_rx) = make_channel!([u8; 4], 3);

        // End Initialize Motor Board //

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

        let screen = Screen::new(0, true, display);

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

        // Define SPI pins
        let kicker_spi_pins = Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };

        // Generate instance of LPSPI3 to manually populate
        let kicker_spi_block = unsafe { LPSPI3::instance() };
        let kicker_spi_temp = Lpspi::new(kicker_spi_block, kicker_spi_pins);

        // Release pins to split into kicker
        let (kicker_spi_block, mut kicker_spi_pins) = kicker_spi_temp.release();

        // Manually configure the data pins for LPSPI3 function
        iomuxc::lpspi::prepare(&mut kicker_spi_pins.sdo); // SDO/MOSI
        iomuxc::lpspi::prepare(&mut kicker_spi_pins.sdi); // SDI/MISO
        iomuxc::lpspi::prepare(&mut kicker_spi_pins.sck); // SCK

        // Initialize SPI and Kicker controll
        let mut kicker_spi = Lpspi::without_pins(kicker_spi_block);

        // Config SPI
        kicker_spi.disabled(|kicker_spi| {
            kicker_spi.set_mode(MODE_3); // CPOL=1, CPHA=1 to match Pico
            kicker_spi.set_clock_hz(board::LPSPI_FREQUENCY, 2_000_000);
        });

        let kicker_controller =
            Kicker::new(gpio1.output(kicker_spi_pins.pcs0), gpio2.output(pins.p37));

        // End Initialize Kicker //

        // Initialize UVLO //
        let batt_uvlo_counter = 0;
        // End Initialize UVLO //

        // Read the dip switch
        iomuxc::configure(&mut pins.p35, PIN_CONFIG);
        let hex0 = gpio2.input(pins.p35);
        iomuxc::configure(&mut pins.p34, PIN_CONFIG);
        let hex1 = gpio2.input(pins.p34);
        iomuxc::configure(&mut pins.p33, PIN_CONFIG);
        let hex2 = gpio4.input(pins.p33);
        iomuxc::configure(&mut pins.p32, PIN_CONFIG);
        let hex3 = gpio2.input(pins.p32);

        adc1.calibrate();
        let mut batt_sense = AnalogInput::new(pins.p15);
        adc1.read_blocking(&mut batt_sense);

        rx_int.clear_triggered();

        initialize_imu::spawn().ok();

        (
            Shared {
                pit0,
                pit1,
                gpt: gpt1,
                radio_spi,
                blocking_delay: delay2,
                rx_int,
                gpio1,
                gpio2,
                robot_status: RobotStatusMessageBuilder::new().robot_id(0).build(),
                control_message: None,
                counter: 0,
                elapsed_time: 0,
                radio,
                imu,
                pit_delay,
                kicker_controller,
                screen,
                kicker_spi,
                adc1,
                batt_sense,

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
                power_switch,
                kill_n,

                // Errors
                imu_init_error: None,
                radio_init_error: None,
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
                batt_uvlo_counter,
                hex0,
                hex1,
                hex2,
                hex3,
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
    #[task(shared=[screen], local=[hex0, hex1, hex2, hex3], priority = 1)]
    async fn initialize_display(mut ctx: initialize_display::Context) {
        let (team, id) = get_team_and_id(
            &ctx.local.hex0,
            &ctx.local.hex1,
            &ctx.local.hex2,
            &ctx.local.hex3,
        );
        TEAM.set(team).ok();
        ROBOT_ID.set(id).ok();
        ctx.shared.screen.lock(|screen| {
            screen.set_team(team == Team::Blue);
            screen.set_robot_id(id);
            screen.init_display().ok();
            screen.draw().ok();
        });
        initialize_radio::spawn().ok();
    }

    /// Initialize the nRF24l01 Radio
    #[task(
        shared = [radio, radio_spi, blocking_delay, radio_init_error],
        priority = 1
    )]
    async fn initialize_radio(ctx: initialize_radio::Context) {
        (
            ctx.shared.radio,
            ctx.shared.radio_spi,
            ctx.shared.blocking_delay,
            ctx.shared.radio_init_error,
        )
            .lock(
                |radio, spi, delay, radio_init_error| match radio.begin(spi, delay) {
                    Ok(_) => {
                        radio.set_pa_level(BASE_AMPLIFICATION_LEVEL, spi, delay);
                        radio.set_channel(CHANNEL, spi, delay);
                        radio.set_payload_size(ControlMessage::size() as u8, spi, delay);
                        radio.open_writing_pipe(
                            BASE_STATION_ADDRESSES[*TEAM.get().unwrap() as usize],
                            spi,
                            delay,
                        );
                        radio.open_reading_pipe(
                            1,
                            ROBOT_RADIO_ADDRESSES[*TEAM.get().unwrap() as usize]
                                [*ROBOT_ID.get().unwrap() as usize],
                            spi,
                            delay,
                        );
                        radio.stop_listening(spi, delay);
                    }
                    Err(err) => *radio_init_error = Some(err),
                },
            );

        check_for_errors::spawn().ok();
    }

    /// Check for any errors with the peripheral drivers
    #[task(
        shared = [
            imu_init_error,
            radio_init_error,
            kicker_service_error,
            gpio1,
            power_switch
        ],
        priority = 1
    )]
    async fn check_for_errors(ctx: check_for_errors::Context) {
        if (
            ctx.shared.imu_init_error,
            ctx.shared.radio_init_error,
            ctx.shared.kicker_service_error,
        )
            .lock(|imu_init_error, radio_init_error, kicker_service_error| {
                imu_init_error.is_some()
                    || radio_init_error.is_some()
                    || kicker_service_error.is_some()
            })
        {
            error_report::spawn().ok();
        } else {
            start_listening::spawn().ok();
        }

        //start the suicide watch
        (ctx.shared.gpio1, ctx.shared.power_switch).lock(|gpio1, power_switch| {
            gpio1.set_interrupt(&power_switch, Some(Trigger::Low));
        });
    }

    /// Report (eventually via the OLED Display) any errors that occurred during
    /// initialization.
    #[task(
        shared = [
            imu_init_error,
            radio_init_error,
            kicker_service_error,
            screen,
        ],
        priority = 1
    )]
    async fn error_report(mut ctx: error_report::Context) {
        let (imu_init_error, radio_init_error, kicker_service_error) = (
            ctx.shared.imu_init_error,
            ctx.shared.radio_init_error,
            ctx.shared.kicker_service_error,
        )
            .lock(|imu_init_error, radio_init_error, kicker_service_error| {
                (
                    imu_init_error.take(),
                    radio_init_error.take(),
                    kicker_service_error.take(),
                )
            });

        log::error!("IMU-INIT: {:?}", imu_init_error);
        log::error!("RADIO-INIT: {:?}", radio_init_error);
        log::error!("KICKER-SERVICE: {:?}", kicker_service_error);

        loop {
            log::error!("IMU-INIT: {:?}", imu_init_error);
            log::error!("RADIO-INIT: {:?}", radio_init_error);
            log::error!("KICKER-SERVICE: {:?}", kicker_service_error);

            ctx.shared.screen.lock(|screen| {
                let err_txt = format!("{:?}", imu_init_error);
                screen.error_update("IMU Init Error", err_txt);
                screen.draw().ok();
            });
            Systick::delay(3000u32.millis()).await;

            Systick::delay(3000u32.millis()).await;
            ctx.shared.screen.lock(|screen| {
                let err_txt = format!("{:?}", radio_init_error);
                screen.error_update("Radio Init Error", err_txt);
                screen.draw().ok();
            });
            Systick::delay(3000u32.millis()).await;
            let err_txt = format!("{:?}", kicker_service_error);
            ctx.shared.screen.lock(move |screen| {
                screen.error_update("Kicker Serv Error", err_txt);
                screen.draw().ok();
            });
            Systick::delay(3000u32.millis()).await;
        }
    }

    /// Have the radio start listening for incoming commands
    #[task(
        shared = [radio, radio_spi, blocking_delay, gpio1, power_switch, gpio2, rx_int, pit0, pit1],
        priority = 1
    )]
    async fn start_listening(mut ctx: start_listening::Context) {
        (
            ctx.shared.radio,
            ctx.shared.radio_spi,
            ctx.shared.blocking_delay,
            ctx.shared.gpio2,
            ctx.shared.rx_int,
        )
            .lock(|radio, spi, delay, gpio2, rx_int| {
                rx_int.clear_triggered();
                gpio2.set_interrupt(rx_int, Some(Trigger::FallingEdge));
                radio.start_listening(spi, delay);
            });

        ctx.shared.pit0.lock(|pit| {
            pit.clear_elapsed();
            pit.set_load_timer_value(MOTION_CONTROL_DELAY_US);
            pit.set_interrupt_enable(true);
            pit.enable();
        });

        ctx.shared.pit1.lock(|pit| {
            pit.clear_elapsed();
            pit.set_load_timer_value(NON_CRITICAL_DELAY_US);
            pit.set_interrupt_enable(true);
            pit.enable();
        })
    }

    /// Interrupt called when the radio receives new data
    #[task(binds = GPIO2_COMBINED_0_15, shared = [rx_int, gpio2, elapsed_time], priority = 2)]
    fn radio_interrupt(ctx: radio_interrupt::Context) {
        if (ctx.shared.rx_int, ctx.shared.gpio2, ctx.shared.elapsed_time).lock(
            |rx_int, gpio2, elapsed_time| {
                if rx_int.is_triggered() {
                    *elapsed_time = 0;
                    rx_int.clear_triggered();
                    gpio2.set_interrupt(rx_int, None);
                    return true;
                }
                false
            },
        ) {
            receive_command::spawn().ok();
        }
    }

    /// This task kills the motor board when the power switch is pressed.
    #[task(binds = GPIO1_COMBINED_16_31)]
    fn power_switch_interrupt(_: power_switch_interrupt::Context) {
        kill_self::spawn().ok();
    }

    #[task(shared=[power_switch, kill_n,gpio2, rx_int, control_message, kicker_controller, kicker_spi, blocking_delay],priority=2)]
    async fn kill_self(mut ctx: kill_self::Context) {
        // Disable new radio events from coming in by disabling the interrupt
        (ctx.shared.gpio2, ctx.shared.rx_int).lock(|gpio2, rx_int| {
            gpio2.set_interrupt(rx_int, None);
        });
        log::info!("Radio Disabled!");

        // Set Control Message to stop all movement
        (ctx.shared.control_message).lock(|message| {
            *message = None;
        });
        log::info!("Freezed Motors!");

        //trigger kicker repeatedly to force discharge
        (
            ctx.shared.kicker_controller,
            ctx.shared.kicker_spi,
            ctx.shared.blocking_delay,
        )
            .lock(|kicker, kicker_spi, delay| {
                let command = KickerCommand {
                    kick_type: KickType::Kick,
                    kick_trigger: KickTrigger::Immediate,
                    kick_strength: 10,
                    charge_allowed: false,
                };
                kicker.service(command, kicker_spi).unwrap();
                delay.delay_ms(100u32);
            });
        log::info!("Discharged Kicker!");

        (ctx.shared.kill_n).lock(|kill_n| {
            kill_n.clear();
        });
        log::info!("Killed Motor Board!");
    }

    /// Task that decodes the command received via the radio.  This task is directly
    /// spawned by the radio_interrupt hardware task.
    #[task(
        shared = [radio, radio_spi, blocking_delay, control_message, counter],
        local = [read_buffer: [u8; ControlMessage::size()] = [0u8; ControlMessage::size()]],
        priority = 2
    )]
    async fn receive_command(ctx: receive_command::Context) {
        // Receive new command
        (
            ctx.shared.radio_spi,
            ctx.shared.blocking_delay,
            ctx.shared.control_message,
            ctx.shared.counter,
            ctx.shared.radio
        ).lock(|spi, delay, control_message, counter, radio| {
            radio.read(ctx.local.read_buffer, spi, delay);
            *control_message = Some(ControlMessage::unpack(ctx.local.read_buffer).unwrap());
            *counter = 0;
        });

        ctx.local.read_buffer.iter_mut().for_each(|byte| *byte = 0);
        Systick::delay(5.millis()).await;
    }

    #[task(
        shared = [radio, rx_int, gpio2, radio_spi, blocking_delay, robot_status],
        local = [status_buffer: [u8; RobotStatusMessage::size()] = [0u8; RobotStatusMessage::size()]],
        priority = 2
    )]
    async fn send_robot_status(ctx: send_robot_status::Context) {
        (
            ctx.shared.radio,
            ctx.shared.radio_spi,
            ctx.shared.blocking_delay,
            ctx.shared.robot_status
        ).lock(|radio, spi, delay, robot_status| {
            robot_status.robot_id = *ROBOT_ID.get().unwrap();
            robot_status.pack(ctx.local.status_buffer).unwrap();

            radio.set_payload_size(RobotStatusMessage::size() as u8, spi, delay);
            radio.stop_listening(spi, delay);

            let _ = radio.write(ctx.local.status_buffer, spi, delay);

            radio.set_payload_size(ControlMessage::size() as u8, spi, delay);
            radio.start_listening(spi, delay);
        });

        (ctx.shared.rx_int, ctx.shared.gpio2).lock(|rx_int, gpio2| {
            rx_int.clear_triggered();
            gpio2.set_interrupt(rx_int, Some(Trigger::Low));
        });

        ctx.local.status_buffer.iter_mut().for_each(|byte| *byte = 0);
    }

    /// Tick task that is called by the motion control and non-critical PIT  timers
    #[task(
        shared = [pit0, pit1],
        priority = 1,
        binds = PIT
    )]
    fn tick(mut ctx: tick::Context) {
        if ctx.shared.pit0.lock(|pit| {
            let elapsed = pit.is_elapsed();
            if elapsed {
                pit.clear_elapsed();
                pit.set_load_timer_value(MOTION_CONTROL_DELAY_US);
                pit.set_interrupt_enable(true);
                pit.enable();
            }
            elapsed
        }) {
            if motion_control_loop::spawn().is_err() {
                log::error!("Motion Control Loop Already Running");
            }
        }

        if ctx.shared.pit1.lock(|pit| {
            let elapsed = pit.is_elapsed();
            if elapsed {
                pit.clear_elapsed();
                pit.set_load_timer_value(NON_CRITICAL_DELAY_US);
                pit.set_interrupt_enable(true);
                pit.enable();
            }
            elapsed
        }) {
            non_critical_task::spawn().ok();
        }
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
            gpio2,
            blocking_delay,
            gpt,
            kicker_controller,
            robot_status,
            kicker_spi,
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
            last_body_velocities: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0),
        ],
        priority = 1,
    )]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            *ctx.local.initialized = true;
            *ctx.local.last_time = ctx.shared.gpt.lock(|gpt| gpt.count());
        }

        let (mut body_velocities, dribbler_speed) =
            ctx.shared
                .control_message
                .lock(|control_message| match control_message {
                    Some(control_message) => (
                        control_message.get_velocity(),
                        control_message.dribbler_speed,
                    ),
                    None => (Vector3::new(0.0, 0.0, 0.0), 0),
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

        if body_velocities != *ctx.local.last_body_velocities {
            *ctx.local.last_body_velocities = body_velocities;
            let wheel_velocities = ctx.local.motion_controller.body_to_wheels(body_velocities);

            ctx.shared
                .dribbler_uart
                .lock(|uart| send_command(dribbler_speed as i32, ctx.local.dribbler_tx, uart, 0));
            ctx.shared
                .motor_one_uart
                .lock(|uart| send_command(wheel_velocities[0], ctx.local.motor_one_tx, uart, 0));
            ctx.shared
                .motor_two_uart
                .lock(|uart| send_command(wheel_velocities[1], ctx.local.motor_two_tx, uart, 0));
            ctx.shared
                .motor_three_uart
                .lock(|uart| send_command(wheel_velocities[2], ctx.local.motor_three_tx, uart, 0));
            ctx.shared
                .motor_four_uart
                .lock(|uart| send_command(wheel_velocities[3], ctx.local.motor_four_tx, uart, 0));
        }

        #[cfg(feature = "debug")]
        log::info!("Moving at {:?}", wheel_velocities);

        // Service the kicker
        if *ctx.local.iteration % KICKER_SERVICE_DELAY_TICKS == 0 {
            let mut kicker_command = ctx.shared.control_message.lock(|control_message| {
                if let Some(control_message) = control_message {
                    (*control_message).into()
                } else {
                    KickerCommand::default()
                }
            });
            kicker_command.charge_allowed = true;
            (
                ctx.shared.kicker_controller,
                ctx.shared.robot_status,
                ctx.shared.kicker_spi,
            )
                .lock(|controller, robot_status, kicker_spi| {
                    let state = controller.service(kicker_command, kicker_spi).unwrap();
                    robot_status.kick_status =
                        kicker_command.kick_trigger != KickTrigger::Disabled;
                    robot_status.ball_sense_status = state.ball_sensed;
                    robot_status.kick_healthy = state.healthy;
                });
        }

        *ctx.local.iteration = ctx.local.iteration.wrapping_add(1);
    }

    #[task(
        shared = [screen, adc1, batt_sense, robot_status],
        local = [batt_uvlo_counter],
        priority = 1
    )]
    async fn non_critical_task(mut ctx: non_critical_task::Context) {
        let battery_sense = (ctx.shared.adc1, ctx.shared.batt_sense)
            .lock(|adc, batt_sense| adc.read_blocking(batt_sense));
        let battery_voltage = (battery_sense as f32) * 3.3 / 1023.0;
        log::info!("Battery Voltage: {}", battery_voltage);

        // Maximum Voltage of batteries is roughly 2.69, so we're making a random
        // linear interpolation between the max and min voltage
        let battery_percent = unsafe {
            ((battery_voltage - MIN_BATTERY_VOLTAGE) / (2.69 - MIN_BATTERY_VOLTAGE) * 100.0)
                .to_int_unchecked()
        };

        let _status = ctx.shared.robot_status.lock(|robot_status| {
            robot_status.battery_voltage = battery_percent;
            robot_status.clone()
        });

        // // Battery is under voltaged so we should die
        if battery_voltage < MIN_BATTERY_VOLTAGE {
            *ctx.local.batt_uvlo_counter += 1;
            if *ctx.local.batt_uvlo_counter >= BATT_UVLO_THRESHOLD {
                kill_self::spawn().ok();
            }
        } else {
            *ctx.local.batt_uvlo_counter = 0;
        }

        ctx.shared.screen.lock(|screen| {
            screen.main_loop_update(
                _status.battery_voltage.into(),
                _status.kick_status,
                _status.ball_sense_status,
                0,
            );
            screen.draw().ok();
        });
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
