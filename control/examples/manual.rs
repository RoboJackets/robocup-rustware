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

    use imxrt_iomuxc::prelude::*;

    use embedded_hal::{blocking::delay::DelayMs, spi::MODE_0};

    use bsp::board::PERCLK_FREQUENCY;
    use bsp::board::{self, LPSPI_FREQUENCY};
    use nalgebra::{Vector3, Vector4};
    use teensy4_bsp as bsp;

    use hal::gpio::Trigger;
    use hal::lpspi::{Lpspi, Pins};
    use hal::pit::Chained01;
    use hal::timer::Blocking;
    use teensy4_bsp::hal;

    use bsp::ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::Radio;

    use rtic_monotonics::systick::*;

    use ncomm_utils::packing::Packable;

    use robojackets_robocup_control::robot::TEAM_NUM;
    use robojackets_robocup_rtp::BASE_STATION_ADDRESSES;
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

    use robojackets_robocup_control::{
        Delay2, FPGAInitError, FPGAProgError, Fpga, Gpio1, Imu, ImuInitError, PitDelay, RFRadio,
        RadioInitError, RadioInterrupt, SharedSPI, BASE_AMPLIFICATION_LEVEL, CHANNEL,
        GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY, RADIO_ADDRESS, ROBOT_ID,
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 200;
    const DIE_TIME_US: u64 = 250_000;

    #[local]
    struct Local {
        motion_controller: MotionControl,
        last_encoders: Vector4<f32>,
        chain_timer: Chained01,
    }

    #[shared]
    struct Shared {
        // Peripherals
        shared_spi: SharedSPI,
        delay2: Delay2,
        rx_int: RadioInterrupt,
        gpio1: Gpio1,
        pit_delay: PitDelay,

        // Drivers
        fpga: Fpga,
        imu: Imu,
        radio: RFRadio,

        // Motion Control
        robot_status: RobotStatusMessage,
        control_message: Option<ControlMessage>,
        counter: u32,
        elapsed_time: u64,

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
            mut gpt2,
            pit: (pit0, pit1, pit2, _pit3),
            ..
        } = board::t41(ctx.device);

        // Setup USB Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        // Chained Pit<0> and Pit<1>
        let mut chained_timer = Chained01::new(pit0, pit1);
        chained_timer.enable();

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

        rx_int.clear_triggered();

        initialize_imu::spawn().ok();

        (
            Shared {
                shared_spi,
                delay2,
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

                // Errors
                imu_init_error: None,
                radio_init_error: None,
                fpga_prog_error: None,
                fpga_init_error: None,
            },
            Local {
                motion_controller: MotionControl::new(),
                last_encoders: Vector4::zeros(),
                chain_timer: chained_timer,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

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

    #[task(
        shared = [radio, shared_spi, delay2, radio_init_error],
        priority = 1
    )]
    async fn initialize_radio(ctx: initialize_radio::Context) {
        (
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.delay2,
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

    #[task(
        shared = [fpga, pit_delay, imu_init_error, fpga_init_error, fpga_prog_error, radio_init_error, delay2],
        priority = 1
    )]
    async fn initialize_fpga(ctx: initialize_fpga::Context) {
        let fully_initialized = (
            ctx.shared.fpga,
            ctx.shared.pit_delay,
            ctx.shared.imu_init_error,
            ctx.shared.fpga_init_error,
            ctx.shared.fpga_prog_error,
            ctx.shared.radio_init_error,
            ctx.shared.delay2,
        )
            .lock(
                |fpga,
                 pit_delay,
                 imu_init_error,
                 fpga_init_error,
                 fpga_prog_error,
                 radio_init_error,
                 delay| {
                    if let Err(err) = fpga.configure(delay) {
                        *fpga_prog_error = Some(err);
                        return false;
                    }
                    pit_delay.delay_ms(10u8);

                    if let Err(err) = fpga.motors_en(true, delay) {
                        *fpga_init_error = Some(err);
                        return false;
                    }

                    imu_init_error.is_none() && radio_init_error.is_none()
                },
            );

        if fully_initialized {
            start_listening::spawn().ok();
        } else {
            error_report::spawn().ok();
        }
    }

    #[task(
        shared = [radio, shared_spi, delay2, gpio1, rx_int],
        priority = 1
    )]
    async fn start_listening(ctx: start_listening::Context) {
        (
            ctx.shared.radio,
            ctx.shared.shared_spi,
            ctx.shared.delay2,
            ctx.shared.gpio1,
            ctx.shared.rx_int,
        )
            .lock(|radio, spi, delay, gpio1, rx_int| {
                rx_int.clear_triggered();
                gpio1.set_interrupt(rx_int, Some(Trigger::FallingEdge));
                radio.start_listening(spi, delay);
            });

        motion_control_loop::spawn().ok();
    }

    #[task(binds = GPIO1_COMBINED_16_31, shared = [rx_int, gpio1, elapsed_time], priority = 2)]
    fn radio_interrupt(ctx: radio_interrupt::Context) {
        if (ctx.shared.rx_int, ctx.shared.gpio1, ctx.shared.elapsed_time).lock(
            |rx_int, gpio1, elapsed_time| {
                // ctx.local.led_pin.toggle();
                if rx_int.is_triggered() {
                    *elapsed_time = 0;
                    rx_int.clear_triggered();
                    gpio1.set_interrupt(&rx_int, None);
                    return true;
                }
                false
            },
        ) {
            receive_command::spawn().ok();
        }
    }

    #[task(
        shared = [radio, rx_int, gpio1, shared_spi, delay2, control_message, robot_status, counter],
        priority = 2
    )]
    async fn receive_command(ctx: receive_command::Context) {
        (
            ctx.shared.shared_spi,
            ctx.shared.delay2,
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
            gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));
        });
    }

    #[task(
        shared = [imu, control_message, counter, elapsed_time, fpga, rx_int, gpio1, delay2],
        local = [motion_controller, last_encoders, chain_timer, initialized: bool = false, iteration: u32 = 0, last_time: u64 = 0],
        priority = 1
    )]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            *ctx.local.initialized = true;
            *ctx.local.last_time = ctx.local.chain_timer.current_timer_value();
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

        let now = ctx.local.chain_timer.current_timer_value();
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
            delta as u32,
        );

        #[cfg(feature = "debug")]
        log::info!("Moving at {:?}", wheel_velocities);

        let encoder_velocities = (ctx.shared.fpga, ctx.shared.delay2).lock(|fpga, delay| {
            let encoder_velocities =
                match fpga.set_velocities(wheel_velocities.into(), dribbler_enabled, delay) {
                    Ok(encoder_velocities) => encoder_velocities,
                    Err(_err) => {
                        #[cfg(feature = "debug")]
                        log::info!("Unable to Read Encoder Values");
                        [0.0; 4]
                    }
                };

            #[cfg(feature = "debug")]
            log::info!("Fpga Status: {:#010b}", fpga.status);

            encoder_velocities
        });

        *ctx.local.last_encoders = Vector4::new(
            encoder_velocities[0],
            encoder_velocities[1],
            encoder_velocities[2],
            encoder_velocities[3],
        );

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_loop::spawn().ok();
    }

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
            log::error!(
                "IMU-INIT: {:?}\nFPGA-PROG: {:?}\nFPGA-INIT: {:?}\nRADIO-INIT: {:?}",
                imu_init_error,
                fpga_prog_error,
                fpga_init_error,
                radio_init_error
            );
            Systick::delay(1_000u32.millis()).await;
        }

        panic!(
            "IMU-INIT: {:?}\nFPGA-PROG: {:?}\nFPGA-INIT: {:?}\nRADIO-INIT: {:?}",
            imu_init_error, fpga_prog_error, fpga_init_error, radio_init_error
        );
    }
}
