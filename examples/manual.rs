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

    use core::convert::Infallible;
    use core::mem::MaybeUninit;

    use imxrt_iomuxc::prelude::*;

    use main::ROBOT_ID;

    use embedded_hal::spi::MODE_0;

    use nalgebra::{Vector3, Vector4};
    use teensy4_bsp as bsp;
    use bsp::board::{self, LPSPI_FREQUENCY};
    use bsp::board::{Lpi2c1, PERCLK_FREQUENCY};

    use teensy4_pins::t41::*;

    use teensy4_bsp::hal as hal;
    use hal::lpspi::{LpspiError, Lpspi, Pins};
    use hal::gpio::{Output, Input, Trigger, Port};
    use hal::gpt::{ClockSource, Gpt1, Gpt2};
    use hal::timer::Blocking;
    use hal::pit::Chained01;
    
    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::{Radio, config::*};

    use rtic_monotonics::systick::*;

    use packed_struct::prelude::*;

    use robojackets_robocup_rtp::{ControlMessage, CONTROL_MESSAGE_SIZE};
    use robojackets_robocup_rtp::{RobotStatusMessage, RobotStatusMessageBuilder, ROBOT_STATUS_SIZE};
    use robojackets_robocup_rtp::{BASE_STATION_ADDRESS, ROBOT_RADIO_ADDRESSES};

    use motion::MotionControl;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    use icm42605_driver::IMU;

    // Constants
    const GPT_FREQUENCY: u32 = 1_000;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 1_000;
    const MAX_COUNTER_VALUE: u32 = 100;

    // Type Definitions
    // FPGA Spi
    type SPI = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
    type Fpga = FPGA<SPI, Output<P9>, P29, Output<P28>, P30, Delay1, hal::lpspi::LpspiError, Infallible>;
    // Shared Spi
    type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
    type RadioCE = Output<P0>;
    type RadioCSN = Output<P6>;
    type RadioInterrupt = Input<P1>;
    // Delays
    type Delay1 = Blocking<Gpt1, GPT_FREQUENCY>;
    type Delay2 = Blocking<Gpt2, GPT_FREQUENCY>;
    // GPIO Ports
    type Gpio1 = Port<1>;
    // IMU
    type Imu = IMU<Lpi2c1>;

    #[local]
    struct Local {
        radio: Radio<RadioCE, RadioCSN, SharedSPI, Delay2, Infallible, LpspiError>,
        motion_controller: MotionControl,
        fpga: Fpga,
        imu: Imu,
        last_encoders: Vector4<f32>,
        chain_timer: Chained01,
    }

    #[shared]
    struct Shared {
        shared_spi: SharedSPI,
        delay2: Delay2,
        rx_int: RadioInterrupt,
        gpio1: Gpio1,
        robot_status: RobotStatusMessage,
        control_message: Option<ControlMessage>,
        counter: u32,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize the Heap
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

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
            pit: (pit0, pit1, pit2, _pit3),
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

        // Chained Pit<0> and Pit<1>
        let mut chained_timer = Chained01::new(pit0, pit1);
        chained_timer.enable();

        // Setup Rx Interrupt
        let rx_int = gpio1.input(pins.p1);
        gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));

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
        spi.disabled(|spi| {
            spi.set_mode(FPGA_SPI_MODE);
        });

        // Initialize IMU
        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
        let mut pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let imu = match IMU::new(i2c, &mut pit_delay) {
            Ok(imu) => imu,
            Err(_err) => panic!("Unable to Initialize IMU"),
        };

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
        let radio_cs = gpio2.output(pins.p6);
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
                delay2,
                rx_int,
                gpio1,
                robot_status: initial_robot_status,
                control_message: None,
                counter: 0,
            },
            Local {
                radio,
                motion_controller: MotionControl::new(),
                fpga,
                imu,
                last_encoders: Vector4::zeros(),
                chain_timer: chained_timer,
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
        #[cfg(feature = "debug")]
        log::info!("Command Received");
        
        if (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            if rx_int.is_triggered() {
                rx_int.clear_triggered();
                gpio1.set_interrupt(&rx_int, None);
                return true;
            }
            false
        }) {
            receive_command::spawn().ok();
        }
    }

    #[task(
        shared = [rx_int, gpio1, shared_spi, delay2, control_message, robot_status, counter],
        local = [radio],
        priority = 2
    )]
    async fn receive_command(ctx: receive_command::Context) {
        (
            ctx.shared.shared_spi,
            ctx.shared.delay2,
            ctx.shared.control_message,
            ctx.shared.robot_status,
            ctx.shared.counter,
        ).lock(|spi, delay, command, robot_status, counter| {
            let mut read_buffer = [0u8; CONTROL_MESSAGE_SIZE];
            ctx.local.radio.read(&mut read_buffer, spi, delay);

            let control_message = match ControlMessage::unpack_from_slice(&read_buffer[..]) {
                Ok(control_message) => control_message,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Error Unpacking Control Command: {:?}", _err);
                    return;
                },
            };

            *counter = 0;

            #[cfg(feature = "debug")]
            log::info!("Control Command Received: {:?}", control_message);

            *command = Some(control_message);

            ctx.local.radio.set_payload_size(ROBOT_STATUS_SIZE as u8, spi, delay);
            ctx.local.radio.stop_listening(spi, delay);

            let packed_data = match robot_status.pack() {
                Ok(bytes) => bytes,
                Err(_err) => {
                    #[cfg(feature = "debug")]
                    log::info!("Error Packing Robot Status: {:?}", _err);

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

    #[task(
        shared = [control_message, counter],
        local = [fpga, motion_controller, imu, last_encoders, chain_timer, initialized: bool = false, iteration: u32 = 0, last_time: u64 = 0],
        priority = 1
    )]
    async fn motion_control_loop(mut ctx: motion_control_loop::Context) {
        if !*ctx.local.initialized {
            match ctx.local.fpga.configure() {
                Ok(_) => {
                    #[cfg(feature = "debug")]
                    log::info!("Fpga Configured")
                },
                Err(_) => panic!("Unable to Configure Fpga"),
            }

            match ctx.local.fpga.motors_en(true) {
                Ok(_status) => {
                    #[cfg(feature = "debug")]
                    log::info!("Enabled motors fpga: {:010b}", _status)
                },
                Err(_) => panic!("Unable to Enable Motors"),
            }
            *ctx.local.initialized = true;

            *ctx.local.last_time = ctx.local.chain_timer.current_timer_value();
        }

        let mut body_velocities = ctx.shared.control_message.lock(|control_message| {
            match control_message {
                Some(control_message) => control_message.get_velocity(),
                None => Vector3::new(0.0, 0.0, 0.0),
            }
        });

        let counter = ctx.shared.counter.lock(|counter| *counter);
        if counter > MAX_COUNTER_VALUE {
            body_velocities = Vector3::new(0.0, 0.0, 0.0);
        }

        let gyro = match ctx.local.imu.gyro_z() {
            Ok(gyro) => gyro,
            Err(_err) => {
                #[cfg(feature = "debug")]
                log::info!("Unable to Read Gyro");
                0.0
            }
        };

        let accel_x = match ctx.local.imu.accel_x() {
            Ok(accel_x) => accel_x,
            Err(_err) => {
                #[cfg(feature = "debug")]
                log::info!("Unable to Read Accel X");
                0.0
            }
        };

        let accel_y = match ctx.local.imu.accel_y() {
            Ok(accel_y) => accel_y,
            Err(_err) => {
                #[cfg(feature = "debug")]
                log::info!("Unable to Read Accel Y");
                0.0
            }
        };

        let now = ctx.local.chain_timer.current_timer_value();
        let delta = *ctx.local.last_time - now;
        *ctx.local.last_time = now;

        let wheel_velocities = ctx.local.motion_controller.control_update(
            Vector3::new(accel_x, accel_y, gyro),
            *ctx.local.last_encoders,
            body_velocities,
            delta,
        );

        let encoder_velocities = match ctx.local.fpga.set_velocities(wheel_velocities.into(), 0.0) {
            Ok(encoder_velocities) => encoder_velocities,
            Err(_err) => {
                #[cfg(feature = "debug")]
                log::info!("Unable to Read Encoder Values");
                [0.0; 4]
            }
        };

        *ctx.local.last_encoders = Vector4::new(
            encoder_velocities[0],
            encoder_velocities[1],
            encoder_velocities[2],
            encoder_velocities[3],
        );

        #[cfg(feature = "debug")]
        if *ctx.local.iteration % 100 == 0 {
            log::info!("Wheel Velocities: {:?} --- Body Velocities: {:?}", wheel_velocities, body_velocities);
        }

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_loop::spawn().ok();
    }
}