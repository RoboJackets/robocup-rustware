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

    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_nrf24l01::{Radio, config::*};

    use rtic_monotonics::systick::*;

    use packed_struct::prelude::*;

    use fpga_rs::{error::FpgaError, duty_cycle::DutyCycle, FPGA};
    use fpga_rs::{FPGA_SPI_MODE, FPGA_SPI_FREQUENCY};

    // Communication
    use robojackets_robocup_rtp::Team;
    use robojackets_robocup_rtp::control_message::ControlMessage;
    use robojackets_robocup_rtp::robot_status_message::{RobotStatusMessage, RobotStatusMessageBuilder};

    use main::motion_control::MotionControl;

    /// Constants
    const GPT_FREQUENCY: u32 = 1_000;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    /// Type Definitions
    // SPI 1
    type SPI = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
    type Fpga = FPGA<SPI, Output<P9>, P15, Output<P16>, P14>;
    // SPI 2
    type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
    type RadioCE = Output<P0>;
    type RadioCSN = Output<P2>;
    type RadioInterrupt = Input<P1>;
    // DELAYS
    type Delay1 = Blocking<Gpt1, GPT_FREQUENCY>;
    type Delay2 = Blocking<Gpt2, GPT_FREQUENCY>;
    // GPIO Ports
    type Gpio1 = Port<1>;

    #[local]
    struct Local {
        fpga: Fpga,
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
        control_command: Option<ControlMessage>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        let board::Resources {
            mut pins,
            mut gpio1,
            mut gpio2,
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
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        // gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        // RX DONE Interrupt setup
        let rx_int = gpio1.input(pins.p1);
        gpio4.set_interrupt(&rx_int, Some(Trigger::RisingEdge));

        // initialize spi 1
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

        // Initialize Shared SPI
        let shared_spi_pins = hal::lpspi::Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = hal::lpspi::Lpspi::new(shared_spi_block, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 10_000_000u32);
            spi.set_mode(MODE_0);
        });

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
        radio.open_writing_pipe(BASE_STATION_ADDRESS, &mut shared_spi, &mut delay2);
        radio.open_reading_pipe(1, ROBOT_RADIO_ADDRESSES[ROBOT_ID as usize], &mut shared_spi, &mut delay2);
        radio.start_listening(&mut shared_spi, &mut delay2);

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

        let initial_robot_status = RobotStatusMessageBuilder::new().build();

        (
            Shared {
                shared_spi,
                delay1,
                delay2,
                rx_int,
                gpio1,
                robot_status: initial_robot_status,
                control_command: None,
            },
            Local {
                fpga,
                radio,
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

    #[task(binds = GPIO1_COMBINED_0_15, shared = [rx_int, gpio1], priority = 1)]
    fn gpio_interrupt(mut ctx: gpio_interrupt::Context) {
        if (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            if rx_int.is_triggered() {
                rx_int.clear_triggered();
                gpio1.set_interrupt(&rx_int, None);
                return true;
            }
            false
        }) {
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
            for _ in 0..5 {
                let report = ctx.local.radio.write(&packed_data, spi, delay);
                if report {
                    break;
                }
            }

            ctx.local.radio.start_listening(spi, delay);
        });

        (ctx.shared.rx_int, ctx.shared.gpio1).lock(|rx_int, gpio1| {
            rx_int.clear_triggered();
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
    }
}