//!
//! This demo example shows how a teensy 4 RTIC application can be set up
//! and spawns a software task that blinks an onboard led.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

///
/// This is a demo example file that turns on and off the onboard led.
///
/// Please follow this example for future examples and sanity tests
///
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use bsp::board;
    use teensy4_bsp as bsp;

    use rtic_monotonics::systick::*;

    use embedded_hal::blocking::delay::DelayMs;
    use embedded_hal::spi::MODE_0;
    use icm42605_driver::{ImuError, IMU};

    use teensy4_bsp::board::PERCLK_FREQUENCY;

    use bsp::hal;
    use hal::lpi2c::ControllerStatus;
    use hal::timer::Blocking;
    use hal::lpspi::Pins;

    use robojackets_robocup_control::{
        Display, Imu, Killn, MotorEn, PitDelay, RFRadio, RadioInitError, RadioSPI,
        GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY, Delay2, BASE_AMPLIFICATION_LEVEL, CHANNEL,
        RADIO_ADDRESS, robot::TEAM_NUM
    };

    use robojackets_robocup_rtp::{CONTROL_MESSAGE_SIZE, BASE_STATION_ADDRESSES};

    use teensy4_pins::t41::{P18, P19};

    use graphics::main_window::MainWindow;
    use display_interface::DisplayError;

    use embedded_graphics::prelude::*;
    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

    use rtic_nrf24l01::Radio;

    #[local]
    struct Local {
        poller: imxrt_log::Poller,

        main_window: MainWindow<'static>,
        latency_placeholder: u16,
    }

    #[shared]
    struct Shared {
        imu: Imu,
        pit_delay: PitDelay,
        display: Display,
        radio: RFRadio,
        radio_spi: RadioSPI,
        blocking_delay: Delay2,

        imu_init_error: Option<ImuError<ControllerStatus>>,
        display_init_error: Option<DisplayError>,
        radio_init_error: Option<RadioInitError>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb,
            pins,
            lpi2c1,
            pit: (_, _, pit2, _),
            mut gpio1,
            mut gpio2,
            mut gpt2,
            lpspi4,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        // End Initialize Timers //

        // Initialize Motor Board //

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();

        delay2.delay_ms(500u32);

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::MHz1);
        let i2c_bus: &'static _ = shared_bus::new_cortexm!(
            imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1> = i2c
        )
        .expect("Failed to initialize shared I2C bus LPI2C1");

        let delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit2);
        let imu = IMU::new(i2c_bus.acquire_i2c());

        let display_interface = I2CDisplayInterface::new(i2c_bus.acquire_i2c());
        let display: Display = Ssd1306::new(
            display_interface,
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();

        let main_window = MainWindow::new(0, "Blue");
        let latency_placeholder: u16 = 0;

        let radio_spi_pins = Pins {
            pcs0: pins.p10,
            sck: pins.p13,
            sdo: pins.p11,
            sdi: pins.p12,
        };
        let mut radio_spi = hal::lpspi::Lpspi::new(lpspi4, radio_spi_pins);
        radio_spi.disabled(|spi| {
            spi.set_clock_hz(bsp::board::LPSPI_FREQUENCY, 5_000_000u32);
            spi.set_mode(MODE_0);
        });
        let radio_cs = gpio1.output(pins.p14);
        let ce = gpio1.output(pins.p41);
        let radio = Radio::new(ce, radio_cs);

        initialize_imu::spawn().ok();

        (
            Shared {
                imu,
                pit_delay: delay,
                display,
                radio,
                radio_spi,
                blocking_delay: delay2,

                imu_init_error: None,
                display_init_error: None,
                radio_init_error: None,
            },
            Local {
                poller,
                main_window,
                latency_placeholder,
            }
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

        init_display::spawn().ok();
    }

    #[task(
        shared = [display, display_init_error],
        priority = 1
    )]
    async fn init_display(ctx: init_display::Context) {
        (
            ctx.shared.display,
            ctx.shared.display_init_error
        ).lock(|display, error| {
            if let Err(err) = display.init() {
                *error = Some(err);
            }
        });
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
                        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, spi, delay);
                        radio.open_writing_pipe(BASE_STATION_ADDRESSES[TEAM_NUM], spi, delay);
                        radio.open_reading_pipe(1, RADIO_ADDRESS, spi, delay);
                        radio.stop_listening(spi, delay);
                    }
                    Err(err) => *radio_init_error = Some(err),
                },
            );

        check_for_errors::spawn().ok();
    }

    #[task(
        shared = [imu_init_error, display_init_error, radio_init_error],
        priority = 1
    )]
    async fn check_for_errors(ctx: check_for_errors::Context) {
        if (
            ctx.shared.imu_init_error,
            ctx.shared.display_init_error,
            ctx.shared.radio_init_error
        ).lock(|imu_error, display_error, radio_error| {
            imu_error.is_some() ||
                display_error.is_some() ||
                radio_error.is_some()
        }) {
            error_report::spawn().ok();
        } else {
            imu_test::spawn().ok();
        }
    }

    #[task(
        shared = [imu_init_error, display_init_error, radio_init_error],
        priority = 1
    )]
    async fn error_report(ctx: error_report::Context) {
        let (
            imu_init_error,
            display_init_error,
            radio_init_error
        ) = (
            ctx.shared.imu_init_error,
            ctx.shared.display_init_error,
            ctx.shared.radio_init_error
        ).lock(|imu_error, display_error, radio_error| {
            (
                imu_error.take(),
                display_error.take(),
                radio_error.take()
            )
        });

        for _ in 0..3 {
            log::error!("IMU ERROR: {:?}", imu_init_error);
            log::error!("DISPLAY ERROR: {:?}", display_init_error);
            log::error!("RADIO ERROR: {:?}", radio_init_error);

            Systick::delay(500u32.millis()).await;
        }

        panic!("CONTROL BOARD IS NOT OK");
    }

    #[task(
        shared = [imu],
        priority = 1,
    )]
    async fn imu_test(mut ctx: imu_test::Context) {
        let (gyro_z, accel_x, accel_y) = ctx.shared.imu.lock(|imu| {
            (
                imu.gyro_z().unwrap_or_default(),
                imu.accel_x().unwrap_or_default(),
                imu.accel_y().unwrap_or_default()
            )
        });

        log::info!("ACCEL X: {}, ACCEL Y: {}, GYRO Z: {}", accel_x, accel_y, gyro_z);

        short_delay::spawn().ok();
    }

    #[task(local = [iteration: u32 = 0], priority = 1)]
    async fn short_delay(ctx: short_delay::Context) {
        Systick::delay(100u32.millis()).await;

        if *ctx.local.iteration < 10 {
            *ctx.local.iteration += 1;
            imu_test::spawn().ok();
        } else {
            main_window_test::spawn().ok();
        }
    }

    #[task(priority=1, shared=[display], local=[latency_placeholder, main_window])]
    async fn main_window_test(mut _cx: main_window_test::Context) {
        for _i in 0..30 {
            *_cx.local.latency_placeholder += 1;
            _cx.local.main_window.latency = *_cx.local.latency_placeholder as u32;
            _cx.local.main_window.team = if *_cx.local.latency_placeholder % 2 == 0 {
                "blue"
            } else {
                "yellow"
            };
            _cx.local.main_window.ball_sense = if *_cx.local.latency_placeholder % 2 == 0 {
                true
            } else {
                false
            };
            _cx.local.main_window.kicker_charged = if *_cx.local.latency_placeholder % 2 == 0 {
                true
            } else {
                false
            };
            _cx.shared.display.lock(|display| {
                display.clear();
                _cx.local.main_window.draw(display).ok();
                display.flush().ok();
            });
            Systick::delay(500_000.micros()).await;
        }
        *_cx.local.latency_placeholder = 0;
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
