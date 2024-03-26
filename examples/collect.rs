//!
//! Collection Collects Data from Basic Movement by the Robot and stores the data
//! in flash memory that can be read via report.rs
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(bigint_helper_methods)]

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

    use main::collect::{MotionControlHeader, MotionControlReading};

    use embedded_hal::spi::MODE_0;

    use teensy4_bsp as bsp;
    use bsp::board::{self, LPSPI_FREQUENCY};
    use bsp::board::{Lpi2c1, PERCLK_FREQUENCY};

    use teensy4_pins::t41::*;

    use teensy4_bsp::hal as hal;
    use hal::lpspi::{LpspiError, Lpspi, Pins};
    use hal::gpio::{Output, Port};
    use hal::gpt::{ClockSource, Gpt1, Gpt2};
    use hal::timer::Blocking;
    
    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_monotonics::systick::*;

    use motion::MotionControl;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    use icm42605_driver::IMU;

    use w25q128::StorageModule;

    // Constants
    const GPT_FREQUENCY: u32 = 1_000;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 8192;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 200;

    // Type Definitions
    // FPGA Spi
    type FpgaSpi = Lpspi<board::LpspiPins<P11, P12, P13, P10>, 4>;
    type Fpga = FPGA<FpgaSpi, Output<P9>, P29, Output<P28>, P30, Delay1, hal::lpspi::LpspiError, Infallible>;
    // Shared SPI
    type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
    // Delays
    type Delay1 = Blocking<Gpt1, GPT_FREQUENCY>;
    type Delay2 = Blocking<Gpt2, GPT_FREQUENCY>;
    // GPIO Ports
    type Gpio1 = Port<1>;

    #[local]
    struct Local {
        fpga: Fpga,
        delay: Option<Delay2>,
        imu: IMU<Lpi2c1>,
        storage_module: StorageModule<Output<P6>, SharedSPI, LpspiError, Infallible>,
        motion_control: MotionControl,
        spi: SharedSPI,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
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
            pit: (pit1, _pit2, _pit3, _pit4),
            ..
        } = board::t41(ctx.device);

        // Setup Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let mut fpga_spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            FPGA_SPI_FREQUENCY,
        );
        fpga_spi.disabled(|spi| {
            spi.set_mode(FPGA_SPI_MODE);
        });

        let cs = gpio2.output(pins.p9);
        let init_b = gpio4.input(pins.p29);
        let config = Config::zero().set_open_drain(OpenDrain::Enabled);
        configure(&mut pins.p28, config);
        let prog_b = gpio3.output(pins.p28);
        let done = gpio3.input(pins.p30);

        let fpga = match FPGA::new(fpga_spi, cs, init_b, prog_b, done, delay1) {
            Ok(fpga) => fpga,
            Err(_) => panic!("Unable to initialize the FPGA"),
        };

        let shared_spi_pins = Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = Lpspi::new(shared_spi_block, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_mode(MODE_0);
        });

        let csn = gpio2.output(pins.p6);

        let mut storage_module = StorageModule::new(csn);
        match storage_module.erase_32k_bytes([0x00, 0x00, 0x00], &mut shared_spi, &mut delay2) {
            Ok(_) => (),
            Err(_) => panic!("Unable to erase storage module"),
        }

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);
        let mut pit_delay = Blocking::<_, PERCLK_FREQUENCY>::from_pit(pit1);
        let imu = match IMU::new(i2c, &mut pit_delay) {
            Ok(imu) => imu,
            Err(_) => panic!("Unable to initialize the IMU"),
        };

        (
            Shared {

            },
            Local {
                fpga,
                delay: Some(delay2),
                imu,
                storage_module,
                motion_control: MotionControl::without_clock(),
                spi: shared_spi,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [fpga, delay, imu, storage_module, motion_control, spi, initialized: bool = false, address: [u8; 3] = [0u8; 3]], priority=1)]
    async fn motion_control_update(ctx: motion_control_update::Context) {
        #[cfg(any(
            feature = "up",
            not(feature = "down"),
            not(feature = "right"),
            not(feature = "left"),
            not(feature = "clockwise"),
            not(feature = "counterclockwise"),
        ))]
        let body_velocity = controller::UP;
        #[cfg(feature = "down")]
        let body_velocity = controller::DOWN;
        #[cfg(feature = "right")]
        let body_velocity = controller::RIGHT;
        #[cfg(feature = "left")]
        let body_velocity = controller::LEFT;
        #[cfg(feature = "clockwise")]
        let body_velocity = controller::CLOCKWISE;
        #[cfg(feature = "counterclockwise")]
        let body_velocity = controller::COUNTERCLOCKWISE;

        let mut delay = ctx.local.delay.take().unwrap();

        if !*ctx.local.initialized {
            if ctx.local.fpga.configure().is_err() {
                panic!("Unable to configure fpga");
            }

            if ctx.local.fpga.motors_en(true).is_err() {
                panic!("Unable to enable motors");
            }

            let header = MotionControlHeader { target_velocity: body_velocity.into() };

            if ctx.local.storage_module.write(
                *ctx.local.address,
                &header.to_bytes(),
                ctx.local.spi,
                &mut delay,
            ).is_err() {
                panic!("Unable to Write to Flash Storage");
            }

            *ctx.local.address = [0, 0, 12];

            *ctx.local.initialized = true;
        }

        let wheel_velocities = ctx.local.motion_control.body_to_wheels(body_velocity);

        let encoder_values = match ctx.local.fpga.set_duty_get_encoders(wheel_velocities.into(), 0.0) {
            Ok(encoder_values) => encoder_values,
            Err(_) => [u8::MAX; 10],
        };

        if ctx.local.fpga.set_duty_cycles(wheel_velocities.into(), 0.0).is_err() {
            log::info!("Unable to set duty cycles");
        }

        let gyro_z = match ctx.local.imu.gyro_z() {
            Ok(z) => z,
            Err(_) => f32::MAX,
        };

        let accel_x = match ctx.local.imu.accel_x() {
            Ok(x) => x,
            Err(_) => f32::MAX,
        };

        let accel_y = match ctx.local.imu.accel_y() {
            Ok(y) => y,
            Err(_) => f32::MAX,
        };

        let reading = MotionControlReading {
            accel_x,
            accel_y,
            gyro_z,
            encoder_values,
            valid: true,
        };

        if ctx.local.storage_module.write(
            *ctx.local.address,
            &reading.to_bytes(),
            ctx.local.spi,
            &mut delay,
        ).is_err() {
            panic!("Unable to Write to Storage Module");
        }

        // Increment Address
        let (value, mut carry) = ctx.local.address[2].carrying_add(23, false);
        ctx.local.address[2] = value;
        for i in (0..2).rev() {
            (ctx.local.address[i], carry) = ctx.local.address[i].carrying_add(0, carry);
        }

        *ctx.local.delay = Some(delay);

        motion_control_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn motion_control_delay(_ctx: motion_control_delay::Context) {
        Systick::delay(MOTION_CONTROL_DELAY_US.micros()).await;

        motion_control_update::spawn().ok();
    }
}