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

    use main::collect::{MotionControlHeader, MotionControlReading};

    use embedded_hal::spi::MODE_0;

    use teensy4_bsp as bsp;
    use bsp::board::{self, LPSPI_FREQUENCY};

    use teensy4_pins::t41::*;

    use teensy4_bsp::hal as hal;
    use hal::lpspi::{LpspiError, Lpspi, Pins};
    use hal::gpio::{Output, Port};
    use hal::gpt::{ClockSource, Gpt1, Gpt2};
    use hal::timer::Blocking;
    
    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_monotonics::systick::*;

    use fpga_rs as fpga;
    use fpga::FPGA;

    use w25q128::error::W25Q128Error;
    use w25q128::status::StatusRegister;
    use w25q128::StorageModule;

    // Constants
    const GPT_FREQUENCY: u32 = 1_000;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 8192;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const MOTION_CONTROL_DELAY_US: u32 = 200;
    const TASK_START_DELAY_MS: u32 = 2_000;

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
        storage_module: StorageModule<Output<P6>, SharedSPI, LpspiError, Infallible>,
        spi: SharedSPI,
        delay: Delay2,
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

        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

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

        let storage_module = StorageModule::new(csn);

        report_data::spawn().ok();

        (
            Shared {

            },
            Local {
                storage_module,
                spi: shared_spi,
                delay: delay2,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [storage_module, spi, delay], priority=1)]
    async fn report_data(ctx: report_data::Context) {
        Systick::delay(TASK_START_DELAY_MS.millis()).await;

        match ctx.local.storage_module.reset_device(ctx.local.spi, ctx.local.delay) {
            Ok(_) => log::info!("reset device"),
            Err(_) => panic!("Unable to reset the device"),
        }

        match ctx.local.storage_module.leave_qpi(ctx.local.spi, ctx.local.delay) {
            Ok(_) => log::info!("Leaving qpi"),
            Err(_) => panic!("Unable to leave qpi"),
        }

        Systick::delay(500u32.millis()).await;

        match ctx.local.storage_module.read_from_status_register(
            StatusRegister::StatusRegister1,
            ctx.local.spi,
            ctx.local.delay,
        ) {
            Ok(status) => log::info!("Status Register 1: {:#02x}", status),
            Err(_) => panic!("Unable to read status register 1"),
        }

        match ctx.local.storage_module.read_from_status_register(
            StatusRegister::StatusRegister2,
            ctx.local.spi,
            ctx.local.delay,
        ) {
            Ok(status) => log::info!("Status Register 2: {:#02x}", status),
            Err(_) => panic!("Unable to read status register 2"),
        }

        match ctx.local.storage_module.read_from_status_register(
            StatusRegister::StatusRegister3,
            ctx.local.spi,
            ctx.local.delay,
        ) {
            Ok(status) => log::info!("Status Register 3: {:#02x}", status),
            Err(_) => panic!("Unable to read status register 3"),
        }

        match ctx.local.storage_module.write_enable(ctx.local.spi, ctx.local.delay) {
            Ok(_) => log::info!("Write Enabled"),
            Err(_) => panic!("Unable to set write enabled"),
        }

        Systick::delay(100u32.millis()).await;

        match ctx.local.storage_module.read_from_status_register(
            StatusRegister::StatusRegister1,
            ctx.local.spi,
            ctx.local.delay,
        ) {
            Ok(status) => log::info!("Status: {:#02x}", status),
            Err(_) => panic!("Unable to read status register 1"),
        }

        log::info!("Erasing Memory");
        match ctx.local.storage_module.erase_32k_bytes([0x00, 0x00, 0x00], ctx.local.spi, ctx.local.delay) {
            Ok(_) => (),
            Err(err) => match err {
                W25Q128Error::UnableToSetWriteEnable => panic!("Unable to set write enable"),
                W25Q128Error::SpiError(_) => panic!("SPI Error"),
                _ => panic!("Other Error"),
            },
        }

        match ctx.local.storage_module.write(
            [0, 0, 0],
            &[1, 2, 3, 4, 5],
            ctx.local.spi,
            ctx.local.delay,
        ) {
            Ok(_) => log::info!("Wrote Test Data"),
            Err(_) => panic!("Unable to write test data"),
        }

        Systick::delay(100u32.millis()).await;

        let mut buffer = [0; 5];
        match ctx.local.storage_module.read(
            [0, 0, 0],
            &mut buffer,
            ctx.local.spi,
            ctx.local.delay,
        ) {
            Ok(_) => log::info!("Read: {:?}", buffer),
            Err(_) => panic!("Unable to read test data"),
        }

        log::info!("Writing Actual Data");

        let header = MotionControlHeader {
            target_velocity: controller::UP.into(),
            valid: true,
        };

        if ctx.local.storage_module.write([0u8; 3], &header.to_bytes(), ctx.local.spi, ctx.local.delay).is_err() {
            panic!("Unable to Write");
        }

        let mut address = [0, 0, 13];
        for _ in 0..5 {
            let reading = MotionControlReading {
                valid: true,
                accel_x: unsafe { (1.0f32 * 1_000.0).to_int_unchecked::<i32>() },
                accel_y: unsafe { (1.0f32 * 1_000.0).to_int_unchecked::<i32>() },
                gyro_z: unsafe { (1.0f32 * 1_000.0).to_int_unchecked::<i32>() },
                encoder_values: [15; 10],
                delta_t: 0,
            };

            if ctx.local.storage_module.write(address, &reading.to_bytes(), ctx.local.spi, ctx.local.delay).is_err() {
                panic!("Unable to Write");
            }

            let (value, mut carry) = address[2].carrying_add(31, false);
            address[2] = value;
            for i in (0..2).rev() {
                (address[i], carry) = address[i].carrying_add(0, carry);
            }
        }

        let reading = MotionControlReading {
            valid: false,
            accel_x: unsafe { 0.0f32.to_int_unchecked::<i32>() },
            accel_y: unsafe { 0.0f32.to_int_unchecked::<i32>() },
            gyro_z: unsafe { 0.0f32.to_int_unchecked::<i32>() },
            encoder_values: [0u8; 10],
            delta_t: 0,
        };

        if ctx.local.storage_module.write(address, &reading.to_bytes(), ctx.local.spi, ctx.local.delay).is_err() {
            panic!("Unable to Write");
        }

        log::info!("Done Writing");

        log::info!("Beginning Reading");

        let mut header = [0u8; 13];
        match ctx.local.storage_module.read(
            [0, 0, 0],
            &mut header,
            ctx.local.spi,
            ctx.local.delay,
        ) {
            Ok(_) => {
                let header = MotionControlHeader::from_bytes(&header);
                log::info!("Reading Header: {:?}", header);
            },
            Err(_) => panic!("Unable to read header"),
        }

        let mut address = [0, 0, 13];
        for _ in 0..5 {
            let mut reading = [0u8; 31];
            match ctx.local.storage_module.read(
                address,
                &mut reading,
                ctx.local.spi,
                ctx.local.delay,
            ) {
                Ok(_) => {
                    let reading = MotionControlReading::from_bytes(&reading);
                    log::info!("Reading: {:?}", reading);
                },
                Err(_) => panic!("Unable to read reading"),
            }

            let (value, mut carry) = address[2].carrying_add(31, false);
            address[2] = value;
            for i in (0..2).rev() {
                (address[i], carry) = address[i].carrying_add(0, carry);
            }
        }

        log::info!("Done");
    }
}