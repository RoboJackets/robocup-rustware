//!
//! Collection Collects Data from Basic Movement by the Robot and stores the data
//! in flash memory that can be read via report.rs
//! 

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_bsp as _;

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
    use hal::lpspi::{LpspiError, Lpspi};
    use hal::gpio::{Output, Port};
    use hal::gpt::{ClockSource, Gpt1, Gpt2};
    use hal::timer::Blocking;
    
    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use rtic_monotonics::systick::*;

    use fpga_rs as fpga;
    use fpga::FPGA;

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
        storage_module: StorageModule<Output<P40>, SharedSPI, LpspiError, Infallible>,
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
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = Lpspi::new(
            shared_spi_block,
            board::LpspiPins {
                pcs0: pins.p38,
                sck: pins.p27,
                sdo: pins.p26,
                sdi: pins.p38,
            }
        );

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 5_000_000u32);
            spi.set_mode(MODE_0);
        });

        let csn = gpio4.output(pins.p40);

        let mut storage_module = StorageModule::new(csn);
        match storage_module.erase_32k_bytes([0x00, 0x00, 0x00], &mut shared_spi, &mut delay2) {
            Ok(_) => (),
            Err(_) => panic!("Unable to erase storage module"),
        }

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

        let mut header = [0u8; 12];
        if ctx.local.storage_module.read([0u8; 3], &mut header, ctx.local.spi, ctx.local.delay).is_err() {
            panic!("Unable to Read Header");
        }
        let header = MotionControlHeader::from_bytes(&header);

        log::info!("Target Velocity: {:?}", header);
        log::info!("...");

        Systick::delay(100u32.millis()).await;

        let mut address = [0, 0, 12];
        loop {
            let mut buffer = [0u8; 23];
            if ctx.local.storage_module.read(address, &mut buffer, ctx.local.spi, ctx.local.delay).is_err() {
                panic!("Unable to Read Data");
            }
            let reading = MotionControlReading::from_bytes(&buffer);

            if reading.valid {
                log::info!(
                    "X: {}, Y: {}, Z: {}, [{}, {}, {}, {}, {}, {}, {}, {}, {}, {}]",
                    reading.accel_x,
                    reading.accel_y,
                    reading.gyro_z,
                    reading.encoder_values[0],
                    reading.encoder_values[1],
                    reading.encoder_values[2],
                    reading.encoder_values[3],
                    reading.encoder_values[4],
                    reading.encoder_values[5],
                    reading.encoder_values[6],
                    reading.encoder_values[7],
                    reading.encoder_values[8],
                    reading.encoder_values[9],
                );
            } else {
                break;
            }
        }

        log::info!("DONE");
    }
}