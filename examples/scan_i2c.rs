//!
//! This Example scans the i2c interface for devices connected.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device=teensy4_bsp, peripherals=true, dispatchers=[GPT2])]
mod app {
    use core::mem::MaybeUninit;

    use embedded_hal::blocking::i2c::WriteRead;

    use super::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use teensy4_bsp::hal as hal;

    use rtic_monotonics::systick::*;
    use board::Lpi2c1;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const TASK_START_DELAY_MS: u32 = 5_000;

    #[shared]
    struct Shared {

    }

    #[local]
    struct Local {
        i2c: Lpi2c1,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize the Heap
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        // Get Peripherals
        let board::Resources {
            pins,
            usb,
            lpi2c1,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::MHz1);

        scan_i2c_devices::spawn().ok();

        (
            Shared {

            },
            Local {
                i2c,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local=[i2c], priority=1)]
    async fn scan_i2c_devices(ctx: scan_i2c_devices::Context) {
        Systick::delay(TASK_START_DELAY_MS.millis()).await;
        log::info!("Scanning For I2C Devices");

        for address in 0..128 {
            log::info!("Writing to Address: {:#01}", address);
            Systick::delay(100u32.millis()).await;
            let mut buffer = [0u8];
            let _ = ctx.local.i2c.write_read(address, &[0x75], &mut buffer);
            log::info!("Received: {:#01}", buffer[0]);
        }

        // let mut buffer = [0u8];
        // if let Err(err) = ctx.local.i2c.write_read(0b1101000, &[0x75], &mut buffer) {
        //     log::info!("Error Occurred: {:?}", err);
        // }

        // log::info!("Who I am: {:#01x}", buffer[0]);

        // for address in 0..=0x7f {
        //     let mut result = ctx.local.i2c.write(address, &[0x00]);
        //     while let Err(err) = result {
        //         if !err.contains(hal::lpi2c::ControllerStatus::BUS_BUSY) {
        //             break;
        //         }

        //         Systick::delay(1u32.millis()).await;

        //         result = ctx.local.i2c.write(address, &[0x00]);
        //     }

        //     if let Ok(_) = result {
        //         log::info!("Device Found at Address: {:#010x}", address);
        //     }
        // }

        log::info!("Completed I2C Devices Scan");
    }
}