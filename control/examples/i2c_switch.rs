//!
//! This Example tests the i2c switch.
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

const SENSOR_ADDRESS: u8 = 0x28;
const SWITCH_ADDRESS: u8 = 0xE2; // Switch depending on our address pin layout


#[rtic::app(device=teensy4_bsp, peripherals=true, dispatchers=[GPT2])]
mod app {
    use core::mem::MaybeUninit;

    use embedded_hal::blocking::i2c::WriteRead;

    use crate::app::shared_resources::i2c_that_needs_to_be_locked;

    use super::*;

    use bsp::board;
    use teensy4_bsp::{self as bsp, ral::adc::HC0::AIEN::W};

    use board::Lpi2c1;
    use rtic_monotonics::systick::*;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    const TASK_START_DELAY_MS: u32 = 5_000;

    #[local]
    struct Local {
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {
        i2c: Lpi2c1,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize the Heap
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        // Get Peripherals
        let board::Resources {
            pins, usb, lpi2c1, ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);

        start_i2c_switch::spawn().ok();

        (Shared { i2c }, Local { poller })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(shared=[i2c], priority=1)]
    async fn start_i2c_switch(mut ctx: start_color_sensor::Context) {
        Systick::delay(TASK_START_DELAY_MS.millis()).await;
        ctx.shared.i2c.lock(|i2c| {
            init_i2c_switch(i2c);
        });
    }

    #[task(shared=[i2c], priority=1)]
    async fn log_sensor_data(mut ctx: log_sensor_data::Context) {
        loop {
            let mut data = SensorData {
                clear: 0, 
                red: 0, 
                green: 0, 
                blue: 0, 
                infrared: 0,
            };
            ctx.shared.i2c.lock(|i2c| {
                read_sensor_values(i2c, &mut data);
            });

            log::info!("Clear: {}", data.clear);
            log::info!("Red: {}", data.red);
            log::info!("Green: {}", data.green);
            log::info!("Blue: {}", data.blue);
            log::info!("Infrared: {}", data.infrared);
        }
    }

    // ??? dunno if needed
    fn write_to_switch(i2c: &mut Lpi2c1, data: &[u8]) {
        match (*i2c).write_read(SWITCH_ADDRESS, data, []) {
            Err(err) => {
                log::info!("An error occured. {:?}", err);
            },
            _ => {},
        }
    }

    // ??? dunno if needed
    fn read_from_switch(i2c: &mut Lpi2c1, data: &mut [u8]) {
        match (*i2c).write_read(SWITCH_ADDRESS, [], data) {
            Err(err) => {
                log::info!("An error occured. {:?}", err);
            },
            _ => {},
        }
    }

    // hopefully would work?
    fn write_to_register(i2c: &mut Lpi2c1, register: u8, data: &[u8]) {
        match (*i2c).write_read(SWITCH_ADDRESS, [1 << register], []) {
            Err(err) => {
                log::info!("An error occured. {:?}", err);
            },
            _ => {},
        }
        match (*i2c).write_read(SENSOR_ADDRESS, data, []) {
            Err(err) => {
                log::info!("An error occured. {:?}", err);
            },
            _ => {},
        }
    }

    // hopefully would work?
    fn read_from_register(i2c: &mut Lpi2c1, register: u8, data: &mut [u8]) {
        match (*i2c).write_read(SWITCH_ADDRESS, [1 << register], []) {
            Err(err) => {
                log::info!("An error occured. {:?}", err);
            },
            _ => {},
        }
        match (*i2c).write_read(SENSOR_ADDRESS, [], data) {
            Err(err) => {
                log::info!("An error occured. {:?}", err);
            },
            _ => {},
        }
    }

    /* keep these for i2c switch testing?
    fn write_to_command(i2c: &mut Lpi2c1, command_code: u8, data: u16) {
        let low_byte: u8 = (data % 0b1_0000_0000) as u8;
        let high_byte: u8 = (data / 0b1_0000_0000) as u8;
        match (*i2c).write_read(SENSOR_ADDRESS, &[command_code, low_byte, high_byte], &mut []) {
            Err(err) => {
                log::info!("An error occured. {:?}", err);
            },
            _ => {},
        }
    }

    fn read_from_command(i2c: &mut Lpi2c1, command_code: u8, data: &mut u16) {
        let mut buffer = [0u8, 0u8];
        match (*i2c).write_read(SENSOR_ADDRESS, &[command_code], &mut buffer) {
            Err(err) => {
                log::info!("An error occured. {:?}", err);
            },
            _ => {},
        };
        *data = (buffer[1] as u16) << 8 + (buffer[0] as u16);
    }
    */

    fn init_i2c_switch(i2c: &mut Lpi2c1) {
        // pull reset high somehow lol
    }

    fn read_sensor_values(i2c: &mut Lpi2c1, sd: &mut SensorData) {
        read_from_command(i2c, 0x04, &mut sd.clear);
        read_from_command(i2c, 0x05, &mut sd.red);
        read_from_command(i2c, 0x06, &mut sd.green);
        read_from_command(i2c, 0x07, &mut sd.blue);
        read_from_command(i2c, 0x08, &mut sd.infrared);
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
