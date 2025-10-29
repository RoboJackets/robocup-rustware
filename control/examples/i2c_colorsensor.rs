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

const SENSOR_ADDRESS: u8 = 0x28;

enum PowerState {
    Shutdown,
    PowerOn,
}

enum ColorsToSense {
    RGOn,
    RGOff,
}

enum DigitalGain {
    X1,
    X2,
    X4,
}

enum Gain {
    X1_2,
    X1,
    X2,
    X4,
}

enum Sensitivity {
    High,
    Low,
}

enum IntegrationTime {
    ms50,
    ms100,
    ms200,
    ms400,
}

enum ForceMode {
    Auto,
    Active,
}

struct SensorData {
    clear: u16,
    red: u16,
    green: u16,
    blue: u16,
    infrared: u16,
}

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

        scan_i2c_devices::spawn().ok();

        (Shared { i2c }, Local { poller })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(shared=[i2c], priority=1)]
    async fn start_color_sensor(mut ctx: start_color_sensor::Context) {
        Systick::delay(TASK_START_DELAY_MS.millis()).await;
        let ps = PowerState::PowerOn;
        let cts = ColorsToSense::RGOn;
        let dg = DigitalGain::X1;
        let g = Gain::X1;
        let s = Sensitivity::High;
        let it = IntegrationTime::ms50;
        let fm = ForceMode::Auto;
        ctx.shared.i2c.lock(|i2c| {
            init_color_sensor(i2c, ps, cts, dg, g, s, it, fm);
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

    fn write_to_command(i2c: &mut Lpi2c1, command_code: u8, data: u16) {
        let low_byte: u8 = (data % 0b1_0000_0000) as u8;
        let high_byte: u8 = (data / 0b1_0000_0000) as u8;
        (*i2c).write_read(SENSOR_ADDRESS, &[command_code, low_byte, high_byte], &mut []);
    }

    fn read_from_command(i2c: &mut Lpi2c1, command_code: u8, data: &mut u16) {
        let mut low_byte: u8 = 0;
        let mut high_byte: u8 = 0;
        (*i2c).write_read(SENSOR_ADDRESS, &[command_code], &mut [low_byte, high_byte]);
        *data = (high_byte as u16) << 8 + (low_byte as u16);
    }

    fn init_color_sensor(i2c: &mut Lpi2c1, ps: PowerState, cts: ColorsToSense, dg: DigitalGain, g: Gain, s: Sensitivity, it: IntegrationTime, fm: ForceMode) {
        let mut config = match ps {
            PowerState::PowerOn => {0b0},
            PowerState::Shutdown => {0b1},
        };
        config = config << 1 + match cts {
            ColorsToSense::RGOn => {0b0},
            ColorsToSense::RGOff => {0b1},
        };
        config = config << 2 + match dg {
            DigitalGain::X1 => {0b00},
            DigitalGain::X2 => {0b01},
            DigitalGain::X4 => {0b10},
        };
        config = config << 2 + match g {
            Gain::X1 => {0b00},
            Gain::X2 => {0b01},
            Gain::X4 => {0b10},
            Gain::X1_2 => {0b11},
        };
        config = config << 4 + match s {
            Sensitivity::High => {0b00},
            Sensitivity::Low => {0b01},
        };
        config = config << 2 + match it {
            IntegrationTime::ms50 => {0b00},
            IntegrationTime::ms100 => {0b01},
            IntegrationTime::ms200 => {0b10},
            IntegrationTime::ms400 => {0b11},
        };
        config = config << 1 + match fm {
            ForceMode::Auto => {0b0},
            ForceMode::Active => {0b1},
        };
        config = config << 3 + match ps {
            PowerState::PowerOn => {0b0},
            PowerState::Shutdown => {0b1},
        };

        write_to_command(i2c, 0x00, config);
    }

    fn read_sensor_values(i2c: &mut Lpi2c1, sd: &mut SensorData) {
        read_from_command(i2c, 0x04, &mut sd.clear);
        read_from_command(i2c, 0x05, &mut sd.red);
        read_from_command(i2c, 0x06, &mut sd.green);
        read_from_command(i2c, 0x07, &mut sd.blue);
        read_from_command(i2c, 0x08, &mut sd.infrared);
    }

    #[task(shared=[i2c], priority=1)]
    async fn scan_i2c_devices(mut ctx: scan_i2c_devices::Context) {
        Systick::delay(TASK_START_DELAY_MS.millis()).await;

        let mut buffer = [0u8];

        ctx.shared.i2c.lock(|i2c| {
            if let Err(err) = i2c.write_read(0b1101000, &[0x75], &mut buffer) {
                log::info!("Error Occurred: {:?}", err);
            }
        });

        log::info!("Who Am I: {:#01x}", buffer[0]);

        log::info!("Completed I2C Devices Scan");
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
