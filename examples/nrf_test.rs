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

extern crate alloc;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use embedded_hal::digital::v2::OutputPin;

    use rtic_nrf24l01::NRF24L01;
    use rtic_nrf24l01::error::Error;
    use rtic_nrf24l01::register::ConfigRegister;
    use rtic_nrf24l01::state::State;
    use teensy4_bsp::hal::lpspi::LpspiError;
    use teensy4_pins::common::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::gpio::{Output, Trigger, Input};

    use rtic_monotonics::systick::*;

    // NRF Configuration
    use rtic_nrf24l01::config;
    use rtic_nrf24l01::register::ContainsStatus;
    const CONFIGURATION: Result<config::Configuration<'static>, config::error::ConfigurationError> = config::Configuration::new(
        config::data_rate::DataRate::R1Mb,
        0u8,
        config::power_amplifier::PowerAmplifier::PAMedium,
        Some(config::bit_correction::BitCorrection::CRC1),
        config::interrupt_mask::InterruptMask::rx(),
        config::address_width::AddressWidth::A3Bytes,
        [
            Some(config::data_pipe::DataPipeConfig::new(true, false, true, b"aaa")),
            None,
            None,
            None,
            None,
            None,
        ],
        b"aaa",
        0u8,
        0u8,
    );
    // End NRF Configuration

    #[local]
    struct Local {
        spi: board::Lpspi4,
        red: Output<P15>,
        fake_cs: Option<Output<P1>>,
        ce: Option<Output<P8>>,
        irq: Input<P9>,
    }

    #[shared]
    struct Shared {

    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            mut gpio1,
            mut gpio2,
            mut gpio4,
            usb,
            lpspi4,
            ..
        } = board::t41(ctx.device);

        // usb logging setup
        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            1_000_000
        );

        let fake_cs = gpio1.output(pins.p1);
        let ce = gpio2.output(pins.p8);
        let irq = gpio2.input(pins.p9);
        gpio2.set_interrupt(&irq, Some(Trigger::RisingEdge));

        let red = gpio1.output(pins.p15);

        blink_led::spawn().ok();

        test_radio::spawn().ok();

        (
            Shared {

            },
            Local {
                spi,
                red,
                fake_cs: Some(fake_cs),
                ce: Some(ce),
                irq,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [red], priority = 1)]
    async fn blink_led(ctx: blink_led::Context) {
        loop {
            log::info!("INFO");

            ctx.local.red.set_high().unwrap();

            Systick::delay(2_500u32.millis()).await;

            ctx.local.red.set_low().unwrap();

            Systick::delay(2_500u32.millis()).await;
        }
    }

    #[task(local = [spi, ce, irq, fake_cs], priority = 1)]
    async fn test_radio(ctx: test_radio::Context) {
        Systick::delay(5_000u32.millis()).await;

        if CONFIGURATION.is_err() {
            log::info!("Invalid Configuration");
            return;
        } else {
            log::info!("Valid Configuration");
        }

        let config = CONFIGURATION.unwrap();
        log::info!("Configuration: {:?}", config);

        let ce = ctx.local.ce.take().unwrap();
        let cs = ctx.local.fake_cs.take().unwrap();

        let mut radio = match NRF24L01::new(Some(cs), ce, config, ctx.local.spi).await {
            Ok(radio) => radio,
            Err(err) => {
                match err {
                    Error::InvalidPipeId => log::info!("Invalid Pipe ID"),
                    Error::TooLargeAckPayload => log::info!("Too Large Ack Payload"),
                    Error::InvalidBufferSize => log::info!("Invalid Buffer Size"),
                    Error::InvalidRegisterBufferSize(register) => log::info!("Invalid Register Buffer Size {:#x} = {}", register, register),
                    Error::InvalidAddressBufferSize => log::info!("Invalid Address Buffer Size"),
                    Error::UnknownRegister => log::info!("Unknown Register"),
                    Error::UnableToConfigureRegister(register, expected, found) => log::info!("Unable to Configure Register: {:#x}\nExpected\n{:#010b}\nFound\n{:#010b}", register, expected, found),
                    Error::GpioError(_) => log::info!("GPIO Error"),
                    Error::SpiError(err) => match err {
                        LpspiError::FrameSize => log::info!("SPI Error Frame Size"),
                        LpspiError::Fifo(direction) => log::info!("SPI Error Direction: {:?}", direction),
                        LpspiError::Busy => log::info!("SPI Error Busy"),
                        LpspiError::NoData => log::info!("SPI Error No Data"),
                    },
                    Error::GpioSpiError(_) => log::info!("Gpio + SPI Error"),
                    _ => log::info!("Unknown Error"),
                };
                return;
            }
        };

        // Read all of the registers
        for register in 0x00..=0x06 {
            let mut config = [0u8];
            match radio.read_register(register, &mut config, ctx.local.spi).await {
                Ok(_) => log::info!("Register {:#x} --- {:#010b}", register, config[0]),
                Err(_) => log::info!("Error Reading Register {:#x}", register),
            }
        }

        let mut base_rx = [0u8; 5];
        match radio.read_register(0x0A, &mut base_rx, ctx.local.spi).await {
            Ok(_) => log::info!("Base Read --- {:#x?}", base_rx),
            Err(_) => log::info!("Error Reading Base Read Address"),
        }

        for register in 0x0B..=0x0F {
            let mut config = [0u8];
            match radio.read_register(register, &mut config, ctx.local.spi).await {
                Ok(_) => log::info!("RX Register {:#x} --- {:#x}", register, config[0]),
                Err(_) => log::info!("Error Reading Register {:#x}", register),
            }
        }

        let mut tx = [0u8; 5];
        match radio.read_register(0x10, &mut tx, ctx.local.spi).await {
            Ok(_) => log::info!("Tx --- {:#x?}", tx),
            Err(_) => log::info!("Error Reading Tx Address"),
        }

        for register in 0x1C..=0x1D {
            let mut config = [0u8];
            match radio.read_register(register, &mut config, ctx.local.spi).await {
                Ok(_) => log::info!("Register {:#x} --- {:#010b}", register, config[0]),
                Err(_) => log::info!("Error Reading Register {:#x}", register),
            }
        }

        match radio.read_status(ctx.local.spi).await {
            Ok(status) => log::info!("Status {:#010b}", status),
            Err(_) => log::info!("Unable to Read Status"),
        }

        for _ in 0..=10 {
            Systick::delay(1_000u32.millis()).await;

            log::info!("Sending Hello World");

            match radio.send_data(b"Hello World ", false, ctx.local.spi).await {
                Ok(_) => log::info!("Sent Hello World"),
                Err(err) => match err {
                    Error::InvalidPipeId => log::info!("Invalid Pipe ID"),
                    Error::TooLargeAckPayload => log::info!("Too Large Ack Payload"),
                    Error::InvalidBufferSize => log::info!("Invalid Buffer Size"),
                    Error::InvalidRegisterBufferSize(register) => log::info!("Invalid Register Buffer Size {:#x} = {}", register, register),
                    Error::InvalidAddressBufferSize => log::info!("Invalid Address Buffer Size"),
                    Error::UnknownRegister => log::info!("Unknown Register"),
                    Error::UnableToConfigureRegister(register, expected, found) => log::info!("Unable to Configure Register: {:#x}\nExpected\n{:#010b}\nFound\n{:#010b}", register, expected, found),
                    Error::GpioError(_) => log::info!("GPIO Error"),
                    Error::SpiError(err) => match err {
                        LpspiError::FrameSize => log::info!("SPI Error Frame Size"),
                        LpspiError::Fifo(direction) => log::info!("SPI Error Direction: {:?}", direction),
                        LpspiError::Busy => log::info!("SPI Error Busy"),
                        LpspiError::NoData => log::info!("SPI Error No Data"),
                    },
                    Error::GpioSpiError(_) => log::info!("Gpio + SPI Error"),
                    _ => log::info!("Unknown Error"),
                },
            }
        }
    }
}