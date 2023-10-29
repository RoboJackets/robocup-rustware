#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use sx127::RadioMode;
    use teensy4_bsp::hal::gpt::Gpt2;
    use teensy4_pins::common::*;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::gpio::{self, Trigger};
    use hal::gpt::{Gpt1, ClockSource};
    use hal::timer::Blocking;

    use rtic_monotonics::systick::*;

    use packed_struct::prelude::*;
    use packed_struct::types::bits::ByteArray;

    use robojackets_robocup_rtp::{Team, MessageType};
    use robojackets_robocup_rtp::control_command::ControlCommand;
    use robojackets_robocup_rtp::control_message::ControlMessage;
    use robojackets_robocup_rtp::robot_status_message::RobotStatusMessage;

    const DELAY_MS: u32 = 5_000;
    const FREQUENCY: i64 = 915;
    const GPT1_FREQUENCY: u32 = 1_000;
    const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    type Delay = Blocking<Gpt1, GPT1_FREQUENCY>;
    type BlockingDelay = Blocking<Gpt2, GPT1_FREQUENCY>;
    type Interrupt = gpio::Input<P4>;
    type Radio = sx127::LoRa<board::Lpspi4, gpio::Output<P8>, gpio::Output<P9>, Delay>;

    #[local]
    struct Local {
        rx_int: Interrupt,
    }

    #[shared]
    struct Shared {
        radio: Radio,
        blocking_delay: BlockingDelay,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        let board::Resources {
            pins,
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

        // systic setup
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // gpt 1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT1_DIVIDER);
        gpt1.set_clock_source(GPT1_CLOCK_SOURCE);
        let delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

        gpt2.disable();
        gpt2.set_divider(GPT1_DIVIDER);
        gpt2.set_clock_source(GPT1_CLOCK_SOURCE);
        let blocking_delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt2);

        // RX DONE Interrupt setup
        let rx_int = gpio4.input(pins.p4);
        gpio1.set_interrupt(&rx_int, Some(Trigger::RisingEdge));

        // initialize spi
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

        // init fake CS pin (TEMPORARY) and required reset pin
        let fake_cs = gpio2.output(pins.p8);
        let reset = gpio2.output(pins.p9);

        // initialize radio
        let radio = match sx127::LoRa::new(spi, fake_cs, reset, FREQUENCY, delay) {
            Ok(radio) => radio,
            Err(err) => match err {
                sx127::Error::VersionMismatch(version) => panic!("Version Mismatch Error {:?}", version),
                sx127::Error::CS(_) => panic!("Chip select issue"),
                sx127::Error::Reset(_) => panic!("Rest Issue"),
                sx127::Error::SPI(_) => panic!("SPI problem"),
                sx127::Error::Transmitting => panic!("Error during spi transmission"),
                sx127::Error::Uninformative => panic!("Uninformative error RIP"),
            }
        };

        init_radio::spawn().unwrap();

        (
            Shared {
                radio,
                blocking_delay,
            },
            Local {
                rx_int
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(shared = [radio], priority = 1)]
    async fn init_radio(mut ctx: init_radio::Context) {
        Systick::delay(DELAY_MS.millis()).await;

        ctx.shared.radio.lock(|radio| {
            log::info!("Lora Version: {:#04x}", radio.get_radio_version().unwrap());

            match radio.set_tx_power(17, 1) {
                Ok(_) => log::info!("Successfully set TX power"),
                Err(_) => panic!("Error setting tx power"),
            };

            log::info!("Successfully Initialized Radio");

            match radio.set_mode(RadioMode::RxContinuous) {
                Ok(_) => log::info!("Radio Listening..."),
                Err(_) => panic!("Couldn't set radio to listening"),
            }
        });
    }

    #[task(binds = GPIO1_COMBINED_0_15, local = [rx_int], shared = [radio, blocking_delay])]
    fn receive_data(mut ctx: receive_data::Context) {
        log::info!("Interrupt Triggered");

        (ctx.shared.radio, ctx.shared.blocking_delay).lock(|radio, blocking_delay| {
            match radio.read_packet() {
                Ok(buffer) => {
                    match MessageType::from(buffer[0]) {
                        MessageType::ControlCommand => {
                            let control_command_length = <ControlCommand as PackedStruct>::ByteArray::len();
                            let control_command = ControlCommand::unpack_from_slice(&buffer[1..(1+control_command_length)]);
                            
                            log::info!("Received Control Command: {:?}", control_command);

                            // Respond with a robot status response
                            let status = RobotStatusMessage::new(
                                Team::Blue,
                                0u8,
                                false,
                                false,
                                false,
                                255u8,
                                0u8,
                                false,
                                [0u16; 18],
                            );

                            let packed_data = match status.pack() {
                                Ok(bytes) => bytes,
                                Err(err) => panic!("Unable to Send Data {:?}", err),
                            };

                            let packed_data = packed_data.as_bytes_slice();

                            let mut packet = [0u8; 255];
                            packet[0..packed_data.len()].copy_from_slice(packed_data);

                            match radio.transmit_payload_busy(packet, packed_data.len()) {
                                Ok(_) => log::info!("Successfully Sent Response"),
                                Err(_) => panic!("Unable to Send Response"),
                            }

                            if radio.set_mode(RadioMode::Stdby).is_err() {
                                panic!("Unable to set to standby");
                            }

                            blocking_delay.block_ms(100);
                        },
                        MessageType::ControlMessage => {
                            let control_message_length = <ControlMessage as PackedStruct>::ByteArray::len();
                            let control_message = ControlMessage::unpack_from_slice(&buffer[1..(1+control_message_length)]);
                            
                            log::info!("Received Control Message: {:?}", control_message);

                            let status = RobotStatusMessage::new(
                                Team::Blue,
                                0u8,
                                true,
                                true,
                                true,
                                255u8,
                                0u8,
                                true,
                                [255u16; 18],
                            );

                            let packed_data = match status.pack() {
                                Ok(bytes) => bytes,
                                Err(err) => panic!("Unable to Send Data {:?}", err),
                            };

                            let packed_data = packed_data.as_bytes_slice();

                            let mut packet = [0u8; 255];
                            packet[0..packed_data.len()].copy_from_slice(packed_data);

                            match radio.transmit_payload_busy(packet, packed_data.len()) {
                                Ok(_) => log::info!("Successfully Sent Response"),
                                Err(_) => panic!("Unable to Send Response"),
                            }

                            if radio.set_mode(RadioMode::Stdby).is_err() {
                                panic!("Unable to set to standby");
                            }

                            blocking_delay.block_ms(100);
                        },
                        _ => log::info!("Unknown Packet Found: {:?}", buffer),
                    }
                },
                Err(_) => panic!("Error while reading data"),
            }

            if radio.set_mode(RadioMode::RxContinuous).is_err() {
                panic!("Unable to Set to Receive");
            }

            ctx.local.rx_int.clear_triggered();

            blocking_delay.block_ms(10);
        });
    }
}