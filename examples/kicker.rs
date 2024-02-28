//!
//! This demo tests the operation of the kicker by utilizing the various kick types of the
//! kicker
//!
//! For Testing Please Connect the Kicker Testing Board which consists of:
//!     3 Buttons (connected to gpio 28, 29, 30, 31)
//!         gpio 29 -> Toggle Immediate, OnBreakBeam, Off
//!         gpio 30 -> Kick
//!         gpio 31 -> Chip
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device=teensy4_bsp, peripherals=true, dispatchers=[GPT2])]
mod app {
    use super::*;

    use core::convert::Infallible;
    use core::mem::MaybeUninit;

    use embedded_hal::spi::MODE_0;

    use teensy4_pins::t41::*;

    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::board::LPSPI_FREQUENCY;

    use bsp::hal as hal;
    use hal::lpspi::{LpspiError, Lpspi};
    use hal::gpt::{ClockSource, Gpt1};
    use hal::gpio::{Output, Input, Trigger, Port};
    use hal::timer::Blocking;

    use bsp::ral as ral;
    use ral::lpspi::LPSPI3;

    use kicker::Kicker;

    use rtic_monotonics::systick::*;

    use robojackets_robocup_rtp::{ControlMessage, ControlMessageBuilder};

    /// Constants
    const GPT_FREQUENCY: u32 = 1_000;
    const GPT_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT_FREQUENCY;

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    /// Type Definitions
    // SPI 2
    type SharedSPI = Lpspi<board::LpspiPins<P26, P39, P27, P38>, 3>;
    type KickerCSN = Output<P34>;
    // DELAYS
    type Delay1 = Blocking<Gpt1, GPT_FREQUENCY>;

    #[local]
    struct Local {
        kicker: Kicker<KickerCSN, SharedSPI, Delay1, Infallible, LpspiError>,
    }

    #[shared]
    struct Shared {
        gpio3: Port<3>,
        gpio4: Port<4>,
        activation_pin: Input<P29>,
        kick_pin: Input<P30>,
        chip_pin: Input<P31>,
        delay: Delay1,
        shared_spi: SharedSPI,
        control_message: ControlMessage,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        let board::Resources {
            pins,
            mut gpio2,
            mut gpio3,
            mut gpio4,
            usb,
            mut gpt1,
            ..
        } = board::t41(ctx.device);

        // usb logging setup
        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // gpt1 as a blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        let shared_spi_pins = hal::lpspi::Pins {
            pcs0: pins.p38,
            sck: pins.p27, 
            sdo: pins.p26,
            sdi: pins.p39,
        };
        let shared_spi_block = unsafe { LPSPI3::instance() };
        let mut shared_spi = hal::lpspi::Lpspi::new(shared_spi_block, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 100_000u32);
            spi.set_mode(MODE_0);
        });
        
        let kicker_csn = gpio2.output(pins.p34);
        let kicker = Kicker::new(kicker_csn);

        let activation_pin = gpio4.input(pins.p29);
        gpio3.set_interrupt(&activation_pin, Some(Trigger::FallingEdge));

        let kick_pin = gpio3.input(pins.p30);
        gpio3.set_interrupt(&kick_pin, Some(Trigger::FallingEdge));

        let chip_pin = gpio3.input(pins.p31);
        gpio2.set_interrupt(&chip_pin, Some(Trigger::FallingEdge));

        kicker_loop::spawn().ok();

        (
            Shared {
                gpio3,
                gpio4,
                activation_pin,
                kick_pin,
                chip_pin,
                delay,
                shared_spi,
                control_message: ControlMessageBuilder::new().build(),
            },
            Local {
                kicker,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local=[kicker], shared=[delay, shared_spi, control_message], priority=1)]
    async fn kicker_loop(ctx: kicker_loop::Context) {
        (
            ctx.shared.delay,
            ctx.shared.shared_spi,
            ctx.shared.control_message,
        ).lock(|delay, spi, control_message| {
            if ctx.local.kicker.service(control_message, spi, delay).is_err() {
                log::info!("Unable to Service the Kicker");
            }
        });

        kicker_delay::spawn().ok();
    }

    #[task(binds=GPIO3_COMBINED_16_31, shared=[gpio3, chip_pin, kick_pin, gpio4, activation_pin, control_message], priority=1)]
    fn kick_chip(ctx: kick_chip::Context) {
        (
            ctx.shared.gpio3,
            ctx.shared.chip_pin,
            ctx.shared.kick_pin,
            ctx.shared.gpio4,
            ctx.shared.activation_pin,
            ctx.shared.control_message
        ).lock(|gpio3, chip_pin, kick_pin, gpio4, activation_pin, control_message| {
            gpio3.set_interrupt(chip_pin, None);
            gpio3.set_interrupt(kick_pin, None);
            gpio4.set_interrupt(activation_pin, None);

            if chip_pin.is_triggered() {
                chip_pin.clear_triggered();
                log::info!("Chip On");

                control_message.shoot_mode = true;
            } else if kick_pin.is_triggered() {
                kick_pin.clear_triggered();
                log::info!("Kick On");

                control_message.shoot_mode = false;
            }
        });

        reset_interrupts::spawn().ok();
    }

    #[task(binds=GPIO4_COMBINED_16_31, shared=[gpio3, chip_pin, kick_pin, gpio4, activation_pin, control_message], priority=1)]
    fn power_kicker(ctx: power_kicker::Context) {
        (
            ctx.shared.gpio3,
            ctx.shared.chip_pin,
            ctx.shared.kick_pin,
            ctx.shared.gpio4,
            ctx.shared.activation_pin,
            ctx.shared.control_message,
        ).lock(|gpio3, chip_pin, kick_pin, gpio4, activation_pin, control_message| {
            activation_pin.clear_triggered();
            gpio4.set_interrupt(activation_pin, None);
            gpio3.set_interrupt(chip_pin, None);
            gpio3.set_interrupt(kick_pin, None);

            control_message.trigger_mode = match *control_message.trigger_mode {
                0 => {
                    log::info!("Immediate Mode On");
                    1.into()
                },
                1 => {
                    log::info!("Breakbeam Mode On");
                    2.into()
                },
                2 => {
                    log::info!("Off");
                    3.into()
                },
                _ => {
                    log::info!("Power Up");
                    0.into()
                }
            };
        });

        reset_interrupts::spawn().ok();
    }

    #[task(shared=[gpio3, gpio4, kick_pin, chip_pin, activation_pin], priority=1)]
    async fn reset_interrupts(ctx: reset_interrupts::Context) {
        Systick::delay(100u32.millis()).await;

        (
            ctx.shared.gpio3,
            ctx.shared.gpio4,
            ctx.shared.kick_pin,
            ctx.shared.chip_pin,
            ctx.shared.activation_pin,
        ).lock(|gpio3, gpio4, kick_pin, chip_pin, activation_pin| {
            gpio3.set_interrupt(kick_pin, Some(Trigger::FallingEdge));
            gpio3.set_interrupt(chip_pin, Some(Trigger::FallingEdge));
            gpio4.set_interrupt(activation_pin, Some(Trigger::FallingEdge));
        });
    }

    #[task(priority=1)]
    async fn kicker_delay(_ctx: kicker_delay::Context) {
        Systick::delay(40u32.millis()).await;

        kicker_loop::spawn().ok();
    }
}