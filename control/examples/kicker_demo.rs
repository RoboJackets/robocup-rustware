//!
//! This test charges the kicker, printing the current voltage of the kicker while
//! powering up for 2 seconds.  Then kicks and stops charging the kicker.
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
    use bsp::board::{self};
    use teensy4_bsp as bsp;

    use rtic_monotonics::systick::*;

    use kicker_controller::{KickTrigger, KickType, Kicker, KickerCommand};

    use robojackets_robocup_control::{
        KickerSpi, KickerCSn, KickerReset, Killn, MotorEn,
    };
    
    use bsp::hal::iomuxc;
    use bsp::ral;
    use bsp::hal::lpspi::{Lpspi, Pins};
    use embedded_hal::spi::MODE_3;
    use ral::lpspi::LPSPI3;

    #[local]
    struct Local {
        kicker_controller: Kicker<KickerCSn, KickerReset>,
        spi: KickerSpi,
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            mut gpio1,
            mut gpio2,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);
        

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();

        // Define SPI pins
        let spi_pins = Pins {
            pcs0: pins.p38,
            sck: pins.p27,
            sdo: pins.p26,
            sdi: pins.p39,
        };

        // Generate instance of LPSPI3 to manually populate
        let spi_block = unsafe { LPSPI3::instance() };
        let spi_temp = Lpspi::new(spi_block, spi_pins);

        // Release pins to split into kicker
        let (spi_block, mut spi_pins) = spi_temp.release();

        // Manually configure the data pins for LPSPI3 function
        iomuxc::lpspi::prepare(&mut spi_pins.sdo);  // SDO/MOSI
        iomuxc::lpspi::prepare(&mut spi_pins.sdi);  // SDI/MISO
        iomuxc::lpspi::prepare(&mut spi_pins.sck);  // SCK

        // Initialize SPI and Kicker controll
        let mut spi = Lpspi::without_pins(spi_block);
        let kicker = Kicker::new(gpio1.output(spi_pins.pcs0), gpio2.output(pins.p37));

        // Config SPI
        spi.disabled(|spi| {
            spi.set_mode(MODE_3);                  // CPOL=1, CPHA=1 to match Pico
            spi.set_clock_hz(board::LPSPI_FREQUENCY, 2_000_000);
        });


        kicker_test::spawn().ok();

        (
            Shared {},
            Local {
                kicker_controller: kicker,
                spi,
                poller,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        local = [kicker_controller, spi],
        priority = 1
    )]
    async fn kicker_test(ctx: kicker_test::Context) {
        
        Systick::delay(1_000u32.millis()).await;
        
        

        log::info!("Charging the Kicker");
        let kicker_command = KickerCommand {
            kick_type: KickType::Kick,
            kick_trigger: KickTrigger::Breakbeam,
            kick_strength: 15,
            charge_allowed: true,
        };
        log::info!("Raw Out: {:?}", kicker_command);
        
        loop {
            let kicker_status = ctx
                .local
                .kicker_controller
                .service(kicker_command, ctx.local.spi)
                .unwrap();
            log::info!("Kicker Status: {:?}", kicker_status);
            Systick::delay(1000u32.millis()).await;
        }

        

        log::info!("KICKING!!!");
        let kicker_command = KickerCommand {
            kick_type: KickType::Kick,
            kick_trigger: KickTrigger::Immediate,
            kick_strength: 15,
            charge_allowed: true,
        };
        let kicker_status = ctx
            .local
            .kicker_controller
            .service(kicker_command, ctx.local.spi)
            .unwrap();
        log::info!("Kicker Status: {:?}", kicker_status);

        log::info!("Charging the Chipper");
        let kicker_command = KickerCommand {
            kick_type: KickType::Chip,
            kick_trigger: KickTrigger::Disabled,
            kick_strength: 5,
            charge_allowed: true,
        };
        for _ in 0..20 {
            let kicker_status = ctx
                .local
                .kicker_controller
                .service(kicker_command, ctx.local.spi)
                .unwrap();
            log::info!("Kicker Status: {:?}", kicker_status);
            Systick::delay(100u32.millis()).await;
        }

        log::info!("CHIPPING!!!!");
        let kicker_command = KickerCommand {
            kick_type: KickType::Chip,
            kick_trigger: KickTrigger::Immediate,
            kick_strength: 15,
            charge_allowed: true,
        };
        let kicker_status = ctx
            .local
            .kicker_controller
            .service(kicker_command, ctx.local.spi)
            .unwrap();
        log::info!("Kicker Status: {:?}", kicker_status);

        Systick::delay(100u32.millis()).await;

        log::info!("Charging the Kicker");
        let kicker_command = KickerCommand {
            kick_type: KickType::Kick,
            kick_trigger: KickTrigger::Disabled,
            kick_strength: 10,
            charge_allowed: true,
        };
        for _ in 0..20 {
            let kicker_status = ctx
                .local
                .kicker_controller
                .service(kicker_command, ctx.local.spi)
                .unwrap();
            log::info!("Kicker Status: {:?}", kicker_status);
            Systick::delay(100u32.millis()).await;
        }

        loop {
            log::info!("Kick on Breakbeam");
            let kicker_command = KickerCommand {
                kick_type: KickType::Kick,
                kick_trigger: KickTrigger::Breakbeam,
                kick_strength: 10,
                charge_allowed: true,
            };
            let kicker_status = ctx
                .local
                .kicker_controller
                .service(kicker_command, ctx.local.spi)
                .unwrap();
            log::info!("Kicker Status: {:?}", kicker_status);
            Systick::delay(100u32.millis()).await;
        }
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
