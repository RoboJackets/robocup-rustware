#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT2])]
mod app {
    use super::*;

    use core::mem::MaybeUninit;

    use embedded_hal::spi::MODE_0;

    use rtic_nrf24l01::{config::*, Radio};

    use teensy4_bsp::board::LPSPI_FREQUENCY;
    use teensy4_bsp::hal::lpspi::Pins;

    use bsp::board;
    use teensy4_bsp as bsp;

    use bsp::hal;
    use hal::timer::Blocking;

    use rtic_monotonics::systick::*;

    use robojackets_robocup_control::{
        Delay2, RadioCE, RadioCSN, RadioSPI, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY,
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        spi: RadioSPI,
        delay: Delay2,
        ce: Option<RadioCE>,
        csn: Option<RadioCSN>,
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins,
            mut gpio1,
            usb,
            mut gpt2,
            lpspi4,
            ..
        } = board::t41(ctx.device);

        // usb logging setup
        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        // systic setup
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // gpt 1 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let delay = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        // Initialize Shared SPI
        let shared_spi_pins = Pins {
            pcs0: pins.p10,
            sck: pins.p13,
            sdo: pins.p11,
            sdi: pins.p12,
        };
        let mut shared_spi = hal::lpspi::Lpspi::new(lpspi4, shared_spi_pins);

        shared_spi.disabled(|spi| {
            spi.set_clock_hz(LPSPI_FREQUENCY, 5_000_000u32);
            spi.set_mode(MODE_0);
        });

        // Init radio cs pin and ce pin
        let radio_cs = gpio1.output(pins.p14);
        let ce = gpio1.output(pins.p41);

        // init fake CS pin (TEMPORARY) and required reset pin

        init_radio::spawn().unwrap();

        (
            Shared {},
            Local {
                delay,
                spi: shared_spi,
                ce: Some(ce),
                csn: Some(radio_cs),
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

    #[task(local = [delay, spi, ce, csn], priority = 1)]
    async fn init_radio(ctx: init_radio::Context) {
        Systick::delay(2_000u32.millis()).await;

        let csn = ctx.local.csn.take().unwrap();
        let ce = ctx.local.ce.take().unwrap();

        let mut radio = Radio::new(ce, csn);

        if let Err(err) = radio.begin(ctx.local.spi, ctx.local.delay) {
            log::info!("Radio Error: {:?}", err);
        }

        radio.set_pa_level(
            power_amplifier::PowerAmplifier::PALow,
            ctx.local.spi,
            ctx.local.delay,
        );

        radio.set_payload_size(4, ctx.local.spi, ctx.local.delay);

        radio.open_writing_pipe(
            [0xE7, 0xE7, 0xE7, 0xE7, 0xE7],
            ctx.local.spi,
            ctx.local.delay,
        );

        radio.open_reading_pipe(
            1,
            [0xC3, 0xC3, 0xC3, 0xC3, 0xC3],
            ctx.local.spi,
            ctx.local.delay,
        );

        radio.start_listening(ctx.local.spi, ctx.local.delay);

        Systick::delay(1_000u32.millis()).await;

        radio.stop_listening(ctx.local.spi, ctx.local.delay);

        log::info!(
            "Configuration: {:?}",
            radio.get_registers(ctx.local.spi, ctx.local.delay)
        );

        let mut payload = 0u32;
        loop {
            let p = crate::to_bytes(&payload);
            log::info!("Sending P: {:?}", p);
            let report = radio.write(&p, ctx.local.spi, ctx.local.delay);

            if report {
                log::info!("Sent Successfully");
                log::info!("Sent {:?}", payload);
                payload += 1;
            } else {
                log::info!("Transmission Failed");
            }

            Systick::delay(1_000u32.millis()).await;

            log::info!(
                "Configuration: {:?}",
                radio.get_registers(ctx.local.spi, ctx.local.delay)
            );
        }
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}

fn to_bytes(value: &u32) -> [u8; 4] {
    [
        (value & 0xFF) as u8,
        ((value & (0xFF << 8)) >> 8) as u8,
        ((value & (0xFF << 16)) >> 16) as u8,
        ((value & (0xFF << 24)) >> 24) as u8,
    ]
}
