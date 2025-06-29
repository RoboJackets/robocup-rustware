#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use rtic_monotonics::systick::*;

use embedded_graphics::prelude::*;
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use graphics::error_screen::ErrorScreen;
    use graphics::main_window::MainWindow;
    use teensy4_bsp::board::Lpi2c1;
    use teensy4_pins::t41::{P18, P19};

    use core::mem::MaybeUninit;

    const HEAP_SIZE: usize = 1024 * 8;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[shared]
    struct Shared {
        display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
            DisplaySize128x64,
            ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
        >,
    }

    #[local]
    struct Local {
        main_window: MainWindow<'static>,
        error_screen: ErrorScreen<'static>,
        latency_placeholder: u16,
        poller: imxrt_log::Poller,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins, lpi2c1, usb, ..
        } = board::t41(cx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 36_000_000, systick_token);

        let i2c: Lpi2c1 = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::MHz1);
        let interface = I2CDisplayInterface::new(i2c);
        let display: Ssd1306<
            I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        > = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        let main_window = MainWindow::new(0, "Blue");
        let error_screen = ErrorScreen::new(
            "Example Program",
            "Lorem ipsum dolor sit amet, consectetur adipiscing elit, 
            sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.",
        );
        let latency_placeholder: u16 = 0;

        init_devices::spawn().ok();
        (
            Shared { display },
            Local {
                main_window,
                error_screen,
                latency_placeholder,
                poller,
            },
        )
    }

    // Loops between the main screen and an error screen with sample text.
    // Does not demonstrate any other devices - Does not read battery, ball sensor, etc.
    #[task(priority=1, shared = [display])]
    async fn init_devices(mut _cx: init_devices::Context) {
        _cx.shared.display.lock(|display| {
            display.init().ok();
        });
        main_window_test::spawn().ok();
    }

    #[task(priority=1, shared=[display], local=[latency_placeholder, main_window])]
    async fn main_window_test(mut _cx: main_window_test::Context) {
        for _i in 0..30 {
            *_cx.local.latency_placeholder += 1;
            _cx.local.main_window.latency = *_cx.local.latency_placeholder as u32;
            _cx.local.main_window.team = if *_cx.local.latency_placeholder % 2 == 0 {
                "blue"
            } else {
                "yellow"
            };
            _cx.local.main_window.ball_sense = if *_cx.local.latency_placeholder % 2 == 0 {
                true
            } else {
                false
            };
            _cx.local.main_window.kicker_charged = if *_cx.local.latency_placeholder % 2 == 0 {
                true
            } else {
                false
            };
            _cx.shared.display.lock(|display| {
                display.clear();
                _cx.local.main_window.draw(display).ok();
                display.flush().ok();
            });
            Systick::delay(5000000.micros()).await;
        }
        *_cx.local.latency_placeholder = 0;
        error_screen_test::spawn().ok();
    }

    #[task(priority=1, shared=[display], local=[error_screen])]
    async fn error_screen_test(mut _cx: error_screen_test::Context) {
        _cx.shared.display.lock(|display| {
            display.clear();
            _cx.local.error_screen.draw(display).ok();
            display.flush().ok();
        });
        Systick::delay(250000000.micros()).await;
        main_window_test::spawn().ok();
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
