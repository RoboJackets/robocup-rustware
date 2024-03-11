#![no_std]
#![no_main]

use bsp::board::{self, LPSPI_FREQUENCY};
use teensy4_bsp as bsp;
use teensy4_panic as _;

use bsp::hal as hal;
use bsp::ral::lpspi::LPSPI3;

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        ..
    } = board::t41(board::instances());

    let spi_pins = hal::lpspi::Pins {
        pcs0: pins.p38,
        sck: pins.p27,
        sdo: pins.p26,
        sdi: pins.p39,
    };
    let spi3 = unsafe { LPSPI3::instance() };
    let mut radio_spi = hal::lpspi::Lpspi::new(spi3, spi_pins);

    radio_spi.disabled(|spi| {
        spi.set_clock_hz(LPSPI_FREQUENCY, 1_000_000);
    });

    loop {

    }
}