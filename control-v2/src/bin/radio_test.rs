#![no_std]
#![no_main]

extern crate alloc;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use core::{cell::RefCell, default::Default, mem::MaybeUninit};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Input, Level, Output, Speed},
};
use embassy_stm32::{i2c, spi};
use embassy_sync::{
    blocking_mutex::{NoopMutex, raw::NoopRawMutex},
    pubsub::PubSubChannel,
};
use embassy_time::{Delay, Duration, Timer};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};
use static_cell::StaticCell;

use control_v2::{
    gpio::{decode_robot_id, decode_team},
    graphics,
    radio::{self, control_command::ControlMessage},
    utils,
};
use rf24::radio::RF24;

// Allocator
use embedded_alloc::LlffHeap;
#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();
const HEAP_SIZE: usize = 1024 * 8;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

/// The channel for publishing ControlMessages received from the radio
pub static COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, ControlMessage, 4, 2, 1>> =
    StaticCell::new();
/// The channel for publishing kicker states
pub static KICKER_STATE_CHANNEL: StaticCell<
    PubSubChannel<NoopRawMutex, kicker_controller::KickerState, 4, 2, 1>,
> = StaticCell::new();
// The channel for publishing the current battery voltage
pub static BATTERY_VOLTAGE_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, f32, 4, 2, 1>> =
    StaticCell::new();

// Gpio Interrupts
bind_interrupts!(pub struct GpioIrqs {
    EXTI0 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI0>;
    EXTI1 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI1>;
    EXTI15_10 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI15_10>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe {
        #[allow(static_mut_refs)]
        HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
    }

    let p = embassy_stm32::init(Default::default());

    // Initialize the dip switches
    let s1 = Input::new(p.PE5, embassy_stm32::gpio::Pull::None);
    let s2 = Input::new(p.PE4, embassy_stm32::gpio::Pull::None);
    let s3 = Input::new(p.PE3, embassy_stm32::gpio::Pull::None);
    let s4 = Input::new(p.PE2, embassy_stm32::gpio::Pull::None);

    let robot_id = decode_robot_id(s1.is_high(), s2.is_high(), s3.is_high());
    let team = decode_team(s4.is_high());

    // Initialize the display
    let display_i2c = i2c::I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        graphics::DisplayIrqs,
        p.DMA1_CH2,
        p.DMA1_CH3,
        i2c::Config::default(),
    );
    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(display_i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    if let Err(_err) = display.init() {
        loop {
            error!("Error Initializing Display");
            Timer::after(Duration::from_secs(5)).await;
        }
    }

    // Initialize the Radio
    let radio_spi = spi::Spi::new(
        p.SPI4,
        p.PE12,
        p.PE14,
        p.PE13,
        p.DMA1_CH4,
        p.DMA1_CH5,
        spi::Config::default(),
    );
    let radio_spi = radio::RADIO_SPI.init(NoopMutex::new(RefCell::new(radio_spi)));
    let csn = Output::new(p.PE11, Level::High, Speed::High);
    let ce = Output::new(p.PE9, Level::Low, Speed::Low);
    let radio_spi = SpiDevice::new(radio_spi, csn);
    let mut rf_radio = RF24::new(ce, radio_spi, Delay);
    radio::init_radio(&mut rf_radio, &mut display, robot_id, team);
    let radio_irq = embassy_stm32::exti::ExtiInput::new(
        p.PE10,
        p.EXTI10,
        embassy_stm32::gpio::Pull::None,
        GpioIrqs,
    );

    // Create communication channels
    let command_channel =
        COMMAND_CHANNEL.init(PubSubChannel::<NoopRawMutex, ControlMessage, 4, 2, 1>::new());
    let battery_voltage_channel =
        BATTERY_VOLTAGE_CHANNEL.init(PubSubChannel::<NoopRawMutex, f32, 4, 2, 1>::new());
    let kicker_channel = KICKER_STATE_CHANNEL.init(PubSubChannel::<
        NoopRawMutex,
        kicker_controller::KickerState,
        4,
        2,
        1,
    >::new());

    // Spawn the Radio Receive Task
    spawner
        .spawn(radio::receive_radio_data(
            radio_irq,
            rf_radio,
            robot_id,
            team,
            command_channel.publisher().unwrap(),
            kicker_channel.subscriber().unwrap(),
            battery_voltage_channel.subscriber().unwrap(),
        ))
        .unwrap();

    // Spawn the Radio Testing Task
    spawner
        .spawn(utils::radio_testing_task(
            command_channel.subscriber().unwrap(),
            kicker_channel.publisher().unwrap(),
            battery_voltage_channel.publisher().unwrap(),
        ))
        .unwrap();
}
